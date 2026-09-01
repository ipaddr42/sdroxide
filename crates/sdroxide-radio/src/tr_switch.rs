//! The station's external transmit/receive switch, as the engines see it.
//!
//! One shared [`TrSwitch`] is handed to every engine in the process, the way
//! [`crate::TxGate`] is, and for a related reason: the relay that grounds the
//! SDR's antenna belongs to the *station*, not to any one radio. Several
//! engines may be running, any of them may key, and the contacts must follow
//! all of them — so each engine publishes whether it is on the air and the
//! switch follows the OR. The last radio to unkey is what releases it; a radio
//! that never keyed cannot.
//!
//! The hardware itself lives behind `sdroxide_relay::RelayHandle`, which is a
//! thread and a port. This type is the arbitration in front of it, and it is
//! deliberately almost nothing: two atomics on the hot path, because
//! [`TrSwitch::publish`] is called on every engine tick and [`TrSwitch::key`]
//! sits in the key-down path at its most time-critical moment.
//!
//! # What the engine owes it
//!
//! * [`TrSwitch::key`] **before** RF, and the caller must wait the returned
//!   lead. That is the one guarantee the whole subsystem exists to make.
//! * [`TrSwitch::unkey`] after RF stops — the hold times are the driver's
//!   business, so this returns at once.
//! * [`TrSwitch::abort`] when a key-down was refused *after* the contacts were
//!   thrown. No RF appeared, so there is nothing to protect on the way out and
//!   the hold would only delay the receiver coming back.
//! * [`TrSwitch::publish`] every tick, for the overs sdroxide does not drive:
//!   a transceiver keyed at its own microphone, or a rig sending CW from its
//!   own keyer.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use sdroxide_types::{FailSafe, RelayConfig, RelayStatus};

/// See the module doc.
#[derive(Default)]
pub struct TrSwitch {
    /// One bit per radio id. Non-zero means the station is on the air.
    on_air: AtomicU64,
    /// The driver, installed by whichever engine owns the configuration.
    driver: std::sync::Mutex<Option<sdroxide_relay::RelayHandle>>,
    /// Whether a switch is configured at all, so the hot path can answer
    /// without taking the lock.
    configured: AtomicBool,
    /// Whether a switch that will not answer should refuse the over.
    refuse: AtomicBool,
    /// Why the switch could not be opened, when it could not. Separate from the
    /// driver's own status because there is no driver to ask.
    open_error: std::sync::Mutex<Option<String>>,
    /// Which radio the transmit-sense line belongs to.
    ///
    /// Here rather than read from each engine's own copy of the configuration,
    /// because those copies drift: the engine that was handed a
    /// `Command::SetRelayConfig` updates its own and the others keep what they
    /// loaded at startup. Every other stale field is cosmetic; this one decides
    /// which receiver gets muted and which key-down gets refused when a
    /// transmitter out in the shack comes up, so it lives with the driver it
    /// came in with.
    sense_radio: AtomicU32,
}

impl TrSwitch {
    pub fn new() -> TrSwitch {
        TrSwitch::default()
    }

    /// Install (or replace, or remove) the driver.
    ///
    /// The old handle is taken out *under* the lock and dropped *outside* it:
    /// dropping one joins a thread, and a key-down queueing behind that would
    /// be a key-down waiting on a serial port to close.
    pub fn install(&self, handle: Option<sdroxide_relay::RelayHandle>, cfg: &RelayConfig) {
        self.configured.store(cfg.enabled(), Ordering::Release);
        self.refuse.store(cfg.fail_safe == FailSafe::RefuseTx, Ordering::Release);
        self.sense_radio.store(cfg.sense.radio, Ordering::Release);
        let old = {
            let mut slot = self.driver.lock().unwrap_or_else(|e| e.into_inner());
            std::mem::replace(&mut *slot, handle)
        };
        drop(old);
    }

    /// Record why there is no driver, for the status line and the refusal.
    pub fn set_open_error(&self, why: Option<String>) {
        *self.open_error.lock().unwrap_or_else(|e| e.into_inner()) = why;
    }

    /// Whether any radio is on the air. A configuration must not be applied
    /// while this is true — rebuilding the driver throws every contact.
    pub fn busy(&self) -> bool {
        self.on_air.load(Ordering::Acquire) != 0
    }

    /// Claim the air for `radio` and throw the contacts. Returns how long the
    /// caller must wait before letting RF out.
    ///
    /// Zero when the station was already on the air: the contacts are closed
    /// already and a second radio joining an over it is sharing has nothing to
    /// wait for.
    pub fn key(&self, radio: u32) -> std::time::Duration {
        let was = self.on_air.fetch_or(bit(radio), Ordering::AcqRel);
        if was != 0 || !self.configured.load(Ordering::Acquire) {
            return std::time::Duration::ZERO;
        }
        match self.driver.lock() {
            Ok(d) => d.as_ref().map(|h| h.key()).unwrap_or_default(),
            Err(_) => std::time::Duration::ZERO,
        }
    }

    /// Release `radio`'s claim. The contacts open only when the last one goes.
    pub fn unkey(&self, radio: u32) {
        let was = self.on_air.fetch_and(!bit(radio), Ordering::AcqRel);
        if was & bit(radio) == 0 || was & !bit(radio) != 0 {
            // Either this radio was not on the air, or somebody else still is.
            return;
        }
        if let Ok(d) = self.driver.lock()
            && let Some(h) = d.as_ref()
        {
            h.unkey();
        }
    }

    /// Release `radio`'s claim with no hold — the key-down was refused after
    /// the contacts were thrown, so nothing ever reached the air.
    pub fn abort(&self, radio: u32) {
        let was = self.on_air.fetch_and(!bit(radio), Ordering::AcqRel);
        if was & !bit(radio) != 0 {
            return; // another radio is genuinely transmitting
        }
        if let Ok(d) = self.driver.lock()
            && let Some(h) = d.as_ref()
        {
            h.abort();
        }
    }

    /// Reconcile `radio`'s bit with what it is actually doing. Called every
    /// engine tick, so it is one atomic in the common case.
    ///
    /// This is the path for the overs sdroxide does not drive and cannot lead:
    /// a transceiver keyed at its own microphone, or a rig sending CW from its
    /// own keyer. The relay follows as soon as the engine notices, which is the
    /// best software can do — see `sdroxide_types::SenseConfig` for the wire
    /// that makes "as soon as the engine notices" mean milliseconds.
    pub fn publish(&self, radio: u32, on_air: bool) {
        let held = self.on_air.load(Ordering::Acquire) & bit(radio) != 0;
        if held == on_air {
            return;
        }
        if on_air {
            let _ = self.key(radio);
        } else {
            self.unkey(radio);
        }
    }

    /// Why a key-down should be refused, if it should.
    ///
    /// Only ever `Some` when the operator asked for [`FailSafe::RefuseTx`] and
    /// the switch is not in a state to protect anything. A switch that is
    /// simply not configured refuses nothing.
    pub fn refusal(&self) -> Option<String> {
        if !self.configured.load(Ordering::Acquire) || !self.refuse.load(Ordering::Acquire) {
            return None;
        }
        if let Some(why) = self.open_error.lock().unwrap_or_else(|e| e.into_inner()).clone() {
            return Some(why);
        }
        let d = self.driver.lock().ok()?;
        let h = d.as_ref()?;
        let st = h.status();
        (!st.present)
            .then(|| st.error.unwrap_or_else(|| "the T/R switch is not answering".to_string()))
    }

    /// What to show the operator.
    pub fn status(&self) -> RelayStatus {
        if !self.configured.load(Ordering::Acquire) {
            return RelayStatus::default();
        }
        if let Some(why) = self.open_error.lock().unwrap_or_else(|e| e.into_inner()).clone() {
            return RelayStatus {
                configured: true,
                present: false,
                describe: String::new(),
                error: Some(why),
                keyed: false,
            };
        }
        match self.driver.lock() {
            Ok(d) => d.as_ref().map(|h| h.status()).unwrap_or_default(),
            Err(_) => RelayStatus::default(),
        }
    }

    /// The most recent sense-line edge nobody has acted on, taken — but only
    /// by the radio the wire belongs to.
    ///
    /// Gated here rather than at the call site so that exactly one engine can
    /// consume an edge: a second one asking would take the edge away from the
    /// radio that was supposed to act on it, which is worse than not asking.
    pub fn take_sense_edge(&self, radio: u32) -> Option<bool> {
        if radio != self.sense_radio.load(Ordering::Acquire) {
            return None;
        }
        let d = self.driver.lock().ok()?;
        d.as_ref()?.take_sense_edge()
    }

    /// Pulse one contact so the operator can check their wiring. Refused by the
    /// driver while anything is on the air.
    pub fn test(&self, channel: u8) {
        if let Ok(d) = self.driver.lock()
            && let Some(h) = d.as_ref()
        {
            h.test(channel);
        }
    }
}

/// One bit per radio id. Ids above 63 share the top bit, which on a station
/// with sixty-four radios is not the problem anybody has.
fn bit(radio: u32) -> u64 {
    1u64 << (radio.min(63))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The case this type exists for. Two radios, one antenna relay: it must
    /// not open when the first one unkeys while the second is still on the air.
    #[test]
    fn the_last_radio_to_unkey_is_what_releases_the_contacts() {
        let s = TrSwitch::new();
        assert!(!s.busy());
        let _ = s.key(0);
        assert!(s.busy());
        let _ = s.key(1);
        s.unkey(0);
        assert!(s.busy(), "radio 1 is still transmitting");
        s.unkey(1);
        assert!(!s.busy());
    }

    #[test]
    fn an_unkey_from_a_radio_that_never_keyed_releases_nothing() {
        let s = TrSwitch::new();
        let _ = s.key(0);
        s.unkey(3);
        assert!(s.busy(), "radio 0 is still on the air");
    }

    #[test]
    fn publish_is_idempotent_and_follows_both_edges() {
        let s = TrSwitch::new();
        s.publish(2, false);
        assert!(!s.busy());
        s.publish(2, true);
        s.publish(2, true);
        assert!(s.busy());
        s.publish(2, false);
        assert!(!s.busy());
    }

    #[test]
    fn radio_id_zero_is_a_real_claim() {
        let s = TrSwitch::new();
        let _ = s.key(0);
        assert!(s.busy(), "id 0 must not read back as \"nobody\"");
    }

    /// Only the radio the wire is in takes the edge — and the others asking
    /// must not consume it out from under it.
    #[test]
    fn a_sense_edge_belongs_to_one_radio() {
        let s = TrSwitch::new();
        let cfg = RelayConfig {
            link: sdroxide_types::RelayLink::Serial,
            sense: sdroxide_types::SenseConfig {
                line: sdroxide_types::SenseLine::Cts,
                active_high: false,
                radio: 1,
            },
            ..RelayConfig::default()
        };
        s.install(None, &cfg);
        // No driver, so nobody gets an edge — but radio 0 must be turned away
        // before the driver is even consulted, which is what the gate is for.
        assert_eq!(s.take_sense_edge(0), None);
        assert_eq!(s.take_sense_edge(1), None);
    }

    #[test]
    fn a_switch_that_is_not_configured_refuses_nothing() {
        let s = TrSwitch::new();
        assert_eq!(s.refusal(), None);
        assert_eq!(s.status(), RelayStatus::default());
    }

    #[test]
    fn a_switch_that_would_not_open_refuses_the_over_when_asked_to() {
        let s = TrSwitch::new();
        let cfg = RelayConfig {
            link: sdroxide_types::RelayLink::Serial,
            fail_safe: FailSafe::RefuseTx,
            ..RelayConfig::default()
        };
        s.install(None, &cfg);
        s.set_open_error(Some("cannot open the T/R switch on /dev/ttyUSB9".into()));
        assert!(s.refusal().is_some_and(|r| r.contains("ttyUSB9")));

        // ...and does not, when the operator chose otherwise.
        let cfg = RelayConfig { fail_safe: FailSafe::WarnOnly, ..cfg };
        s.install(None, &cfg);
        assert_eq!(s.refusal(), None, "WarnOnly transmits and says so instead");
    }
}
