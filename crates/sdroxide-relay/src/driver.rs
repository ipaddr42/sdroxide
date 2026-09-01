//! Deciding which contacts should be closed right now, and putting that on a
//! thread.
//!
//! Two halves, the same split `sdroxide-limerfe` makes next door.
//!
//! [`Sequencer`] is pure and clock-injected. It holds the on-air state and the
//! timings and answers "what should the hardware be set to at this instant".
//! Every timing decision this subsystem makes is in it, and every one of them
//! is testable with nothing plugged in — which matters more here than anywhere
//! else in the program, because the thing being timed is a receiver's front end
//! against a kilowatt.
//!
//! [`spawn`] puts a transport on a thread, because a serial write blocks for
//! ten milliseconds and the engine's loop cannot spare it.
//!
//! # The rules
//!
//! * **Once a contact is closed for an over it stays closed for that over.**
//!   The lead times are a *rise* schedule, never a fall one. Re-deriving them
//!   mid-over — which a keyer, a QSK CW string or back-to-back FT8 slots will
//!   ask for many times a minute — would otherwise drop an antenna relay under
//!   live RF.
//! * **A half-applied state is not a state.** A failed transaction forgets what
//!   the hardware was told and writes everything again.
//! * **Give up rather than retry forever**, and say so once.
//! * **Shutdown is unconditional.** Whatever happened above, the contacts go
//!   back to receive on the way out.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TryRecvError};
use sdroxide_types::{RelayConfig, RelayStatus};

use crate::frame::{self, ChannelMask};
use crate::trace::{self, Trace};
use crate::transport::RelayTransport;

/// Consecutive failures before the hardware is declared gone. The LimeRFE's
/// number, for the same reason: two is a glitch, three is a fault.
pub const MAX_TRIES: u32 = 3;

/// How long the thread sleeps between looks when a sense line is wired.
///
/// A sequencer's granularity *is* its accuracy, so this is deliberately much
/// shorter than the LimeRFE's — but it is also the floor on how quickly a
/// transmitter keying itself is noticed, which is the entire value of the sense
/// input. Five milliseconds of `TIOCMGET` two hundred times a second is not
/// measurable next to the receive chain.
const SENSE_TICK: Duration = Duration::from_millis(5);

/// How long it sleeps without one. Nothing else arrives except through the
/// channel, which wakes it, and through the schedule, which it wakes for
/// exactly — so this is only a backstop.
const IDLE_TICK: Duration = Duration::from_millis(200);

/// How often to ask a board that can answer what it is actually set to. Only
/// while receiving: a read-back at key-down would put a round trip between the
/// operator's thumb and their antenna relay for no gain.
const READBACK_INTERVAL: Duration = Duration::from_secs(5);

/// How long a test pulse holds a contact closed. Long enough to hear the relay
/// and see the light, short enough that a mis-wired board is not left keying an
/// amplifier while the operator thinks about it.
const TEST_PULSE: Duration = Duration::from_millis(600);

/// Whether the hardware is there. Not configuration: it either answers or it
/// does not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Presence {
    Present,
    /// Stopped answering, and left alone until the configuration is applied
    /// again.
    Absent,
}

/// What is holding the station on the air. Two independent sources, OR'd —
/// the program's own transmit state, and a line watched on the hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// The engine says a radio in this process is transmitting.
    Engine,
    /// The sense input says a transmitter out in the shack is.
    Sense,
}

/// One channel's place in the schedule, resolved once so the hot path is
/// arithmetic on a small array rather than a walk through the configuration.
#[derive(Debug, Clone, Copy)]
struct Slot {
    bit: ChannelMask,
    /// How long after the key-down edge this contact closes. Derived: the
    /// channel with the longest lead closes at once and the rest are staggered
    /// behind it, so that they all lead the RF by exactly what they asked for.
    rise: Duration,
    /// How long after the key-up edge it opens.
    fall: Duration,
}

/// What the hardware should be doing, given who is on the air and when they
/// started. Pure — no transport, no clock of its own.
#[derive(Debug)]
pub struct Sequencer {
    slots: Vec<Slot>,
    managed: ChannelMask,
    /// Channels whose *transmit* state is the de-energised one. Applied on the
    /// way out to the transport, so nothing below here knows about polarity.
    inverted: ChannelMask,
    max_lead: Duration,
    engine_on_air: bool,
    sense_on_air: bool,
    /// When the OR of the two last changed.
    edge: Instant,
    /// Logical: which contacts are in their transmit state.
    asserted: ChannelMask,
    /// Physical: what the transport last accepted. `None` is "unknown", which
    /// is the starting state and what a failure resets it to.
    applied: Option<ChannelMask>,
    test: Option<(ChannelMask, Instant)>,
    failures: u32,
    presence: Presence,
}

impl Sequencer {
    pub fn new(cfg: &RelayConfig, now: Instant) -> Sequencer {
        let mut s = Sequencer {
            slots: Vec::new(),
            managed: 0,
            inverted: 0,
            max_lead: Duration::ZERO,
            engine_on_air: false,
            sense_on_air: false,
            edge: now,
            asserted: 0,
            applied: None,
            test: None,
            failures: 0,
            presence: Presence::Present,
        };
        s.adopt(cfg);
        s
    }

    fn adopt(&mut self, cfg: &RelayConfig) {
        let max_lead = u64::from(cfg.max_lead_ms());
        self.max_lead = Duration::from_millis(max_lead);
        self.slots.clear();
        self.managed = 0;
        self.inverted = 0;
        for ch in cfg.active_channels() {
            let bit = frame::bit(ch.index);
            if bit == 0 {
                continue;
            }
            self.managed |= bit;
            if !ch.active_high {
                self.inverted |= bit;
            }
            let lead = u64::from(cfg.lead_for(ch)).min(max_lead);
            self.slots.push(Slot {
                bit,
                rise: Duration::from_millis(max_lead - lead),
                fall: Duration::from_millis(u64::from(cfg.hold_for(ch))),
            });
        }
        // A contact that is no longer managed is not ours to hold.
        self.asserted &= self.managed;
    }

    /// Adopt a new configuration. The hardware is written in full afterwards —
    /// the polarity may have flipped under us, and a mask that happens to match
    /// would then mean the opposite of what it did a moment ago.
    pub fn set_config(&mut self, cfg: &RelayConfig) {
        self.adopt(cfg);
        self.applied = None;
        // Applying a configuration is the operator saying "try again".
        self.failures = 0;
        self.presence = Presence::Present;
    }

    pub fn presence(&self) -> Presence {
        self.presence
    }

    pub fn on_air(&self) -> bool {
        self.engine_on_air || self.sense_on_air
    }

    /// The physical mask that means "everything receiving" — what the hardware
    /// is put back to at shutdown, without having to know what state it was in.
    pub fn receive_mask(&self) -> ChannelMask {
        self.inverted & self.managed
    }

    /// Tell the sequencer that one of the two on-air sources changed.
    pub fn set_source(&mut self, source: Source, on: bool, now: Instant) {
        let was = self.on_air();
        match source {
            Source::Engine => self.engine_on_air = on,
            Source::Sense => self.sense_on_air = on,
        }
        if self.on_air() != was {
            self.edge = now;
        }
    }

    /// Drop everything immediately, with no hold: for a key-down that was
    /// refused after the contacts were already thrown. No RF appeared, so there
    /// is nothing to protect on the way out.
    pub fn abort(&mut self, now: Instant) {
        self.engine_on_air = false;
        self.sense_on_air = false;
        self.asserted = 0;
        self.test = None;
        self.edge = now;
    }

    /// Close one contact for [`TEST_PULSE`], so an operator can hear the relay
    /// and check their wiring with the transmitter cold. Refused while anything
    /// is on the air — throwing a relay under live RF is exactly what this
    /// whole subsystem exists to avoid.
    pub fn test(&mut self, channel: u8, now: Instant) -> bool {
        let bit = frame::bit(channel) & self.managed;
        if bit == 0 || self.on_air() {
            return false;
        }
        self.test = Some((bit, now + TEST_PULSE));
        true
    }

    /// Move `asserted` to where the clock says it should be.
    ///
    /// Monotonic within an over in each direction: while on the air contacts
    /// are only ever added, and while receiving only ever removed. That is what
    /// makes a re-key inside the hold window free — the contact was never
    /// going to be dropped, so there is nothing to cancel.
    fn advance(&mut self, now: Instant) {
        if let Some((_, until)) = self.test
            && now >= until
        {
            self.test = None;
        }
        if self.on_air() {
            for s in &self.slots {
                if self.asserted & s.bit == 0 && now.saturating_duration_since(self.edge) >= s.rise
                {
                    self.asserted |= s.bit;
                }
            }
        } else {
            for s in &self.slots {
                if self.asserted & s.bit != 0 && now.saturating_duration_since(self.edge) >= s.fall
                {
                    self.asserted &= !s.bit;
                }
            }
        }
    }

    /// The physical mask the hardware should be holding.
    pub fn want(&self) -> ChannelMask {
        let logical = self.asserted | self.test.map(|(b, _)| b).unwrap_or(0);
        (logical ^ self.inverted) & self.managed
    }

    /// What to write, if anything. `None` means the hardware already holds it.
    pub fn due(&mut self, now: Instant) -> Option<ChannelMask> {
        if self.presence == Presence::Absent {
            return None;
        }
        self.advance(now);
        let want = self.want();
        (self.applied != Some(want)).then_some(want)
    }

    pub fn on_ack(&mut self, applied: ChannelMask) {
        self.applied = Some(applied);
        self.failures = 0;
    }

    /// Forget what the hardware is holding, so the next tick writes it all
    /// again. For a read-back that disagreed: the board is there, so this is
    /// not a failure, but what it holds is not what we think.
    pub fn invalidate(&mut self) {
        self.applied = None;
    }

    pub fn on_error(&mut self) {
        // Whatever the hardware is holding, it is not what we think.
        self.applied = None;
        self.failures = self.failures.saturating_add(1);
        if self.failures >= MAX_TRIES {
            self.presence = Presence::Absent;
        }
    }

    /// When the schedule next has something to do, so the thread can sleep
    /// exactly that long instead of spinning. `None` when it is settled.
    pub fn next_change(&self, now: Instant) -> Option<Instant> {
        let mut soonest: Option<Instant> = None;
        let mut note = |t: Instant| {
            if t > now {
                soonest = Some(soonest.map_or(t, |s: Instant| s.min(t)));
            }
        };
        if self.on_air() {
            for s in &self.slots {
                if self.asserted & s.bit == 0 {
                    note(self.edge + s.rise);
                }
            }
        } else {
            for s in &self.slots {
                if self.asserted & s.bit != 0 {
                    note(self.edge + s.fall);
                }
            }
        }
        if let Some((_, until)) = self.test {
            note(until);
        }
        soonest
    }
}

/// Messages into the switch's thread. Every one is last-value-wins.
#[derive(Debug, Clone)]
pub enum Ctrl {
    Config(Box<RelayConfig>),
    /// The engine's own transmit state.
    Key(bool),
    /// A key-down that was refused after the contacts were thrown.
    Abort,
    /// Pulse one contact, for checking the wiring.
    Test(u8),
    Shutdown,
}

/// A handle on the thread driving one station's T/R switch.
pub struct RelayHandle {
    tx: Sender<Ctrl>,
    join: Option<std::thread::JoinHandle<()>>,
    describe: String,
    status: Arc<std::sync::Mutex<RelayStatus>>,
    /// The last unconsumed sense-line edge, `true` for "the transmitter out
    /// there is keyed". Last-value-wins: an edge nobody read before the next
    /// one arrived was never going to be acted on separately.
    sense_edge: Arc<std::sync::Mutex<Option<bool>>>,
    /// One command's cost on this link, from the transport.
    settle: Duration,
    /// The longest lead any driven channel asks for, in milliseconds. An atomic
    /// because the caller of [`RelayHandle::key`] is on the engine's thread at
    /// its most time-critical moment and must not queue behind a lock the
    /// worker might be holding.
    lead_ms: Arc<AtomicU64>,
}

impl RelayHandle {
    /// Throw the contacts for an over, and say how long to wait before letting
    /// RF out.
    ///
    /// The wait is the operator's longest lead plus what one command costs on
    /// this link — asked rather than assumed, because a 9600-baud board and a
    /// CDC one are a factor of two apart and a caller that guessed the fast one
    /// would let drive out into a relay that is still moving.
    pub fn key(&self) -> Duration {
        let _ = self.tx.send(Ctrl::Key(true));
        Duration::from_millis(self.lead_ms.load(Ordering::Relaxed)) + self.settle
    }

    /// End the over. Returns at once: the hold times are the worker's business,
    /// and the engine has a receiver to get back to.
    pub fn unkey(&self) {
        let _ = self.tx.send(Ctrl::Key(false));
    }

    /// Drop everything now, with no hold — a key-down that was refused after
    /// the contacts were thrown.
    pub fn abort(&self) {
        let _ = self.tx.send(Ctrl::Abort);
    }

    pub fn set_config(&self, cfg: RelayConfig) {
        self.lead_ms.store(u64::from(cfg.max_lead_ms()), Ordering::Relaxed);
        let _ = self.tx.send(Ctrl::Config(Box::new(cfg)));
    }

    pub fn test(&self, channel: u8) {
        let _ = self.tx.send(Ctrl::Test(channel));
    }

    pub fn describe(&self) -> &str {
        &self.describe
    }

    pub fn status(&self) -> RelayStatus {
        self.status.lock().map(|s| s.clone()).unwrap_or_default()
    }

    /// The most recent sense-line edge nobody has acted on yet, taken.
    pub fn take_sense_edge(&self) -> Option<bool> {
        self.sense_edge.lock().ok().and_then(|mut s| s.take())
    }

    /// What one command costs on this link, without the operator's lead.
    pub fn settle(&self) -> Duration {
        self.settle
    }
}

impl Drop for RelayHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(Ctrl::Shutdown);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// Put a transport on its own thread and drive it from a sequencer.
pub fn spawn(mut transport: Box<dyn RelayTransport>, cfg: RelayConfig) -> RelayHandle {
    let (tx, rx) = crossbeam_channel::unbounded::<Ctrl>();
    let describe = transport.describe();
    let settle = transport.round_trip();
    let lead_ms = Arc::new(AtomicU64::new(u64::from(cfg.max_lead_ms())));
    let status = Arc::new(std::sync::Mutex::new(RelayStatus {
        configured: true,
        present: true,
        describe: describe.clone(),
        error: None,
        keyed: false,
    }));
    let sense_edge = Arc::new(std::sync::Mutex::new(None));

    let status_thread = Arc::clone(&status);
    let sense_thread = Arc::clone(&sense_edge);
    let lead_thread = Arc::clone(&lead_ms);
    // What this switch was told, kept for a report. A relay that clicks and
    // passes no signal, or one that never clicks at all, is diagnosed from
    // exactly this — and on these boards there is no other record anywhere.
    let t = Trace::new();
    t.set_link(&describe);
    trace::remember(&t);

    let join = std::thread::Builder::new()
        .name("sdroxide-relay".into())
        .spawn(move || {
            let mut seq = Sequencer::new(&cfg, Instant::now());
            run(
                &mut *transport,
                &mut seq,
                &cfg,
                &rx,
                &status_thread,
                &sense_thread,
                &lead_thread,
                &t,
            );
            // Back to receive, whatever happened above. The same "shutdown is
            // best-effort but unconditional" rule the USB drivers apply to
            // their radios — and here it is the difference between leaving a
            // station's antenna grounded and leaving an amplifier keyed.
            let stood_down = transport.apply(seq.receive_mask());
            t.note(
                "shutdown: contacts back to receive",
                match &stood_down {
                    Ok(()) => "ok".to_string(),
                    Err(e) => format!("FAILED: {e}"),
                },
            );
            if let Ok(mut s) = status_thread.lock() {
                s.keyed = false;
            }
        })
        .expect("spawn sdroxide-relay thread");

    RelayHandle { tx, join: Some(join), describe, status, sense_edge, settle, lead_ms }
}

#[allow(clippy::too_many_arguments)]
fn run(
    transport: &mut dyn RelayTransport,
    seq: &mut Sequencer,
    initial: &RelayConfig,
    rx: &Receiver<Ctrl>,
    status: &std::sync::Mutex<RelayStatus>,
    sense_edge: &std::sync::Mutex<Option<bool>>,
    lead_ms: &AtomicU64,
    trace: &Trace,
) {
    let mut cfg = initial.clone();
    let mut sensed: Option<bool> = None;
    let mut next_readback = Instant::now() + READBACK_INTERVAL;

    loop {
        // Drain the whole channel before acting: a configuration change and a
        // key-down that arrived together should produce one decision.
        loop {
            match rx.try_recv() {
                Ok(msg) => {
                    if !apply_ctrl(msg, seq, &mut cfg, lead_ms, trace) {
                        return;
                    }
                }
                Err(TryRecvError::Disconnected) => return,
                Err(TryRecvError::Empty) => break,
            }
        }

        // The sense line, first and every tick: it is the one thing here whose
        // whole value is how quickly it is noticed.
        if cfg.sense.line != sdroxide_types::SenseLine::Off {
            match transport.sense() {
                Ok(Some(level)) => {
                    let keyed = level == cfg.sense.active_high;
                    if sensed != Some(keyed) {
                        sensed = Some(keyed);
                        seq.set_source(Source::Sense, keyed, Instant::now());
                        if let Ok(mut s) = sense_edge.lock() {
                            *s = Some(keyed);
                        }
                        trace.note(
                            format!("sense line {}", cfg.sense.line.label()),
                            if keyed { "transmitting" } else { "receiving" },
                        );
                    }
                }
                Ok(None) => {}
                Err(e) => tracing::debug!("T/R switch sense line: {e}"),
            }
        }

        let now = Instant::now();
        if let Some(want) = seq.due(now) {
            match transport.apply(want) {
                Ok(()) => {
                    seq.on_ack(want);
                    if let Ok(mut s) = status.lock() {
                        s.error = None;
                        s.present = true;
                        s.keyed = seq.on_air();
                    }
                    trace.note(format!("contacts {want:#010b}"), "ok");
                    tracing::debug!("T/R switch contacts set to {want:#010b}");
                }
                Err(e) => {
                    seq.on_error();
                    let gone = seq.presence() == Presence::Absent;
                    trace.note(format!("contacts {want:#010b}"), format!("FAILED: {e}"));
                    if let Ok(mut s) = status.lock() {
                        s.present = !gone;
                        s.error = Some(if gone {
                            format!("the T/R switch stopped answering and has been left alone: {e}")
                        } else {
                            e.to_string()
                        });
                    }
                    if gone {
                        // Said once, not once per tick.
                        tracing::warn!("T/R switch gave up after {MAX_TRIES} failures: {e}");
                    } else {
                        tracing::debug!("T/R switch write failed, will retry: {e}");
                    }
                }
            }
        }

        // Ask a board that can answer whether it agrees, but only while
        // receiving and only occasionally. A board that has quietly stopped
        // agreeing is a flat supply or a stuck relay — a different fault from a
        // dead cable, and one nothing else here would ever notice.
        if !seq.on_air() && Instant::now() >= next_readback {
            next_readback = Instant::now() + READBACK_INTERVAL;
            match transport.read_back() {
                Ok(Some(actual)) => {
                    let want = seq.want();
                    if actual != want {
                        trace.note(
                            format!("read back {actual:#010b}"),
                            format!("expected {want:#010b}"),
                        );
                        tracing::warn!(
                            "the T/R switch reports its contacts as {actual:#010b}, not the \
                             {want:#010b} it was set to"
                        );
                        if let Ok(mut s) = status.lock() {
                            s.error = Some(
                                "the T/R switch reports different contacts from the ones it was \
                                 set to — check its supply"
                                    .to_string(),
                            );
                        }
                        // Make the next tick write them again.
                        seq.invalidate();
                    }
                }
                Ok(None) => {}
                Err(e) => tracing::debug!("T/R switch read-back: {e}"),
            }
        }

        // Sleep until the schedule's next move, the next sense poll, or a
        // message — whichever comes first.
        let now = Instant::now();
        let base =
            if cfg.sense.line == sdroxide_types::SenseLine::Off { IDLE_TICK } else { SENSE_TICK };
        let wait = match seq.next_change(now) {
            Some(at) => base.min(at.saturating_duration_since(now)).max(Duration::from_micros(200)),
            None => base,
        };
        match rx.recv_timeout(wait) {
            Ok(msg) => {
                if !apply_ctrl(msg, seq, &mut cfg, lead_ms, trace) {
                    return;
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
        }
    }
}

/// Apply one message. `false` means the thread should stop.
///
/// One function rather than two arms of the loop, because the drain at the top
/// and the wake-up at the bottom must never disagree about what a message
/// means — and a `Config` handled in one place and not the other is the kind of
/// bug that only shows up on a busy station.
fn apply_ctrl(
    msg: Ctrl,
    seq: &mut Sequencer,
    cfg: &mut RelayConfig,
    lead_ms: &AtomicU64,
    trace: &Trace,
) -> bool {
    match msg {
        Ctrl::Config(c) => {
            *cfg = *c;
            lead_ms.store(u64::from(cfg.max_lead_ms()), Ordering::Relaxed);
            seq.set_config(cfg);
            trace.note("configuration applied", cfg.sequence_note());
        }
        Ctrl::Key(on) => {
            seq.set_source(Source::Engine, on, Instant::now());
            trace.note(if on { "key-down" } else { "key-up" }, "");
        }
        Ctrl::Abort => {
            seq.abort(Instant::now());
            trace.note("abort", "key-down refused after the contacts were thrown");
        }
        Ctrl::Test(ch) => {
            let ok = seq.test(ch, Instant::now());
            trace.note(format!("test pulse on channel {ch}"), if ok { "ok" } else { "refused" });
        }
        Ctrl::Shutdown => return false,
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::{RelayChannel, RelayLink, RelayRole};

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    /// Ask what to write and say it was written — the pair the thread always
    /// performs together.
    fn settle(s: &mut Sequencer, at: Instant) {
        if let Some(m) = s.due(at) {
            s.on_ack(m);
        }
    }

    /// A station with the arrangement this whole subsystem was written for:
    /// channel 1 grounds the SDR (long lead, long hold), channel 2 keys an
    /// amplifier (short lead, no hold).
    fn station() -> RelayConfig {
        RelayConfig {
            link: RelayLink::Serial,
            channels: vec![
                RelayChannel {
                    index: 1,
                    role: RelayRole::SdrAntenna,
                    label: "SDR".into(),
                    active_high: true,
                    lead_ms: 25,
                    hold_ms: 40,
                },
                RelayChannel {
                    index: 2,
                    role: RelayRole::Amplifier,
                    label: "PA".into(),
                    active_high: true,
                    lead_ms: 5,
                    // Deliberately not zero: the fall order is what these tests
                    // are about, and a zero hold would make the amplifier's
                    // release indistinguishable from the key-up itself.
                    hold_ms: 5,
                },
            ],
            ..RelayConfig::default()
        }
    }

    const SDR: ChannelMask = 0b01;
    const PA: ChannelMask = 0b10;

    /// The claim the module docs make, and the one an operator is trusting with
    /// their front end: the antenna relay throws first and comes back last.
    #[test]
    fn the_antenna_leads_the_amplifier_in_and_follows_it_out() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);

        s.set_source(Source::Engine, true, t0);
        // The longest lead closes at once: it is the one the engine's wait is
        // measured against, so it has the whole lead to itself.
        assert_eq!(s.due(t0), Some(SDR), "the antenna throws immediately at key-down");
        s.on_ack(SDR);
        // ...and the shorter lead is staggered behind it by the difference, so
        // that it too leads the RF by exactly the 5 ms it asked for.
        assert_eq!(s.due(t0 + ms(19)), None, "the amplifier is not keyed yet");
        assert_eq!(s.due(t0 + ms(20)), Some(SDR | PA), "25 − 5 = 20 ms later, the amplifier keys");
        s.on_ack(SDR | PA);

        // RF stops at t1. The amplifier drops first.
        let t1 = t0 + ms(1000);
        s.set_source(Source::Engine, false, t1);
        assert_eq!(s.due(t1), None, "nothing drops on the instant of key-up");
        assert_eq!(s.due(t1 + ms(5)), Some(SDR), "the amplifier unkeys after its 5 ms");
        s.on_ack(SDR);
        assert_eq!(s.due(t1 + ms(39)), None, "the antenna is still grounded");
        assert_eq!(s.due(t1 + ms(40)), Some(0), "and comes back only after its own hold");
    }

    /// Fast CW, QSK and back-to-back FT8 slots all re-key inside the hold
    /// window many times a minute. If that dropped and re-threw the antenna
    /// relay it would do it under live RF, which is the exact failure this
    /// subsystem exists to prevent.
    #[test]
    fn a_re_key_inside_the_hold_window_never_drops_a_contact() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Engine, true, t0);
        settle(&mut s, t0);
        settle(&mut s, t0 + ms(20));
        assert_eq!(s.want(), SDR | PA);

        let t1 = t0 + ms(500);
        s.set_source(Source::Engine, false, t1);
        // Two milliseconds later — well inside both holds — the operator keys
        // again.
        let t2 = t1 + ms(2);
        s.set_source(Source::Engine, true, t2);
        assert_eq!(s.due(t2), None, "both contacts were already closed and stay closed");
        assert_eq!(s.want(), SDR | PA);
        // And nothing drops later either: the fall schedule belongs to the
        // key-up that is no longer current.
        assert_eq!(s.due(t2 + ms(100)), None);
        assert_eq!(s.want(), SDR | PA);
    }

    /// The lead schedule is a rise schedule. A re-key must not un-assert a
    /// short-lead channel that is already closed just because the new edge has
    /// not reached its rise time yet.
    #[test]
    fn a_re_key_does_not_re_stagger_a_contact_that_is_already_closed() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Engine, true, t0);
        settle(&mut s, t0);
        settle(&mut s, t0 + ms(20));

        let t1 = t0 + ms(100);
        s.set_source(Source::Engine, false, t1);
        let t2 = t1 + ms(1);
        s.set_source(Source::Engine, true, t2);
        // The amplifier's rise is 20 ms after the edge, and the edge is now t2.
        assert_eq!(s.due(t2), None, "the amplifier stays keyed rather than dropping for 20 ms");
    }

    /// Polarity is resolved on the way out, and it is the one fail-safe that
    /// survives the program dying — so an active-low channel must read as
    /// energised while *receiving*.
    #[test]
    fn an_active_low_channel_is_energised_while_receiving() {
        let cfg = RelayConfig {
            link: RelayLink::Serial,
            channels: vec![RelayChannel {
                index: 1,
                role: RelayRole::SdrAntenna,
                label: "SDR".into(),
                active_high: false,
                lead_ms: 10,
                hold_ms: 10,
            }],
            ..RelayConfig::default()
        };
        let t0 = Instant::now();
        let mut s = Sequencer::new(&cfg, t0);
        assert_eq!(s.due(t0), Some(SDR), "receiving means the coil is held on");
        s.on_ack(SDR);
        s.set_source(Source::Engine, true, t0);
        assert_eq!(s.due(t0), Some(0), "and transmitting means it is released");
        assert_eq!(s.receive_mask(), SDR, "shutdown must put it back, not clear everything");
    }

    #[test]
    fn an_abort_drops_everything_at_once_because_no_rf_appeared() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Engine, true, t0);
        settle(&mut s, t0);
        assert_eq!(s.want(), SDR);

        s.abort(t0 + ms(1));
        assert_eq!(s.due(t0 + ms(1)), Some(0), "no hold: there was nothing on the air to protect");
    }

    #[test]
    fn three_failures_and_the_hardware_is_left_alone() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Engine, true, t0);
        for _ in 0..MAX_TRIES {
            assert!(s.due(t0).is_some(), "it keeps trying until it has given up");
            s.on_error();
        }
        assert_eq!(s.presence(), Presence::Absent);
        assert_eq!(s.due(t0), None, "and then stops talking to it");

        // Applying a configuration is the operator saying "try again".
        s.set_config(&station());
        assert_eq!(s.presence(), Presence::Present);
        assert!(s.due(t0).is_some());
    }

    #[test]
    fn a_failure_forgets_what_the_hardware_was_holding() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Engine, true, t0);
        settle(&mut s, t0);
        assert_eq!(s.due(t0), None, "settled");
        s.on_error();
        assert_eq!(s.due(t0), Some(SDR), "a half-applied state is not a state; write it all again");
    }

    /// The sense line and the program's own transmit state are independent, and
    /// the contacts follow either. A station whose rig is keyed at the
    /// microphone while sdroxide thinks it is receiving must still be switched.
    #[test]
    fn either_source_holds_the_contacts() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        s.set_source(Source::Sense, true, t0);
        assert!(s.on_air());
        assert_eq!(s.due(t0), Some(SDR));
        s.on_ack(SDR);

        // The engine keys too, then unkeys; the sensed transmitter is still up,
        // so the over never ended and the schedule never restarted.
        s.set_source(Source::Engine, true, t0 + ms(1));
        s.set_source(Source::Engine, false, t0 + ms(2));
        assert!(s.on_air(), "the rig out in the shack is still transmitting");
        assert_eq!(
            s.due(t0 + ms(500)),
            Some(SDR | PA),
            "the amplifier closed on its own stagger and nothing was released"
        );
        s.on_ack(SDR | PA);

        // Only when the sensed transmitter drops does the hold schedule run.
        let t1 = t0 + ms(500);
        s.set_source(Source::Sense, false, t1);
        assert_eq!(s.due(t1 + ms(5)), Some(SDR));
        s.on_ack(SDR);
        assert_eq!(s.due(t1 + ms(40)), Some(0));
    }

    #[test]
    fn a_test_pulse_is_refused_while_anything_is_on_the_air() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        assert!(s.test(2, t0), "with the transmitter cold it is allowed");
        assert_eq!(s.due(t0), Some(PA));
        s.on_ack(PA);
        // And it ends on its own rather than needing anybody to remember.
        assert_eq!(s.due(t0 + TEST_PULSE), Some(0));
        s.on_ack(0);

        s.set_source(Source::Engine, true, t0);
        assert!(!s.test(2, t0), "throwing a relay under live RF is the thing we are preventing");
    }

    #[test]
    fn a_channel_with_no_job_is_never_written() {
        let cfg = RelayConfig {
            link: RelayLink::Serial,
            channels: vec![
                RelayChannel { index: 1, role: RelayRole::SdrAntenna, ..RelayChannel::default() },
                RelayChannel { index: 2, role: RelayRole::Unused, ..RelayChannel::default() },
            ],
            ..RelayConfig::default()
        };
        let t0 = Instant::now();
        let mut s = Sequencer::new(&cfg, t0);
        s.set_source(Source::Engine, true, t0);
        assert_eq!(s.due(t0), Some(SDR), "channel 2 belongs to the operator, not to us");
    }

    /// The thread sleeps on this rather than spinning, so a wrong answer here
    /// is a contact that opens late.
    #[test]
    fn the_next_wake_up_is_the_next_thing_the_schedule_has_to_do() {
        let t0 = Instant::now();
        let mut s = Sequencer::new(&station(), t0);
        assert_eq!(s.next_change(t0), None, "settled and receiving: nothing to wake for");

        s.set_source(Source::Engine, true, t0);
        settle(&mut s, t0);
        assert_eq!(s.next_change(t0), Some(t0 + ms(20)), "the amplifier's stagger");

        settle(&mut s, t0 + ms(20));
        assert_eq!(s.next_change(t0 + ms(20)), None, "everything is closed");

        let t1 = t0 + ms(100);
        s.set_source(Source::Engine, false, t1);
        assert_eq!(s.next_change(t1), Some(t1 + ms(5)), "the amplifier drops first");
    }
}
