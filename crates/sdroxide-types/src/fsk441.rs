//! FSK441 vocabulary shared by the engine, the wire protocol and the panel.
//!
//! **Interface facts only.** FSK441's 4-FSK front end, its ping search and its
//! PUA-43 alphabet live in `sdroxide-dsp`, which carries the decoder. What is
//! here is what the UI and the wire need to talk about FSK441 at all: how long
//! a slot is, how the periods are named. Nothing here is derived from WSJT-X
//! source, so this crate stays permissive-intent and wasm-clean. Please keep it
//! that way.

use serde::{Deserialize, Serialize};

/// FSK441's T/R period — the two the mode is worked in.
///
/// FSK441 is the original high-speed meteor-scatter mode: 4-FSK at 441 baud on
/// four tones 441 Hz apart, transmitting a message repeatedly through the whole
/// period. The receiver hears only the brief reflections off underdense meteor
/// trails — pings of ten to a few hundred milliseconds — so a decode carries
/// the time *into* the slot it was found at rather than sitting at a fixed
/// offset, exactly as [`crate::Mode::Msk144`]'s do.
///
/// The 30-second period is the band convention; 15 seconds is the fast end,
/// where the shortened message is repeated more often and both stations turn
/// around sooner. All that changes between them is how much audio one slot
/// holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Fsk441Period {
    /// 15-second T/R period. The fast end.
    P15,
    /// 30-second T/R period. The band convention and the default.
    #[default]
    P30,
}

impl Fsk441Period {
    /// Every period, shortest slot first — the order a picker reads them in,
    /// and the *wire* order, since postcard numbers variants by declaration.
    pub const ALL: [Fsk441Period; 2] = [Fsk441Period::P15, Fsk441Period::P30];

    /// The label a chip or log row shows: the period in seconds.
    pub fn label(self) -> &'static str {
        match self {
            Fsk441Period::P15 => "15",
            Fsk441Period::P30 => "30",
        }
    }

    /// Slot period in seconds — how often a transmission may start.
    pub fn slot_s(self) -> f64 {
        match self {
            Fsk441Period::P15 => 15.0,
            Fsk441Period::P30 => 30.0,
        }
    }

    /// The nominal on-air burst, in seconds.
    ///
    /// FSK441 does not key a fixed frame and stop: the operator transmits the
    /// message over and over through the whole period, and a meteor's trail
    /// catches whatever part of it is passing. The figure here is the longest
    /// single pass — 46 characters of three dits at 441 baud — which is the
    /// unit the slot bar can mark, as MSK144's one 72 ms frame is for that mode.
    pub fn burst_s(self) -> f64 {
        46.0 * 3.0 / 441.0
    }

    /// Delay from the slot boundary to the first dit. FSK441 keys right on the
    /// boundary; the operator's clock is the only thing that decides.
    pub fn start_delay_s(self) -> f64 {
        0.0
    }

    /// The whole clock for this period, in the shape every other slotted mode
    /// states it in — so the scheduler and the panel take FSK441's timing from
    /// the same type as FT8's, exactly as [`crate::Fst4Period`] lets them.
    pub fn slot_timing(self) -> crate::SlotTiming {
        crate::SlotTiming {
            slot_s: self.slot_s(),
            tx_offset_s: self.start_delay_s(),
            burst_s: self.burst_s(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every period's pass has to fit inside its own slot, and each slot has to
    /// divide a minute so the two ends cannot drift apart.
    #[test]
    fn every_periods_pass_fits_its_slot() {
        for p in Fsk441Period::ALL {
            let t = p.slot_timing();
            assert!(
                t.tx_offset_s + t.burst_s < t.slot_s,
                "FSK441-{}: a {} s pass keyed {} s in overruns a {} s slot",
                p.label(),
                t.burst_s,
                t.tx_offset_s,
                t.slot_s
            );
            let (long, short) = if t.slot_s >= 60.0 { (t.slot_s, 60.0) } else { (60.0, t.slot_s) };
            assert!((long / short).fract() < 1e-9, "FSK441-{}: not a minute's worth", p.label());
        }
    }

    /// The period is the whole choice, so the two must actually differ.
    #[test]
    fn the_periods_are_distinct() {
        assert_eq!(Fsk441Period::P15.slot_s(), 15.0);
        assert_eq!(Fsk441Period::P30.slot_s(), 30.0);
        assert_eq!(Fsk441Period::default(), Fsk441Period::P30);
        assert_ne!(Fsk441Period::P15.label(), Fsk441Period::P30.label());
    }
}
