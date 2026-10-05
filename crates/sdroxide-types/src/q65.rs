//! Q65 vocabulary shared by the engine, the wire protocol and the panel.
//!
//! **Interface facts only.** Q65's sync pattern, QRA LDPC tables and sub-mode
//! geometry live in `sdroxide-digi`, which already carries the GPL obligation
//! for mfsk-core. What is here is what the UI and the wire need to talk about
//! Q65 at all: which sub-mode is selected, how long its slot is, how the
//! sub-modes are named. Nothing here is derived from WSJT-X source, so this
//! crate stays permissive-intent and wasm-clean.
//!
//! # The two axes
//!
//! Q65 sub-modes vary along two independent axes, and the WSJT-X name is
//! `<period><letter>` — Q65-60A, Q65-120E and so on:
//!
//! * **T/R period** — 15, 30, 60, 120 or 300 seconds. Longer is deeper and
//!   slower, exactly as FST4's periods are.
//! * **Tone-spacing letter** A–E — spacing = baud × 2^(letter−1), so a higher
//!   letter is wider and tolerates more Doppler spread. **A** is the narrow,
//!   sensitive terrestrial choice; **E** is for the fast-fading paths (microwave
//!   EME, ionoscatter) where a wide signal is the only one that survives.
//!
//! mfsk-core wires ten of the combinations; the ones absent here (e.g.
//! Q65-30B) are not in the crate.

use serde::{Deserialize, Serialize};

/// A Q65 sub-mode: period and tone-spacing letter together.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Q65Mode {
    /// Q65-15A — 15 s period, narrow. Terrestrial and meteor work.
    A15,
    /// Q65-30A — 30 s period, narrow. The terrestrial weak-signal standard.
    #[default]
    A30,
    /// Q65-60A — 60 s period, narrow. The EME band convention.
    A60,
    /// Q65-60B — 60 s, double spacing.
    B60,
    /// Q65-60C — 60 s, four times spacing.
    C60,
    /// Q65-60D — 60 s, eight times spacing. Microwave EME.
    D60,
    /// Q65-60E — 60 s, sixteen times spacing. The widest 60 s sub-mode.
    E60,
    /// Q65-120D — 120 s, eight times spacing. Rainscatter and troposcatter.
    D120,
    /// Q65-120E — 120 s, sixteen times spacing. 6 m ionoscatter.
    E120,
    /// Q65-300A — 300 s, narrow. The deepest: optical scatter.
    A300,
}

impl Q65Mode {
    /// Every sub-mode, in declaration order — which is the *wire* order, since
    /// postcard numbers variants by it — and also the order a picker lists
    /// them in: the short terrestrial ones first, then the 60 s EME lineup
    /// widening letter by letter, then the long scatter modes.
    pub const ALL: [Q65Mode; 10] = [
        Q65Mode::A15,
        Q65Mode::A30,
        Q65Mode::A60,
        Q65Mode::B60,
        Q65Mode::C60,
        Q65Mode::D60,
        Q65Mode::E60,
        Q65Mode::D120,
        Q65Mode::E120,
        Q65Mode::A300,
    ];

    /// The WSJT-X name minus the "Q65-" prefix: the period and the letter.
    pub fn label(self) -> &'static str {
        match self {
            Q65Mode::A15 => "15A",
            Q65Mode::A30 => "30A",
            Q65Mode::A60 => "60A",
            Q65Mode::B60 => "60B",
            Q65Mode::C60 => "60C",
            Q65Mode::D60 => "60D",
            Q65Mode::E60 => "60E",
            Q65Mode::D120 => "120D",
            Q65Mode::E120 => "120E",
            Q65Mode::A300 => "300A",
        }
    }

    /// Slot period in seconds.
    pub fn slot_s(self) -> f64 {
        match self {
            Q65Mode::A15 => 15.0,
            Q65Mode::A30 => 30.0,
            Q65Mode::A60 | Q65Mode::B60 | Q65Mode::C60 | Q65Mode::D60 | Q65Mode::E60 => 60.0,
            Q65Mode::D120 | Q65Mode::E120 => 120.0,
            Q65Mode::A300 => 300.0,
        }
    }

    /// Samples per symbol at 12 kHz, mfsk-core's own sub-mode geometry.
    pub fn nsps(self) -> usize {
        match self {
            Q65Mode::A15 => 1_800,
            Q65Mode::A30 => 3_600,
            Q65Mode::A60 | Q65Mode::B60 | Q65Mode::C60 | Q65Mode::D60 | Q65Mode::E60 => 7_200,
            Q65Mode::D120 | Q65Mode::E120 => 16_000,
            Q65Mode::A300 => 41_472,
        }
    }

    /// Delay from the slot boundary to the first symbol: half a second for the
    /// 15 s and 30 s sub-modes and one second for the rest, as WSJT-X keys
    /// them (`Modulator.cpp`: 500 ms for Q65 at `nsps <= 3600`) and centres its
    /// search (`q65.f90`). A decode's DT is measured from it.
    pub fn start_delay_s(self) -> f64 {
        if self.nsps() <= 3_600 { 0.5 } else { 1.0 }
    }

    /// On-air duration of one transmission: 85 symbols at this sub-mode's
    /// geometry.
    pub fn burst_s(self) -> f64 {
        85.0 * self.nsps() as f64 / 12_000.0
    }

    /// The whole clock for this sub-mode, in the shape every other slotted mode
    /// states it in.
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

    /// Every Q65 burst has to start and finish inside its own slot.
    #[test]
    fn every_sub_modes_burst_fits_its_slot() {
        for m in Q65Mode::ALL {
            let t = m.slot_timing();
            assert!(
                t.tx_offset_s + t.burst_s < t.slot_s,
                "Q65-{}: a {} s burst keyed {} s in overruns a {} s slot",
                m.label(),
                t.burst_s,
                t.tx_offset_s,
                t.slot_s
            );
            let (long, short) = if t.slot_s >= 60.0 { (t.slot_s, 60.0) } else { (60.0, t.slot_s) };
            assert!((long / short).fract() < 1e-9, "Q65-{}: not a minute's worth", m.label());
        }
    }

    /// The two axes are independent: period comes from the name's number,
    /// spacing multiplier from its letter.
    #[test]
    fn the_sub_mode_names_encode_the_two_axes() {
        assert_eq!(Q65Mode::A60.slot_s(), Q65Mode::E60.slot_s(), "60 s is 60 s");
        // A60..E60 all share the 60 s geometry — the letter is *tone spacing*,
        // not symbol length — so their bursts are identical even though the
        // signals are 1× to 16× as wide.
        assert!((Q65Mode::A60.burst_s() - Q65Mode::E60.burst_s()).abs() < 1e-9);
        // The long modes are genuinely longer.
        assert!(Q65Mode::A300.burst_s() > Q65Mode::A60.burst_s());
        assert!(Q65Mode::E120.burst_s() > Q65Mode::A60.burst_s());
    }
}
