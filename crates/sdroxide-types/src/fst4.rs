//! FST4 vocabulary shared by the engine, the wire protocol and the panel.
//!
//! **Interface facts only.** FST4's Costas arrays, sync pattern, LDPC(240,101)
//! tables and GFSK shaper live in `sdroxide-digi`, which already carries the
//! GPL obligation for mfsk-core. What is here is what the UI and the wire need
//! to talk about FST4 at all: how long a slot is, how the periods are named.
//! Nothing here is derived from WSJT-X source, so this crate stays
//! permissive-intent and wasm-clean. Please keep it that way.

use serde::{Deserialize, Serialize};

/// FST4's T/R period — the five sub-modes mfsk-core wires.
///
/// FST4 is the slow weak-signal mode for EME, troposcatter and LF/MF
/// propagation: all five periods share one 160-symbol frame, one FEC code and
/// one 77-bit message, and differ only in how long a symbol lasts — which
/// trades sensitivity against how long a contact takes. A 15-second period is
/// a fast terrestrial signal; a 300-second one digs tens of dB under the noise
/// for a moonbounce path.
///
/// The letter suffix WSJT-X uses (FST4-60**A**) names the tone-spacing
/// multiplier within a period; only the **A** variant of each period is wired
/// here, which is the one used for the deep weak-signal work, so the period is
/// the whole choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Fst4Period {
    /// FST4-15 — 15 s slots. The fast end; terrestrial and meteor work.
    P15,
    /// FST4-30 — 30 s slots.
    P30,
    /// FST4-60A — 60 s slots. The band convention and the default.
    #[default]
    P60,
    /// FST4-120 — 120 s slots. Deep weak-signal work.
    P120,
    /// FST4-300 — 300 s slots. The deepest: LF/MF and EME at the noise floor.
    P300,
}

impl Fst4Period {
    /// Every period, shortest slot first — the order a picker reads them in,
    /// and the *wire* order, since postcard numbers variants by declaration.
    pub const ALL: [Fst4Period; 5] =
        [Fst4Period::P15, Fst4Period::P30, Fst4Period::P60, Fst4Period::P120, Fst4Period::P300];

    /// The label a chip or log row shows: the WSJT-X period number.
    pub fn label(self) -> &'static str {
        match self {
            Fst4Period::P15 => "15",
            Fst4Period::P30 => "30",
            Fst4Period::P60 => "60",
            Fst4Period::P120 => "120",
            Fst4Period::P300 => "300",
        }
    }

    /// Slot period in seconds — how often a transmission may start.
    pub fn slot_s(self) -> f64 {
        match self {
            Fst4Period::P15 => 15.0,
            Fst4Period::P30 => 30.0,
            Fst4Period::P60 => 60.0,
            Fst4Period::P120 => 120.0,
            Fst4Period::P300 => 300.0,
        }
    }

    /// Samples per symbol at 12 kHz, from mfsk-core's own sub-mode geometry
    /// (`FST4_{15,30,60A,120,300}_GFSK`). These are the figures that make the
    /// burst length right; the values are WSJT-X's `nsps` for each period.
    pub fn nsps(self) -> usize {
        match self {
            Fst4Period::P15 => 720,
            Fst4Period::P30 => 1_680,
            Fst4Period::P60 => 3_888,
            Fst4Period::P120 => 8_200,
            Fst4Period::P300 => 21_504,
        }
    }

    /// Delay from the slot boundary to the first symbol, in seconds: one
    /// second for every period but FST4-15, which keys half a second in so its
    /// 9.6 s burst leaves room to decode. WSJT-X's `Modulator.cpp` and
    /// mfsk-core's own sub-modes both say so, and a decode's DT is measured
    /// from it.
    pub fn start_delay_s(self) -> f64 {
        match self {
            Fst4Period::P15 => 0.5,
            _ => 1.0,
        }
    }

    /// On-air duration of one transmission: 160 symbols at this period's
    /// geometry.
    pub fn burst_s(self) -> f64 {
        160.0 * self.nsps() as f64 / 12_000.0
    }

    /// The whole clock for this period, in the shape every other slotted mode
    /// states it in — so the scheduler and the panels take FST4's timing from
    /// the same type as FT8's, exactly as [`crate::Js8Speed::slot_timing`]
    /// lets them for the other mode whose slot is a setting.
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

    /// Every FST4 burst has to start and finish inside its own slot, and for
    /// the periods that divide a minute the boundary has to land where a
    /// clock shows it.
    #[test]
    fn every_periods_burst_fits_its_slot() {
        for p in Fst4Period::ALL {
            let t = p.slot_timing();
            assert!(
                t.tx_offset_s + t.burst_s < t.slot_s,
                "FST4-{}: a {} s burst keyed {} s in overruns a {} s slot",
                p.label(),
                t.burst_s,
                t.tx_offset_s,
                t.slot_s
            );
            // 15, 30 and 60 divide a minute; 120 and 300 are whole minutes.
            let (long, short) = if t.slot_s >= 60.0 { (t.slot_s, 60.0) } else { (60.0, t.slot_s) };
            assert!((long / short).fract() < 1e-9, "FST4-{}: not a minute's worth", p.label());
        }
    }

    /// The geometry matches mfsk-core's sub-mode tables — a wrong `nsps` is a
    /// burst that decodes at the wrong length, which reads as nothing decoding
    /// at all.
    #[test]
    fn the_period_geometry_matches_mfsk_core() {
        // 160 symbols x nsps / 12000.
        assert!((Fst4Period::P60.burst_s() - 51.84).abs() < 1e-9);
        assert!((Fst4Period::P15.burst_s() - 9.6).abs() < 1e-9);
        assert!((Fst4Period::P300.burst_s() - 286.72).abs() < 1e-9);
    }
}
