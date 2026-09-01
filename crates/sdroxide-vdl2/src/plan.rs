//! The channel plan, and which of it a given receiver window can reach.
//!
//! # Why fixed channels rather than a wideband burst detector
//!
//! Every VDL2 transmission in the world is on one of seven frequencies. Watching
//! the whole 300 kHz for a burst and then going back for the samples means
//! keeping a wideband history long enough to re-extract from, and the answer it
//! produces is a channel that was in this table all along. Parking a
//! downconverter on each channel costs one [`sdroxide_dsp::Ddc`] apiece and
//! nothing else, and it cannot miss the start of a transmission because it was
//! already listening — which for a burst whose first sixteen symbols are the
//! only synchronisation there is, is the whole game.
//!
//! # One window, seven channels
//!
//! The plan spans 325 kHz including the outer channels' own bandwidth, so a
//! single half-megahertz window covers the lot on any receiver in this tree. A
//! narrower one reaches fewer, and which ones it reaches depends on where the
//! hardware centre happens to be — so the window slides inside the front end's
//! span to take in as much of the plan as it can, and the panel is told what it
//! got.
//!
//! Source: the European VDL2 frequency assignments (ICAO EUR Doc 011); 136.975
//! is the worldwide Common Signalling Channel.

use sdroxide_types::{VDL2_CHANNEL_SPACING_HZ, VDL2_CHANNELS_HZ, VDL2_CSC_HZ, VDL2_PLAN_CENTER_HZ};

/// One channel of the plan.
pub struct Channel {
    pub center_hz: f64,
    pub label: &'static str,
    /// The Common Signalling Channel: in use worldwide, and where every link
    /// starts.
    pub csc: bool,
}

/// The channels, ascending.
pub const CHANNELS: [Channel; 7] = [
    Channel { center_hz: VDL2_CHANNELS_HZ[0], label: "VDL2 (to 2027)", csc: false },
    Channel { center_hz: VDL2_CHANNELS_HZ[1], label: "VDL2 ground", csc: false },
    Channel { center_hz: VDL2_CHANNELS_HZ[2], label: "VDL2 air", csc: false },
    Channel { center_hz: VDL2_CHANNELS_HZ[3], label: "VDL2 air", csc: false },
    Channel { center_hz: VDL2_CHANNELS_HZ[4], label: "VDL2 ground", csc: false },
    Channel { center_hz: VDL2_CHANNELS_HZ[5], label: "VDL2", csc: false },
    Channel { center_hz: VDL2_CSC_HZ, label: "common signalling channel", csc: true },
];

/// Fraction of a front end's span the window may claim.
///
/// The outer edges of any receiver's window are where its own anti-alias filter
/// is already rolling off, and a channel sitting in the roll-off decodes badly
/// or not at all. The same three quarters the ISM and ADS-B lanes use, for the
/// same reason.
pub const USABLE_FRACTION: f64 = 0.75;

/// What the lane asks its window downconverter for.
///
/// Not simply the plan's span divided by [`USABLE_FRACTION`]: a
/// [`sdroxide_dsp::Ddc`] decimates by a whole number and rounds to the *nearest*
/// one, so a target sitting exactly on the requirement can round the wrong way
/// and land under it. On an RTL-SDR at 2.4 Msps that is the difference between a
/// 480 kHz window that holds all seven channels and a 400 kHz one that holds
/// five. Half a megahertz leaves room for the rounding on every front end here.
pub const WINDOW_TARGET_RATE_HZ: f64 = 500_000.0;

/// What each channel's own downconverter asks for.
///
/// Ten samples a symbol nominally; an integer decimation lands somewhere near
/// it, between about nine and twelve on the front ends in this tree. The floor
/// that matters is not the symbol timing — that is interpolated — but the
/// neighbouring channel 50 kHz away: the rate has to put it outside the Nyquist
/// band, or it folds back on top of the signal where no filter can reach it.
pub const CHANNEL_TARGET_RATE_HZ: f64 = 105_000.0;

/// Distance from the plan's outermost channel centres to its edges.
pub fn span_hz() -> f64 {
    let lo = CHANNELS[0].center_hz - VDL2_CHANNEL_SPACING_HZ / 2.0;
    let hi = CHANNELS[CHANNELS.len() - 1].center_hz + VDL2_CHANNEL_SPACING_HZ / 2.0;
    hi - lo
}

/// Where the window wants to sit to reach the whole plan.
pub fn ideal_center_hz() -> f64 {
    VDL2_PLAN_CENTER_HZ
}

/// Whether a channel is inside a window, with its own bandwidth and the front
/// end's roll-off allowed for.
pub fn fits(center_hz: f64, window_center_hz: f64, window_rate_hz: f64) -> bool {
    let half = window_rate_hz * USABLE_FRACTION / 2.0;
    (center_hz - window_center_hz).abs() + VDL2_CHANNEL_SPACING_HZ / 2.0 <= half
}

/// Indices of the channels a window reaches, ascending.
pub fn channels_in_window(window_center_hz: f64, window_rate_hz: f64) -> Vec<usize> {
    (0..CHANNELS.len())
        .filter(|&i| fits(CHANNELS[i].center_hz, window_center_hz, window_rate_hz))
        .collect()
}

/// Where to put a window of `window_rate_hz` inside a front end's span.
///
/// The ideal centre where the span reaches it, and as close as the span allows
/// otherwise — a receiver that cannot hold the whole plan should still hold as
/// much of it as it can rather than refusing.
pub fn window_center_for(hw_center_hz: f64, hw_rate_hz: f64, window_rate_hz: f64) -> f64 {
    let slack = (hw_rate_hz * USABLE_FRACTION - window_rate_hz) / 2.0;
    if slack <= 0.0 {
        return hw_center_hz;
    }
    ideal_center_hz().clamp(hw_center_hz - slack, hw_center_hz + slack)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The plan's span and its ideal centre are derived from the table, so
    /// adding a channel moves them rather than leaving them stale.
    #[test]
    fn the_span_and_centre_come_from_the_table() {
        assert_eq!(span_hz(), 325_000.0);
        let lo = CHANNELS[0].center_hz - VDL2_CHANNEL_SPACING_HZ / 2.0;
        assert_eq!(ideal_center_hz(), lo + span_hz() / 2.0);
        assert_eq!(CHANNELS.iter().filter(|c| c.csc).count(), 1);
    }

    /// A window at the target rate holds the whole plan — which is the thing
    /// that rate exists to guarantee.
    #[test]
    fn the_target_rate_holds_the_whole_plan() {
        let got = channels_in_window(ideal_center_hz(), WINDOW_TARGET_RATE_HZ);
        assert_eq!(got.len(), CHANNELS.len(), "only {got:?} fit");
    }

    /// ...and so does the rate every front end in this tree actually lands on.
    ///
    /// The downconverter rounds its decimation to the nearest whole number, so
    /// the achievable rates are a ladder rather than a dial; this is the ladder,
    /// and every rung above 440 kHz has to hold all seven.
    #[test]
    fn the_rates_front_ends_land_on_hold_the_plan() {
        for &in_rate in &[2_400_000.0f64, 2_048_000.0, 2_500_000.0, 2_025_000.0, 8_000_000.0] {
            let rate = sdroxide_dsp::Ddc::rate_for(in_rate, WINDOW_TARGET_RATE_HZ);
            let got = channels_in_window(ideal_center_hz(), rate);
            assert_eq!(
                got.len(),
                CHANNELS.len(),
                "{in_rate} gives a {rate} Hz window holding only {got:?}"
            );
        }
    }

    /// A narrow window keeps what it can reach and says so, rather than
    /// refusing outright. An RTL-SDR at its slowest still hears three channels.
    #[test]
    fn a_narrow_window_keeps_what_it_reaches() {
        let got = channels_in_window(ideal_center_hz(), 250_000.0);
        assert!(!got.is_empty() && got.len() < CHANNELS.len(), "{got:?}");
        // ...and one narrow enough for a single channel gets the one it is on.
        let got = channels_in_window(VDL2_CSC_HZ, 40_000.0);
        assert_eq!(got, vec![6]);
    }

    /// The window slides inside the front end's span to reach the plan, and
    /// stops at the edge rather than asking for samples that are not there.
    #[test]
    fn the_window_slides_towards_the_plan_but_not_past_the_span() {
        // A wide front end centred elsewhere: the window reaches the plan.
        let c = window_center_for(137_500_000.0, 8_000_000.0, 500_000.0);
        assert_eq!(c, ideal_center_hz());
        // A narrow one centred well away: the window goes as far as it can.
        let c = window_center_for(137_500_000.0, 1_000_000.0, 500_000.0);
        assert!(c > 137_300_000.0 && c < 137_500_000.0, "{c}");
        // No room at all: it stays on the hardware centre and reaches nothing.
        let c = window_center_for(137_500_000.0, 500_000.0, 500_000.0);
        assert_eq!(c, 137_500_000.0);
        assert!(channels_in_window(c, 500_000.0).is_empty());
    }

    /// The per-channel rate does not fold the neighbouring channel onto the
    /// signal — the one kind of interference no later filter can undo.
    ///
    /// A neighbour sits 50 kHz away. At a channel rate below 100 kHz it is
    /// outside the Nyquist band and comes back somewhere else in it, and where
    /// it lands is what matters: anywhere outside the receive filter's passband
    /// is harmless, and on top of the signal is fatal. A rate near 58 kHz would
    /// put it exactly there.
    #[test]
    fn the_channel_rate_does_not_fold_the_neighbour_onto_the_signal() {
        let passband_hz = crate::demod::CUTOFF_SYMS * crate::demod::SYMBOL_RATE;
        for &window in &[480_000.0f64, 500_000.0, 506_250.0, 512_000.0, 250_000.0] {
            let rate = sdroxide_dsp::Ddc::rate_for(window, CHANNEL_TARGET_RATE_HZ);
            // Where 50 kHz appears after sampling at `rate`, folded into
            // (-rate/2, rate/2].
            let folded = {
                let f = (VDL2_CHANNEL_SPACING_HZ * 2.0).rem_euclid(rate);
                if f > rate / 2.0 { f - rate } else { f }
            };
            assert!(
                folded.abs() > passband_hz * 2.0,
                "a {window} Hz window gives {rate} Hz channels, putting the neighbour \
                 at {folded} Hz — inside the passband"
            );
            let sps = rate / crate::demod::SYMBOL_RATE;
            assert!((3.0..20.0).contains(&sps), "{window} Hz gives {sps} samples per symbol");
        }
    }
}
