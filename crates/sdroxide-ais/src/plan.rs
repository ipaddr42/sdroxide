//! The two AIS channels, and which of them a given receiver window can reach.
//!
//! # Why two channels and not four
//!
//! AIS 1 (161.975 MHz) and AIS 2 (162.025 MHz) carry the service. A station
//! alternates between them slot by slot, so *both* have to be listened to or
//! half of every vessel's reports are lost — which does not look like half a
//! signal, it looks like a vessel that reports at half the rate and jumps.
//!
//! There are two more allocations, 156.775 and 156.825 MHz, used for the
//! long-range message 27 to satellites. They are not in this plan: they sit
//! five megahertz down, so reaching them and the main pair together needs a
//! window no receiver in this tree would spend on it, and what arrives there is
//! a once-every-three-minutes position with no name attached. A receiver parked
//! on them would decode message 27 through this same lane if it were added; the
//! decision here is only about where the window goes.
//!
//! # One window, two channels
//!
//! The pair spans 75 kHz including the outer channels' own slots, so a window
//! of 100 kHz covers both on any receiver in this tree. A narrower one reaches
//! whichever channel it happens to be over — half the traffic, which is a real
//! answer and better than a refusal — and the panel is told which.
//!
//! Source: ITU-R M.1371-5 §2.1 and the Appendix 18 marine VHF channel table
//! (channels 87B and 88B, 2087 and 2088 in the four-digit form).

use sdroxide_types::{
    AIS_BIT_RATE, AIS_CHANNEL_A_HZ, AIS_CHANNEL_B_HZ, AIS_CHANNEL_SPACING_HZ, AIS_GOOD_SPS,
    AIS_PLAN_CENTER_HZ,
};

/// One channel of the plan.
pub struct Channel {
    pub center_hz: f64,
    /// `"A"` or `"B"` — what every AIS display in the world calls them.
    pub label: &'static str,
    /// The Appendix 18 channel number, for an operator setting a marine set.
    pub marine: &'static str,
}

/// The channels, ascending.
pub const CHANNELS: [Channel; 2] = [
    Channel { center_hz: AIS_CHANNEL_A_HZ, label: "A", marine: "87B" },
    Channel { center_hz: AIS_CHANNEL_B_HZ, label: "B", marine: "88B" },
];

/// Fraction of a front end's span the window may claim.
///
/// The outer edges of any receiver's window are where its own anti-alias filter
/// is already rolling off, and a channel sitting in the roll-off decodes badly
/// or not at all. The same three quarters the ISM, ADS-B and VDL2 lanes use,
/// for the same reason.
pub const USABLE_FRACTION: f64 = 0.75;

/// What the lane asks its window down-converter for.
///
/// Not simply the plan's span divided by [`USABLE_FRACTION`]: a
/// [`sdroxide_dsp::Ddc`] decimates by a whole number and rounds to the
/// *nearest* one, so a target sitting exactly on the requirement can round the
/// wrong way and land under it. 150 kHz leaves room for that rounding on every
/// front end here — an RTL-SDR at 2.4 Msps lands on exactly 150 kHz, and one at
/// 2.048 Msps on 146.3.
pub const WINDOW_TARGET_RATE_HZ: f64 = 150_000.0;

/// The widest a channel stream is allowed to be.
///
/// More samples a bit buys nothing past about ten — the timing estimate is
/// already finer than the noise — and every one of them is a complex multiply
/// in the receive filter, on two channels, forever.
pub const CHANNEL_MAX_RATE_HZ: f64 = 160_000.0;

/// How far from baseband the *other* channel has to land after decimation
/// before it is out of the way.
///
/// An AIS signal occupies about ±7 kHz. A neighbour folded inside this is
/// inside the receive filter's passband, where no later stage can reach it —
/// which is the one kind of interference that cannot be undone, and the trap
/// [`channel_decimation`] exists to avoid.
pub const NEIGHBOUR_GUARD_HZ: f64 = 15_000.0;

/// Distance from the plan's outermost channel centres to its edges.
pub fn span_hz() -> f64 {
    let lo = CHANNELS[0].center_hz - AIS_CHANNEL_SPACING_HZ / 2.0;
    let hi = CHANNELS[CHANNELS.len() - 1].center_hz + AIS_CHANNEL_SPACING_HZ / 2.0;
    hi - lo
}

/// Where the window wants to sit to reach both channels.
pub fn ideal_center_hz() -> f64 {
    AIS_PLAN_CENTER_HZ
}

/// Whether a channel is inside a window, with its own slot and the front end's
/// roll-off allowed for.
pub fn fits(center_hz: f64, window_center_hz: f64, window_rate_hz: f64) -> bool {
    let half = window_rate_hz * USABLE_FRACTION / 2.0;
    (center_hz - window_center_hz).abs() + AIS_CHANNEL_SPACING_HZ / 2.0 <= half
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
/// otherwise — a receiver that cannot hold both channels should still hold one
/// rather than refusing.
pub fn window_center_for(hw_center_hz: f64, hw_rate_hz: f64, window_rate_hz: f64) -> f64 {
    let slack = (hw_rate_hz * USABLE_FRACTION - window_rate_hz) / 2.0;
    if slack <= 0.0 {
        return hw_center_hz;
    }
    ideal_center_hz().clamp(hw_center_hz - slack, hw_center_hz + slack)
}

/// Where `f` appears after sampling at `rate`, folded into `(-rate/2, rate/2]`.
fn fold_hz(f: f64, rate: f64) -> f64 {
    let x = f.rem_euclid(rate);
    if x > rate / 2.0 { x - rate } else { x }
}

/// How far the channel down-converter should decimate the window.
///
/// # Why this is chosen here rather than left to a target rate
///
/// Every other lane in the tree hands [`sdroxide_dsp::Ddc`] a wanted rate and
/// takes whatever integer decimation it rounds to. That cannot be done here,
/// because of an arithmetic coincidence peculiar to this plan: the two AIS
/// channels are **50 kHz apart**, so a channel rate anywhere near 50 kHz folds
/// one channel onto the other. At 50.0 it lands exactly on top of it; at 48 —
/// which is what a bare "five samples a bit" target would ask for — it lands
/// 2 kHz off it, which is worse, because it is then inside the receive filter
/// and beating with the ship being decoded.
///
/// Aliasing is the one kind of interference no later stage can undo, which is
/// what makes this worth choosing rather than accepting. Everything *else* the
/// neighbour could do is handled by [`crate::demod::RxFilter`], which sits at
/// ±9 kHz in front of the gate and removes it outright at any rate where it has
/// not already folded inside the passband.
///
/// So the decimation is picked outright: the largest one whose rate still
/// leaves [`AIS_GOOD_SPS`] samples a bit, is no wider than
/// [`CHANNEL_MAX_RATE_HZ`], and puts the neighbour at least
/// [`NEIGHBOUR_GUARD_HZ`] from baseband. Largest, because the rate is what
/// every sample of the receive filter and the discriminator costs — and a
/// decimation of one is a perfectly good answer where the arithmetic works out
/// that way, because the receive filter is in the chain either way.
pub fn channel_decimation(window_rate_hz: f64, both_channels: bool) -> usize {
    let min_rate = AIS_GOOD_SPS * AIS_BIT_RATE;
    let neighbour = 2.0 * AIS_CHANNEL_SPACING_HZ;
    let mut best = None;
    for m in 1..=64usize {
        let rate = window_rate_hz / m as f64;
        if rate < min_rate {
            break;
        }
        if rate > CHANNEL_MAX_RATE_HZ {
            continue;
        }
        if both_channels && fold_hz(neighbour, rate).abs() < NEIGHBOUR_GUARD_HZ {
            continue;
        }
        best = Some(m);
    }
    // Nothing satisfied everything: take the rate itself, which happens only on
    // a window too narrow to hold both channels anyway — and there the window's
    // own decimating filter has already removed the neighbour.
    best.unwrap_or(1)
}

/// The rate a channel stream will run at, given the window it is taken from.
pub fn channel_rate_for(window_rate_hz: f64, both_channels: bool) -> f64 {
    window_rate_hz / channel_decimation(window_rate_hz, both_channels) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The plan's span and its ideal centre come from the table, so a change
    /// there moves them rather than leaving them stale.
    #[test]
    fn the_span_and_centre_come_from_the_table() {
        assert_eq!(span_hz(), 75_000.0);
        let lo = CHANNELS[0].center_hz - AIS_CHANNEL_SPACING_HZ / 2.0;
        assert_eq!(ideal_center_hz(), lo + span_hz() / 2.0);
    }

    /// A window at the target rate holds both channels — which is the thing
    /// that rate exists to guarantee — and so does every rate the front ends in
    /// this tree actually land on.
    #[test]
    fn the_rates_front_ends_land_on_hold_both_channels() {
        for &in_rate in
            &[2_400_000.0f64, 2_048_000.0, 2_500_000.0, 1_024_000.0, 250_000.0, 8_000_000.0]
        {
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
    /// refusing outright.
    #[test]
    fn a_narrow_window_keeps_the_channel_it_is_over() {
        let got = channels_in_window(AIS_CHANNEL_A_HZ, 50_000.0);
        assert_eq!(got, vec![0]);
        let got = channels_in_window(AIS_CHANNEL_B_HZ, 50_000.0);
        assert_eq!(got, vec![1]);
        // Parked between them and too narrow for either: nothing.
        assert!(channels_in_window(ideal_center_hz(), 50_000.0).is_empty());
    }

    /// The window slides inside the front end's span to reach the plan, and
    /// stops at the edge rather than asking for samples that are not there.
    #[test]
    fn the_window_slides_towards_the_plan_but_not_past_the_span() {
        let c = window_center_for(160_000_000.0, 8_000_000.0, 150_000.0);
        assert_eq!(c, ideal_center_hz());
        let c = window_center_for(162_400_000.0, 800_000.0, 150_000.0);
        assert!(c > ideal_center_hz() && c < 162_400_000.0, "{c}");
        let c = window_center_for(162_400_000.0, 150_000.0, 150_000.0);
        assert_eq!(c, 162_400_000.0);
    }

    /// The one that matters: whatever window a front end lands on, the other
    /// AIS channel must not fold onto the one being decoded — and the channel
    /// stream must never be the window itself, because a decimation of one has
    /// no filter in it at all.
    ///
    /// 50 kHz apart is a coincidence with teeth: the obvious "five samples a
    /// bit" target of 48 kHz would put the neighbour 2 kHz from baseband, dead
    /// centre of the passband, on every receiver at once.
    #[test]
    fn the_channel_rate_never_folds_the_other_channel_onto_this_one() {
        for &in_rate in &[
            2_400_000.0f64,
            2_048_000.0,
            2_500_000.0,
            2_025_000.0,
            1_024_000.0,
            900_001.0,
            250_000.0,
            240_000.0,
            8_000_000.0,
            32_400_000.0,
        ] {
            let window = sdroxide_dsp::Ddc::rate_for(in_rate, WINDOW_TARGET_RATE_HZ);
            let m = channel_decimation(window, true);
            let rate = window / m as f64;
            let folded = fold_hz(2.0 * AIS_CHANNEL_SPACING_HZ, rate);
            assert!(
                folded.abs() >= NEIGHBOUR_GUARD_HZ,
                "a {window} Hz window gives {rate} Hz channels, putting the other \
                 channel at {folded} Hz — inside the passband"
            );
            let sps = rate / AIS_BIT_RATE;
            assert!((AIS_GOOD_SPS..=17.0).contains(&sps), "{window} Hz gives {sps} samples a bit");
            // And the Ddc, handed this rate as its target, agrees on the
            // decimation — otherwise the guard above would be about a stream
            // nothing produces.
            assert_eq!(sdroxide_dsp::Ddc::rate_for(window, rate), rate);
        }
    }

    /// A window holding only one channel has no neighbour to fold — its own
    /// decimating filter took it out — so the rate is chosen on samples a bit
    /// alone and a narrow receiver is not refused for a hazard that is not
    /// there.
    #[test]
    fn a_single_channel_window_is_not_held_to_the_neighbour_guard() {
        let m = channel_decimation(48_000.0, false);
        assert_eq!(m, 1);
        assert!(channel_rate_for(48_000.0, false) / AIS_BIT_RATE >= AIS_GOOD_SPS);
    }
}
