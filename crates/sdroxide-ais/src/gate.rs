//! The slot gate: turns a continuously running channel into discrete
//! transmissions.
//!
//! AIS is self-organising TDMA. The minute is divided into 2250 slots of
//! 26.67 ms on each channel, a station claims one, transmits 256 bits into it
//! and is quiet again. Even a busy estuary leaves most slots empty, so the gate
//! watches one channel's baseband power, notices when a transmission arrives,
//! and hands the whole thing — ramp-up included — to the decoder as one buffer
//! it can measure over.
//!
//! # Why the whole burst has to arrive at once
//!
//! Everything downstream is estimated *over the transmission*: the slicing
//! level (which is where the receiver's frequency error ends up, once the
//! discriminator has turned it into a DC offset) and the bit timing. Both want
//! all 256 bits and neither is meaningful over a fragment, which is what a
//! streaming demodulator would have to work with. Gating first is what makes
//! them one-shot measurements rather than loops that have to acquire.
//!
//! # The noise floor
//!
//! A threshold means nothing without a floor, and the floor has to be learned
//! from a channel that is usually quiet and sometimes not. The obvious rule —
//! only update while the gate is shut — deadlocks both ways: seeded too low the
//! gate never shuts and the floor never updates; seeded too high it never opens,
//! and again never updates.
//!
//! So the floor tracks *down fast and up very slowly*. It converges on the
//! quietest power seen recently, which on a channel like this is the noise, and
//! a transmission lasting a hundredth of the rise time barely moves it.
//! Min-tracking of a noisy quantity settles below its mean, so the estimate is
//! scaled back up by [`FLOOR_BIAS`] — a constant this module's own test
//! *measures* against noise of known power rather than reasons about and hopes
//! for.
//!
//! The design is the VDL2 gate's, which is the ISM decoder's before it, with
//! AIS's numbers. It is not shared code: every constant below is derived from
//! the slot structure, and the one that matters most — [`HANG_BLOCKS`] — is set
//! from the gap between two adjacent slots, a quantity neither of the others
//! has.
//!
//! Source: ITU-R M.1371-5 §3.1 (the TDMA frame) and Table 26 (the transmission
//! packet).

use sdroxide_dsp::Complex32;
use sdroxide_types::AIS_BIT_RATE;

/// Samples per power measurement.
///
/// At a channel rate near 75 kHz this is 0.43 ms, about four bits. Long enough
/// that the mean of `|z|²` is a usable estimate, and short enough that the
/// GMSK envelope — which is constant, so this is only about the filter's own
/// ripple — cannot look like a gap in the transmission.
const BLOCK: usize = 32;

/// Per-block smoothing when the measurement is below the current floor.
const FLOOR_FALL: f32 = 0.05;
/// Per-block smoothing when it is above — three orders of magnitude slower.
const FLOOR_RISE: f32 = 0.00005;

/// Correction from where min-tracking settles to the actual mean noise power.
///
/// Measured by [`tests::the_floor_estimate_lands_on_the_true_noise_power`].
/// Changing `BLOCK`, `FLOOR_FALL` or `FLOOR_RISE` changes this number, and that
/// test is what will say so.
const FLOOR_BIAS: f32 = 1.34;

/// How far the power must fall back below the open threshold before the
/// transmission is over, as a power ratio — 6 dB of hysteresis.
const CLOSE_RATIO: f32 = 0.25;

/// Blocks below the close threshold before the gate shuts.
///
/// The number that decides whether two ships in *adjacent* slots arrive as two
/// transmissions or as one welded lump that fits neither. A slot is 26.67 ms
/// and a transmission is at most 256 bits of it; the standard's own timing
/// budget leaves the last part of the slot for ramp-down, propagation delay and
/// slot-boundary jitter, so the quiet between two occupied slots is on the
/// order of two milliseconds. Three blocks is 1.3 ms at 75 kHz — inside that
/// gap, and still long enough to ride the momentary dip a run of alternating
/// bits puts in a filtered envelope.
const HANG_BLOCKS: u32 = 3;

/// Shortest transmission worth decoding, in bits on the air.
///
/// The smallest real one is a long-range position report: 24 training bits,
/// two flags, 96 bits of data and its check sequence, about 152 bits. Below
/// that it is a click.
const MIN_BURST_BITS: f64 = 140.0;

/// Longest transmission kept, in bits on the air.
///
/// A message may claim up to five consecutive slots and transmits across them
/// without a gap, which is 1280 bits. The cap is a little over twice that, so a
/// pair of welded transmissions is still decoded and a channel sitting under a
/// stuck carrier is dropped rather than accumulated.
const MAX_BURST_BITS: f64 = 2_700.0;

/// Pre-trigger history, in bits.
///
/// Must hold the transmitter's ramp-up *and* the gate's own detection lag,
/// because the training sequence starts immediately after the ramp and the
/// timing estimate wants all of it. Eight bits of ramp plus four blocks of lag
/// is under thirty; forty is ample.
const PRE_BITS: f64 = 40.0;

/// One captured transmission.
#[derive(Debug, Clone)]
pub struct Burst {
    /// Baseband samples, pre-trigger history first.
    pub iq: Vec<Complex32>,
    pub rate_hz: f64,
    /// Absolute RF centre of the channel it came from.
    pub center_hz: f64,
    /// Peak block power against the channel's learned floor, dB.
    pub snr_db: f32,
    /// Peak block power in absolute dBFS — the comparable figure when both
    /// channels hear the same ship, since `snr_db` is referred to each
    /// channel's own floor.
    pub peak_dbfs: f32,
}

/// Slot gate for one channel.
pub struct Gate {
    rate_hz: f64,
    center_hz: f64,
    open_ratio: f32,

    inbuf: Vec<Complex32>,
    floor: f32,
    tracked: f32,
    seeded: bool,

    pre: Vec<Complex32>,
    pre_w: usize,
    pre_filled: usize,

    cur: Vec<Complex32>,
    cur_pre: usize,
    open: bool,
    hang: u32,
    peak: f32,

    min_samples: usize,
    max_samples: usize,

    /// Transmissions the gate opened on, and ones dropped for running past
    /// [`MAX_BURST_BITS`]. Reported so a channel sitting under a carrier looks
    /// like what it is rather than like a broken decoder.
    pub opened: u64,
    pub overlong: u64,
}

impl Gate {
    pub fn new(rate_hz: f64, center_hz: f64, threshold_db: f32) -> Gate {
        let sps = rate_hz / AIS_BIT_RATE;
        let pre = (PRE_BITS * sps).round().max(BLOCK as f64) as usize;
        Gate {
            rate_hz,
            center_hz,
            open_ratio: 10f32.powf(threshold_db / 10.0),
            inbuf: Vec::with_capacity(BLOCK * 2),
            floor: 0.0,
            tracked: 0.0,
            seeded: false,
            pre: vec![Complex32::default(); pre],
            pre_w: 0,
            pre_filled: 0,
            cur: Vec::new(),
            cur_pre: 0,
            open: false,
            hang: 0,
            peak: 0.0,
            min_samples: (MIN_BURST_BITS * sps) as usize,
            max_samples: (MAX_BURST_BITS * sps) as usize,
            opened: 0,
            overlong: 0,
        }
    }

    /// Change the threshold without losing the learned floor — the noise did
    /// not move because the operator dragged a slider.
    pub fn set_threshold_db(&mut self, db: f32) {
        self.open_ratio = 10f32.powf(db / 10.0);
    }

    /// Current noise-floor estimate as dBFS.
    pub fn floor_dbfs(&self) -> f32 {
        10.0 * self.floor.max(1e-30).log10()
    }

    /// Feed baseband samples; append any completed transmissions to `out`.
    pub fn push(&mut self, iq: &[Complex32], out: &mut Vec<Burst>) {
        let mut buf = std::mem::take(&mut self.inbuf);
        buf.extend_from_slice(iq);
        let mut pos = 0usize;
        while pos + BLOCK <= buf.len() {
            let block = &buf[pos..pos + BLOCK];
            let power = block.iter().map(|z| z.norm_sqr()).sum::<f32>() / BLOCK as f32;
            self.advance(block, power, out);
            pos += BLOCK;
        }
        if pos > 0 {
            buf.drain(..pos);
        }
        self.inbuf = buf;
    }

    fn advance(&mut self, block: &[Complex32], power: f32, out: &mut Vec<Burst>) {
        if !self.seeded {
            self.tracked = power;
            self.seeded = true;
        } else {
            let alpha = if power < self.tracked { FLOOR_FALL } else { FLOOR_RISE };
            self.tracked += alpha * (power - self.tracked);
        }
        self.floor = self.tracked * FLOOR_BIAS;
        let open_at = self.floor * self.open_ratio;

        if !self.open {
            self.push_pre(block);
            if power > open_at {
                self.open = true;
                self.hang = 0;
                self.peak = power;
                self.opened += 1;
                self.cur.clear();
                self.take_pre_into_cur();
                self.cur_pre = self.cur.len();
            }
            return;
        }

        self.cur.extend_from_slice(block);
        self.peak = self.peak.max(power);

        if power < open_at * CLOSE_RATIO {
            self.hang += 1;
        } else {
            self.hang = 0;
        }

        let too_long = self.cur.len() > self.max_samples;
        if self.hang < HANG_BLOCKS && !too_long {
            return;
        }

        self.open = false;
        // The hangover is not trimmed. The decoder finds the frame by hunting
        // for a flag, so trailing silence costs it nothing, and cutting close
        // would risk clipping the last bits of a transmission that ended on a
        // run of alternating levels.
        if too_long {
            self.overlong += 1;
        } else if self.cur.len().saturating_sub(self.cur_pre) >= self.min_samples {
            out.push(Burst {
                iq: std::mem::take(&mut self.cur),
                rate_hz: self.rate_hz,
                center_hz: self.center_hz,
                snr_db: 10.0 * (self.peak / self.floor.max(1e-30)).max(1e-30).log10(),
                peak_dbfs: 10.0 * self.peak.max(1e-30).log10(),
            });
        }
        self.cur.clear();
        // The pre-trigger ring now holds the tail of the transmission just
        // emitted; dropping it stops the next one being prefixed with the last.
        self.pre_filled = 0;
        self.pre_w = 0;
    }

    fn push_pre(&mut self, block: &[Complex32]) {
        for &z in block {
            self.pre[self.pre_w] = z;
            self.pre_w = (self.pre_w + 1) % self.pre.len();
            self.pre_filled = (self.pre_filled + 1).min(self.pre.len());
        }
    }

    fn take_pre_into_cur(&mut self) {
        let n = self.pre_filled;
        let len = self.pre.len();
        let start = (self.pre_w + len - n) % len;
        for i in 0..n {
            self.cur.push(self.pre[(start + i) % len]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tx::{Noise, TxParams, modulate};

    /// [`FLOOR_BIAS`] is measured, not guessed: feed noise of known power and
    /// check the settled estimate against it.
    #[test]
    fn the_floor_estimate_lands_on_the_true_noise_power() {
        let mut n = Noise::new(0x1234_5678);
        // Two components at sigma each, so the mean power is 2·sigma².
        let sigma = 0.03f32;
        let want = 2.0 * sigma * sigma;
        let mut g = Gate::new(75_000.0, super::super::plan::CHANNELS[0].center_hz, 8.0);
        let mut out = Vec::new();
        let mut buf = vec![Complex32::default(); 4096];
        for _ in 0..40 {
            for s in buf.iter_mut() {
                *s = n.gaussian(sigma);
            }
            g.push(&buf, &mut out);
        }
        let err_db = 10.0 * (g.floor / want).log10();
        assert!(err_db.abs() < 1.0, "floor is {err_db} dB from the true noise power");
        assert!(out.is_empty(), "noise alone opened the gate {} times", out.len());
    }

    /// A transmission is caught whole, with its ramp in front of it — which is
    /// the only reason the training sequence survives the gate's detection lag.
    #[test]
    fn a_transmission_is_caught_with_its_ramp() {
        let rate = 75_000.0;
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let burst = modulate(&[0x55u8; 21], &p);

        let mut n = Noise::new(99);
        let mut g = Gate::new(rate, 161_975_000.0, 8.0);
        let mut out = Vec::new();
        let mut quiet = vec![Complex32::default(); 20_000];
        n.add(&mut quiet, 0.01);
        g.push(&quiet, &mut out);
        let mut sig = burst.clone();
        n.add(&mut sig, 0.01);
        g.push(&sig, &mut out);
        let mut quiet2 = vec![Complex32::default(); 20_000];
        n.add(&mut quiet2, 0.01);
        g.push(&quiet2, &mut out);

        assert_eq!(out.len(), 1, "expected exactly one transmission");
        let b = &out[0];
        assert!(b.iq.len() >= burst.len(), "the transmission was clipped");
        assert!(b.snr_db > 20.0, "signal-to-noise {}", b.snr_db);
        assert_eq!(b.center_hz, 161_975_000.0);
    }

    /// Two ships in adjacent slots must arrive as two transmissions. This is
    /// what [`HANG_BLOCKS`] is set for, and welding them would cost both: the
    /// timing and slicing levels are measured once per burst.
    #[test]
    fn two_adjacent_slots_do_not_weld_into_one() {
        let rate = 75_000.0;
        let sps = rate / AIS_BIT_RATE;
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let a = modulate(&[0x5au8; 21], &p);
        let b = modulate(&[0xa5u8; 21], &p);

        let mut n = Noise::new(7);
        let mut g = Gate::new(rate, 161_975_000.0, 8.0);
        let mut out = Vec::new();
        let mut quiet = vec![Complex32::default(); 20_000];
        n.add(&mut quiet, 0.01);
        g.push(&quiet, &mut out);

        // The slot is 256 bits; the transmission is shorter, and what is left
        // is the gap the gate has to find.
        let gap = ((256.0 * sps) as usize).saturating_sub(a.len());
        assert!(gap > 0, "the test's own slot arithmetic is wrong");
        let mut run = a.clone();
        run.extend(std::iter::repeat_n(Complex32::default(), gap));
        run.extend_from_slice(&b);
        n.add(&mut run, 0.01);
        g.push(&run, &mut out);
        g.push(&quiet, &mut out);

        assert_eq!(out.len(), 2, "the two slots were welded into one burst");
    }

    /// A carrier that never stops is dropped and counted, not accumulated until
    /// the process runs out of memory.
    #[test]
    fn a_stuck_carrier_is_dropped_and_counted() {
        let rate = 75_000.0;
        let mut g = Gate::new(rate, 161_975_000.0, 8.0);
        let mut out = Vec::new();
        let mut n = Noise::new(5);
        let mut quiet = vec![Complex32::default(); 8192];
        n.add(&mut quiet, 0.01);
        g.push(&quiet, &mut out);

        // A little past the cap, but by less than the shortest transmission:
        // the gate drops the over-long run and then re-opens on what is left,
        // and what is left has to be too short to be emitted. Feeding it a
        // whole extra slot's worth would have the gate emit that as a burst,
        // which is correct behaviour and not what this test is about.
        let len = (MAX_BURST_BITS * rate / AIS_BIT_RATE) as usize + 200;
        let carrier = vec![Complex32::new(1.0, 0.0); len];
        g.push(&carrier, &mut out);
        g.push(&quiet, &mut out);
        assert!(out.is_empty(), "an endless carrier was emitted as a transmission");
        assert_eq!(g.overlong, 1);
    }

    /// The cap clears the longest transmission the standard allows — five
    /// consecutive slots, sent as one.
    #[test]
    fn the_cap_clears_the_longest_transmission_the_standard_allows() {
        // Five consecutive slots, sent without a gap, is 1280 bits on the air.
        let longest = 5.0 * 256.0;
        assert!(MAX_BURST_BITS > longest * 1.2, "{MAX_BURST_BITS} leaves no margin over {longest}");
    }
}
