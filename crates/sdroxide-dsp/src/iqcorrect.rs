//! Front-end IQ correction for a zero-IF receiver: the DC spike in the middle
//! of the span, and the mirror image of every signal reflected about it.
//!
//! # What it corrects
//!
//! A quadrature front end mixes straight to baseband with two analogue paths
//! that are never quite identical, and the two defects that leaves both sit in
//! the spectrum where they are impossible to miss:
//!
//! * **DC** — the tuner's own LO leaks back into its mixer and demodulates
//!   itself, and the converter adds an offset of its own. On an RTL-SDR the
//!   8-bit samples are also centred on a value that is not exactly mid-scale.
//!   All of it lands on the centre of the span as a permanent carrier.
//! * **Amplitude and phase imbalance** — if the I and Q paths differ in gain,
//!   or their phases are not exactly 90° apart, the stream is no longer
//!   analytic and every signal appears again mirrored about the centre. On a
//!   typical dongle that image is 25–40 dB down, which is enough to look like a
//!   station that is not there.
//!
//! Neither can be fixed by tuning: the R820T family has no offset-tuning mode
//! to move the LO out of the passband, so a receiver that wants a clean centre
//! has to correct in DSP.
//!
//! # How
//!
//! DC is a leaky running mean, subtracted — see [`ComplexDcBlock`].
//!
//! The imbalance correction is the classic orthogonalise-and-normalise
//! estimator, run as a closed loop on its own output:
//!
//! ```text
//! i' = i
//! q' = gain · (q − alpha · i)
//! ```
//!
//! For any stream that is circularly symmetric — noise, and any collection of
//! signals that does not happen to be a mirror image of itself — I and Q are
//! uncorrelated and carry equal power. Both are properties of the *output*, so
//! every [`BLOCK`] samples the loop measures how far its own output misses them
//! and steps the two coefficients a fraction of the way towards closing the
//! gap: `alpha` until `E[i·q']` is zero, `gain` until `E[q'²]` matches `E[i²]`.
//! Convergence is deliberately unhurried — [`TAU_S`] — so the estimate settles
//! on the front end rather than chasing a fading signal, and the residual
//! jitter stays far below the image it is there to remove.
//!
//! That fraction is derived from the sample rate rather than fixed, because a
//! block is a *count of samples* and the loop is trying to hold a time
//! constant. A sound card's quadrature output — a rig's I/Q on a 48 kHz stereo
//! input — runs fifty times slower than a dongle, so a step sized for the
//! dongle would take the better part of a minute to converge there, which an
//! operator reads as the correction not working at all.
//!
//! The estimator needs no knowledge of what is being received, but there is one
//! input it cannot read: a baseband that is genuinely symmetric about the
//! centre — two equally strong carriers either side of it, or a double-sideband
//! signal centred exactly on the dial — is indistinguishable from an imbalance,
//! because that is precisely what an imbalance manufactures. A block that
//! measures something no front end could be ([`IMPLAUSIBLE_ALPHA`],
//! [`IMPLAUSIBLE_RATIO`]) is therefore thrown away rather than learned from,
//! and what a milder one can do is bounded by the clamps ([`MAX_ALPHA`],
//! [`MAX_GAIN`]) and unwound as soon as the band stops looking like that.
//!
//! DC removal has no such escape: a carrier that really does sit on the centre
//! frequency *is* DC, so it goes with the offset — an AM station tuned dead on
//! comes out with its carrier stripped and its envelope detector distorting.
//! Tuning a kilohertz off, or switching the correction off, are both answers;
//! it is a large part of why this is a switch and not a permanent fixture.
//! (Front ends whose driver can park the LO clear of the passband —
//! `IqSource::lo_offset_hz` — never meet the question at all.)

use crate::Complex32;
use crate::demod::ComplexDcBlock;

/// Samples per estimator update. At 2.4 Msps this is ~7 ms, and 16 k samples
/// measure a correlation to about 1 % — an order of magnitude finer than the
/// imbalance being corrected, before the loop's step smooths it further.
const BLOCK: usize = 16_384;

/// How long the estimate takes to close the bulk of the gap, in seconds. The
/// step taken per block follows from it and the sample rate — one block is
/// `BLOCK / rate` seconds of signal, and taking that fraction of the error each
/// time makes the loop an exponential with this time constant.
const TAU_S: f64 = 0.35;

/// Bounds on that step. The floor keeps a very fast front end from crawling;
/// the ceiling is what a slow one runs into — at 48 kHz a block is 341 ms, so
/// the time constant [`TAU_S`] asks for is shorter than a single measurement
/// and the loop can do no better than take a quarter of each. That still
/// settles in about a second and a half, and a quarter of a block's measurement
/// noise (~1 % over [`BLOCK`] samples) leaves the residual image far below what
/// the front end started with.
const MU_MIN: f64 = 0.005;
const MU_MAX: f64 = 0.25;

/// Largest phase skew the loop may apply, in radians of quadrature error
/// (0.2 ≈ 11°). Hardware needing more than this is broken, so a larger
/// estimate means the loop has been pulled by a signal that breaks its
/// assumptions rather than by the front end.
const MAX_ALPHA: f64 = 0.2;

/// Largest Q-branch gain correction, as a ratio. Same reasoning as
/// [`MAX_ALPHA`]: 1.25 (≈ 2 dB) is already outside what a working front end
/// produces.
const MAX_GAIN: f64 = 1.25;

/// A block that measures a quadrature error past ~30° is not measuring the
/// front end: the spectrum in it is close to a mirror image of itself — two
/// carriers either side of the centre, or a double-sideband signal on it —
/// which is indistinguishable from an imbalance and would drag the loop
/// somewhere it cannot see. Such a block is discarded rather than learned from,
/// which is also what makes the pull recoverable: nothing was taken from it.
const IMPLAUSIBLE_ALPHA: f64 = 0.5;

/// The same for the power ratio: no front end has one path 3:1 above the other,
/// so a block that says so is describing its signal, not the hardware.
const IMPLAUSIBLE_RATIO: f64 = 3.0;

/// DC removal plus adaptive amplitude/phase imbalance correction, applied to a
/// raw device stream before anything else sees it.
pub struct IqCorrect {
    dc: ComplexDcBlock,
    /// Whether the imbalance loop runs at all, or this is a DC blocker wearing
    /// the same coat — see [`IqCorrect::dc_only`].
    balance: bool,
    /// Fraction of each block's measured error folded into the estimate,
    /// derived from the sample rate so the loop's time constant is [`TAU_S`]
    /// whatever the front end's rate is.
    mu: f64,
    /// How much I to subtract from Q to square up the quadrature.
    alpha: f64,
    /// Q-branch gain that equalises the two paths' power.
    gain: f64,
    /// Samples accumulated into the sums below since the last update.
    n: usize,
    sum_ii: f64,
    sum_qq: f64,
    sum_iq: f64,
}

impl IqCorrect {
    /// DC removal and imbalance correction both. `corner_hz` is the DC
    /// blocker's corner — see [`ComplexDcBlock`].
    pub fn new(corner_hz: f64, sample_rate: f64) -> Self {
        Self::build(corner_hz, sample_rate, true)
    }

    /// DC removal alone, at whatever corner is asked for.
    ///
    /// For the operator who wants the spike in the middle of the waterfall gone
    /// and nothing else touched — the imbalance loop is the half that can be
    /// fooled by a mirror-symmetric band, so leaving it off has to stay
    /// possible. The reverse pairing does not: an uncorrected offset multiplies
    /// into `E[i·q]` as a constant bias, so the imbalance loop always removes
    /// DC first whether it was asked to or not.
    pub fn dc_only(corner_hz: f64, sample_rate: f64) -> Self {
        Self::build(corner_hz, sample_rate, false)
    }

    fn build(corner_hz: f64, sample_rate: f64, balance: bool) -> Self {
        let mu = if sample_rate > 0.0 {
            (BLOCK as f64 / sample_rate / TAU_S).clamp(MU_MIN, MU_MAX)
        } else {
            MU_MAX
        };
        IqCorrect {
            dc: ComplexDcBlock::new(corner_hz, sample_rate),
            balance,
            mu,
            alpha: 0.0,
            gain: 1.0,
            n: 0,
            sum_ii: 0.0,
            sum_qq: 0.0,
            sum_iq: 0.0,
        }
    }

    /// Correct a block in place.
    pub fn process(&mut self, buf: &mut [Complex32]) {
        self.dc.process(buf);
        if !self.balance {
            return;
        }
        // Cut at the estimator's own block boundary and run the samples
        // between two of them straight through. The coefficients only ever
        // change at such a boundary, so this corrects exactly what the
        // sample-at-a-time loop corrected — but the loop below has no branch
        // and no shared accumulator in it, which is the difference between a
        // vector of samples per iteration and one. At 2.4 Msps this pass was
        // measured at a seventh of the whole receive thread.
        let mut at = 0;
        while at < buf.len() {
            let take = (BLOCK - self.n).min(buf.len() - at);
            let (alpha, gain) = (self.alpha as f32, self.gain as f32);
            let (mut ii, mut qq, mut iq) = (0.0f64, 0.0f64, 0.0f64);
            for s in &mut buf[at..at + take] {
                let i = s.re;
                let q = gain * (s.im - alpha * i);
                s.im = q;
                ii += (i as f64) * (i as f64);
                qq += (q as f64) * (q as f64);
                iq += (i as f64) * (q as f64);
            }
            self.sum_ii += ii;
            self.sum_qq += qq;
            self.sum_iq += iq;
            self.n += take;
            at += take;
            if self.n >= BLOCK {
                self.update();
            }
        }
    }

    /// Forget the estimate and start converging again. Worth doing when the
    /// correction is switched back on after a spell off the air, so it does not
    /// resume with a coefficient measured on a band the receiver has left.
    pub fn reset(&mut self) {
        self.alpha = 0.0;
        self.gain = 1.0;
        self.n = 0;
        self.sum_ii = 0.0;
        self.sum_qq = 0.0;
        self.sum_iq = 0.0;
    }

    /// The current quadrature-error correction in radians, and the Q-branch
    /// gain. Diagnostics: what the loop has converged on says how good the
    /// front end is.
    pub fn estimate(&self) -> (f64, f64) {
        (self.alpha, self.gain)
    }

    /// Fold one block of statistics into the estimate.
    fn update(&mut self) {
        let (ii, qq, iq) = (self.sum_ii, self.sum_qq, self.sum_iq);
        self.n = 0;
        self.sum_ii = 0.0;
        self.sum_qq = 0.0;
        self.sum_iq = 0.0;
        // A silent block — a stalled front end, or the gain turned all the way
        // down — carries no information about the imbalance. Hold the estimate.
        if ii <= f64::MIN_POSITIVE || qq <= f64::MIN_POSITIVE {
            return;
        }
        // Residual correlation, in units of Q per I. Subtracting `rho·i` from
        // the output would zero it outright; `alpha` reaches the output through
        // `gain`, and only a fraction `mu` of the step is taken.
        let rho = iq / ii;
        if rho.abs() <= IMPLAUSIBLE_ALPHA {
            self.alpha = (self.alpha + self.mu * rho / self.gain).clamp(-MAX_ALPHA, MAX_ALPHA);
        }
        // Same idea for the power ratio: the full correction is sqrt(ii/qq).
        let ratio = (ii / qq).sqrt();
        if (1.0 / IMPLAUSIBLE_RATIO..=IMPLAUSIBLE_RATIO).contains(&ratio) {
            self.gain = (self.gain * ratio.powf(self.mu)).clamp(1.0 / MAX_GAIN, MAX_GAIN);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustfft::FftPlanner;

    /// Pseudo-random noise, so the tests do not depend on a random crate.
    fn noise(n: usize) -> Vec<Complex32> {
        let mut state = 0x2545_f491_4f6c_dd1d_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 8_388_608.0 - 1.0
        };
        (0..n).map(|_| Complex32::new(0.1 * next(), 0.1 * next())).collect()
    }

    /// Apply a known front-end defect: gain and phase imbalance, then DC.
    fn spoil(buf: &mut [Complex32], gain: f32, phase_rad: f32, dc: Complex32) {
        for s in buf {
            let i = s.re;
            let q = gain * (s.im * phase_rad.cos() + s.re * phase_rad.sin());
            *s = Complex32::new(i + dc.re, q + dc.im);
        }
    }

    /// Power in the bin holding a tone at `cycles_per_sample`, and in its
    /// mirror, in dB relative to full scale.
    fn tone_and_image_db(buf: &[Complex32], cycles_per_sample: f64) -> (f64, f64) {
        let n = buf.len();
        let fft = FftPlanner::<f32>::new().plan_fft_forward(n);
        let mut scratch: Vec<Complex32> = buf.to_vec();
        fft.process(&mut scratch);
        let bin = (cycles_per_sample * n as f64).round() as usize;
        let power = |k: usize| 20.0 * (scratch[k % n].norm() as f64 + 1e-30).log10();
        (power(bin), power(n - bin))
    }

    #[test]
    fn dc_offset_is_removed() {
        let mut buf = noise(200_000);
        spoil(&mut buf, 1.0, 0.0, Complex32::new(0.05, -0.03));
        let mut corr = IqCorrect::new(20.0, 2_400_000.0);
        corr.process(&mut buf);
        // The blocker needs a moment to charge; judge it on the settled tail.
        let tail = &buf[100_000..];
        let mean: Complex32 =
            tail.iter().sum::<Complex32>() / Complex32::new(tail.len() as f32, 0.0);
        assert!(mean.norm() < 1e-3, "residual DC {mean:?}");
    }

    #[test]
    fn the_mirror_image_is_suppressed() {
        // A tone a quarter of the way up the span, through a front end with 5 %
        // gain error and 3° of quadrature error — an ordinary dongle. Two
        // seconds of it at 2.4 Msps, because the loop is deliberately slow.
        let f = 0.25_f64;
        let n = 5_000_000;
        let mut buf: Vec<Complex32> = (0..n)
            .map(|k| {
                let ph = (std::f64::consts::TAU * f * k as f64) as f32;
                Complex32::new(0.2 * ph.cos(), 0.2 * ph.sin())
            })
            .collect();
        spoil(&mut buf, 1.05, 3.0_f32.to_radians(), Complex32::new(0.05, -0.03));

        let window = 1 << 14;
        let (tone_before, image_before) = tone_and_image_db(&buf[..window], f);
        let before = tone_before - image_before;

        let mut corr = IqCorrect::new(20.0, 2_400_000.0);
        corr.process(&mut buf);
        let (tone_after, image_after) = tone_and_image_db(&buf[n - window..], f);
        let after = tone_after - image_after;

        assert!(before < 40.0, "the spoiled signal should have a visible image: {before:.1} dB");
        assert!(
            after > before + 30.0,
            "image rejection {before:.1} dB -> {after:.1} dB, expected 30 dB better"
        );
        // Inverting `spoil` exactly means alpha = g·sin φ and gain = 1/(g·cos φ).
        let (alpha, gain) = corr.estimate();
        let (want_alpha, want_gain) = {
            let (g, p) = (1.05_f64, 3.0_f64.to_radians());
            (g * p.sin(), 1.0 / (g * p.cos()))
        };
        assert!(
            (alpha - want_alpha).abs() < 0.005 && (gain - want_gain).abs() < 0.005,
            "converged on ({alpha:.4}, {gain:.4}), wanted ({want_alpha:.4}, {want_gain:.4})"
        );
    }

    /// The input the estimator cannot read: a double-sideband signal centred on
    /// the dial is a mirror image of itself, so it *looks* like an all-I,
    /// no-Q front end. The loop must recognise that as impossible rather than
    /// swing the Q branch to the stop chasing it.
    #[test]
    fn a_mirror_symmetric_signal_does_not_pull_the_loop() {
        let n = 5_000_000;
        let mut buf = noise(n);
        for (k, s) in buf.iter_mut().enumerate() {
            // A carrier on the centre with 1 kHz of amplitude modulation, ten
            // times the noise: real-valued baseband, no Q content at all.
            let m = (std::f64::consts::TAU * 1e3 / 2.4e6 * k as f64) as f32;
            s.re += 1.0 + 0.5 * m.cos();
        }
        let mut corr = IqCorrect::new(20.0, 2_400_000.0);
        corr.process(&mut buf);
        let (alpha, gain) = corr.estimate();
        assert!(
            alpha.abs() < 0.01 && (gain - 1.0).abs() < 0.01,
            "pulled to ({alpha:.4}, {gain:.4}) by a signal it cannot measure"
        );
    }

    /// The same front-end defect on a rig's sound card rather than a dongle.
    /// The loop counts *samples*, so a step sized for 2.4 Msps takes fifty
    /// times as long here — most of a minute, which is indistinguishable from
    /// the correction not working. Five seconds of a 48 kHz card has to be
    /// enough, and is what makes the step a function of the rate.
    #[test]
    fn a_sound_card_rate_converges_in_seconds() {
        let rate = 48_000.0;
        let f = 0.25_f64;
        let n = 5 * rate as usize;
        let mut buf: Vec<Complex32> = (0..n)
            .map(|k| {
                let ph = (std::f64::consts::TAU * f * k as f64) as f32;
                Complex32::new(0.2 * ph.cos(), 0.2 * ph.sin())
            })
            .collect();
        spoil(&mut buf, 1.05, 3.0_f32.to_radians(), Complex32::new(0.05, -0.03));

        let window = 1 << 14;
        let (tone_before, image_before) = tone_and_image_db(&buf[..window], f);
        let before = tone_before - image_before;

        let mut corr = IqCorrect::new(20.0, rate);
        corr.process(&mut buf);
        let (tone_after, image_after) = tone_and_image_db(&buf[n - window..], f);
        assert!(
            tone_after - image_after > before + 30.0,
            "image rejection {before:.1} dB -> {:.1} dB after 5 s at 48 kHz",
            tone_after - image_after
        );
    }

    /// DC removal without the imbalance loop: the half an operator can ask for
    /// on its own, because it is the other half that a band which mirrors
    /// itself can mislead.
    #[test]
    fn dc_only_leaves_the_imbalance_alone() {
        let f = 0.25_f64;
        let n = 400_000;
        let mut buf: Vec<Complex32> = (0..n)
            .map(|k| {
                let ph = (std::f64::consts::TAU * f * k as f64) as f32;
                Complex32::new(0.2 * ph.cos(), 0.2 * ph.sin())
            })
            .collect();
        spoil(&mut buf, 1.05, 3.0_f32.to_radians(), Complex32::new(0.05, -0.03));

        let window = 1 << 14;
        let (tone_before, image_before) = tone_and_image_db(&buf[..window], f);

        let mut corr = IqCorrect::dc_only(20.0, 48_000.0);
        corr.process(&mut buf);

        let tail = &buf[n - window..];
        let mean: Complex32 =
            tail.iter().sum::<Complex32>() / Complex32::new(tail.len() as f32, 0.0);
        assert!(mean.norm() < 1e-3, "residual DC {mean:?}");

        let (tone_after, image_after) = tone_and_image_db(tail, f);
        assert!(
            ((tone_after - image_after) - (tone_before - image_before)).abs() < 3.0,
            "the image moved: {:.1} dB -> {:.1} dB",
            tone_before - image_before,
            tone_after - image_after
        );
        assert_eq!(corr.estimate(), (0.0, 1.0), "the loop must not have run");
    }

    #[test]
    fn a_clean_stream_is_left_alone() {
        let mut buf = noise(400_000);
        let reference = buf.clone();
        let mut corr = IqCorrect::new(20.0, 2_400_000.0);
        corr.process(&mut buf);
        let worst = buf[200_000..]
            .iter()
            .zip(&reference[200_000..])
            .map(|(a, b)| (a - b).norm())
            .fold(0.0_f32, f32::max);
        assert!(worst < 5e-3, "clean input perturbed by {worst}");
        let (alpha, gain) = corr.estimate();
        assert!(alpha.abs() < 0.01 && (gain - 1.0).abs() < 0.01, "drifted: {alpha} {gain}");
    }
}
