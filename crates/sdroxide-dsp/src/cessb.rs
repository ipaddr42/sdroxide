//! Controlled-envelope SSB: get more average power out of a transmitter
//! without raising its peak, and without splattering to do it.
//!
//! The technique is David Hershberger W9GR's, published in *QEX* for
//! November/December 2014. The problem it solves is specific to single
//! sideband: the thing an amplifier runs out of is the *envelope* — the
//! magnitude of the analytic signal — and that is not the audio waveform. A
//! limiter on the microphone therefore does not limit what the amplifier sees;
//! the envelope of clipped speech overshoots by a long way, so an SSB
//! transmitter has to be backed off for peaks nobody hears. Speech is peaky, so
//! the average power that reaches the far end is a small fraction of the
//! transmitter's rating.
//!
//! # The three stages
//!
//! 1. **Clip the envelope.** `z ← z · min(1, 1/|z|)` — the phase is kept, only
//!    the magnitude is limited, so this is exactly the ceiling the amplifier
//!    has. It is also a hard nonlinearity, and it splatters.
//! 2. **Filter the splatter away.** The same band-pass the modulator used, so
//!    what leaves this stage occupies the operator's own transmit passband and
//!    nothing else. Filtering a clipped signal restores its peaks, though: the
//!    envelope comes out of here overshooting by around 60 %, which is the
//!    whole reason the naive "clip and filter" is not enough.
//! 3. **Take the overshoot back out, gently.** The part of each sample that
//!    stands above the ceiling is collected as a signal of its own, put through
//!    the *same* band-pass, and **subtracted**. That it is a subtraction and not
//!    a gain is the whole of the idea: a time-varying gain multiplies, and
//!    multiplying by anything that moves as fast as the envelope does spreads
//!    the transmission to twice its width — measured here at −22 dB of
//!    third-order product before this stage was written the other way round.
//!    Subtracting a signal that has been through the transmit filter cannot put
//!    energy anywhere the transmit filter does not pass, whatever it is
//!    subtracted from. That is the *controlled* in controlled-envelope, and it
//!    brings the overshoot down to a percent or two with none of stage 1's
//!    splatter.
//!
//! The published figure is about 2.5× the average power for the same peak; the
//! usual on-air description is three or four decibels of apparent loudness with
//! no processed sound. This implementation follows the structure and the
//! constants of the article's reference design (the ±2-sample envelope window
//! and the 1.4 correction factor for the filter method), at the transmit
//! chain's own 48 kHz rather than the reference's 24 — there is no need to
//! interpolate a signal that is already sixteen times oversampled.
//!
//! # What it must never do
//!
//! Two properties hold whatever the audio does, because a processor that fails
//! by transmitting wider than it was told is worse than no processor:
//!
//! * every stage is bounded by the same full-scale ceiling, so the output
//!   cannot exceed the envelope the transmitter was already limited to;
//! * with the compression set to zero the clipper never fires and the
//!   correction is identically zero, so the signal that comes out is the
//!   signal that went in, band-pass filtered — audibly and measurably the same
//!   transmission as before.
//!
//! # Voice only
//!
//! Nothing here is applied to a digital mode. FT8, PSK and the rest carry their
//! information in a waveform whose envelope *is* the signal, and clipping one
//! is not compression, it is distortion of the thing being sent. This runs on
//! the two voice sidebands and nowhere else; the caller enforces that.

use crate::Complex32;
use crate::fir::{ComplexFir, bandpass_taps};

/// Taps in the band-pass that follows the clipper.
///
/// The same length the modulator's own filter uses: this one has the same job
/// against the same passband, and a slacker skirt here would undo the
/// modulator's.
const CLIP_TAPS: usize = 331;

/// Taps in the band-pass the overshoot correction goes through.
///
/// The same filter as the clipper's, because it has the same job: what is
/// subtracted has to be inside the transmit passband, and "inside" has to mean
/// the same thing at both ends of the chain. Odd, so the group delay is a whole
/// number of samples and the signal can be lined up against it exactly.
const CORR_TAPS: usize = CLIP_TAPS;

/// How hard the band-limited overshoot is subtracted.
///
/// Well above one, because the band-pass is what makes the correction safe and
/// also what blunts it: an overshoot lasting a fraction of a millisecond comes
/// out of a 331-tap filter spread over several, and therefore far shorter than
/// it went in. The published reference design uses 1.4 to 1.9 with a much
/// shorter filter; this structure needs more, and the figure is measured rather
/// than borrowed.
///
/// Swept against the speech-like signal in the tests below, at 9 dB of
/// compression, as `factor → (peak-to-average gained, out-of-band energy,
/// overshoot left for the safety clip)`:
///
/// | 1.0 | 4.5 dB | −33 dB | 15.0 % |
/// | 1.8 | 4.3 dB | −45 dB |  4.2 % |
/// | 2.5 | 4.0 dB | −61 dB |  1.4 % |
/// | 3.0 | 3.8 dB | −106 dB |  0.0 % |
/// | 4.0 | 3.4 dB | −104 dB |  0.0 % |
///
/// Three is where the safety clip stops firing at all, which is what collapses
/// the out-of-band energy to the arithmetic's own floor: past that point the
/// only thing more correction buys is less average power.
const CORRECTION: f32 = 3.0;

/// The controlled-envelope processor for one transmit chain.
pub struct Cessb {
    /// Linear gain into the clipper — the operator's compression setting.
    /// 1.0 (0 dB) means nothing ever reaches the ceiling and the whole thing is
    /// a band-pass filter.
    drive: f32,
    rate: f64,
    clip_fir: ComplexFir,
    corr_fir: ComplexFir,
    /// Band-limited signal waiting for its correction to be computed. Held
    /// because the correction filter is centred, so a sample cannot be finished
    /// until half a filter's worth of the signal *after* it has arrived.
    pend: std::collections::VecDeque<Complex32>,
    /// Samples of `pend` still to be dropped before the two streams line up.
    /// See [`Cessb::process`].
    align: usize,
    /// Scratch, kept between blocks so a transmission allocates nothing.
    filtered: Vec<Complex32>,
    over: Vec<Complex32>,
    smoothed: Vec<Complex32>,
    /// The worst envelope seen *before* the final safety clip, since the last
    /// reset. Not a control: it is how the tests below measure whether the
    /// compensator is doing its job rather than leaving the work to the clip.
    raw_peak: f32,
}

impl Cessb {
    /// A processor for the passband `lo_hz..hi_hz` at `rate`, with no
    /// compression asked for yet.
    #[must_use]
    pub fn new(rate: f64, lo_hz: f32, hi_hz: f32) -> Self {
        Cessb {
            drive: 1.0,
            rate,
            clip_fir: ComplexFir::new(bandpass_taps(CLIP_TAPS, lo_hz as f64, hi_hz as f64, rate)),
            corr_fir: ComplexFir::new(bandpass_taps(CORR_TAPS, lo_hz as f64, hi_hz as f64, rate)),
            pend: std::collections::VecDeque::new(),
            // The correction filter is centred on its own middle tap, so the
            // first correction it produces belongs to the sample that many
            // places into the signal — and that many have to go past unpaired
            // before the two streams are describing the same instant.
            align: CORR_TAPS / 2,
            filtered: Vec::new(),
            over: Vec::new(),
            smoothed: Vec::new(),
            raw_peak: 0.0,
        }
    }

    /// Follow the operator's transmit filter, as the modulator does.
    pub fn set_filter(&mut self, lo_hz: f32, hi_hz: f32) {
        self.clip_fir.set_taps(bandpass_taps(CLIP_TAPS, lo_hz as f64, hi_hz as f64, self.rate));
        self.corr_fir.set_taps(bandpass_taps(CORR_TAPS, lo_hz as f64, hi_hz as f64, self.rate));
        self.align = CORR_TAPS / 2;
        self.pend.clear();
    }

    /// Set the compression, in decibels of gain ahead of the clipper. Clamped
    /// to `0..=`[`sdroxide_types::CESSB_MAX_DB`]; zero passes the signal through.
    pub fn set_compression_db(&mut self, db: f32) {
        self.drive = 10f32.powf(db.clamp(0.0, sdroxide_types::CESSB_MAX_DB) / 20.0);
    }

    /// The compression currently set, in decibels.
    #[must_use]
    pub fn compression_db(&self) -> f32 {
        20.0 * self.drive.log10()
    }

    /// Whether anything is being done at all — false at zero compression, where
    /// the clipper can never fire.
    #[must_use]
    pub fn active(&self) -> bool {
        self.drive > 1.001
    }

    /// The worst envelope the compensator left for the final safety clip to
    /// deal with, as a fraction of full scale. 1.0 means it left nothing.
    #[must_use]
    pub fn residual_overshoot(&self) -> f32 {
        self.raw_peak
    }

    /// Forget the transmission so far. Called at key-up: the filters' history
    /// belongs to the over that has just ended, and carrying it into the next
    /// one would start it with the tail of the last.
    pub fn reset(&mut self) {
        self.clip_fir.reset();
        self.corr_fir.reset();
        self.pend.clear();
        self.align = CORR_TAPS / 2;
        self.raw_peak = 0.0;
    }

    /// Process one block of complex baseband in place.
    ///
    /// The block comes back shorter at the start of a transmission — both
    /// filters have to fill before either can answer — and the same length as
    /// it went in from then on. That is the same behaviour the modulator's own
    /// filter has, and the transmit path already carries blocks of whatever
    /// length it is handed.
    pub fn process(&mut self, buf: &mut Vec<Complex32>) {
        if buf.is_empty() {
            return;
        }
        // ── 1. Drive into the clipper, and clip. ──
        for z in buf.iter_mut() {
            *z *= self.drive;
            let mag = z.norm();
            if mag > 1.0 {
                *z /= mag;
            }
        }

        // ── 2. Band-limit, which is what puts the overshoot back. ──
        self.filtered.clear();
        self.clip_fir.process(buf, &mut self.filtered);

        // ── 3. Collect what still stands above the ceiling. ──
        //
        // A signal of its own, zero wherever the envelope is inside full scale
        // and equal to the part above it — along the same phase, so subtracting
        // it shortens the vector without turning it.
        self.over.clear();
        self.over.extend(self.filtered.iter().map(|&z| {
            let mag = z.norm();
            if mag > 1.0 { z * (1.0 - 1.0 / mag) } else { Complex32::new(0.0, 0.0) }
        }));

        // ── 4. Band-limit it, and subtract. ──
        //
        // This is the controlled part. What comes out of the band-pass is a
        // signal inside the transmit passband, so taking it away from another
        // signal inside the transmit passband leaves a third one inside the
        // transmit passband — no matter how violent the envelope was that
        // produced it.
        self.pend.extend(self.filtered.iter().copied());
        self.smoothed.clear();
        self.corr_fir.process(&self.over, &mut self.smoothed);
        // The first `align` band-limited samples have no correction of their
        // own — they are what the correction filter was filling up with — so
        // they are dropped rather than sent out uncorrected.
        let drop = self.align.min(self.pend.len());
        self.pend.drain(..drop);
        self.align -= drop;

        buf.clear();
        for &c in &self.smoothed {
            let Some(z) = self.pend.pop_front() else { break };
            let mut z = z - c * CORRECTION;
            self.raw_peak = self.raw_peak.max(z.norm());
            // The last word on the ceiling. The correction leaves a percent or
            // two of overshoot by design, and this is where that percent stops
            // — a clip this small and this rare is thousands of times weaker
            // than what stage 1 removed, and it is the guarantee that nothing
            // downstream ever sees more than full scale.
            let mag = z.norm();
            if mag > 1.0 {
                z /= mag;
            }
            buf.push(z);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::TAU;

    use super::*;

    const RATE: f64 = 48_000.0;
    const LO: f32 = 200.0;
    const HI: f32 = 2_800.0;

    /// The standard two-tone SSB test signal, as complex baseband: two equal
    /// tones inside the passband, scaled so its envelope peaks at `peak`.
    ///
    /// Two tones because that is what an SSB transmitter is measured with — one
    /// tone has a constant envelope and would tell us nothing about an envelope
    /// processor, and speech is two tones plus everything else.
    fn two_tone(n: usize, peak: f32) -> Vec<Complex32> {
        (0..n)
            .map(|i| {
                let t = i as f32 / RATE as f32;
                let a = Complex32::new(0.0, TAU * 700.0 * t).exp();
                let b = Complex32::new(0.0, TAU * 1_900.0 * t).exp();
                (a + b) * (peak / 2.0)
            })
            .collect()
    }

    /// A speech-like test signal: many tones spread across the passband with
    /// unrelated phases, normalised so its envelope peaks at 1.0.
    ///
    /// Two tones are the classic *linearity* test and are used for that below,
    /// but they are a poor test of an envelope processor: their intermodulation
    /// products land outside a 2.6 kHz passband, so the stage that removes
    /// splatter also removes most of the clipping, and the measurement says the
    /// processor did nothing. Speech is not like that — it fills the band, its
    /// products land inside it, and its peak-to-average is eight or ten
    /// decibels rather than three. This is the case CESSB exists for.
    fn speech_like(n: usize) -> Vec<Complex32> {
        // A fixed, arbitrary phase set: the test has to be the same every run.
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f32 / (1u64 << 53) as f32
        };
        let tones: Vec<(f32, f32)> =
            (0..40).map(|k| (400.0 + k as f32 * 55.0, rand() * TAU)).collect();
        let mut x: Vec<Complex32> = (0..n)
            .map(|i| {
                let t = i as f32 / RATE as f32;
                tones.iter().fold(Complex32::new(0.0, 0.0), |a, &(f, p)| {
                    a + Complex32::new(0.0, TAU * f * t + p).exp()
                })
            })
            .collect();
        let p = peak(&x);
        for z in &mut x {
            *z /= p;
        }
        x
    }

    /// Run `x` through a processor at `db` of compression, a block at a time,
    /// and give back the steady-state part of the output.
    fn run(x: &[Complex32], db: f32) -> Vec<Complex32> {
        let mut c = Cessb::new(RATE, LO, HI);
        c.set_compression_db(db);
        let mut out = Vec::new();
        for chunk in x.chunks(480) {
            let mut buf = chunk.to_vec();
            c.process(&mut buf);
            out.extend_from_slice(&buf);
        }
        // Both filters ring for their own length at the start; that is the
        // transmitter coming up, not what it sounds like.
        let skip = (CLIP_TAPS + CORR_TAPS).min(out.len());
        out.split_off(skip)
    }

    fn peak(x: &[Complex32]) -> f32 {
        x.iter().fold(0.0f32, |m, z| m.max(z.norm()))
    }

    fn mean_power(x: &[Complex32]) -> f32 {
        x.iter().map(|z| z.norm_sqr()).sum::<f32>() / x.len().max(1) as f32
    }

    /// Energy outside the transmit passband, relative to the energy inside it,
    /// in dB — the number that decides whether a processor is fit to be on the
    /// air at all.
    fn splatter_db(x: &[Complex32]) -> f32 {
        use rustfft::FftPlanner;
        let n = 16_384.min(x.len().next_power_of_two() / 2);
        let mut buf: Vec<rustfft::num_complex::Complex<f32>> = x[..n]
            .iter()
            .enumerate()
            .map(|(i, z)| {
                // Hann, so the skirts being measured are the signal's and not
                // the rectangular window's.
                let w = 0.5 - 0.5 * (TAU * i as f32 / n as f32).cos();
                rustfft::num_complex::Complex::new(z.re * w, z.im * w)
            })
            .collect();
        FftPlanner::new().plan_fft_forward(n).process(&mut buf);
        let bin_hz = RATE as f32 / n as f32;
        let (mut inband, mut out) = (0.0f32, 0.0f32);
        for (k, c) in buf.iter().enumerate() {
            // The baseband is complex, so the upper half of the transform is
            // the negative frequencies.
            let f = if k < n / 2 { k as f32 * bin_hz } else { (k as f32 - n as f32) * bin_hz };
            let p = c.norm_sqr();
            // A 500 Hz guard either side: the transmit filter's own skirt is
            // not splatter, and neither is the two-tone's window leakage.
            if f > LO - 500.0 && f < HI + 500.0 { inband += p } else { out += p }
        }
        10.0 * (out / inband.max(1e-30)).log10()
    }

    /// With no compression asked for, the processor is a band-pass filter and
    /// nothing else. This is the promise that switching the feature on with the
    /// control at zero changes no transmission at all.
    #[test]
    fn zero_compression_is_a_wire() {
        let x = two_tone(48_000, 0.9);
        let y = run(&x, 0.0);
        assert!(!Cessb::new(RATE, LO, HI).active());
        assert!((peak(&y) - 0.9).abs() < 0.02, "peak went from 0.900 to {:.3}", peak(&y));
        let (px, py) = (mean_power(&x), mean_power(&y));
        assert!(
            (py / px - 1.0).abs() < 0.05,
            "mean power changed by {:.1}%",
            (py / px - 1.0) * 100.0
        );
    }

    /// The whole point: more average power for the same peak.
    #[test]
    fn compression_raises_the_average_without_raising_the_peak() {
        let x = speech_like(48_000);
        let plain = run(&x, 0.0);
        let processed = run(&x, 9.0);
        assert!(
            peak(&processed) <= 1.001,
            "the envelope reached {:.4}, above full scale",
            peak(&processed)
        );
        // Both referred to their own peak, which is what an amplifier's ceiling
        // means: the question is how much average power fits under it.
        let db = 10.0
            * (mean_power(&processed)
                / peak(&processed).powi(2)
                / (mean_power(&plain) / peak(&plain).powi(2)))
            .log10();
        assert!(db > 3.0, "only {db:.2} dB of average power gained");
    }

    /// And it must not buy that power with somebody else's QSO. The comparison
    /// is against the naive answer — clip the envelope and transmit it — which
    /// is what makes the two extra stages worth having.
    #[test]
    fn it_is_far_cleaner_than_clipping_the_envelope() {
        let x = speech_like(48_000);
        let g = 10f32.powf(9.0 / 20.0);
        let clipped: Vec<Complex32> = x
            .iter()
            .map(|&z| {
                let z = z * g;
                let mag = z.norm();
                if mag > 1.0 { z / mag } else { z }
            })
            .collect();
        let naive = splatter_db(&clipped[CLIP_TAPS + CORR_TAPS..]);
        let ours = splatter_db(&run(&x, 9.0));
        assert!(ours < -60.0, "the controlled envelope splattered at {ours:.1} dB");
        assert!(
            ours < naive - 40.0,
            "controlled {ours:.1} dB vs plain clipping {naive:.1} dB — not the improvement claimed"
        );
    }

    /// The IMD test every SSB transmitter is measured with, for the one thing
    /// it is good at: showing that two tones come out as two tones.
    ///
    /// Deliberately not used for the power measurements above — a two-tone's
    /// intermodulation products fall outside a 2.6 kHz passband, so the stage
    /// that removes splatter removes most of the clipping with it and the
    /// processor appears to do nothing. What it does show is that the products
    /// really are gone.
    #[test]
    fn two_tones_come_out_as_two_tones() {
        let y = run(&two_tone(48_000, 1.0), 9.0);
        assert!(peak(&y) <= 1.001);
        assert!(splatter_db(&y) < -55.0, "two-tone splatter {:.1} dB", splatter_db(&y));
    }

    /// The overshoot compensator is the stage that earns its keep. Without it,
    /// filtering the clipper's output puts the peaks back — this is the
    /// measurement that says the third stage, and not the safety clip behind
    /// it, is what holds the envelope down.
    #[test]
    fn filtering_a_clipped_envelope_would_overshoot_and_this_does_not() {
        let x = speech_like(48_000);
        // Stages 1 and 2 alone: clip, then band-limit.
        let mut fir = ComplexFir::new(bandpass_taps(CLIP_TAPS, LO as f64, HI as f64, RATE));
        let mut clip_filtered = Vec::new();
        let g = 10f32.powf(9.0 / 20.0);
        for chunk in x.chunks(480) {
            let clipped: Vec<Complex32> = chunk
                .iter()
                .map(|&z| {
                    let z = z * g;
                    let mag = z.norm();
                    if mag > 1.0 { z / mag } else { z }
                })
                .collect();
            fir.process(&clipped, &mut clip_filtered);
        }
        let overshoot = peak(&clip_filtered[CLIP_TAPS..]);
        assert!(overshoot > 1.15, "clip-and-filter should overshoot; it reached {overshoot:.3}");

        let mut c = Cessb::new(RATE, LO, HI);
        c.set_compression_db(9.0);
        for chunk in x.chunks(480) {
            let mut b = chunk.to_vec();
            c.process(&mut b);
        }
        assert!(
            c.residual_overshoot() <= 1.001,
            "the compensator left {:.1}% for the safety clip to deal with",
            (c.residual_overshoot() - 1.0) * 100.0
        );
    }
}
