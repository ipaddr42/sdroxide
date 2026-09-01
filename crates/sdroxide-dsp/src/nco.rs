use std::f64::consts::TAU;

use crate::Complex32;

/// How many samples one advance of the f64 phasor covers.
///
/// The mixer's cost used to be its recurrence: `phasor *= step` in f64 is a
/// four-multiply complex product whose result the *next* sample depends on, so
/// a machine that can start a multiply every cycle still spends the full
/// latency of one on every sample, and nothing about the loop can be widened.
/// On an RX-888 at 32.4 Msps — two of these run there, one for the receive
/// chain and one for the panadapter's zoom lane — that was measured as the
/// single largest thing in the DSP thread, 22 % of it.
///
/// So the phasor advances once per block of this many samples, by `step^LANES`,
/// and the samples inside the block are reached by multiplying it with the
/// precomputed `step^0 … step^{LANES-1}`. Those products are independent of
/// each other, which is what lets the loop pipeline. Eight because that is
/// where the gain flattens: measured 3.5× at 32.4 Msps, and the tail handling
/// costs more as the block grows.
const LANES: usize = 8;

/// Numerically controlled oscillator: multiplies a signal by e^{j2πft}.
/// Complex-recurrence in f64 with periodic renormalization — phase-continuous
/// across frequency changes, sub-0.01 Hz accuracy at Msps rates.
pub struct Nco {
    phasor: num_complex::Complex<f64>,
    step: num_complex::Complex<f64>,
    /// `step^0 … step^{LANES-1}` in f32 — where in the block each sample sits.
    lanes: [Complex32; LANES],
    /// `step^LANES`: what advances [`Self::phasor`] from one block to the next.
    block_step: num_complex::Complex<f64>,
    renorm: u32,
}

/// Samples between pulling the phasor back onto the unit circle. Counted in
/// samples rather than in multiplies so the interval is the same however the
/// block above is sized — the drift per multiply has not changed, there are
/// simply fewer of them.
const RENORM_SAMPLES: u32 = 1 << 16;

crate::simd::kernel! {
    /// The whole-block half of [`Nco::mix`].
    ///
    /// `dst` and `src` are both a whole number of [`LANES`]-sample blocks. The
    /// f64 phasor is carried across blocks — that recurrence is what keeps the
    /// mixer's phase exact over hours — while the samples inside a block are
    /// reached from the precomputed lane table and so are independent of one
    /// another, which is what lets this widen.
    fn mix_run / mix_run_portable / mix_run_avx2 / mix_run_avx512 (
        dst: &mut [Complex32],
        src: &[Complex32],
        phasor: &mut num_complex::Complex<f64>,
        block_step: num_complex::Complex<f64>,
        lanes: &[Complex32; LANES],
    ) {
        for (ys, xs) in dst.chunks_exact_mut(LANES).zip(src.chunks_exact(LANES)) {
            let base = Complex32::new(phasor.re as f32, phasor.im as f32);
            for ((y, &x), &lane) in ys.iter_mut().zip(xs).zip(lanes) {
                *y = x * (base * lane);
            }
            *phasor *= block_step;
        }
    }
}

crate::simd::kernel! {
    /// [`mix_run`] for a caller that already owns the buffer being shifted.
    fn mix_in_place_run / mix_in_place_run_portable / mix_in_place_run_avx2 / mix_in_place_run_avx512 (
        samples: &mut [Complex32],
        phasor: &mut num_complex::Complex<f64>,
        block_step: num_complex::Complex<f64>,
        lanes: &[Complex32; LANES],
    ) {
        for xs in samples.chunks_exact_mut(LANES) {
            let base = Complex32::new(phasor.re as f32, phasor.im as f32);
            for (x, &lane) in xs.iter_mut().zip(lanes) {
                *x *= base * lane;
            }
            *phasor *= block_step;
        }
    }
}

impl Nco {
    /// `freq_hz` may be negative. Positive shifts the signal up in frequency.
    pub fn new(freq_hz: f64, sample_rate: f64) -> Self {
        let mut nco = Nco {
            phasor: num_complex::Complex::new(1.0, 0.0),
            step: num_complex::Complex::new(1.0, 0.0),
            lanes: [Complex32::new(1.0, 0.0); LANES],
            block_step: num_complex::Complex::new(1.0, 0.0),
            renorm: 0,
        };
        nco.set_freq(freq_hz, sample_rate);
        nco
    }

    /// Phase-continuous retune.
    pub fn set_freq(&mut self, freq_hz: f64, sample_rate: f64) {
        let w = TAU * freq_hz / sample_rate;
        self.step = num_complex::Complex::new(w.cos(), w.sin());
        // The lane table and the block step are the same recurrence walked
        // once here instead of once per sample. `phasor` is untouched, which
        // is what keeps the retune phase-continuous.
        let mut p = num_complex::Complex::new(1.0f64, 0.0);
        for lane in self.lanes.iter_mut() {
            *lane = Complex32::new(p.re as f32, p.im as f32);
            p *= self.step;
        }
        self.block_step = p;
    }

    /// samples[i] *= phasor, in place.
    ///
    /// Same recurrence as [`Nco::mix`], for callers that already own the buffer
    /// they want shifted and would otherwise copy it just to mix it.
    pub fn mix_in_place(&mut self, samples: &mut [Complex32]) {
        let whole = samples.len() - samples.len() % LANES;
        mix_in_place_run(&mut samples[..whole], &mut self.phasor, self.block_step, &self.lanes);
        self.renorm += whole as u32;
        self.maybe_renorm();
        for x in &mut samples[whole..] {
            let p = Complex32::new(self.phasor.re as f32, self.phasor.im as f32);
            *x *= p;
            self.advance_sample();
        }
    }

    /// out[i] = input[i] * phasor (appends to `out`).
    pub fn mix(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        let start = out.len();
        out.resize(start + input.len(), Complex32::new(0.0, 0.0));
        let dst = &mut out[start..];

        let whole = input.len() - input.len() % LANES;
        mix_run(&mut dst[..whole], &input[..whole], &mut self.phasor, self.block_step, &self.lanes);
        self.renorm += whole as u32;
        self.maybe_renorm();
        for (y, &x) in dst[whole..].iter_mut().zip(&input[whole..]) {
            let p = Complex32::new(self.phasor.re as f32, self.phasor.im as f32);
            *y = x * p;
            self.advance_sample();
        }
    }

    /// Move it on by a single sample — the tail of a block that did not divide
    /// evenly, and the only path a caller passing very short buffers takes.
    #[inline]
    fn advance_sample(&mut self) {
        self.phasor *= self.step;
        self.renorm += 1;
        self.maybe_renorm();
    }

    #[inline]
    fn maybe_renorm(&mut self) {
        if self.renorm >= RENORM_SAMPLES {
            self.renorm = 0;
            self.phasor /= self.phasor.norm();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The block-at-a-time recurrence has to be the per-sample one it replaced,
    /// to the precision the output is carried at.
    ///
    /// Both the frequency (does sample *n* have the phase of sample *n*?) and
    /// the continuation across calls (does a block boundary land mid-buffer
    /// without a phase step?) are in this: the reference is walked one sample
    /// at a time over the whole run, and the buffer lengths deliberately do not
    /// divide by the lane count.
    #[test]
    fn a_block_of_lanes_is_the_same_phase_as_stepping_one_at_a_time() {
        let fs = 32_400_000.0;
        let mut nco = Nco::new(-1_234_567.0, fs);

        // The reference: the recurrence exactly as it was written before.
        let w = TAU * -1_234_567.0 / fs;
        let step = num_complex::Complex::new(w.cos(), w.sin());
        let mut phasor = num_complex::Complex::new(1.0f64, 0.0);
        let mut n = 0u32;

        let input: Vec<Complex32> = (0..70_003)
            .map(|i| Complex32::new((i % 17) as f32 * 0.05, (i % 23) as f32 * 0.03))
            .collect();

        let mut got = Vec::new();
        // Lengths that are not multiples of LANES, so every call ends on a
        // tail and the next one has to pick the phase up mid-block.
        for chunk in input.chunks(4_093) {
            nco.mix(chunk, &mut got);
        }

        for (i, x) in input.iter().enumerate() {
            let p = Complex32::new(phasor.re as f32, phasor.im as f32);
            let want = x * p;
            assert!((got[i] - want).norm() < 2e-6, "sample {i}: {:?} vs {:?}", got[i], want);
            phasor *= step;
            n += 1;
            if n >= RENORM_SAMPLES {
                n = 0;
                phasor /= phasor.norm();
            }
        }
    }

    /// A tone comes out where it was asked for, over a run long past the
    /// renormalisation interval — the block step is a power of the sample step
    /// and a mis-derived one would show as a frequency error, not a wobble.
    #[test]
    fn the_shift_is_the_frequency_it_was_given() {
        let fs = 2_000_000.0;
        let shift = 137_000.0;
        let mut nco = Nco::new(shift, fs);
        let mut buf: Vec<Complex32> = (0..300_000).map(|_| Complex32::new(1.0, 0.0)).collect();
        nco.mix_in_place(&mut buf);

        // Phase advance per sample, averaged over the tail of the run.
        let mut adv = 0.0f64;
        let tail = &buf[200_000..];
        for w in tail.windows(2) {
            adv += (w[1] * w[0].conj()).arg() as f64;
        }
        let hz = adv / (tail.len() - 1) as f64 / TAU * fs;
        assert!((hz - shift).abs() < 1.0, "read {hz:.2} Hz, asked {shift}");

        // And the amplitude has not wandered off the unit circle.
        let mag = tail.iter().map(|c| c.norm()).fold(0.0f32, f32::max);
        assert!((mag - 1.0).abs() < 1e-4, "amplitude drifted to {mag}");
    }

    /// `mix` and `mix_in_place` are the same mixer.
    #[test]
    fn the_two_entry_points_agree() {
        let fs = 1_536_000.0;
        let input: Vec<Complex32> = (0..10_005)
            .map(|i| Complex32::new((i % 13) as f32 * 0.07, (i % 29) as f32 * 0.02))
            .collect();

        let mut a = Nco::new(48_000.0, fs);
        let mut out = Vec::new();
        a.mix(&input, &mut out);

        let mut b = Nco::new(48_000.0, fs);
        let mut in_place = input.clone();
        b.mix_in_place(&mut in_place);

        for (x, y) in out.iter().zip(&in_place) {
            assert!((x - y).norm() < 1e-6);
        }
    }
}
