//! Binaural audio — the receive passband spread across the stereo image, so
//! that signals at different pitches arrive from different directions and
//! tuning one floats it from one ear to the other (issue #263).
//!
//! This is the software form of KK7B's binaural receiver, and of the `BIN`
//! button on a FlexRadio: what it buys is not a prettier noise but a second
//! cue. Two CW signals 200 Hz apart are two tones in one ear, which the ear has
//! to separate by pitch alone; the same pair placed left and right of centre
//! are two *sources*, which is the problem hearing evolved to solve. The noise
//! spreads over the whole image while a signal stays a point in it, so a note
//! that was buried in a diffuse hiss stands out of it.
//!
//! On SSB the same arithmetic buys the second half of that and not the first: a
//! voice fills the passband rather than sitting at a point in it, so two
//! stations do not separate, but the noise still decorrelates across the image
//! while the station stays coherent in the middle of it. Which modes are
//! offered it is the caller's business ([`sdroxide_types::Mode::binaural_audio`]);
//! nothing here knows or cares what is in the passband.
//!
//! **How the placement is made.** A sound reaches the nearer ear first, and it
//! is that interaural delay — plus, higher up, the head's shadow — that the
//! brain reads as a direction. So the pair produced here is the mono audio
//! plus and minus one *side* signal:
//!
//! ```text
//! L = m + s      R = m − s
//! ```
//!
//! `s` is built from the audio's quadrature partner (its Hilbert transform),
//! weighted by how far the frequency sits from the centre of the passband:
//!
//! ```text
//! s(n) = Re[ a(n−N)·e^{ j(Nθ+α)}/2 − a(n+N)·e^{−j(Nθ−α)}/2 ],   a = m + j·m̂
//! ```
//!
//! For a tone at ω that comes to `s = A·sin(N(ω−θ))·sin(ωn+α)`: a quadrature
//! signal whose amplitude follows a sine of the offset from the passband centre
//! θ. Adding and subtracting it turns that amplitude into an interaural *phase*
//! difference — zero at the centre of the passband, ±90° at its edges, with `N`
//! chosen to put a quarter period of the sine there. `α` tilts the side signal
//! partly back into phase with the mono, which adds an interaural *level*
//! difference of the same sign as the delay: the two cues then agree, which is
//! what makes the effect survive on speakers as well as on headphones.
//!
//! Three properties are worth stating, because they are what make this safe to
//! leave switched on:
//!
//! * **`L + R = 2m`, exactly.** The mono downmix is the ordinary audio, sample
//!   for sample — so a remote client, which is sent `(L+R)/2`, hears no comb
//!   filtering and no coloration at all. A pseudo-stereo built the classic way
//!   out of a delay (`L = m + m(t−τ)`, `R = m − m(t−τ)`) cannot say that: its
//!   two ears are complementary comb filters, and everything downstream of the
//!   downmix inherits one of them.
//! * **Neither ear is a filtered copy of the signal.** `s` is in quadrature
//!   with `m`, so the only level difference between the ears is the deliberate
//!   `α`. A tone crossing the image changes direction rather than audibility:
//!   the pair's total power rises by at most 3 dB at the passband edges and
//!   never falls, and the quieter ear stays within about 1 dB of the mono.
//! * **The pan law is bounded.** `sin(N(ω−θ))` never exceeds 1, whatever
//!   arrives, so audio outside the passband — a rig's hiss above a CW filter
//!   whose width sdroxide only *assumes* — cannot be run away with. The worst
//!   case is one ear at √3 (+4.8 dB), which the clamp on the way out catches.
//!
//! Latency is [`Binaural::latency`] samples, a few milliseconds: the Hilbert
//! transformer's group delay plus the half-span of the pan filter. It falls on
//! the speaker path only — the decoders, the recorder and the remote stream all
//! tap the audio upstream of this.

use std::collections::VecDeque;
use std::f64::consts::TAU;

use crate::{Complex32, RealFir};

/// Half-length of the Hilbert transformer, in seconds of audio.
///
/// This is what decides how low a frequency the quadrature partner is still
/// honest at, and so how far down the passband the placement holds: a windowed
/// transformer's response falls away towards DC over roughly its first couple
/// of frequency bins, and the bin spacing is the reciprocal of its length.
///
/// 5.3 ms is about 500 taps at 48 kHz, which at audio rates is nothing.
/// Measured against the 150–2850 Hz voice passband, the ends of the image come
/// out at 4.81 dB / 89.5° at 150 Hz against an ideal 4.77 dB / 90° — the error
/// only becomes visible below the band anything is received in (5.48 dB / 81°
/// at 100 Hz), and it is a gentler placement rather than an artefact.
/// Expressed in time rather than in taps so that it means the same thing on a
/// 44.1 kHz sound card as on a 96 kHz one.
const HILBERT_SECS: f64 = 0.0053;

/// How much of the pan is carried as a level difference rather than as a delay,
/// as the angle the side signal is rotated by.
///
/// At the edge of the passband −30° puts one ear 4.8 dB above the other, with
/// that same ear leading by 90°. Both cues then point the same way, which
/// matters because only one of them survives a loudspeaker: headphones deliver
/// the interaural delay intact, a pair of speakers largely does not, and an
/// operator listening on speakers should still hear the image move rather than
/// hear nothing at all. Negative because that is the sign that makes the louder
/// ear the leading one — see the module comment's expansion of `L` and `R`.
const CUE_TILT_RAD: f64 = -std::f64::consts::FRAC_PI_6;

/// Narrowest passband the pan is stretched across, in Hz. A hand-edited filter
/// of no width would otherwise ask for a pan filter longer than the audio it is
/// spreading.
const MIN_HALF_WIDTH_HZ: f32 = 25.0;

/// The receive passband spread across the stereo image. See the module comment.
pub struct Binaural {
    rate: f64,
    /// The audio's quadrature partner: an odd-symmetric linear-phase FIR whose
    /// response is −j·sgn(ω), i.e. the Hilbert transform.
    hilbert: RealFir,
    /// That transformer's group delay, in samples — half its length.
    group: usize,
    /// Half-span of the pan filter, in samples: a quarter period of the pan
    /// sine, so that the passband edges land at ±90° of interaural phase.
    span: usize,
    /// What the two ends of that span are rotated by, `α` folded in.
    past: Complex32,
    future: Complex32,
    /// The passband `span`, `past` and `future` were built for, so that a block
    /// which changes nothing rebuilds nothing.
    filter: (f32, f32),
    /// The mono audio, delayed by `group + span` so that it lines up with the
    /// side signal built from it.
    delay: VecDeque<f32>,
    /// The last `2·span` samples of the analytic signal, so the pan filter can
    /// reach `span` either side of the sample it is producing.
    window: VecDeque<Complex32>,
    /// This block's quadrature audio.
    quad: Vec<f32>,
}

impl Binaural {
    /// A widener for audio at `rate`, spreading the passband `lo_hz..hi_hz` —
    /// the receiver's filter edges, as [`sdroxide_types::RxState`] holds them.
    pub fn new(rate: f64, lo_hz: f32, hi_hz: f32) -> Self {
        let mut b = Binaural {
            rate: 0.0,
            hilbert: RealFir::new(vec![1.0]),
            group: 0,
            span: 1,
            past: Complex32::default(),
            future: Complex32::default(),
            filter: (f32::NAN, f32::NAN),
            delay: VecDeque::new(),
            window: VecDeque::new(),
            quad: Vec::new(),
        };
        b.set_rate(rate);
        b.set_passband(lo_hz, hi_hz);
        b
    }

    /// Rebuild for a new audio rate. A no-op when nothing has moved, so this
    /// can be asserted per block — the speaker's rate changes under a running
    /// receiver whenever the operator picks another sound card.
    pub fn set_rate(&mut self, rate: f64) {
        if rate < 1_000.0 || (rate - self.rate).abs() < 0.01 {
            return;
        }
        self.rate = rate;
        let half = (rate * HILBERT_SECS).round().max(8.0) as usize;
        self.group = half;
        self.hilbert = RealFir::new(hilbert_taps(2 * half + 1));
        // Primed with the history a full window needs, so that the very first
        // block is answered sample for sample. A stage that swallowed its
        // filter length on the first block would leave the two ears offset by
        // that many samples for the rest of the session — and the offset
        // between the ears is exactly the quantity this is built to control.
        self.quad.clear();
        self.hilbert.process(&vec![0.0; 2 * half], &mut self.quad);
        self.quad.clear();
        // The pan filter's span is in samples, so it moves with the rate too.
        let (lo, hi) = self.filter;
        self.filter = (f32::NAN, f32::NAN);
        self.set_passband(lo, hi);
    }

    /// Point the image at the receiver's passband: its centre goes to the
    /// centre of the head, its edges to the ears.
    ///
    /// The edges are as [`sdroxide_types::RxState`] holds them — Hz relative to
    /// the carrier, with the sign carrying the sideband. What matters here is
    /// the *audio* the demodulator makes of them, which is the same band folded
    /// onto positive frequencies; a passband straddling the carrier (AM, FM)
    /// demodulates to audio from DC up to its wider edge.
    ///
    /// On the lower sideband the audio spectrum is the radio one reversed, and
    /// the image is reversed with it — so a signal to the right on the
    /// panadapter is a signal to the right in the headphones whichever sideband
    /// the receiver is on.
    pub fn set_passband(&mut self, lo_hz: f32, hi_hz: f32) {
        if !lo_hz.is_finite() || !hi_hz.is_finite() || (lo_hz, hi_hz) == self.filter {
            return;
        }
        self.filter = (lo_hz, hi_hz);
        let (a, b) = (lo_hz.abs(), hi_hz.abs());
        // A passband with the carrier inside it demodulates to audio from DC
        // up: its two halves land on top of each other.
        let lo = if lo_hz.signum() == hi_hz.signum() { a.min(b) } else { 0.0 };
        let hi = a.max(b);
        let half = f64::from(((hi - lo) / 2.0).max(MIN_HALF_WIDTH_HZ));
        let centre = f64::from((lo + hi) / 2.0);

        // A quarter period of the pan sine across half the passband, which is
        // what puts ±90° of interaural phase on the two edges.
        let span = (self.rate / (4.0 * half)).round().clamp(1.0, 4096.0) as usize;
        // Swapping the ears is what mirrors the image, and negating the side
        // signal is what swaps the ears — level cue and delay cue together.
        let amp = if hi_hz < 0.0 { -0.5 } else { 0.5 };
        let theta = TAU * centre / self.rate * span as f64;
        let rot = |ang: f64| {
            let (s, c) = (ang + CUE_TILT_RAD).sin_cos();
            Complex32::new((amp * c) as f32, (amp * s) as f32)
        };
        self.past = rot(theta);
        self.future = rot(-theta);

        if span != self.span || self.delay.len() != self.group + span {
            self.span = span;
            // One transient block when the operator drags a filter edge, which
            // is the price of not carrying two spans' worth of state around.
            self.delay.clear();
            self.delay.resize(self.group + span, 0.0);
            self.window.clear();
            self.window.resize(2 * span, Complex32::default());
        }
    }

    /// How far behind the input the pair comes out, in samples.
    pub fn latency(&self) -> usize {
        self.group + self.span
    }

    /// Spread `audio` across the stereo image, appending one sample to each of
    /// `left` and `right` for every sample in.
    pub fn process(&mut self, audio: &[f32], left: &mut Vec<f32>, right: &mut Vec<f32>) {
        self.quad.clear();
        self.hilbert.process(audio, &mut self.quad);
        // Both hold unless a caller built this and never gave it a passband:
        // the transformer is primed, and the delay line is sized with the span.
        if self.quad.len() != audio.len() || self.delay.len() != self.group + self.span {
            debug_assert!(false, "binaural stage was not primed");
            left.extend_from_slice(audio);
            right.extend_from_slice(audio);
            return;
        }
        left.reserve(audio.len());
        right.reserve(audio.len());
        for (i, &x) in audio.iter().enumerate() {
            self.delay.push_back(x);
            // The analytic signal at this step: the audio delayed by the
            // transformer's group delay, paired with the quadrature partner
            // that came out of it.
            let a = Complex32::new(self.delay[self.span], self.quad[i]);
            let old = self.window.pop_front().unwrap_or_default();
            self.window.push_back(a);
            let s = (old * self.past - a * self.future).re;
            let m = self.delay.pop_front().unwrap_or(0.0);
            // Clamped last, as everywhere else on the speaker path: the side
            // signal can put one ear up to 4.8 dB above the mono it came from,
            // and audio already at full scale would clip either way round.
            left.push((m + s).clamp(-1.0, 1.0));
            right.push((m - s).clamp(-1.0, 1.0));
        }
    }
}

/// Taps for a Hilbert transformer of `ntaps` (odd), in the orientation
/// [`RealFir`] applies them.
///
/// The kernel is the classic `2/(πk)` on odd offsets from the centre, windowed
/// — its response is −j·sgn(ω), which turns a cosine into a sine and so hands
/// back the quadrature partner of whatever comes in. Hann rather than the
/// Blackman-Harris the low-pass designs use: this filter's whole job is to be
/// *accurate at the bottom of the audio band*, where a wider main lobe costs
/// directly, and there is no adjacent channel here whose leakage the deeper
/// sidelobes would be bought against.
///
/// Negated because [`RealFir`] correlates rather than convolves — it applies
/// its taps without reversing them, and this kernel is odd-symmetric, so the
/// mirroring a symmetric low-pass never notices is a sign flip here. The same
/// trap as the shift in [`crate::bandpass_taps`], and the reason
/// `the_transformer_turns_a_cosine_into_a_sine` exists.
fn hilbert_taps(ntaps: usize) -> Vec<f32> {
    let mid = (ntaps - 1) / 2;
    let w = crate::window::hann(ntaps);
    (0..ntaps)
        .map(|i| {
            let k = i as isize - mid as isize;
            if k % 2 == 0 { 0.0 } else { (-2.0 / (std::f64::consts::PI * k as f64)) as f32 * w[i] }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;
    /// The CW passband sdroxide opens on: 500 Hz centred on a 700 Hz sidetone.
    const LO: f32 = 450.0;
    const HI: f32 = 950.0;
    /// A whole number of cycles at every frequency measured below (all are
    /// multiples of 5 Hz), so the bin the ears are read out of is exact.
    const WINDOW: usize = 9_600;

    fn tone(n: usize, hz: f64, amp: f32) -> Vec<f32> {
        (0..n).map(|i| amp * (TAU * hz * i as f64 / RATE).cos() as f32).collect()
    }

    /// Amplitude and phase of `x` at `hz`, as one complex number.
    fn bin(x: &[f32], hz: f64) -> num_complex::Complex<f64> {
        let w = TAU * hz / RATE;
        let s: num_complex::Complex<f64> = x
            .iter()
            .enumerate()
            .map(|(n, &v)| f64::from(v) * num_complex::Complex::from_polar(1.0, -w * n as f64))
            .sum();
        s * 2.0 / x.len() as f64
    }

    /// Run a steady tone through a fresh widener and read both ears at it:
    /// `(level difference in dB, interaural phase in degrees, left amplitude)`.
    /// Both are positive when the *left* ear leads or is the louder.
    fn measure(hz: f64, lo: f32, hi: f32) -> (f64, f64, f64) {
        let mut b = Binaural::new(RATE, lo, hi);
        let skip = b.latency() + 64;
        let (mut l, mut r) = (Vec::new(), Vec::new());
        b.process(&tone(skip + WINDOW, hz, 0.25), &mut l, &mut r);
        let (bl, br) = (bin(&l[skip..], hz), bin(&r[skip..], hz));
        let mut ipd = (bl.arg() - br.arg()).to_degrees();
        if ipd > 180.0 {
            ipd -= 360.0;
        } else if ipd < -180.0 {
            ipd += 360.0;
        }
        (20.0 * (bl.norm() / br.norm()).log10(), ipd, bl.norm())
    }

    /// The property everything else rests on: the transformer really does
    /// produce the quadrature partner, in the sign that makes a cosine a sine
    /// rather than its negative. Getting that backwards would swap the ears
    /// and nothing else in this file would notice.
    #[test]
    fn the_transformer_turns_a_cosine_into_a_sine() {
        let mid = (RATE * HILBERT_SECS) as usize;
        let mut fir = RealFir::new(hilbert_taps(2 * mid + 1));
        let mut out = Vec::new();
        fir.process(&tone(8192, 700.0, 1.0), &mut out);
        // Unprimed, the filter drops its whole window, so its output runs the
        // group delay *ahead* of the input index rather than behind it.
        let err = out
            .iter()
            .enumerate()
            .skip(64)
            .map(|(i, &got)| {
                let want = (TAU * 700.0 * (i + mid) as f64 / RATE).sin() as f32;
                (got - want).abs()
            })
            .fold(0.0, f32::max);
        assert!(err < 0.02, "quadrature partner is off by {err}");
    }

    /// A signal at the centre of the passband — the sidetone pitch itself, the
    /// one the operator is tuned to — sits in the middle of the head.
    #[test]
    fn the_centre_of_the_passband_is_centred() {
        let (ild, ipd, _) = measure(700.0, LO, HI);
        assert!(ild.abs() < 0.3, "level difference {ild} dB at the centre");
        assert!(ipd.abs() < 5.0, "phase difference {ipd}° at the centre");
    }

    /// …and one either side of it is off to that side, with the level cue and
    /// the delay cue agreeing about which side that is.
    #[test]
    fn the_passband_edges_reach_the_ears() {
        let (ild, ipd, _) = measure(HI.into(), LO, HI);
        assert!(ild < -3.0, "the high edge should favour the right ear, got {ild} dB");
        assert!(ipd < -60.0, "the right ear should lead at the high edge, got {ipd}°");

        let (ild, ipd, _) = measure(LO.into(), LO, HI);
        assert!(ild > 3.0, "the low edge should favour the left ear, got {ild} dB");
        assert!(ipd > 60.0, "the left ear should lead at the low edge, got {ipd}°");
    }

    /// The image moves across the head as the pitch changes — which is what
    /// tuning a CW signal does, and the whole point of the feature.
    #[test]
    fn the_image_sweeps_with_the_pitch() {
        let mut last = f64::INFINITY;
        for step in 0..=10 {
            let hz = f64::from(LO) + f64::from(HI - LO) * f64::from(step) / 10.0;
            let (_, ipd, _) = measure(hz, LO, HI);
            assert!(ipd < last, "the image jumped back at {hz} Hz: {ipd}° after {last}°");
            last = ipd;
        }
    }

    /// A tone crossing the image changes direction, not audibility.
    #[test]
    fn a_tone_keeps_its_level_across_the_image() {
        let (_, _, centre) = measure(700.0, LO, HI);
        for hz in [500.0, 600.0, 800.0, 900.0] {
            let (ild, _, left) = measure(hz, LO, HI);
            let right = left / 10f64.powf(ild / 20.0);
            for (ear, amp) in [("left", left), ("right", right)] {
                let db = 20.0 * (amp / centre).log10();
                assert!(
                    (-2.0..5.0).contains(&db),
                    "{hz} Hz came out {db} dB from the centre in the {ear} ear"
                );
            }
        }
    }

    /// The mono downmix is the audio, sample for sample. That is what a remote
    /// client is sent, and what keeps this a local effect.
    #[test]
    fn the_two_ears_sum_back_to_the_mono_audio() {
        let mut b = Binaural::new(RATE, LO, HI);
        // Well below full scale, so nothing meets the clamp on the way out.
        let x = tone(4096, 620.0, 0.3);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        b.process(&x, &mut l, &mut r);
        let d = b.latency();
        for i in d..x.len() {
            let sum = 0.5 * (l[i] + r[i]);
            assert!(
                (sum - x[i - d]).abs() < 1e-5,
                "downmix at {i} is {sum}, the audio was {}",
                x[i - d]
            );
        }
    }

    /// One sample out for every sample in, from the very first block: the mixer
    /// pairs the two ears by arrival, so a stage that swallowed its filter
    /// length once would leave them offset for good.
    #[test]
    fn every_block_is_answered_sample_for_sample() {
        let mut b = Binaural::new(RATE, LO, HI);
        for len in [1usize, 7, 480, 1, 4096, 33] {
            let (mut l, mut r) = (Vec::new(), Vec::new());
            b.process(&tone(len, 700.0, 0.2), &mut l, &mut r);
            assert_eq!(l.len(), len);
            assert_eq!(r.len(), len);
        }
    }

    /// On the lower sideband the audio spectrum is the radio one reversed, so
    /// the image is reversed with it and the panadapter still agrees with the
    /// headphones.
    #[test]
    fn the_lower_sideband_mirrors_the_image() {
        let (usb_ild, usb_ipd, _) = measure(HI.into(), LO, HI);
        let (lsb_ild, lsb_ipd, _) = measure(HI.into(), -HI, -LO);
        assert!(
            (usb_ild + lsb_ild).abs() < 0.3,
            "levels {usb_ild} and {lsb_ild} dB are not mirrored"
        );
        assert!(
            (usb_ipd + lsb_ipd).abs() < 5.0,
            "phases {usb_ipd}° and {lsb_ipd}° are not mirrored"
        );
    }

    /// A voice passband is spread the same way, and the pan reaches its ends:
    /// the bottom of the audio to one ear, the top to the other, the middle in
    /// the middle. Nothing here knows it is looking at speech rather than a
    /// note — which is exactly why SSB needed no new arithmetic.
    #[test]
    fn a_voice_passband_is_spread_end_to_end() {
        // The 2.7 kHz SSB filter sdroxide opens on.
        let (lo, hi) = (150.0f32, 2850.0f32);
        let (ild, ipd, _) = measure(1500.0, lo, hi);
        assert!(ild.abs() < 0.5, "level difference {ild} dB in the middle of the voice");
        assert!(ipd.abs() < 8.0, "phase difference {ipd}° in the middle of the voice");

        let (low_ild, low_ipd, _) = measure(300.0, lo, hi);
        let (high_ild, high_ipd, _) = measure(2700.0, lo, hi);
        assert!(low_ild > 2.0, "the bottom of the voice should lean left, got {low_ild} dB");
        assert!(high_ild < -2.0, "the top of the voice should lean right, got {high_ild} dB");
        assert!(low_ipd > 30.0, "the left ear should lead at the bottom, got {low_ipd}°");
        assert!(high_ipd < -30.0, "the right ear should lead at the top, got {high_ipd}°");
    }

    /// A passband with the carrier inside it — AM, FM — demodulates to audio
    /// from DC up, and the image is stretched across that instead.
    #[test]
    fn a_carrier_centred_passband_spreads_from_dc() {
        // −5 to +5 kHz is 0..5 kHz of audio, so 2.5 kHz is the middle of it.
        let (ild, ipd, _) = measure(2500.0, -5000.0, 5000.0);
        assert!(ild.abs() < 0.5, "level difference {ild} dB at the centre");
        assert!(ipd.abs() < 8.0, "phase difference {ipd}° at the centre");
        let (ild, _, _) = measure(4200.0, -5000.0, 5000.0);
        assert!(ild < -2.0, "the top of the audio should lean right, got {ild} dB");
    }

    /// Nothing outside the passband can be run away with: the pan law is a
    /// sine, not a ramp, so a rig's hiss above a CW filter whose width we only
    /// *assume* stays bounded instead of being multiplied by the tilt.
    #[test]
    fn out_of_band_audio_is_not_amplified() {
        let (_, _, centre) = measure(700.0, LO, HI);
        for hz in [1500.0, 2400.0, 3600.0] {
            let (_, _, amp) = measure(hz, LO, HI);
            let db = 20.0 * (amp / centre).log10();
            assert!(db < 5.5, "{hz} Hz came out {db} dB above the centre's level");
        }
    }

    /// A hand-edited filter of no width must not ask for a pan filter longer
    /// than the audio it is spreading.
    #[test]
    fn a_zero_width_filter_is_survivable() {
        let mut b = Binaural::new(RATE, 700.0, 700.0);
        assert!(b.latency() < RATE as usize);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        b.process(&tone(4096, 700.0, 0.3), &mut l, &mut r);
        assert_eq!(l.len(), 4096);
        assert!(l.iter().all(|s| s.is_finite()));
        assert!(r.iter().all(|s| s.is_finite()));
    }
}
