//! GMSK reception: the receive filter, the frequency discriminator, the bit
//! timing and the slicer.
//!
//! AIS transmits 9600 bit/s GMSK with a modulation index of 0.5 and a Gaussian
//! pre-filter of `BT = 0.4`. A modulation index of one half means the carrier
//! moves ±2400 Hz, so what comes out of a frequency discriminator is a bipolar
//! baseband waveform — a smoothed NRZ signal — and everything below treats it
//! as one.
//!
//! ```text
//!  channel I/Q ─▶ Rx filter ─▶ Gate ─▶ arg(z·conj(z₋₁)) ─▶ low-pass ─▶ sample ─▶ level
//!                  ±9 kHz              frequency            0.7·Rb     timing
//! ```
//!
//! The receive filter is [`RxFilter`] and runs *in front of the gate*, not
//! behind it. That is not a detail: the two AIS channels are 50 kHz apart and
//! the down-converter that separates them may legitimately decimate by one, in
//! which case it contains no filter at all. Gating an unfiltered stream would
//! open channel A's gate on channel B's transmissions, weld two ships' slots
//! into one, and measure the noise floor over both channels at once. Filtering
//! first makes every one of those questions go away, and it is what lets
//! [`crate::plan::channel_decimation`] choose a decimation on the merits rather
//! than to keep a filter in the chain.
//!
//! # Why a discriminator and not a coherent detector
//!
//! GMSK *is* continuous-phase FSK, and a coherent MSK detector would be about
//! 2 dB better in noise. It would also need carrier phase, and this receiver
//! does not have it: an uncalibrated RTL-SDR at 162 MHz is twenty to fifty
//! parts per million out, which is three to eight kilohertz — larger than the
//! whole frequency deviation.
//!
//! A discriminator does not care. A constant carrier offset comes out as a
//! constant *added to* the baseband waveform, so it is a DC offset on a bipolar
//! signal, and [`Timing::slice`] is where it goes: the slicing level is
//! measured from the burst itself rather than assumed to be zero. That single
//! property is why this decoder works on a dongle nobody has calibrated, and it
//! is worth more here than 2 dB.
//!
//! What *does* set a limit on the offset is [`CHANNEL_CUTOFF_HZ`], because a
//! filter cut close to the signal's shoulder takes a slice off it — see there.
//!
//! # Why the polarity does not matter
//!
//! The line is NRZI-encoded — a zero is a *change* of level — so the decision
//! that matters is "did this bit differ from the last", not "was it high". A
//! receiver that has its I and Q swapped, or a front end that inverts the
//! spectrum, flips the sign of everything here and decodes exactly the same
//! frames. Nothing in this crate has to know which way round the deviation is.
//!
//! # Why the timing is estimated once per burst rather than tracked
//!
//! A slot is 256 bits. At a hundred parts per million between transmitter and
//! receiver — which is more than either is allowed to be — the clock slips
//! 0.026 of a bit across the whole transmission. There is nothing for a
//! tracking loop to track, and a loop is exactly where a timing recovery goes
//! wrong: it has an S-curve with a second zero half a symbol away, and locking
//! to the wrong one gives a decoder that samples every bit at its transition
//! and finds nothing, on a signal that looks perfect.
//!
//! So the phase is *measured*, once, by the Oerder–Meyr estimator: the squared
//! envelope of a bipolar waveform has a spectral line at the bit rate whose
//! phase is where the bit centres are. One complex accumulation over the burst,
//! no acquisition, no ambiguity — and [`tests::the_timing_estimate_finds_the_bit_centre`]
//! is what proves the sign of it, because a sign error here is a decoder that
//! samples every transition and reports an empty sea.
//!
//! Sources: ITU-R M.1371-5 §3.2.2 (modulation) and §3.3 (the packet); the
//! timing estimator is M. Oerder and H. Meyr, "Digital filter and square timing
//! recovery", IEEE Trans. Comm. 36(5), 1988.

use std::f64::consts::TAU;

use sdroxide_dsp::{Complex32, lowpass_taps};
use sdroxide_types::AIS_BIT_RATE;

/// Where the receive filter cuts, in Hz either side of the channel centre.
///
/// **This number is the decoder's whole frequency-offset budget**, and it is
/// worth being plain about why. A carrier offset is harmless everywhere else in
/// this chain — the discriminator turns it into a DC level and
/// [`Timing::slice`] measures that level rather than assuming it — so the only
/// thing that can lose a transmission to a mistuned receiver is this filter
/// cutting a slice off the top of it.
///
/// A GMSK signal at this rate and index occupies about ±6 kHz, and a windowed
/// sinc spends another 2.4 kHz on its transition band ([`RX_SPAN_BITS`]), so
/// fourteen leaves **±5 kHz** of tolerance — 31 parts per million at 162 MHz.
/// That covers a dongle with a TCXO several times over and an ordinary crystal
/// one about half the time. Past it the operator has to set the front end's
/// frequency correction, and the panel says so rather than leaving them to
/// guess: [`Timing::slice`] *is* the offset, in units of the deviation, so
/// every decoded transmission measures it and
/// [`sdroxide_types::AisStatus::offset_hz`] reports it.
///
/// Wider would buy more tolerance and cost sensitivity — the noise admitted is
/// proportional to this — and the frequency correction is a setting that
/// already exists, so this is where the trade sits.
///
/// It is nowhere near wide enough to admit the *other* AIS channel, which is
/// 50 kHz away and far into this filter's stopband —
/// [`tests::the_receive_filter_rejects_the_other_channel`] measures that, and
/// [`tests::the_offset_budget_is_what_the_cutoff_says_it_is`] measures this.
pub const CHANNEL_CUTOFF_HZ: f64 = 14_000.0;

/// Where the baseband filter cuts, as a fraction of the bit rate.
///
/// Two jobs at once, pulling opposite ways. As a matched filter it wants to be
/// narrow: the frequency pulse of `BT = 0.4` GMSK is mostly inside half the bit
/// rate. As the input to the timing estimator it must be *wider* than half the
/// bit rate, because the spectral line the estimator reads is produced by the
/// overlap of the spectrum with itself shifted by the bit rate, and a signal
/// cut at exactly half has no overlap to produce it. Seven tenths leaves 20 %
/// excess bandwidth, which is a usable line, and costs almost nothing in noise
/// because the signal has little left up there anyway.
pub const BASEBAND_CUTOFF_BITS: f64 = 0.7;

/// Bits the receive filter spans.
///
/// Long enough that the transition band does not eat the offset budget
/// [`CHANNEL_CUTOFF_HZ`] is chosen for: a windowed sinc's transition is about
/// `4/ntaps` of the sample rate, so eight bits' worth puts the passband edge
/// some 2.4 kHz inside the nominal cutoff rather than 3.2 kHz inside it. That
/// difference is a whole kilohertz of tolerance, and it is why the number is
/// not the six it started as.
const RX_SPAN_BITS: f64 = 8.0;

/// Bits the baseband filter spans.
const BB_SPAN_BITS: f64 = 4.0;

/// Sub-bit phases tried, in bits, in the order they are tried.
///
/// The estimate first, and then a short sweep either side of it. The sweep is
/// not there to rescue a broken estimate — it could not, and
/// [`tests::the_timing_estimate_finds_the_bit_centre`] is what stops one — but
/// because a burst clipped at one end, or one where the gate opened late,
/// biases the estimate by a fraction of a bit, and a fraction of a bit is the
/// difference between every frame and none. Each try costs one pass of a
/// slicer over 256 bits.
const PHASE_TRIES: [f64; 5] = [0.0, 0.15, -0.15, 0.3, -0.3];

/// Smallest eye worth slicing, as a fraction of the full frequency deviation.
///
/// A real transmission has most of the deviation open even after the Gaussian
/// filter has closed some of it down; a carrier has none. A tenth is far below
/// anything a signal produces and far above anything a carrier does.
const MIN_EYE: f32 = 0.10;

/// Where the bits are, and what separates a high from a low.
#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// Position of the first bit centre in the filtered baseband, samples.
    pub phase: f64,
    /// Samples a bit.
    pub sps: f64,
    /// The level a sample is compared against.
    ///
    /// Measured from the burst rather than taken to be zero, because this is
    /// where the receiver's carrier offset ends up — see the module note. The
    /// midpoint of the tenth and ninetieth percentiles rather than the mean:
    /// an HDLC line is not balanced (a run of ones holds the level), and a mean
    /// would follow the imbalance into one of the two clusters.
    pub slice: f32,
    /// Half the distance between those two percentiles — the eye's half
    /// opening, in the discriminator's own units. Reported because a burst with
    /// no eye is a burst with no signal in it, however loud the gate found it.
    pub eye: f32,
}

impl Timing {
    /// How far off frequency the transmitter was, in Hz, as this receiver sees
    /// it.
    ///
    /// The slicing level is the middle of the two frequency levels, so in units
    /// where one is the full deviation it *is* the carrier offset. Which end of
    /// it is the receiver and which the ship cannot be told apart from one
    /// burst — but every ship being 4 kHz off in the same direction can only be
    /// the receiver.
    pub fn offset_hz(&self) -> f32 {
        self.slice * (AIS_BIT_RATE / 4.0) as f32
    }
}

/// The channel's receive filter: a running complex low-pass at
/// ±[`CHANNEL_CUTOFF_HZ`], keeping its history across blocks.
///
/// Streaming rather than per-burst because it sits in front of the gate — see
/// the module note — and the gate is what cuts the stream into bursts.
pub struct RxFilter {
    taps: Vec<f32>,
    hist: Vec<Complex32>,
}

impl RxFilter {
    pub fn new(rate_hz: f64) -> RxFilter {
        let sps = rate_hz / AIS_BIT_RATE;
        let ntaps = ((RX_SPAN_BITS * sps).round() as usize).max(9) | 1;
        RxFilter { taps: lowpass_taps(ntaps, CHANNEL_CUTOFF_HZ / rate_hz), hist: Vec::new() }
    }

    pub fn ntaps(&self) -> usize {
        self.taps.len()
    }

    /// Filter a block, appending to `out`. Output sample `i` is the input
    /// delayed by half the filter.
    pub fn process(&mut self, iq: &[Complex32], out: &mut Vec<Complex32>) {
        self.hist.extend_from_slice(iq);
        let n = self.taps.len();
        if self.hist.len() < n {
            return;
        }
        for i in 0..=(self.hist.len() - n) {
            let mut acc = Complex32::default();
            for (k, &h) in self.taps.iter().enumerate() {
                acc += self.hist[i + k] * h;
            }
            out.push(acc);
        }
        // Keep the tail the next block's first outputs need.
        let drop = self.hist.len() - (n - 1);
        self.hist.drain(..drop);
    }
}

/// One channel's GMSK demodulator: discriminates a burst, measures it, and
/// hands back line levels at whatever bit phase it is asked for.
///
/// Holds its scratch buffers so a burst does not allocate.
pub struct Demod {
    rate_hz: f64,
    sps: f64,
    /// Baseband filter taps.
    bb: Vec<f32>,

    f: Vec<f32>,
    /// The filtered discriminator output — what everything downstream reads.
    y: Vec<f32>,
    /// Sorted copy of `y`, for the percentiles.
    sorted: Vec<f32>,
}

impl Demod {
    pub fn new(rate_hz: f64) -> Demod {
        let sps = rate_hz / AIS_BIT_RATE;
        let bb_taps = ((BB_SPAN_BITS * sps).round() as usize).max(5) | 1;
        Demod {
            rate_hz,
            sps,
            bb: lowpass_taps(bb_taps, BASEBAND_CUTOFF_BITS * AIS_BIT_RATE / rate_hz),
            f: Vec::new(),
            y: Vec::new(),
            sorted: Vec::new(),
        }
    }

    pub fn samples_per_bit(&self) -> f64 {
        self.sps
    }

    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    /// Discriminate and measure one already-filtered burst. `None` when it is
    /// too short, or carries no eye.
    pub fn prepare(&mut self, iq: &[Complex32]) -> Option<Timing> {
        if iq.len() < 8 {
            return None;
        }
        // ── the discriminator ──
        //
        // The phase advance from one sample to the next *is* the instantaneous
        // frequency, and `arg` of the product with the conjugate is that
        // advance with no unwrapping to get wrong. Scaled so that one unit is
        // the full ±2400 Hz deviation a modulation index of one half asks for,
        // which is what makes `Timing::eye` and `Timing::slice` numbers worth
        // printing: an eye near one is a clean signal, and a slicing level near
        // one is a receiver a whole deviation off frequency.
        let scale = (self.rate_hz / (AIS_BIT_RATE / 4.0) / TAU) as f32;
        self.f.clear();
        self.f.reserve(iq.len().saturating_sub(1));
        for w in iq.windows(2) {
            let d = w[1] * w[0].conj();
            self.f.push(d.im.atan2(d.re) * scale);
        }

        // ── the baseband filter ──
        let m = self.bb.len();
        if self.f.len() <= m {
            return None;
        }
        self.y.clear();
        self.y.reserve(self.f.len() - m + 1);
        for i in 0..=(self.f.len() - m) {
            let mut acc = 0f32;
            for (k, &h) in self.bb.iter().enumerate() {
                acc += self.f[i + k] * h;
            }
            self.y.push(acc);
        }
        if (self.y.len() as f64) < 2.0 * self.sps {
            return None;
        }

        let (slice, eye) = self.levels();
        // A burst with no eye is not this waveform: a bare carrier, a click, a
        // repeater's tail. It has a slicing level like anything else, and
        // running a slicer over it can only manufacture bits out of rounding.
        //
        // Noise is *not* what this catches — the discriminator of noise swings
        // over the whole ±π and would pass any eye test written — and the frame
        // check sequence is the right layer to refuse that.
        if eye < MIN_EYE {
            return None;
        }
        // The estimator reads the squared *signal*, so it has to see the
        // waveform about its own slicing level rather than about zero — a
        // carrier offset that shifts the whole thing sideways would otherwise
        // turn a symmetric bipolar waveform into a unipolar one, whose square
        // has its line at the wrong place.
        let phase = self.bit_phase(slice);
        Some(Timing { phase, sps: self.sps, slice, eye })
    }

    /// The slicing level and the eye's half opening, as percentiles of the
    /// burst.
    fn levels(&mut self) -> (f32, f32) {
        self.sorted.clear();
        self.sorted.extend_from_slice(&self.y);
        self.sorted.sort_unstable_by(f32::total_cmp);
        let n = self.sorted.len();
        let lo = self.sorted[n / 10];
        let hi = self.sorted[n - 1 - n / 10];
        ((lo + hi) / 2.0, (hi - lo) / 2.0)
    }

    /// Oerder–Meyr: the phase of the bit-rate line in the squared waveform.
    ///
    /// `|y|²` of a bipolar waveform is large at the bit centres and small where
    /// it crosses between them, so its component at the bit rate is a sinusoid
    /// whose peak is exactly where a slicer should sample. One accumulation
    /// gives that component; its argument gives the peak.
    fn bit_phase(&self, slice: f32) -> f64 {
        let (mut re, mut im) = (0f64, 0f64);
        for (i, &v) in self.y.iter().enumerate() {
            let p = f64::from((v - slice) * (v - slice));
            let th = TAU * i as f64 / self.sps;
            re += p * th.cos();
            im -= p * th.sin();
        }
        // `X ≈ (B·N/2)·exp(-j·2π·n₀/sps)` for a squared waveform peaking at
        // n₀, so the peak is at `-arg(X)·sps/2π`, taken into the first bit.
        (-im.atan2(re) / TAU * self.sps).rem_euclid(self.sps)
    }

    /// Slice the prepared burst into line levels at `timing`, shifted by
    /// `offset` bits.
    ///
    /// The levels are what the line was doing, not the data: the NRZI decode
    /// belongs to [`crate::hdlc`], because it is the framing that knows where
    /// the stream begins.
    pub fn slice(&self, timing: &Timing, offset: f64, out: &mut Vec<bool>) {
        out.clear();
        let mut t = timing.phase + offset * timing.sps;
        while t < 0.0 {
            t += timing.sps;
        }
        let last = self.y.len() as f64 - 2.0;
        while t <= last {
            let i = t as usize;
            let frac = (t - i as f64) as f32;
            // Linear interpolation is ample: the waveform is band-limited to
            // 0.7 of the bit rate and sampled at four to sixteen times it, so
            // the error is orders of magnitude under the noise the decision is
            // already making.
            let v = self.y[i] + (self.y[i + 1] - self.y[i]) * frac;
            out.push(v > timing.slice);
            t += timing.sps;
        }
    }

    /// The phase offsets to try, in the order to try them.
    pub fn phase_tries() -> &'static [f64] {
        &PHASE_TRIES
    }

    /// The filtered discriminator output of the last prepared burst — for
    /// tests, and for anything that wants to look at the eye.
    pub fn baseband(&self) -> &[f32] {
        &self.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tx::{Noise, TxParams, modulate};

    /// The estimator has to land on the *centre* of a bit and not on the
    /// transition between two — the two are half a bit apart, both are
    /// stationary points of the same curve, and one of them decodes nothing.
    ///
    /// Measured against a transmitter whose bit boundaries this test knows,
    /// which is the only way to check the sign of the arithmetic in
    /// [`Demod::bit_phase`]. A sign error here is invisible in every other
    /// test in this crate, because the phase sweep in [`crate::channel`] would
    /// quietly find the right answer on its third try.
    #[test]
    fn the_timing_estimate_finds_the_bit_centre() {
        for &rate in &[50_000.0f64, 75_000.0, 76_800.0, 96_000.0] {
            let p = TxParams { sample_rate: rate, ..TxParams::default() };
            // A frame's worth of varied data, so the estimate is driven by
            // random transitions rather than by the training tone alone.
            let payload: Vec<u8> = (0..21u8).map(|i| i.wrapping_mul(37) ^ 0x5c).collect();
            let iq = modulate(&payload, &p);
            let mut rx = RxFilter::new(rate);
            let mut z = Vec::new();
            rx.process(&iq, &mut z);
            let mut d = Demod::new(rate);
            let t = d.prepare(&z).expect("a modulated burst demodulates");

            // The transmitter puts bit centres at `first_bit_center` + k·sps in
            // its own output; the two filters delay that by their group delays,
            // and the discriminator by half a sample.
            let delay = (rx.ntaps() / 2) as f64 + (d.bb.len() / 2) as f64 + 0.5;
            let want = (p.first_bit_center_sample() - delay).rem_euclid(t.sps);
            let err = {
                let mut e = t.phase - want;
                while e > t.sps / 2.0 {
                    e -= t.sps;
                }
                while e < -t.sps / 2.0 {
                    e += t.sps;
                }
                e / t.sps
            };
            assert!(
                err.abs() < 0.12,
                "at {rate} Hz the estimate is {err} bits from the bit centre \
                 (got {}, wanted {want}, sps {})",
                t.phase,
                t.sps
            );
            assert!(t.eye > 0.4, "the eye should be most of the deviation, not {}", t.eye);
        }
    }

    /// A carrier offset larger than the whole frequency deviation must not
    /// stop anything: it is a DC offset on the baseband waveform, and the
    /// slicing level is measured rather than assumed.
    #[test]
    fn an_uncalibrated_receiver_still_has_an_eye() {
        let rate = 75_000.0;
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let payload: Vec<u8> = (0..21u8).map(|i| i.wrapping_mul(91) ^ 0x33).collect();
        let mut iq = modulate(&payload, &p);
        // 4 kHz — an RTL-SDR twenty-five parts per million out at 162 MHz, and
        // nearly twice the ±2400 Hz the modulation itself uses.
        crate::tx::shift(&mut iq, 4_000.0, rate);

        let mut rx = RxFilter::new(rate);
        let mut z = Vec::new();
        rx.process(&iq, &mut z);
        let mut d = Demod::new(rate);
        let t = d.prepare(&z).expect("an offset burst still demodulates");
        // 4 kHz is 1.67 of the ±2400 Hz deviation, and that is where it ends
        // up: as a slicing level, not as a lost frame.
        assert!(
            (t.offset_hz() - 4_000.0).abs() < 400.0,
            "the offset should show up as a measured 4 kHz, not {}",
            t.offset_hz()
        );
        assert!(t.eye > 0.5, "and the eye should survive it: {}", t.eye);
    }

    /// The offset budget is the one [`CHANNEL_CUTOFF_HZ`] claims, and it is
    /// measured rather than argued: the eye has to stay open out to ±5 kHz —
    /// 31 parts per million at 162 MHz — and it is allowed to shut past there.
    ///
    /// A cliff is what this looks like when it goes wrong, and it goes wrong
    /// silently: every ship decodes on the bench and none on the air, because
    /// the bench signal was generated on frequency.
    #[test]
    fn the_offset_budget_is_what_the_cutoff_says_it_is() {
        let rate = 75_000.0;
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let payload: Vec<u8> = (0..21u8).map(|i| i.wrapping_mul(53) ^ 0x1e).collect();
        for hz in [0.0f64, 2_000.0, 4_000.0, 5_000.0, -5_000.0] {
            let mut iq = modulate(&payload, &p);
            crate::tx::shift(&mut iq, hz, rate);
            let mut rx = RxFilter::new(rate);
            let mut z = Vec::new();
            rx.process(&iq, &mut z);
            let mut d = Demod::new(rate);
            let t = d.prepare(&z).unwrap_or_else(|| panic!("{hz} Hz off produced no eye at all"));
            assert!(t.eye > 0.5, "{hz} Hz off leaves an eye of only {}", t.eye);
            // ...and the offset is not just survived, it is measured — which is
            // what lets the panel tell an operator to set a correction.
            assert!(
                (f64::from(t.offset_hz()) - hz).abs() < 400.0,
                "{hz} Hz off measured as {}",
                t.offset_hz()
            );
        }
    }

    /// A bare carrier has no eye, and is refused rather than sliced into bits
    /// that were never sent.
    ///
    /// This is what [`MIN_EYE`] is for. It is deliberately *not* what refuses
    /// noise — the discriminator of noise swings over the whole ±π and would
    /// pass any eye test written — and the module note says which layer does.
    #[test]
    fn a_bare_carrier_has_no_eye() {
        let rate = 75_000.0;
        let mut n = Noise::new(4242);
        let mut buf = vec![Complex32::new(0.5, 0.0); 4096];
        n.add(&mut buf, 0.002);
        let mut rx = RxFilter::new(rate);
        let mut z = Vec::new();
        rx.process(&buf, &mut z);
        let mut d = Demod::new(rate);
        assert!(d.prepare(&z).is_none(), "a carrier was accepted as a transmission");
    }

    /// The receive filter is what keeps the other AIS channel out, and it has
    /// to do it whether or not the down-converter decimated — which is the
    /// whole reason it sits in front of the gate.
    #[test]
    fn the_receive_filter_rejects_the_other_channel() {
        let rate = 125_000.0; // a window that legitimately decimates by one
        let mut neighbour = vec![Complex32::default(); 8192];
        for (i, z) in neighbour.iter_mut().enumerate() {
            let th = TAU * 50_000.0 * i as f64 / rate;
            *z = Complex32::new(th.cos() as f32, th.sin() as f32);
        }
        let mut rx = RxFilter::new(rate);
        let mut out = Vec::new();
        rx.process(&neighbour, &mut out);
        let tail = &out[out.len() / 2..];
        let after = tail.iter().map(|z| z.norm()).sum::<f32>() / tail.len() as f32;
        let db = 20.0 * after.max(1e-9).log10();
        assert!(db < -50.0, "the other channel is only {db} dB down");
    }
}
