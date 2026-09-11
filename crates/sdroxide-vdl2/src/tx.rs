//! A VDL2 transmitter, in software.
//!
//! Public, and shared by the unit tests, the `vdl2_iq` example and the engine's
//! integration test, rather than living inside a test module. The reason is the
//! one the ADS-B decoder next door records: a transmitter and a receiver written
//! by the same hand agree with each other by construction, and the only defence
//! is to make *the transmitter under test* the same one everything else uses, so
//! that a convention cannot be fixed in one place and left wrong in the other.
//!
//! It is also deliberately made to misbehave in the ways a real transmitter
//! does. Every one of these has hidden a bug in some decoder:
//!
//! - **a fractional start time**, because a burst does not arrive on a sample
//!   boundary;
//! - **a carrier offset**, because the 656 Hz that spends the whole decision
//!   margin is 4.8 parts per million and a receiver is rarely that good;
//! - **a symbol clock error**, because the samples per symbol are not a round
//!   number and never quite what either end thinks;
//! - **either pulse shape**, because the standard names raised cosine for the
//!   transmitter and does not always make plain whether the root is taken on one
//!   side or split across both — a receiver has to cope with what it meets;
//! - **spectral inversion**, because a front end with high-side injection
//!   mirrors everything.
//!
//! Source: the same standard the decoder is written from; this is its inverse.

use sdroxide_dsp::Complex32;

use crate::block::{self, InterleaveOrder};
use crate::demod::{ALPHA, SYMBOL_RATE, UNGRAY, rc_impulse, rrc_impulse};
use crate::header;
use crate::scramble::Lfsr;
use crate::sync::UW_INCREMENTS;

/// Which pulse the transmitter shapes with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Shape {
    /// Full raised cosine — a Nyquist pulse on its own, which is what the
    /// standard names.
    #[default]
    Rc,
    /// Root raised cosine, the half that pairs with a matching filter in the
    /// receiver.
    Rrc,
}

/// How a burst is put on the air.
#[derive(Debug, Clone, Copy)]
pub struct TxParams {
    pub sample_rate: f64,
    /// Carrier offset from the channel centre.
    pub freq_offset_hz: f64,
    /// Symbol clock error, parts per million.
    pub clock_ppm: f64,
    pub amplitude: f32,
    pub shape: Shape,
    /// The receive chain mirrors the spectrum.
    pub inverted: bool,
    /// Symbols of transmitter ramp-up before the synchronisation word.
    pub ramp_syms: usize,
}

impl Default for TxParams {
    fn default() -> Self {
        TxParams {
            sample_rate: 96_000.0,
            freq_offset_hz: 0.0,
            clock_ppm: 0.0,
            amplitude: 1.0,
            shape: Shape::Rc,
            inverted: false,
            ramp_syms: 5,
        }
    }
}

/// Symbols the pulse is truncated to, either side.
const SPAN_SYMS: f64 = 8.0;

/// The bit stream a burst carries: the header, then the interleaved and
/// Reed-Solomon-coded data field, scrambled as one run.
///
/// The frame goes into the field the way it goes out on the air — inside two
/// `0x7E` flags, bit-stuffed, and padded to a whole octet with zeros (see
/// [`crate::hdlc`]). The length the header states is the length of *that*, in
/// bits, before the padding. Until issue #265 this wrapped nothing and stated
/// the bare frame's length, which made a generator that agreed with this
/// decoder and with no transmitter in the sky.
pub fn burst_bits(frame: &[u8], order: InterleaveOrder) -> Vec<u8> {
    let framed = crate::hdlc::frame_bits(frame);
    let mut field_bits = framed.clone();
    // The Reed-Solomon layer works in octets, so the last one is filled out.
    // With zeros: measured off the air on every frame in the issue #265
    // recording that carried any padding at all.
    while !field_bits.len().is_multiple_of(8) {
        field_bits.push(0);
    }
    let data: Vec<u8> = field_bits
        .chunks_exact(8)
        .map(|c| c.iter().enumerate().fold(0u8, |o, (k, &b)| o | ((b & 1) << k)))
        .collect();
    let field = block::encode(&data, order);
    let mut bits = Vec::with_capacity(header::HEADER_BITS + field.len() * 8);
    bits.extend_from_slice(&header::encode(framed.len() as u32, 0));
    for &o in &field {
        for k in 0..8 {
            bits.push((o >> k) & 1);
        }
    }
    // A D8PSK symbol is three bits and the total is not a multiple of three;
    // the tail is scrambler output, which the receiver never reaches.
    while bits.len() % 3 != 0 {
        bits.push(0);
    }
    Lfsr::new().apply(&mut bits);
    bits
}

/// The phase increments of a whole burst, in eighths of a turn: the ramp-up,
/// the synchronisation word, then the data.
pub fn burst_increments(frame: &[u8], ramp_syms: usize, order: InterleaveOrder) -> Vec<u8> {
    let bits = burst_bits(frame, order);
    let mut inc = vec![0u8; ramp_syms];
    inc.extend_from_slice(&UW_INCREMENTS);
    // Most significant bit of the symbol first — see
    // `SymbolReader::next_symbol`, which is the half of this that has to agree
    // with a real transmitter rather than with this one.
    for c in bits.chunks_exact(3) {
        let v = (c[0] << 2) | (c[1] << 1) | c[2];
        inc.push(UNGRAY[v as usize]);
    }
    inc
}

/// Add one burst to `out`, starting at fractional sample position `at`.
///
/// `out` is grown as needed and the burst is *added*, so two stations colliding
/// on one channel is written the way it happens.
pub fn modulate_at(frame: &[u8], p: &TxParams, at: f64, out: &mut Vec<Complex32>) {
    let inc = burst_increments(frame, p.ramp_syms, InterleaveOrder::RoundRobin);
    modulate_increments_at(&inc, p, at, out);
}

/// [`modulate_at`] from phase increments that are already in hand, rather than
/// from a frame this module encoded.
///
/// What that is for: a burst *captured off the air* can be replayed through the
/// whole receiver by carrying its symbols rather than its samples — a hundred
/// and fifty bytes instead of a hundred kilobytes — and everything from the
/// symbol decision inwards is then tested against a real transmitter instead of
/// against this one. That distinction is the whole of issue #265: the encoder
/// above and the decoder agreed with each other about the bit order inside a
/// symbol and about there being no HDLC wrapper, and were both wrong.
///
/// `inc` includes the ramp-up and the synchronisation word; `p.ramp_syms` says
/// how many of the leading symbols the envelope ramps over.
pub fn modulate_increments_at(inc: &[u8], p: &TxParams, at: f64, out: &mut Vec<Complex32>) {
    let sps = p.sample_rate / SYMBOL_RATE * (1.0 + p.clock_ppm * 1e-6);
    let span = (SPAN_SYMS * sps).ceil() as isize;

    let first = (at - SPAN_SYMS * sps).floor().max(0.0) as usize;
    let need = (at + (inc.len() as f64 + SPAN_SYMS + 1.0) * sps).ceil() as usize + 1;
    if out.len() < need {
        out.resize(need, Complex32::default());
    }
    // Built into scratch and added at the end, rather than written straight into
    // `out`. The carrier offset and the spectral mirror are whole-buffer
    // operations, and applying them in place would rotate every burst already
    // there — which is exactly what a recording of two stations on two channels
    // is made of.
    let mut scratch = vec![Complex32::default(); need - first];

    let mut phase = 0f64;
    for (n, &d) in inc.iter().enumerate() {
        phase += f64::from(d) * std::f64::consts::FRAC_PI_4;
        // The ramp: amplitude rises over the run-up symbols so the transmission
        // does not start with a step, which would splatter across the band.
        let env = if n < p.ramp_syms { (n + 1) as f64 / (p.ramp_syms + 1) as f64 } else { 1.0 };
        let a = Complex32::new((env * phase.cos()) as f32, (env * phase.sin()) as f32);
        let centre = at + n as f64 * sps;
        let lo = (centre.floor() as isize - span).max(first as isize);
        let hi = centre.ceil() as isize + span;
        for i in lo..=hi {
            let i = i as usize;
            if i >= need {
                break;
            }
            let t = (i as f64 - centre) / sps;
            let g = match p.shape {
                Shape::Rc => rc_impulse(t, ALPHA),
                Shape::Rrc => rrc_impulse(t, ALPHA),
            };
            scratch[i - first] += a * (g as f32) * p.amplitude;
        }
    }

    // The carrier offset, and the mirror. The phase runs on the *absolute*
    // sample index, so two bursts on the same channel agree about where the
    // carrier is — which is what a receiver measuring a frequency error assumes.
    let w = 2.0 * std::f64::consts::PI * p.freq_offset_hz / p.sample_rate;
    for (k, z) in scratch.iter_mut().enumerate() {
        if p.freq_offset_hz != 0.0 {
            let ph = w * (first + k) as f64;
            *z *= Complex32::new(ph.cos() as f32, ph.sin() as f32);
        }
        if p.inverted {
            *z = z.conj();
        }
    }
    for (k, z) in scratch.into_iter().enumerate() {
        out[first + k] += z;
    }
}

/// A burst on its own, with `lead` samples of silence in front of it.
pub fn modulate(frame: &[u8], p: &TxParams, lead: f64) -> Vec<Complex32> {
    let mut out = vec![Complex32::default(); lead.ceil() as usize];
    modulate_at(frame, p, lead, &mut out);
    // A little silence after, so the gate has a falling edge to close on.
    out.resize(out.len() + (p.sample_rate * 0.005) as usize, Complex32::default());
    out
}

/// A deterministic noise source. There is no `rand` in this tree, and a test
/// that fails one run in fifty is worse than no test at all.
pub struct Noise(u64);

impl Noise {
    pub fn new(seed: u64) -> Noise {
        Noise(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// One complex Gaussian sample with the given per-component deviation.
    pub fn gaussian(&mut self, sigma: f32) -> Complex32 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let th = 2.0 * std::f64::consts::PI * u2;
        Complex32::new((r * th.cos()) as f32 * sigma, (r * th.sin()) as f32 * sigma)
    }

    /// Add noise across a whole buffer.
    pub fn add(&mut self, buf: &mut [Complex32], sigma: f32) {
        for s in buf.iter_mut() {
            *s += self.gaussian(sigma);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sync::UNIQUE_WORD;

    /// The transmitter emits the synchronisation word the standard publishes,
    /// symbol for symbol, with the ramp-up in front of it.
    #[test]
    fn the_burst_opens_with_the_synchronisation_word() {
        let frame = vec![0u8; 20];
        let inc = burst_increments(&frame, 5, InterleaveOrder::RoundRobin);
        assert_eq!(&inc[..5], &[0, 0, 0, 0, 0], "the ramp holds phase");
        assert_eq!(&inc[5..21], &UW_INCREMENTS[..]);
        // ...and those increments Gray-encode back to the published symbols.
        let syms: Vec<u8> = inc[5..21].iter().map(|&d| crate::demod::GRAY[d as usize]).collect();
        assert_eq!(syms, UNIQUE_WORD.to_vec());
    }

    /// The bit stream is the header, then the coded field, and it is exactly as
    /// long as the header's own length field implies — where that length is the
    /// *flagged and stuffed* field's, not the bare frame's (issue #265).
    #[test]
    fn the_bit_stream_is_the_length_the_header_claims() {
        for n in [11usize, 40, 100, 250] {
            let frame: Vec<u8> = (0..n).map(|i| i as u8).collect();
            let bits = burst_bits(&frame, InterleaveOrder::RoundRobin);
            let framed = crate::hdlc::frame_bits(&frame);
            let l = block::layout(framed.len().div_ceil(8));
            let want = header::HEADER_BITS + l.total_octets * 8;
            assert_eq!(bits.len(), want.next_multiple_of(3), "{n} octets");

            // And the header at the front really says how long the field is.
            let mut head = bits[..header::HEADER_BITS].to_vec();
            Lfsr::new().apply(&mut head);
            let h = header::decode(&head).expect("clean header");
            assert_eq!(h.trlen_bits as usize, framed.len());
            // Which is the frame, both flags, and whatever was stuffed.
            assert!(h.trlen_bits as usize >= n * 8 + 16);
        }
    }

    /// A modulated burst is finite, bounded and centred — the properties every
    /// downstream measurement assumes.
    #[test]
    fn a_modulated_burst_is_well_behaved() {
        let frame = vec![0x5au8; 30];
        let p = TxParams { sample_rate: 96_000.0, ..TxParams::default() };
        let iq = modulate(&frame, &p, 100.0);
        assert!(iq.len() > 1000);
        assert!(iq.iter().all(|s| s.re.is_finite() && s.im.is_finite()));
        let peak = iq.iter().map(|s| s.norm_sqr()).fold(0f32, f32::max).sqrt();
        assert!((0.5..3.0).contains(&peak), "peak {peak}");
        // Silence before the pulse's own leading tail, which reaches eight
        // symbols back from the first ramp symbol — that spread is the shaping
        // filter doing its job, not the burst starting early.
        assert!(iq[..20].iter().all(|s| s.norm_sqr() < 1e-12));
    }

    /// The noise source is deterministic and roughly the deviation it is asked
    /// for, so a signal-to-noise figure in a test means something.
    #[test]
    fn the_noise_source_is_deterministic_and_scaled() {
        let mut a = Noise::new(1);
        let mut b = Noise::new(1);
        for _ in 0..100 {
            assert_eq!(a.gaussian(1.0), b.gaussian(1.0));
        }
        let mut n = Noise::new(7);
        let mut acc = 0f64;
        let count = 20_000;
        for _ in 0..count {
            acc += f64::from(n.gaussian(0.5).norm_sqr());
        }
        // Two components at sigma 0.5 each: mean power 2 * 0.25.
        let mean = acc / f64::from(count);
        assert!((mean - 0.5).abs() < 0.02, "mean power {mean}");
    }
}
