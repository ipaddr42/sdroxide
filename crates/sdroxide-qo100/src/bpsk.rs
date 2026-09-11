//! BPSK-400 demodulation and AO-40 "uncoded" frame decode for the QO-100
//! narrowband beacon.
//!
//! # Protocol, confirmed rather than guessed
//!
//! Every number below is taken from Daniel Estévez's (EA4GPZ) gr-satellites
//! project (GPL-3.0-or-later, <https://github.com/daniestevez/gr-satellites>)
//! — he wrote and maintains the QO-100 beacon decoder there — cross-checked
//! against his write-up at
//! <https://destevez.net/2017/05/decoding-ao-40-uncoded-telemetry/>. Nothing
//! here is ported from that code; these are protocol facts (a sync pattern,
//! a frame length, a CRC variant), not an implementation.
//!
//! - `python/satyaml/QO-100.yml`: 10489.750 MHz, "DBPSK Manchester", 400 baud,
//!   framing "AO-40 uncoded" (the satellite also sends an FEC-coded variant on
//!   alternate frames; this module does not attempt that one — see the crate
//!   doc).
//! - `python/components/deframers/ao40_uncoded_deframer.py`: sync word
//!   `'00111001000101011110110100110000'` (32 bits, MSB first —
//!   [`SYNC_WORD`]/[`SYNC_LEN`]), frame length `512 + 2` bytes
//!   ([`PAYLOAD_BYTES`]/[`CRC_BYTES`]), `crc16_ccitt_false` (poly 0x1021,
//!   init 0xFFFF, not reflected, no xorout), sync-word threshold 3 bit errors
//!   ([`SYNC_MAX_ERRORS`]).
//! - `python/telemetry/qo100.py`: the payload is plain ASCII text, no binary
//!   header — `packet[:-2].decode('ascii')`.
//! - The write-up: data is differentially encoded first (`1` = a phase
//!   change, `0` = none), then Manchester encoded.
//!
//! # Demodulation: search, not a tracking loop
//!
//! There is no way to test a Costas/Gardner loop's acquisition behaviour
//! against the real signal from here, so this does not attempt one. Instead —
//! matching `sdroxide_ism`'s slicer, which solves the same problem the same
//! way — [`acquire`] tries a grid of candidate frequency offsets and, at each,
//! every chip-timing phase and Manchester bit-parity, decodes the whole block
//! and checks the sync word and CRC. Whichever combination validates the CRC
//! *is* the calibration answer: its frequency offset is exactly how far the
//! beacon sits from where it was assumed to be.
//!
//! Before that sweep, [`coarse_carrier_hz`] takes one look at the block's
//! power spectrum for the beacon's give-away shape — two lobes with a null
//! between them at the carrier and another near ±[`CHIP_RATE`] out, left/right
//! symmetric. When it finds one, the sweep starts there instead of at DC and,
//! if that estimate is further out than the configured half-width, reaches out
//! to it — so a station whose LNB has never been calibrated is still found.
//! The estimate never decides a lock: a wrong one only reorders the grid the
//! CRC was going to be tried against anyway.
//!
//! The differential+Manchester combination is decoded in one pass, without
//! ever resolving absolute carrier phase: comparing each chip against the one
//! immediately before it (a delay-and-multiply, not a coherent reference)
//! gives a flip/no-flip bit that is robust to whatever the residual carrier
//! phase is doing, and — because Manchester always flips at a bit's own
//! midpoint but the *inter-bit* transition flips exactly when the
//! differentially-encoded source bit is `0` — keeping only the inter-bit
//! comparisons recovers the original data directly. See the derivation in
//! this module's tests.

use rustfft::FftPlanner;
use sdroxide_dsp::Complex32;

/// Chips per second on the air. Manchester encoding sends two of these per
/// data bit.
pub const CHIP_RATE: f64 = 800.0;
/// Data bits per second — half the chip rate, Manchester's own cost.
pub const BAUD: f64 = 400.0;

/// The AO-40 uncoded beacon's sync pattern, MSB first, right-aligned in a
/// u32. See the module doc for where this number comes from.
const SYNC_WORD: u32 = 0x3915_ED30;
const SYNC_LEN: u32 = 32;
/// Bit errors tolerated in the sync match — gr-satellites' own default
/// (`ao40_uncoded_deframer`'s `syncword_threshold`).
const SYNC_MAX_ERRORS: u32 = 3;

const PAYLOAD_BYTES: usize = 512;
const CRC_BYTES: usize = 2;
const FRAME_BYTES: usize = PAYLOAD_BYTES + CRC_BYTES;
const FRAME_BITS: usize = FRAME_BYTES * 8;

/// One whole frame's time on the air, sync word included — 10.36 s, matching
/// destevez.net's own figure for it. [`crate::controller::Qo100Controller`]
/// sizes its rolling buffer off this so a frame can never fall between two
/// analysis windows unseen.
pub const FRAME_SECONDS: f64 = (SYNC_LEN as usize + FRAME_BITS) as f64 / BAUD;

/// Chip-timing phases tried per frequency candidate. A sixteenth of a chip is
/// closer to optimal than the timing error a 400 baud link accumulates over
/// one frame, so trying more would be measuring noise — the same reasoning
/// `sdroxide_ism::slice` uses for its own `PHASES`.
const TIMING_PHASES: usize = 8;

/// A successfully decoded frame: how far the beacon actually sits from the
/// frequency [`acquire`] was told to assume, and what it said.
#[derive(Debug, Clone, PartialEq)]
pub struct Qo100Lock {
    pub offset_hz: f64,
    pub text: String,
}

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, not reflected, no xorout.
/// The variant `ao40_uncoded_deframer.py` names (`crc16_ccitt_false`); the
/// well-known check value for `"123456789"` is `0x29B1`, asserted in this
/// module's tests.
fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// Differential-encode `bits` (`1` = phase change, `0` = none) — the
/// transmit-side half of the demodulation this module reverses, and needed
/// again itself by [`refine_offset_hz`], which has to know exactly which
/// chip polarities a *decoded* frame implies in order to strip them back out.
fn differential_encode(bits: &[bool]) -> Vec<bool> {
    let mut state = false;
    bits.iter()
        .map(|&d| {
            state ^= d;
            state
        })
        .collect()
}

/// Manchester-encode `e` into chip polarities (`true` = one BPSK sense,
/// `false` = the other — which is which is arbitrary and resolved by nothing
/// here, exactly as [`chip_flips`] needs it to be).
fn manchester_chips(e: &[bool]) -> Vec<bool> {
    e.iter().flat_map(|&b| [b, !b]).collect()
}

/// The full on-air chip sequence (differential, then Manchester) for
/// `source_bits` — sync word included, in transmit order.
fn source_chips(source_bits: &[bool]) -> Vec<bool> {
    manchester_chips(&differential_encode(source_bits))
}

/// Delay-and-multiply detection between every adjacent chip: `true` where the
/// phase flipped from one chip to the next. Robust to a residual carrier
/// frequency error too small to notice between two chips 1.25 ms apart, which
/// is the whole reason this is used instead of a coherent (absolute-phase)
/// slicer — see the module doc.
fn chip_flips(chips: &[Complex32]) -> Vec<bool> {
    chips.windows(2).map(|w| (w[1] * w[0].conj()).re < 0.0).collect()
}

/// The differentially-encoded, Manchester-encoded data bits `chip_flips`
/// implies, for one of the two possible Manchester bit-parities (which half
/// of each adjacent-chip pair is the *inter-bit* boundary — the intra-bit
/// transition Manchester guarantees every bit carries no information and is
/// skipped). See the module doc's derivation.
///
/// `parity == 0`: bit boundaries are flips at odd indices of `flips` (chip 1
/// to chip 2, chip 3 to chip 4, …). `parity == 1`: the other set.
fn data_bits(flips: &[bool], parity: usize) -> Vec<bool> {
    flips.iter().skip(parity).step_by(2).map(|&f| !f).collect()
}

/// `bits`, packed MSB-first into bytes. Trailing bits short of a full byte
/// are dropped.
fn pack_bits(bits: &[bool]) -> Vec<u8> {
    bits.as_chunks::<8>()
        .0
        .iter()
        .map(|c| c.iter().fold(0u8, |acc, &b| (acc << 1) | b as u8))
        .collect()
}

/// The bit offset in `bits` where [`SYNC_WORD`] matches within
/// [`SYNC_MAX_ERRORS`], and how many errors it took — the *first* match, read
/// left to right, since a frame beacon transmits continuously and the first
/// candidate in a block is as good as any later one.
fn find_sync(bits: &[bool], from: usize) -> Option<usize> {
    if from + SYNC_LEN as usize > bits.len() {
        return None;
    }
    let mut window: u32 = 0;
    for (i, &b) in bits.iter().enumerate().skip(from) {
        window = (window << 1) | b as u32;
        if i + 1 < from + SYNC_LEN as usize {
            continue;
        }
        if (window ^ SYNC_WORD).count_ones() <= SYNC_MAX_ERRORS {
            return Some(i + 1 - SYNC_LEN as usize);
        }
    }
    None
}

/// One matched, CRC-valid frame's essentials: the bits it actually carried
/// (sync word included, so its exact on-air chips can be reconstructed) and
/// the decoded text, plus where in `data_bits`-space the sync word began —
/// [`refine_offset_hz`] needs all of it to re-locate the exact samples.
struct FrameMatch {
    db_start: usize,
    source_bits: Vec<bool>,
    text: String,
}

/// Try to decode one AO-40 uncoded frame from `bits` (already chip- and
/// Manchester-decoded data bits — see [`data_bits`]). `None` when nothing in
/// range is both a sync match and a CRC-valid frame after it.
///
/// Keeps searching past a sync match whose CRC fails, rather than stopping at
/// the first one: a 32-bit pattern matched within 3 errors turns up by pure
/// chance often enough in a block this long (noise, or another satellite's
/// frame ahead of the real one) that giving up there would refuse a perfectly
/// good frame sitting right after it.
fn decode_frame(bits: &[bool]) -> Option<FrameMatch> {
    let mut from = 0;
    while let Some(start) = find_sync(bits, from) {
        let frame_bits = bits.get(start + SYNC_LEN as usize..);
        if let Some(frame_bits) = frame_bits
            && frame_bits.len() >= FRAME_BITS
        {
            let frame = pack_bits(&frame_bits[..FRAME_BITS]);
            let want = u16::from_be_bytes([frame[PAYLOAD_BYTES], frame[PAYLOAD_BYTES + 1]]);
            let got = crc16_ccitt_false(&frame[..PAYLOAD_BYTES]);
            if got == want {
                let mut source_bits = Vec::with_capacity(SYNC_LEN as usize + FRAME_BITS);
                source_bits.extend((0..SYNC_LEN).rev().map(|i| (SYNC_WORD >> i) & 1 != 0));
                source_bits.extend_from_slice(&frame_bits[..FRAME_BITS]);
                return Some(FrameMatch {
                    db_start: start,
                    source_bits,
                    text: String::from_utf8_lossy(&frame[..PAYLOAD_BYTES]).into_owned(),
                });
            }
        }
        from = start + 1;
    }
    None
}

/// Extract one complex sample per chip from `iq` (already mixed to the
/// candidate frequency), starting `phase` samples in, at `samples_per_chip`
/// spacing. Nearest-sample selection — no interpolation — which is enough at
/// the oversampling ratios this runs at (12+ samples/chip in practice).
fn chip_samples(iq: &[Complex32], samples_per_chip: f64, phase: usize) -> Vec<Complex32> {
    let mut out = Vec::with_capacity((iq.len() as f64 / samples_per_chip) as usize);
    let mut pos = phase as f64;
    while (pos as usize) < iq.len() {
        out.push(iq[pos as usize]);
        pos += samples_per_chip;
    }
    out
}

/// One coarse-search hit: everything [`refine_offset_hz`] needs to re-find
/// the exact samples the matched frame came from.
struct CoarseMatch {
    parity: usize,
    phase: usize,
    m: FrameMatch,
}

/// Try every chip-timing phase and Manchester parity against `iq` (already
/// mixed to one candidate frequency, `rate_hz` samples/s). `Some` on the
/// first combination whose sync word and CRC both check out.
fn try_frequency(iq: &[Complex32], rate_hz: f64) -> Option<CoarseMatch> {
    let samples_per_chip = rate_hz / CHIP_RATE;
    if samples_per_chip < 2.0 {
        return None; // not enough oversampling for chip_samples to mean anything
    }
    let phase_step = (samples_per_chip / TIMING_PHASES as f64).max(1.0);
    for p in 0..TIMING_PHASES {
        let phase = (p as f64 * phase_step) as usize;
        let chips = chip_samples(iq, samples_per_chip, phase);
        if chips.len() < 3 {
            continue;
        }
        let flips = chip_flips(&chips);
        for parity in 0..2 {
            let bits = data_bits(&flips, parity);
            if let Some(m) = decode_frame(&bits) {
                return Some(CoarseMatch { parity, phase, m });
            }
        }
    }
    None
}

/// Precisely measure the residual carrier frequency of a frame
/// [`try_frequency`] already found, on top of whatever coarse offset `iq` was
/// already mixed by.
///
/// The coarse chip-rate delay-detector that found the frame tolerates
/// whatever residual frequency error let it decode at all — a wide window,
/// [`CHIP_RATE`] Hz wide before it repeats — which is exactly what makes it
/// useless for saying *where inside that window* the true carrier sits. Now
/// that the frame is known exactly, its modulation can be stripped from every
/// *raw* sample (not just one per chip) and the residual phase averaged at
/// the full sample rate instead: far more samples to average over, and an
/// alias period of `rate_hz` rather than [`CHIP_RATE`] — wide enough that no
/// realistic search width can be fooled by it.
fn refine_offset_hz(iq: &[Complex32], rate_hz: f64, cm: &CoarseMatch) -> f64 {
    let samples_per_chip = rate_hz / CHIP_RATE;
    let chips = source_chips(&cm.m.source_bits);
    let chip0 = cm.parity + 2 * cm.m.db_start;
    let sample_start = cm.phase + (chip0 as f64 * samples_per_chip).round() as usize;
    let sample_end = (cm.phase
        + ((chip0 + chips.len()) as f64 * samples_per_chip).round() as usize)
        .min(iq.len());
    if sample_end <= sample_start + 1 {
        return 0.0;
    }
    let mut sum = Complex32::new(0.0, 0.0);
    let mut prev: Option<Complex32> = None;
    for (offset, &z) in iq[sample_start..sample_end].iter().enumerate() {
        let chip_idx = (offset as f64 / samples_per_chip) as usize;
        let Some(&bit) = chips.get(chip_idx) else { break };
        let clean = z * if bit { -1.0f32 } else { 1.0f32 };
        if let Some(p) = prev {
            sum += clean * p.conj();
        }
        prev = Some(clean);
    }
    if sum.norm() < 1e-6 {
        return 0.0;
    }
    sum.arg() as f64 * rate_hz / std::f64::consts::TAU
}

/// How often the mixing phasor in [`mix_decimate`] is pulled back onto the
/// unit circle. A complex multiply per sample is what makes the mixer cheap,
/// but it also compounds the rounding error of the one before it, and left
/// alone the phasor's magnitude walks away from 1 and takes the mixed
/// amplitude with it. Renormalising costs one square root every `RENORM`
/// samples — nothing beside the multiply it rides on.
const RENORM: usize = 4096;

/// Mix `iq` down by `shift_hz` and integer-decimate by `deci` in one pass,
/// each output sample the mean of its `deci` inputs. That boxcar is a crude
/// anti-alias filter, but its nulls sit exactly at multiples of the output
/// rate — where any energy would fold — and the beacon is 400 baud, far
/// inside the output passband, so nothing that carries the frame is touched.
/// Output rate is `rate_hz / deci`.
///
/// The phasor advances by one complex multiply per sample rather than a
/// `sin`/`cos` pair. This function is the whole cost of an [`acquire`] sweep —
/// it runs once per candidate frequency over the entire ~24 s buffer, and a
/// profile puts ~96 % of the sweep inside it — so the two transcendentals it
/// used to evaluate per input sample were, in the end, most of what the
/// decoder did. See [`RENORM`] for what keeps the recurrence honest.
fn mix_decimate(iq: &[Complex32], rate_hz: f64, shift_hz: f64, deci: usize) -> Vec<Complex32> {
    let deci = deci.max(1);
    let w = -std::f64::consts::TAU * shift_hz / rate_hz;
    // The recurrence runs in f64 and only its *output* is narrowed. Rounding
    // the step to f32 would put a fixed error into the angle, and a fixed
    // angle error per sample is a phase ramp: harmless per sample, ~0.1 rad by
    // the far end of a 24 s buffer. The differential demodulator would not
    // even notice that, but it is a lie about where the carrier is, and this
    // function's callers are trying to measure exactly that.
    let (sr, si) = (w.cos(), w.sin());
    let (mut pr, mut pi) = (1.0f64, 0.0f64);
    let mut out = Vec::with_capacity(iq.len() / deci + 1);
    let mut acc = Complex32::new(0.0, 0.0);
    let mut k = 0usize;
    for (n, &z) in iq.iter().enumerate() {
        acc += z * Complex32::new(pr as f32, pi as f32);
        (pr, pi) = (pr * sr - pi * si, pr * si + pi * sr);
        if n % RENORM == RENORM - 1 {
            let m = (pr * pr + pi * pi).sqrt();
            (pr, pi) = (pr / m, pi / m);
        }
        k += 1;
        if k == deci {
            out.push(acc / deci as f32);
            acc = Complex32::new(0.0, 0.0);
            k = 0;
        }
    }
    out
}

/// A confident twin-lobe estimate ([`coarse_carrier_hz`]) is allowed to pull
/// [`acquire`]'s sweep this far from the assumed centre even when
/// `search_half_width_hz` is smaller — generous enough for any real LNB that
/// has never been calibrated, and capped so a stray estimate can never send
/// the sweep clear across the capture.
const ACQ_RANGE_MAX_HZ: f64 = 60_000.0;
/// Margin kept either side of a seed when it decides how far the sweep reaches.
const SEED_MARGIN_HZ: f64 = 2_000.0;

/// The dense sub-grid tried right on the spectral seed before the coarse
/// sweep: `±FINE_SPAN_HZ` in `FINE_STEP_HZ` steps. Fine enough that the
/// residual carrier error is small next to what the payload CRC can survive,
/// narrow enough to add only a dozen-odd candidates.
const FINE_STEP_HZ: f64 = 25.0;
const FINE_SPAN_HZ: f64 = 250.0;

/// Welch power spectrum of `iq`: a Blackman–Harris window, 50 % overlap, the
/// mean of every whole segment's periodogram, left in natural FFT order.
fn welch_psd(iq: &[Complex32], nfft: usize) -> Vec<f32> {
    let fft = FftPlanner::<f32>::new().plan_fft_forward(nfft);
    let win = sdroxide_dsp::blackman_harris(nfft);
    let mut acc = vec![0.0f32; nfft];
    let mut seg = vec![Complex32::new(0.0, 0.0); nfft];
    let hop = (nfft / 2).max(1);
    let mut segments = 0u32;
    let mut start = 0usize;
    while start + nfft <= iq.len() {
        for (s, (&x, &w)) in seg.iter_mut().zip(iq[start..start + nfft].iter().zip(&win)) {
            *s = x * w;
        }
        fft.process(&mut seg);
        for (a, s) in acc.iter_mut().zip(&seg) {
            *a += s.norm_sqr();
        }
        segments += 1;
        start += hop;
    }
    if segments > 0 {
        let inv = 1.0 / segments as f32;
        acc.iter_mut().for_each(|v| *v *= inv);
    }
    acc
}

/// Where the beacon's carrier sits and how convincingly the spectrum said so.
/// `hz` is relative to the assumed centre (DC).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CarrierEstimate {
    pub hz: f64,
    /// Depth of the central null between the two lobes, dB.
    pub null_depth_db: f32,
    /// Left/right evenness of the two lobes, 0..1.
    pub symmetry: f32,
    /// Lobe level over the scanned band's noise floor, dB.
    pub snr_db: f32,
}

/// Scan `[lo_hz, hi_hz]` of `iq`'s power spectrum for the beacon's give-away
/// shape and return the best match, or `None` when nothing in the band looks
/// like it.
///
/// The beacon is DBPSK + Manchester at 400 baud, so it shows not as a carrier
/// peak but as a *pair* of lobes with a null between them at the carrier and
/// another null near ±[`CHIP_RATE`] Hz further out (see the module doc). Every
/// candidate centre is scored by exactly that shape — two shoulders up, a
/// notch in the middle, quiet past the outer nulls, left/right symmetric — and
/// the best is returned only if it clears every part of that test by a margin.
/// A bare carrier, an SSB signal or noise all fail it. Anything in the band
/// that is *not* two symmetric lobes is simply not what wins.
pub(crate) fn estimate_carrier(
    iq: &[Complex32],
    rate_hz: f64,
    lo_hz: f64,
    hi_hz: f64,
) -> Option<CarrierEstimate> {
    if rate_hz <= 0.0 || hi_hz - lo_hz < 1_500.0 {
        return None;
    }
    // ~15–25 Hz bins: fine enough to resolve the ±CHIP_RATE null structure,
    // coarse enough that even a short buffer still averages many segments.
    let nfft = ((rate_hz / 22.0).round() as usize).clamp(512, 32_768).next_power_of_two();
    if iq.len() < nfft * 2 {
        return None;
    }
    let psd = welch_psd(iq, nfft);
    let bin_hz = rate_hz / nfft as f64;
    let edge = rate_hz * 0.47;
    let (lo, hi) = (lo_hz.max(-edge), hi_hz.min(edge));

    // Linear PSD at a signed frequency, nearest bin, wrapping natural FFT
    // order; `None` past the transform's own edge.
    let at = |f: f64| -> Option<f32> {
        let k = (f / bin_hz).round() as i64;
        let k = if k < 0 { k + nfft as i64 } else { k };
        (0..nfft as i64).contains(&k).then(|| psd[k as usize])
    };
    // Mean linear PSD over `c ± [lo, hi]`, both sides together.
    let band_mean = |c: f64, blo: f64, bhi: f64| -> Option<f32> {
        let (mut sum, mut n) = (0.0f32, 0u32);
        let mut d = blo;
        while d <= bhi {
            if let (Some(a), Some(b)) = (at(c + d), at(c - d)) {
                sum += a + b;
                n += 2;
            }
            d += bin_hz;
        }
        (n > 0).then(|| sum / n as f32)
    };

    // A robust noise floor for the SNR gate: the median across the scanned
    // band, which the beacon's own two lobes cannot lift far.
    let mut span: Vec<f32> = Vec::new();
    let mut f = lo;
    while f <= hi {
        if let Some(p) = at(f) {
            span.push(p);
        }
        f += bin_hz;
    }
    if span.len() < 32 {
        return None;
    }
    span.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let floor = span[span.len() / 2].max(1e-20);
    let db = |x: f32| 10.0 * (x.max(1e-20) as f64).log10();

    let mut best: Option<(f64, CarrierEstimate)> = None; // (score, est)
    let mut fc = lo;
    while fc <= hi {
        if let (Some(lobe), Some(notch), Some(outer)) = (
            band_mean(fc, 200.0, 600.0),
            band_mean(fc, 0.0, 60.0),
            band_mean(fc, CHIP_RATE + 200.0, CHIP_RATE + 700.0),
        ) {
            let null_depth = db(lobe) - db(notch);
            let lobe_excess = db(lobe) - db(outer);
            let snr = db(lobe) - db(floor);

            // Left/right evenness across the lobe band, 1.0 = perfectly even.
            let (mut asym, mut m) = (0.0f64, 0u32);
            let mut d = 200.0;
            while d <= 600.0 {
                if let (Some(a), Some(b)) = (at(fc + d), at(fc - d)) {
                    let (a, b) = (a as f64, b as f64);
                    if a + b > 0.0 {
                        asym += (a - b).abs() / (a + b);
                        m += 1;
                    }
                }
                d += bin_hz;
            }
            let sym = if m > 0 { 1.0 - asym / m as f64 } else { 0.0 };

            if null_depth >= 2.5 && lobe_excess >= 3.0 && snr >= 3.0 && sym >= 0.55 {
                let score =
                    null_depth.min(12.0) + lobe_excess.min(12.0) + snr.min(12.0) + 8.0 * sym;
                if best.is_none_or(|(s, _)| score > s) {
                    best = Some((
                        score,
                        CarrierEstimate {
                            hz: fc,
                            null_depth_db: null_depth as f32,
                            symmetry: sym as f32,
                            snr_db: snr as f32,
                        },
                    ));
                }
            }
        }
        fc += bin_hz;
    }
    best.map(|(_, e)| e)
}

/// A coarse carrier estimate across `±search_half_width_hz` (plus a little), in
/// Hz relative to the assumed centre — the seed [`acquire`] starts its CRC
/// sweep from. `None` when nothing beacon-shaped is in range.
pub(crate) fn coarse_carrier_hz(
    iq: &[Complex32],
    rate_hz: f64,
    search_half_width_hz: f64,
) -> Option<f64> {
    if search_half_width_hz <= 0.0 {
        return None;
    }
    let reach = (search_half_width_hz + 1_500.0).min(rate_hz * 0.47);
    estimate_carrier(iq, rate_hz, -reach, reach).map(|e| e.hz)
}

/// Search `iq` (complex baseband, `rate_hz` samples/s, the beacon assumed to
/// sit somewhere within `±search_half_width_hz` of DC) for one CRC-valid
/// AO-40 uncoded frame, stepping the candidate frequency by `freq_step_hz`.
/// The frequency reported is refined well past that step's own resolution —
/// see [`refine_offset_hz`] — so `freq_step_hz` only needs to be fine enough
/// to land *somewhere* inside a real signal's capture range, not to measure
/// it.
///
/// Each candidate is mixed down to `demod_rate_hz` (a fixed ~16 kHz — all the
/// 400 baud beacon ever needs) *before* the chip search runs, no matter how
/// wide `rate_hz` made the capture. Without that step the per-candidate work
/// scaled with the capture rate while the candidate count scaled with the
/// search width, so the total grew with the *square* of the width and the
/// widest settings ran many times slower than real time.
///
/// When [`coarse_carrier_hz`] finds the beacon's shape, the sweep is a dense
/// grid on that seed alone — the spectrum has already said where the carrier
/// is, and the coarse grid behind it would be hundreds of full-buffer
/// mix-downs re-answering the same question. A seed past
/// `search_half_width_hz` is still followed out to it (capped at
/// [`ACQ_RANGE_MAX_HZ`] and Nyquist), which is what lets an uncalibrated
/// station be found at all. With no seed the old centre-outward walk of the
/// whole grid runs unchanged.
/// `cancel` is polled between candidates so the engine can drop the
/// controller (turning the decoder off, or changing the search width) without
/// waiting out a search in progress.
///
/// `iq` needs to span at least one whole frame (`FRAME_BITS` bits plus the
/// sync word, [`BAUD`] bits/s) for a frame to have any chance of falling
/// inside it whole; the caller — [`crate::controller::Qo100Controller`] —
/// keeps a rolling window comfortably longer than that so no alignment of the
/// buffer against the frame can miss one.
pub fn acquire(
    iq: &[Complex32],
    rate_hz: f64,
    search_half_width_hz: f64,
    freq_step_hz: f64,
    demod_rate_hz: f64,
    cancel: &std::sync::atomic::AtomicBool,
) -> Option<Qo100Lock> {
    use std::sync::atomic::Ordering;
    if freq_step_hz <= 0.0 || rate_hz <= 0.0 || demod_rate_hz <= 0.0 {
        return None;
    }
    let deci = (rate_hz / demod_rate_hz).round().max(1.0) as usize;
    let dr = rate_hz / deci as f64;

    // The spectral seed, and how far it lets the grid reach: never narrower
    // than the configured half-width, no wider than a confident seed needs
    // (plus a margin), and never past a sane cap or Nyquist.
    let seed_hz = coarse_carrier_hz(iq, rate_hz, search_half_width_hz);
    let reach_hz = match seed_hz {
        Some(s) => (s.abs() + SEED_MARGIN_HZ)
            .min(ACQ_RANGE_MAX_HZ)
            .min(rate_hz * 0.47)
            .max(search_half_width_hz),
        None => search_half_width_hz,
    };
    let steps = (reach_hz / freq_step_hz).round().max(0.0) as i64;
    let seed_step = seed_hz.map(|s| (s / freq_step_hz).round() as i64).unwrap_or(0);

    // The candidate mix-down frequencies, in order.
    //
    // First — when the spectrum gave a confident seed — a *dense* sweep right
    // on it: the coarse `freq_step_hz` grid leaves a residual carrier error of
    // up to half a step, which the chip detector tolerates for the sync word
    // but which lifts the payload bit-error rate enough that the CRC over 514
    // bytes almost never checks out. A few Hz of residual instead of ~75 is
    // what turns "carrier + sync, no CRC" into a decode.
    let mut freqs: Vec<f64> = Vec::new();
    if let Some(s) = seed_hz {
        let fine = (FINE_SPAN_HZ / FINE_STEP_HZ).round() as i64;
        for k in 0..=2 * fine {
            let d = if k % 2 == 0 { k / 2 } else { -(k / 2 + 1) };
            freqs.push(s + d as f64 * FINE_STEP_HZ);
        }
    }
    // With no seed, the full coarse grid: `0, +1, -1, +2, -2, …` from DC, the
    // plain centre-out sweep this function has always made.
    //
    // With a seed, that grid is *not* walked. The seed is a carrier estimate
    // good to a bin — ~15 Hz at the rates the engine uses, and the estimator's
    // own tests hold it inside a few hundred — so the dense grid above already
    // brackets its error, and the several hundred coarse candidates behind it
    // are several hundred full-buffer mix-downs looking for a carrier the
    // spectrum has already placed. Skipping them is what brings the drift
    // search in [`acquire_debug`] inside the window it has to run in: the
    // sweep goes from ~335 candidates to 21, and the price is that a seed the
    // shape test got wrong is no longer caught by a sweep behind it. The gates
    // in [`estimate_carrier`] are what that price is paid against — a bare
    // carrier, an SSB signal and noise all fail them (see its tests).
    if seed_hz.is_none() {
        let mut coarse: Vec<i64> = (-steps..=steps).collect();
        coarse.sort_by_key(|&s| ((s - seed_step).unsigned_abs(), s < seed_step));
        freqs.extend(coarse.iter().map(|&s| s as f64 * freq_step_hz));
    }

    for coarse_hz in freqs {
        if cancel.load(Ordering::Relaxed) {
            return None;
        }
        let mixed = mix_decimate(iq, rate_hz, coarse_hz, deci);
        if let Some(cm) = try_frequency(&mixed, dr) {
            let offset_hz = coarse_hz + refine_offset_hz(&mixed, dr, &cm);
            return Some(Qo100Lock { offset_hz, text: cm.m.text });
        }
    }
    None
}

/// How far the last [`acquire_debug`] pass got, for the operator's
/// step-by-step decoder readout. None of it gates anything — pure
/// instrumentation, so a missing stage points straight at where the chain
/// breaks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DecodeProgress {
    /// The pass had chip-rate structure to work on — a twin-lobe estimate
    /// landed, or the sync correlator found something close.
    pub carrier: bool,
    /// The 32-bit AO-40 sync word matched within [`SYNC_MAX_ERRORS`] somewhere.
    pub sync: bool,
    /// Fewest sync-word bit errors found, 0..=32; [`u8::MAX`] if no bitstream
    /// was run at all.
    pub sync_bit_errors: u8,
    /// How many separate positions in the best chip-timing/parity bitstream
    /// matched the sync word within [`SYNC_MAX_ERRORS`]. A real frame in the
    /// window shows one or two, at very few errors; a 32-bit pattern matched
    /// within 3 errors also turns up by chance roughly once every few windows
    /// on noise, as a lone hit near the full error budget — so this is what
    /// tells "sync lit, CRC dark" apart from a genuine frame alignment the
    /// payload demod is then failing.
    pub sync_matches: u8,
    /// A CRC-valid frame was decoded.
    pub crc_ok: bool,
}

impl Default for DecodeProgress {
    fn default() -> Self {
        Self {
            carrier: false,
            sync: false,
            sync_bit_errors: u8::MAX,
            sync_matches: 0,
            crc_ok: false,
        }
    }
}

/// Fewest bit errors between [`SYNC_WORD`] and any 32-bit window of `bits`.
fn min_sync_distance(bits: &[bool]) -> Option<u8> {
    if bits.len() < SYNC_LEN as usize {
        return None;
    }
    let mut window: u32 = 0;
    let mut best = SYNC_LEN;
    for (i, &b) in bits.iter().enumerate() {
        window = (window << 1) | b as u32;
        if i + 1 >= SYNC_LEN as usize {
            best = best.min((window ^ SYNC_WORD).count_ones());
        }
    }
    Some(best as u8)
}

/// Every bit offset in `bits` where [`SYNC_WORD`] matches within
/// [`SYNC_MAX_ERRORS`], collapsing a run of near-adjacent hits (one real sync
/// preamble can match at a couple of neighbouring offsets when a chip slips)
/// into a single count — so the length of the result is "how many distinct
/// frame alignments look present", the number [`DecodeProgress::sync_matches`]
/// reports.
fn sync_positions(bits: &[bool]) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    if bits.len() < SYNC_LEN as usize {
        return out;
    }
    let mut window: u32 = 0;
    for (i, &b) in bits.iter().enumerate() {
        window = (window << 1) | b as u32;
        if i + 1 >= SYNC_LEN as usize && (window ^ SYNC_WORD).count_ones() <= SYNC_MAX_ERRORS {
            let pos = i + 1 - SYNC_LEN as usize;
            if out.last().is_none_or(|&p| pos - p > 4) {
                out.push(pos);
            }
        }
    }
    out
}

/// Across every chip-timing phase and Manchester parity of `iq` (already mixed
/// to one candidate): the fewest sync-word bit errors any of them reaches, and
/// how many distinct sync matches the bitstream that reached it carries. A
/// "how close was this, and is it one real frame or a chance hit" that does
/// not need a CRC pass to mean something.
fn sync_scan(iq: &[Complex32], rate_hz: f64) -> Option<(u8, u8)> {
    let samples_per_chip = rate_hz / CHIP_RATE;
    if samples_per_chip < 2.0 {
        return None;
    }
    let phase_step = (samples_per_chip / TIMING_PHASES as f64).max(1.0);
    let mut best: Option<(u8, u8)> = None;
    for p in 0..TIMING_PHASES {
        let phase = (p as f64 * phase_step) as usize;
        let chips = chip_samples(iq, samples_per_chip, phase);
        if chips.len() < 3 {
            continue;
        }
        let flips = chip_flips(&chips);
        for parity in 0..2 {
            let bits = data_bits(&flips, parity);
            let Some(d) = min_sync_distance(&bits) else { continue };
            if best.is_none_or(|(bd, _)| d < bd) {
                let n = sync_positions(&bits).len().min(u8::MAX as usize) as u8;
                best = Some((d, n));
            }
        }
    }
    best
}

/// De-rotate a frequency drift of `slope` Hz/s plus curvature `accel` Hz/s²
/// out of `iq` (sample rate `rate_hz`), pivoting on the centre sample so only
/// the *shape* of the walk is removed — the mean offset is left for
/// [`acquire`]'s own grid to find. This is what lets a frame decode on a
/// station whose LNB (or SDR clock) is still walking while it warms up:
/// without it the carrier smears across the 10.36 s frame and the payload CRC
/// never checks out even though the sync word matches.
///
/// A warming LNB's LO does not drift at a *constant* rate — it curves — so the
/// `accel` term earns its place over a whole frame even when the last second
/// of tracker estimates looked linear; a straight-line de-rotation leaves a
/// residual chirp that is still enough to fail the payload CRC.
fn dechirp(iq: &[Complex32], rate_hz: f64, slope: f64, accel: f64) -> Vec<Complex32> {
    if (slope == 0.0 && accel == 0.0) || rate_hz <= 0.0 {
        return iq.to_vec();
    }
    // Instantaneous frequency `slope * t + 0.5 * accel * t^2` integrates to
    // phase `2*PI * (0.5 * slope * t^2 + accel * t^3 / 6)`; `t = (n - n0) /
    // rate`. Multiply by its conjugate.
    let n0 = iq.len() as f64 / 2.0;
    let ks = -std::f64::consts::PI * slope / (rate_hz * rate_hz);
    let ka = -std::f64::consts::PI * accel / (3.0 * rate_hz * rate_hz * rate_hz);
    iq.iter()
        .enumerate()
        .map(|(n, &z)| {
            let d = n as f64 - n0;
            let ph = ks * d * d + ka * d * d * d;
            z * Complex32::new(ph.cos() as f32, ph.sin() as f32)
        })
        .collect()
}

/// Half-width and step, in Hz/s, of the drift-*rate* grid [`acquire_debug`]
/// tries around the caller's estimate. Wide enough to bracket a warming LNB,
/// fine enough that the residual chirp across a frame is a fraction of a bit.
const DRIFT_GRID_HALF_HZ_S: f64 = 6.0;
const DRIFT_GRID_STEP_HZ_S: f64 = 1.5;

/// Half-width and step, in Hz/s², of the *curvature* grid tried around the
/// caller's estimate. `0.0` is always tried first (centre-out), so a station
/// whose drift really is linear pays exactly the sweep it did before this
/// second-order term existed; widen the half-width here if a fast-warming LNB
/// still will not decode and `DRIFT` shows a large curvature.
const ACCEL_GRID_HALF_HZ_S2: f64 = 3.0;
const ACCEL_GRID_STEP_HZ_S2: f64 = 3.0;

/// [`acquire`], plus a [`DecodeProgress`] snapshot of how far the pass got and
/// a small **drift** search around `drift_hz_per_s` / `drift_accel_hz_s2`: the
/// frame decoder needs the carrier coherent across a whole 10.36 s frame,
/// which a drifting — and, as it warms, *accelerating* — LNB breaks, so each
/// candidate is de-rotated ([`dechirp`]) with a trial drift rate and curvature
/// before the ordinary frequency sweep runs on it. The two arguments are the
/// caller's own estimates (0.0 = "no idea"); the grid is centred on them, with
/// zero curvature tried first, so a good estimate keeps the search short and a
/// linearly-drifting station pays what it did before the curvature term.
///
/// The grid runs only when the spectrum can still see the beacon's twin-lobe
/// shape in the buffer de-rotated by the caller's estimate. A window with no
/// beacon in it gets one plain sweep instead of the whole grid — the
/// difference between a decoder that keeps up with the air and one that runs
/// an order of magnitude behind it, since a window that decodes nothing is
/// the case the grid would otherwise walk in full every time.
#[allow(clippy::too_many_arguments)]
pub(crate) fn acquire_debug(
    iq: &[Complex32],
    rate_hz: f64,
    search_half_width_hz: f64,
    freq_step_hz: f64,
    demod_rate_hz: f64,
    drift_hz_per_s: f64,
    drift_accel_hz_s2: f64,
    cancel: &std::sync::atomic::AtomicBool,
) -> (Option<Qo100Lock>, DecodeProgress) {
    use std::sync::atomic::Ordering;

    // Is there a beacon in this window at all?
    //
    // Asked once, of the buffer de-rotated by the caller's own estimate — the
    // centre of the grid below, and so the de-rotation most likely to leave
    // the shape standing. It decides whether the grid runs, because every one
    // of its points costs a whole [`acquire`] sweep and a drift search over a
    // window holding no beacon is that cost multiplied by the size of the
    // grid, spent looking for something that is not there. A station whose
    // beacon the shape test cannot see is not left worse off than it was
    // before the grid existed: it gets the single plain sweep below.
    let centred = dechirp(iq, rate_hz, drift_hz_per_s, drift_accel_hz_s2);
    let seeded = coarse_carrier_hz(&centred, rate_hz, search_half_width_hz).is_some();

    // Curvature outer, drift-rate inner — both centre-outward from the
    // estimate, so (zero curvature, best rate) is tried first.
    let s_steps =
        if seeded { (DRIFT_GRID_HALF_HZ_S / DRIFT_GRID_STEP_HZ_S).round() as i64 } else { 0 };
    let a_steps =
        if seeded { (ACCEL_GRID_HALF_HZ_S2 / ACCEL_GRID_STEP_HZ_S2).round() as i64 } else { 0 };
    'grid: for ka in 0..=2 * a_steps {
        let da = if ka % 2 == 0 { ka / 2 } else { -(ka / 2 + 1) };
        let accel = drift_accel_hz_s2 + da as f64 * ACCEL_GRID_STEP_HZ_S2;
        for ks in 0..=2 * s_steps {
            if cancel.load(Ordering::Relaxed) {
                break 'grid;
            }
            let ds = if ks % 2 == 0 { ks / 2 } else { -(ks / 2 + 1) };
            let slope = drift_hz_per_s + ds as f64 * DRIFT_GRID_STEP_HZ_S;
            // Unseeded, the grid is one point and the estimate behind it is
            // unconfirmed, so the sweep runs on the buffer as it arrived.
            let buf = if seeded { dechirp(iq, rate_hz, slope, accel) } else { iq.to_vec() };
            if let Some(lock) =
                acquire(&buf, rate_hz, search_half_width_hz, freq_step_hz, demod_rate_hz, cancel)
            {
                return (
                    Some(lock),
                    DecodeProgress {
                        carrier: true,
                        sync: true,
                        sync_bit_errors: 0,
                        sync_matches: 1,
                        crc_ok: true,
                    },
                );
            }
        }
    }

    // Nothing decoded at any drift — probe the un-dechirped buffer for how far
    // it got, so the readout still says something.
    let mut progress = DecodeProgress::default();
    if rate_hz > 0.0 && demod_rate_hz > 0.0 {
        let deci = (rate_hz / demod_rate_hz).round().max(1.0) as usize;
        let dr = rate_hz / deci as f64;
        let seed = coarse_carrier_hz(iq, rate_hz, search_half_width_hz);
        let mixed = mix_decimate(iq, rate_hz, seed.unwrap_or(0.0), deci);
        if let Some((d, n)) = sync_scan(&mixed, dr) {
            progress.sync_bit_errors = d;
            progress.sync = u32::from(d) <= SYNC_MAX_ERRORS;
            progress.sync_matches = n;
        }
        progress.carrier = seed.is_some()
            || progress.sync
            || (progress.sync_bit_errors != u8::MAX && progress.sync_bit_errors <= 10);
    }
    (None, progress)
}

// `pub(crate)` so `controller`'s own test module can build a real on-air frame
// through `synth_signal` without a second copy of the synthesis living there.
#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// xorshift64* — cheap, deterministic, good enough for test noise and
    /// jitter. The same technique `sdroxide_radio::source::SigGenSource`
    /// uses for its own noise floor, kept local here rather than pulling in
    /// a `rand` dependency no other first-party crate in this workspace
    /// carries.
    struct TestRng(u64);

    impl TestRng {
        fn new(seed: u64) -> Self {
            Self(seed.max(1))
        }

        /// Uniform in `[0, 1)`.
        fn next_unit(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
        }

        /// Uniform in `[lo, hi)`.
        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_unit() * (hi - lo)
        }
    }

    #[test]
    fn crc16_ccitt_false_matches_the_published_check_value() {
        // The catalogue check value for CRC-16/CCITT-FALSE: CRC of the ASCII
        // string "123456789".
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29B1);
    }

    fn pack_msb(bits: &[bool]) -> Vec<u8> {
        pack_bits(bits)
    }

    fn unpack_msb(bytes: &[u8]) -> Vec<bool> {
        bytes.iter().flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 != 0)).collect()
    }

    /// Build one complete on-air AO-40 uncoded frame (sync + payload + CRC),
    /// as data bits — before differential/Manchester encoding.
    fn build_frame_bits(text: &str) -> Vec<bool> {
        let mut payload = text.as_bytes().to_vec();
        payload.resize(PAYLOAD_BYTES, b' ');
        let crc = crc16_ccitt_false(&payload);
        let mut frame = payload;
        frame.extend_from_slice(&crc.to_be_bytes());
        let sync_bits: Vec<bool> = (0..SYNC_LEN).rev().map(|i| (SYNC_WORD >> i) & 1 != 0).collect();
        let mut bits = sync_bits;
        bits.extend(unpack_msb(&frame));
        bits
    }

    /// Synthesize `rate_hz` samples/s of complex baseband carrying `text` as
    /// an AO-40 uncoded frame, with some quiet chips either side (so a
    /// frame boundary lands away from the buffer's own edges, like it would
    /// in a continuously-transmitting real signal sliced at an arbitrary
    /// time), a constant frequency offset, a random starting phase and chip
    /// timing offset, and light noise.
    pub(crate) fn synth_signal(
        text: &str,
        rate_hz: f64,
        offset_hz: f64,
        noise: f32,
        seed: u64,
    ) -> Vec<Complex32> {
        let mut rng = TestRng::new(seed);
        let data_bits = build_frame_bits(text);
        let e = differential_encode(&data_bits);
        let chips: Vec<bool> = manchester_chips(&e);

        let samples_per_chip = rate_hz / CHIP_RATE;
        let lead_chips = 40usize; // quiet-ish run before the frame, like real air time
        let total_chips = lead_chips + chips.len() + 10;
        let start_phase: f64 = rng.range(0.0, std::f64::consts::TAU);
        let sub_chip_offset: f64 = rng.range(0.0, 1.0); // sub-sample timing error

        let mut out = Vec::with_capacity((total_chips as f64 * samples_per_chip) as usize + 8);
        let mut n = 0usize;
        for c in 0..total_chips {
            let bit = if c < lead_chips || c >= lead_chips + chips.len() {
                // Filler chips outside the frame — a real receiver sees the
                // *previous* frame's tail and the *next* one's head here, not
                // silence, so this is the more honest test. Pseudo-random
                // rather than a fixed pattern: a periodic filler is itself
                // periodic Manchester data and can correlate with the sync
                // word by construction, not by chance — exactly what this is
                // supposed to rule out.
                rng.range(0.0, 1.0) < 0.5
            } else {
                chips[c - lead_chips]
            };
            let sym = if bit { -1.0f32 } else { 1.0f32 };
            let n_this_chip = (((c + 1) as f64 + sub_chip_offset) * samples_per_chip) as usize - n;
            for _ in 0..n_this_chip {
                let carrier_phase =
                    start_phase + std::f64::consts::TAU * offset_hz * n as f64 / rate_hz;
                let noise_c = Complex32::new(
                    rng.range(-1.0, 1.0) as f32 * noise,
                    rng.range(-1.0, 1.0) as f32 * noise,
                );
                out.push(
                    Complex32::new(
                        sym * carrier_phase.cos() as f32,
                        sym * carrier_phase.sin() as f32,
                    ) + noise_c,
                );
                n += 1;
            }
        }
        out
    }

    const TEST_RATE: f64 = 16_000.0;
    const TEST_STEP: f64 = 150.0;

    /// `acquire` with the production demod rate and no cancellation, so the
    /// tests exercise exactly the mix-down-then-search path the worker uses.
    fn acq(iq: &[Complex32], rate_hz: f64, half_width_hz: f64) -> Option<Qo100Lock> {
        acquire(
            iq,
            rate_hz,
            half_width_hz,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            &std::sync::atomic::AtomicBool::new(false),
        )
    }

    #[test]
    fn a_clean_frame_at_zero_offset_decodes_and_reports_no_drift() {
        let iq = synth_signal("QO-100 TEST TELEMETRY LINE ONE", TEST_RATE, 0.0, 0.0, 1);
        let lock = acq(&iq, TEST_RATE, 50.0).expect("should lock");
        // Refined well past the coarse search grid — see `refine_offset_hz`.
        assert!(lock.offset_hz.abs() < 1.0, "found {}", lock.offset_hz);
        assert!(lock.text.starts_with("QO-100 TEST TELEMETRY LINE ONE"), "{:?}", lock.text);
    }

    /// The whole point of the feature: a beacon that is not exactly where the
    /// dial assumes it is still gets found, and the frequency the search
    /// lands on *is* the calibration answer — refined well past the coarse
    /// search grid's own step, not just "the nearest step".
    #[test]
    fn a_drifted_frame_is_found_and_the_drift_is_reported() {
        for &true_offset in &[37.0f64, -68.0, 91.0] {
            let iq = synth_signal("DRIFT CASE", TEST_RATE, true_offset, 0.02, 2);
            let lock = acq(&iq, TEST_RATE, 150.0)
                .unwrap_or_else(|| panic!("should lock at offset {true_offset}"));
            assert!(
                (lock.offset_hz - true_offset).abs() <= 1.0,
                "true {true_offset}, found {}",
                lock.offset_hz
            );
            assert!(lock.text.starts_with("DRIFT CASE"));
        }
    }

    #[test]
    fn noise_with_no_signal_never_reports_a_lock() {
        let mut rng = TestRng::new(3);
        let n = (TEST_RATE * 12.0) as usize;
        let iq: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32))
            .collect();
        assert!(acq(&iq, TEST_RATE, 100.0).is_none());
    }

    /// The cost regression guard: a search at a *realistic* capture rate and
    /// width — the engine's default ±5 kHz, captured at 16 kHz — still finds
    /// the beacon, and the mix-down-per-candidate path keeps the sweep short
    /// enough to matter (the earlier code searched every candidate at the
    /// full capture rate and this width was already seconds of work). Every
    /// other test runs a ±50–150 Hz search, which is why the blow-up went
    /// unnoticed.
    #[test]
    fn a_default_width_search_at_a_realistic_rate_still_locks() {
        // ±5 kHz search wants a capture a little over 2.5× wide — the same
        // rule `Engine::qo100_target_rate_hz` follows.
        let rate = 12_500.0f64.max(16_000.0);
        let iq = synth_signal("REALISTIC WIDTH", rate, 3_200.0, 0.02, 7);
        let started = std::time::Instant::now();
        let lock = acq(&iq, rate, 5_000.0).expect("should still lock at the default width");
        assert!((lock.offset_hz - 3_200.0).abs() <= 2.0, "found {}", lock.offset_hz);
        assert!(lock.text.starts_with("REALISTIC WIDTH"));
        assert!(
            started.elapsed().as_secs() < 5,
            "default-width search took {:?} — the per-candidate cost has regressed",
            started.elapsed()
        );
    }

    /// A cancelled search returns without walking the grid.
    #[test]
    fn a_cancelled_search_bails_out() {
        let iq = synth_signal("CANCELLED", TEST_RATE, 40.0, 0.0, 1);
        let cancel = std::sync::atomic::AtomicBool::new(true);
        assert!(
            acquire(&iq, TEST_RATE, 20_000.0, TEST_STEP, crate::controller::DEMOD_RATE_HZ, &cancel)
                .is_none()
        );
    }

    #[test]
    fn a_corrupted_payload_bit_is_refused_by_the_crc() {
        let clean = build_frame_bits("SHOULD APPEAR");
        assert!(decode_frame(&clean).is_some(), "the undamaged frame must decode");

        let mut corrupted = clean.clone();
        // Flip a bit well inside the payload, away from the sync word.
        let i = SYNC_LEN as usize + 100;
        corrupted[i] = !corrupted[i];
        assert!(decode_frame(&corrupted).is_none(), "a single flipped payload bit must fail CRC");
    }

    #[test]
    fn find_sync_tolerates_a_few_bit_errors_but_not_many() {
        let sync_bits: Vec<bool> = (0..SYNC_LEN).rev().map(|i| (SYNC_WORD >> i) & 1 != 0).collect();
        let mut noisy = sync_bits.clone();
        noisy[3] = !noisy[3];
        noisy[10] = !noisy[10];
        noisy[20] = !noisy[20];
        assert_eq!(find_sync(&noisy, 0), Some(0), "3 errors is within threshold");
        let mut too_noisy = noisy.clone();
        too_noisy[15] = !too_noisy[15];
        assert_eq!(find_sync(&too_noisy, 0), None, "4 errors is not");
    }

    #[test]
    fn pack_and_unpack_bits_round_trip() {
        let bytes = [0x5Au8, 0x00, 0xFF, 0x81];
        assert_eq!(pack_msb(&unpack_msb(&bytes)), bytes);
    }

    /// The chip pipeline in complete isolation — one sample per chip, no
    /// noise, no frequency or timing offset — pins down whether [`data_bits`]
    /// really is the inverse of encode-then-Manchester for *some* parity
    /// (bit 0 of the source has no predecessor to compare against, so the
    /// match is against `want[1..]`), independent of the search/synthesis
    /// machinery built on top of it.
    #[test]
    fn chip_pipeline_round_trips_at_unit_oversampling() {
        let want = build_frame_bits("ROUND TRIP CHECK");
        let chip_bits = source_chips(&want);
        let chips: Vec<Complex32> =
            chip_bits.iter().map(|&b| Complex32::new(if b { -1.0 } else { 1.0 }, 0.0)).collect();
        let flips = chip_flips(&chips);
        let ok = (0..2).any(|parity| {
            let got = data_bits(&flips, parity);
            got.len() >= want.len() - 1 && got[..want.len() - 1] == want[1..]
        });
        assert!(ok, "neither parity reconstructed the source bits");
    }

    /// The demodulator itself (no search) tolerates a real residual frequency
    /// error, not just an exact match — otherwise the coarse search grid in
    /// [`acquire`] would have to be implausibly fine to ever land inside a
    /// real signal's capture window at all.
    #[test]
    fn the_demod_tolerates_a_realistic_residual_frequency_error_unaided() {
        let iq = synth_signal("CAPTURE RANGE", TEST_RATE, 100.0, 0.0, 1);
        assert!(try_frequency(&iq, TEST_RATE).is_some());
    }

    /// Stack `copies` of one synthesized frame end to end — a longer buffer so
    /// [`welch_psd`] has several segments to average, like the worker's
    /// rolling window gives it.
    fn stacked(
        text: &str,
        rate_hz: f64,
        offset_hz: f64,
        noise: f32,
        seed: u64,
        copies: usize,
    ) -> Vec<Complex32> {
        let one = synth_signal(text, rate_hz, offset_hz, noise, seed);
        let mut out = Vec::with_capacity(one.len() * copies);
        for _ in 0..copies {
            out.extend_from_slice(&one);
        }
        out
    }

    #[test]
    fn the_coarse_estimator_reads_the_carrier_off_the_twin_lobes() {
        // A real Manchester beacon 4 kHz off centre, captured wide enough to
        // see both lobes and a stretch of clean floor either side.
        let iq = stacked("COARSE EST", 20_000.0, 4_000.0, 0.05, 7, 3);
        let est = coarse_carrier_hz(&iq, 20_000.0, 8_000.0).expect("the twin lobes should be seen");
        assert!((est - 4_000.0).abs() <= 200.0, "estimated {est}");
    }

    #[test]
    fn the_coarse_estimator_declines_on_noise_and_on_a_bare_carrier() {
        let n = (20_000.0f64 * 6.0) as usize;

        let mut rng = TestRng::new(41);
        let noise: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32))
            .collect();
        assert!(coarse_carrier_hz(&noise, 20_000.0, 8_000.0).is_none(), "noise is not a beacon");

        // A plain unmodulated carrier 3 kHz off: a peak, not two lobes with a
        // null — must be rejected, or it would seed the search wrong.
        let tone: Vec<Complex32> = (0..n)
            .map(|i| {
                let ph = std::f64::consts::TAU * 3_000.0 * i as f64 / 20_000.0;
                Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        assert!(
            coarse_carrier_hz(&tone, 20_000.0, 8_000.0).is_none(),
            "a bare carrier is not a beacon"
        );
    }

    /// The acquisition-range win: a beacon sitting outside the configured
    /// half-width is out of reach of the plain centre-out sweep, but the
    /// spectral seed pulls the grid out to it and it still locks.
    #[test]
    fn a_seed_pulls_the_search_past_the_configured_half_width() {
        // ±8 kHz width, captured 2.5× wide (the engine's rule); beacon at
        // 9 kHz — 1 kHz beyond the width, well inside the capture.
        let rate = 20_000.0;
        let iq = stacked("OUT OF WIDTH", rate, 9_000.0, 0.02, 11, 3);
        let lock = acq(&iq, rate, 8_000.0).expect("the seed should extend the reach to it");
        assert!((lock.offset_hz - 9_000.0).abs() <= 2.0, "found {}", lock.offset_hz);
        assert!(lock.text.starts_with("OUT OF WIDTH"), "{:?}", lock.text);
    }

    #[test]
    fn the_coarse_estimator_needs_a_buffer_worth_averaging() {
        // Fewer than two FFT segments' worth of samples: no estimate at all,
        // so `acquire` just falls back to the plain centre-out sweep.
        let short = synth_signal("TOO SHORT", TEST_RATE, 40.0, 0.0, 1);
        assert!(coarse_carrier_hz(&short[..1024], TEST_RATE, 150.0).is_none());
    }

    /// A seed near the true offset must not cost the plain sweep its
    /// precision: a narrow-width search still lands on the exact drift.
    #[test]
    fn a_seed_does_not_disturb_a_narrow_width_result() {
        let iq = synth_signal("SEEDED NARROW", TEST_RATE, 91.0, 0.02, 2);
        let lock = acq(&iq, TEST_RATE, 150.0).expect("still locks");
        assert!((lock.offset_hz - 91.0).abs() <= 1.0, "found {}", lock.offset_hz);
        assert!(lock.text.starts_with("SEEDED NARROW"));
    }

    #[test]
    fn estimate_carrier_finds_a_beacon_parked_in_a_positive_window() {
        // Beacon parked at +12 kHz; a +5..+20 kHz window must find it and
        // report a healthy null and symmetry.
        let iq = stacked("PARKED", 60_000.0, 12_000.0, 0.05, 3, 3);
        let e =
            estimate_carrier(&iq, 60_000.0, 5_000.0, 20_000.0).expect("twin lobes in the window");
        assert!((e.hz - 12_000.0).abs() <= 250.0, "hz {}", e.hz);
        assert!(e.null_depth_db >= 2.5 && e.symmetry >= 0.55 && e.snr_db >= 3.0, "{e:?}");
    }

    #[test]
    fn estimate_carrier_ignores_a_beacon_outside_the_window() {
        // A real beacon at +2 kHz, but the window starts at +6 kHz.
        let iq = stacked("OUTSIDE", 60_000.0, 2_000.0, 0.02, 4, 3);
        assert!(estimate_carrier(&iq, 60_000.0, 6_000.0, 20_000.0).is_none());
    }

    #[test]
    fn acquire_debug_lights_every_stage_on_a_clean_frame() {
        let iq = synth_signal("PROGRESS", TEST_RATE, 40.0, 0.0, 1);
        let (lock, p) = acquire_debug(
            &iq,
            TEST_RATE,
            300.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            0.0,
            0.0,
            &std::sync::atomic::AtomicBool::new(false),
        );
        assert!(lock.is_some());
        assert!(p.carrier && p.sync && p.crc_ok, "{p:?}");
        assert_eq!(p.sync_bit_errors, 0);
    }

    /// Apply a frequency chirp of `slope` Hz/s plus curvature `accel` Hz/s² to
    /// `iq` — the inverse of [`dechirp`].
    fn add_chirp(iq: &[Complex32], rate_hz: f64, slope: f64, accel: f64) -> Vec<Complex32> {
        let n0 = iq.len() as f64 / 2.0;
        let ks = std::f64::consts::PI * slope / (rate_hz * rate_hz);
        let ka = std::f64::consts::PI * accel / (3.0 * rate_hz * rate_hz * rate_hz);
        iq.iter()
            .enumerate()
            .map(|(n, &z)| {
                let d = n as f64 - n0;
                let ph = ks * d * d + ka * d * d * d;
                z * Complex32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect()
    }

    #[test]
    fn a_chirped_frame_decodes_once_the_drift_is_de_rotated() {
        // A carrier walking 60 Hz/s across the frame — a warming LNB. Far past
        // what the straight chip detector tolerates, but with the tracker's
        // drift estimate as the grid centre acquire_debug recovers it.
        let rate = TEST_RATE;
        let clean = synth_signal("CHIRP CASE", rate, 0.0, 0.0, 5);
        let chirped = add_chirp(&clean, rate, 60.0, 0.0);
        let cancel = std::sync::atomic::AtomicBool::new(false);

        assert!(
            acquire(&chirped, rate, 300.0, TEST_STEP, crate::controller::DEMOD_RATE_HZ, &cancel)
                .is_none(),
            "a straight sweep cannot follow a 60 Hz/s chirp"
        );

        let (lock, p) = acquire_debug(
            &chirped,
            rate,
            300.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            60.0, // the tracker's drift estimate
            0.0,
            &cancel,
        );
        assert!(lock.is_some() && p.crc_ok, "the drift grid should recover it: {p:?}");
        assert!(lock.unwrap().text.starts_with("CHIRP CASE"));
    }

    #[test]
    fn a_curved_drift_decodes_once_the_curvature_is_de_rotated() {
        // A carrier whose drift *rate* is itself moving — a warming LNB. Zero
        // mean slope, pure curvature: near the buffer centre it looks almost
        // stationary, but the ends smear hundreds of Hz, and a symmetric curve
        // is exactly what no linear de-rotation or static offset can straighten.
        // The rate is scaled up here so a single ~10 s synth buffer stands in
        // for the decoder's real ~24 s window, where a few Hz/s² is enough.
        let rate = TEST_RATE;
        let clean = synth_signal("CURVED CASE", rate, 0.0, 0.0, 9);
        let curved = add_chirp(&clean, rate, 0.0, 40.0);
        let cancel = std::sync::atomic::AtomicBool::new(false);

        let (no_curve, _) = acquire_debug(
            &curved,
            rate,
            300.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            0.0,
            0.0, // the drift grid alone, no curvature — cannot follow it
            &cancel,
        );
        assert!(no_curve.is_none(), "a rate-only grid cannot straighten a curved drift");

        let (lock, p) = acquire_debug(
            &curved,
            rate,
            300.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            0.0,
            40.0, // the tracker's curvature estimate
            &cancel,
        );
        assert!(lock.is_some() && p.crc_ok, "the curvature grid should recover it: {p:?}");
        assert!(lock.unwrap().text.starts_with("CURVED CASE"));
    }

    #[test]
    fn dechirp_is_the_inverse_of_a_chirp() {
        let rate = TEST_RATE;
        let clean = synth_signal("INVERSE", rate, 30.0, 0.0, 2);
        let round_trip = dechirp(&add_chirp(&clean, rate, 12.0, 4.0), rate, 12.0, 4.0);
        let cancel = std::sync::atomic::AtomicBool::new(false);
        // The de-chirped copy decodes exactly like the original.
        let lock =
            acquire(&round_trip, rate, 300.0, TEST_STEP, crate::controller::DEMOD_RATE_HZ, &cancel)
                .expect("round-tripped frame still decodes");
        assert!((lock.offset_hz - 30.0).abs() <= 1.0, "offset {}", lock.offset_hz);
    }

    #[test]
    fn acquire_debug_on_noise_reports_no_crc_and_ran_a_probe() {
        let mut rng = TestRng::new(9);
        let n = (TEST_RATE * 12.0) as usize;
        let noise: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32))
            .collect();
        let (lock, p) = acquire_debug(
            &noise,
            TEST_RATE,
            300.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            0.0,
            0.0,
            &std::sync::atomic::AtomicBool::new(false),
        );
        assert!(lock.is_none());
        assert!(!p.crc_ok);
        assert_ne!(p.sync_bit_errors, u8::MAX, "the sync probe should still have run");
    }

    /// The mixing phasor advances by a complex multiply per sample rather than
    /// a `sin`/`cos` pair, which is most of what an [`acquire`] sweep costs.
    /// The recurrence has to stay on the unit circle over a buffer as long as
    /// the decoder's, or it would quietly scale the mixed signal.
    #[test]
    fn the_mixer_recurrence_stays_on_the_unit_circle_over_a_whole_window() {
        // What `mix_decimate` did before the recurrence: the phase evaluated
        // from scratch at every sample.
        fn reference(iq: &[Complex32], rate_hz: f64, shift_hz: f64, deci: usize) -> Vec<Complex32> {
            let w = -std::f64::consts::TAU * shift_hz / rate_hz;
            let mut out = Vec::new();
            let mut acc = Complex32::new(0.0, 0.0);
            let mut k = 0usize;
            for (n, &z) in iq.iter().enumerate() {
                let ph = w * n as f64;
                acc += z * Complex32::new(ph.cos() as f32, ph.sin() as f32);
                k += 1;
                if k == deci {
                    out.push(acc / deci as f32);
                    acc = Complex32::new(0.0, 0.0);
                    k = 0;
                }
            }
            out
        }

        // A buffer the length of a real decode window, so the drift the
        // recurrence could accumulate has the whole run to accumulate over.
        let rate = 62_500.0;
        let n = (rate * FRAME_SECONDS * 2.3) as usize;
        let mut rng = TestRng::new(17);
        let iq: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32))
            .collect();

        let got = mix_decimate(&iq, rate, 4_321.0, 4);
        let want = reference(&iq, rate, 4_321.0, 4);
        assert_eq!(got.len(), want.len());
        let worst = got.iter().zip(&want).map(|(a, b)| (a - b).norm()).fold(0.0f32, f32::max);
        assert!(worst < 1e-5, "worst sample differs by {worst}, out by more than rounding");

        // …and it is no further out at the end of the buffer than at the
        // start. A phasor leaving the unit circle drifts *with* the run, so
        // the tail is where it would show; measured against the reference
        // rather than against itself, so the noise the two share cancels and
        // what is left is the recurrence's own error.
        let level = |c: &[Complex32]| c.iter().map(|z| z.norm()).sum::<f32>() / c.len() as f32;
        let ratio = |r: std::ops::Range<usize>| level(&got[r.clone()]) / level(&want[r]);
        let (head, tail) = (ratio(0..1_000), ratio(got.len() - 1_000..got.len()));
        assert!((tail - head).abs() < 1e-4, "gain drifts across the window: {head} -> {tail}");
    }

    /// A window with no beacon in it must cost one sweep, not a whole drift
    /// grid's worth. The grid is a per-point [`acquire`], so running it over a
    /// window that decodes nothing — which is every window on a station whose
    /// LNB phase noise the frame decoder cannot survive — is what put the
    /// decoder an order of magnitude behind the air it was reading.
    ///
    /// Asserted as a ratio of two runs in the same process rather than a wall
    /// clock, so it says the same thing on a loaded machine as an idle one.
    #[test]
    fn a_window_with_no_beacon_costs_one_sweep_not_a_whole_drift_grid() {
        let rate = 20_000.0;
        let n = (rate * 12.0) as usize;
        let mut rng = TestRng::new(23);
        let noise: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32))
            .collect();
        let cancel = std::sync::atomic::AtomicBool::new(false);

        let t = std::time::Instant::now();
        assert!(
            acquire(&noise, rate, 5_000.0, TEST_STEP, crate::controller::DEMOD_RATE_HZ, &cancel)
                .is_none()
        );
        let one_sweep = t.elapsed();

        let t = std::time::Instant::now();
        let (lock, _) = acquire_debug(
            &noise,
            rate,
            5_000.0,
            TEST_STEP,
            crate::controller::DEMOD_RATE_HZ,
            0.0,
            0.0,
            &cancel,
        );
        let whole_pass = t.elapsed();
        assert!(lock.is_none());

        // One sweep, one spectrum check and the sync probe — not the 27 sweeps
        // the drift grid would be. Generous enough not to fail on scheduling.
        assert!(
            whole_pass < one_sweep * 4,
            "a beaconless window cost {whole_pass:?} against a single sweep's {one_sweep:?}"
        );
    }
}
