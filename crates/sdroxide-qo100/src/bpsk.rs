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
//! The differential+Manchester combination is decoded in one pass, without
//! ever resolving absolute carrier phase: comparing each chip against the one
//! immediately before it (a delay-and-multiply, not a coherent reference)
//! gives a flip/no-flip bit that is robust to whatever the residual carrier
//! phase is doing, and — because Manchester always flips at a bit's own
//! midpoint but the *inter-bit* transition flips exactly when the
//! differentially-encoded source bit is `0` — keeping only the inter-bit
//! comparisons recovers the original data directly. See the derivation in
//! this module's tests.

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

/// Mix `iq` down by `shift_hz` and integer-decimate by `deci` in one pass,
/// each output sample the mean of its `deci` inputs. That boxcar is a crude
/// anti-alias filter, but its nulls sit exactly at multiples of the output
/// rate — where any energy would fold — and the beacon is 400 baud, far
/// inside the output passband, so nothing that carries the frame is touched.
/// Output rate is `rate_hz / deci`.
fn mix_decimate(iq: &[Complex32], rate_hz: f64, shift_hz: f64, deci: usize) -> Vec<Complex32> {
    let deci = deci.max(1);
    let w = -std::f64::consts::TAU * shift_hz / rate_hz;
    let mut out = Vec::with_capacity(iq.len() / deci + 1);
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
/// Candidates are tried from the centre outward, so a beacon near the assumed
/// frequency — the common case for a roughly-calibrated station — is found
/// without walking the whole grid first. `cancel` is polled between
/// candidates so the engine can drop the controller (turning the decoder off,
/// or changing the search width) without waiting out a search in progress.
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
    let steps = (search_half_width_hz / freq_step_hz).round() as i64;
    // 0, +1, -1, +2, -2, … — centre outward.
    let order = (0..=2 * steps).map(|i| if i % 2 == 0 { i / 2 } else { -(i / 2 + 1) });
    for step in order {
        if cancel.load(Ordering::Relaxed) {
            return None;
        }
        let coarse_hz = step as f64 * freq_step_hz;
        let mixed = mix_decimate(iq, rate_hz, coarse_hz, deci);
        if let Some(cm) = try_frequency(&mixed, dr) {
            let offset_hz = coarse_hz + refine_offset_hz(&mixed, dr, &cm);
            return Some(Qo100Lock { offset_hz, text: cm.m.text });
        }
    }
    None
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
}
