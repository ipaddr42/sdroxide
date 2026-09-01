//! One VDL2 channel, end to end: gate, synchronise, demodulate, unscramble,
//! repair, and hand over an AVLC frame.
//!
//! ```text
//!  baseband ─▶ Gate ─▶ Rrc::run ─▶ sync::find ─▶ SymbolReader
//!                                                    │ bits
//!                        Lfsr (one per burst) ◀───────┤
//!                                                    ▼
//!                          header::decode ─▶ block::decode ─▶ avlc::parse
//! ```
//!
//! # The counters are the diagnosis
//!
//! An empty panel has half a dozen causes, and they want completely different
//! answers: no aerial, an aerial on the wrong socket, a receiver tuned
//! elsewhere, a channel full of something that is not VDL2, or a decoder with a
//! bit order backwards. [`Counters`] is arranged so that the *first* stage that
//! stops counting names the cause:
//!
//! - `bursts` zero — nothing is arriving. Aerial, or the receiver is not here.
//! - `bursts` up, `syncs` zero — something is arriving and it is not VDL2.
//! - `syncs` up, `header_ok` zero — it *is* VDL2, and the bits are being read
//!   wrongly. A decoder problem, not a propagation one.
//! - `header_ok` up, `rs_fail` high — real frames, arriving damaged.
//! - `rs_ok` up, `fcs_bad` high — the error correction is fine and something
//!   above it is not. The sharpest single signal there is that this decoder,
//!   rather than the radio path, is what is wrong.
//!
//! # The bit stream is continuous
//!
//! Twenty-five header bits is not a whole number of three-bit symbols, so the
//! data field begins **two bits into the ninth symbol**. Rather than treat that
//! as a special case, the reader appends symbols to one bit vector and the
//! scrambler runs over it in two passes with one register: the header first, and
//! then — once the header has said how long the transmission is — the rest,
//! continuing from where the register stopped. Nothing has to know where a
//! symbol boundary fell.

use sdroxide_dsp::Complex32;

use crate::avlc;
use crate::block::{self, InterleaveOrder};
use crate::demod::{ChannelFilter, SYMBOL_RATE, SymbolReader};
use crate::gate::{Burst, Gate};
use crate::header;
use crate::scramble::Lfsr;
use crate::sync;

/// Symbols the matched filter spans.
const RRC_SPAN_SYMS: usize = 8;
/// Sub-sample positions in the filter bank. A sixteenth of a symbol is well
/// inside what the refinement resolves.
const RRC_PHASES: usize = 16;

/// How strong a candidate has to be, as a fraction of the transmission's own
/// peak power, before the correlator's score is believed.
///
/// The correlation score is normalised, which divides out exactly the thing that
/// tells a real symbol from the shaping filter's tail. On a synthetic burst the
/// tail eight symbols ahead of the first real symbol scores 0.78 against the
/// true position's 0.98 — indistinguishable by score, and an order of magnitude
/// apart in level — and the tail's mean level sits at 0.22 of the
/// transmission's peak power against the true position's 0.5 and up.
const MIN_MAG_FRACTION: f32 = 0.30;

/// What came off the air.
#[derive(Debug, Clone)]
pub struct Decoded {
    pub frame: avlc::Frame,
    pub center_hz: f64,
    pub snr_db: f32,
    pub rssi_dbfs: f32,
    pub rs_corrected: usize,
    pub evm_deg: f32,
    pub freq_err_hz: f32,
    /// The AVLC frame as it arrived, check sequence included.
    pub raw: Vec<u8>,
}

/// Where every burst ended up.
#[derive(Debug, Clone, Copy, Default)]
pub struct Counters {
    pub bursts: u64,
    pub overlong: u64,
    pub syncs: u64,
    pub header_ok: u64,
    pub header_corrected: u64,
    pub header_bad: u64,
    pub length_insane: u64,
    pub truncated: u64,
    pub rs_ok: u64,
    pub rs_fail: u64,
    pub rs_corrected: u64,
    pub multiblock: u64,
    pub multiblock_ok: u64,
    pub fcs_bad: u64,
    pub malformed: u64,
    pub frames: u64,
}

/// One channel's receiver.
pub struct ChannelRx {
    center_hz: f64,
    rate_hz: f64,
    sps: f64,
    rrc: ChannelFilter,
    gate: Gate,
    order: InterleaveOrder,
    counters: Counters,
    // Scratch, kept so a busy channel does not allocate per burst.
    bursts: Vec<Burst>,
    mf: Vec<Complex32>,
    bits: Vec<u8>,
    field: Vec<u8>,
}

impl ChannelRx {
    pub fn new(center_hz: f64, rate_hz: f64, threshold_db: f32) -> ChannelRx {
        let sps = rate_hz / SYMBOL_RATE;
        ChannelRx {
            center_hz,
            rate_hz,
            sps,
            rrc: ChannelFilter::new(sps, RRC_SPAN_SYMS, RRC_PHASES),
            gate: Gate::new(rate_hz, center_hz, threshold_db),
            order: InterleaveOrder::default(),
            counters: Counters::default(),
            bursts: Vec::new(),
            mf: Vec::new(),
            bits: Vec::new(),
            field: Vec::new(),
        }
    }

    pub fn center_hz(&self) -> f64 {
        self.center_hz
    }

    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    pub fn samples_per_symbol(&self) -> f64 {
        self.sps
    }

    pub fn counters(&self) -> &Counters {
        &self.counters
    }

    pub fn floor_dbfs(&self) -> f32 {
        self.gate.floor_dbfs()
    }

    pub fn set_threshold_db(&mut self, db: f32) {
        self.gate.set_threshold_db(db);
    }

    /// Which reading of the block interleaving to use. See [`crate::block`] —
    /// the round-robin one is this decoder's, and the alternative is here so it
    /// can be tried against a recording rather than argued about.
    pub fn set_interleave_order(&mut self, order: InterleaveOrder) {
        self.order = order;
    }

    /// Feed baseband; append everything decoded to `out`.
    pub fn push(&mut self, iq: &[Complex32], out: &mut Vec<Decoded>) {
        self.bursts.clear();
        let mut bursts = std::mem::take(&mut self.bursts);
        self.gate.push(iq, &mut bursts);
        for b in &bursts {
            self.decode_burst(b, out);
        }
        bursts.clear();
        self.bursts = bursts;
        self.counters.overlong = self.gate.overlong;
    }

    /// Decode every transmission in one gated burst.
    ///
    /// The search and the decode are interleaved rather than run in two passes:
    /// a candidate that decodes says where the frame ended, so the search can
    /// resume past it, and a candidate that does not means the search resumes
    /// one sample later and tries again. Two passes would have to guess how far
    /// to skip, and guessing too far steps over the true position — which is
    /// exactly what a shaped pulse's leading tail arranges, since it correlates
    /// respectably a whole symbol-word early.
    pub fn decode_burst(&mut self, burst: &Burst, out: &mut Vec<Decoded>) {
        self.counters.bursts += 1;
        self.rrc.run(&burst.iq, &mut self.mf);
        let peak = self.mf.iter().map(|z| z.norm_sqr()).fold(0f32, f32::max);
        let min_mag = peak * MIN_MAG_FRACTION;

        let mut from = 0usize;
        // A transmission cannot hold more synchronisation words than it has
        // symbols; the bound is only here so that a pathological buffer cannot
        // spin.
        let cap = burst.iq.len() / self.sps as usize + 2;
        for _ in 0..cap {
            let Some(found) =
                sync::find_next(&burst.iq, &self.mf, &self.rrc, self.sps, from, min_mag)
            else {
                break;
            };
            self.counters.syncs += 1;
            match self.decode_from(&burst.iq, &found.lock, burst) {
                Some((d, end)) => {
                    self.counters.frames += 1;
                    out.push(d);
                    from = end.max(found.resume);
                }
                None => from = found.resume,
            }
        }
    }

    /// Read one transmission from a lock. Returns what came out and the sample
    /// position just past it, so the search can resume there.
    fn decode_from(
        &mut self,
        iq: &[Complex32],
        lock: &sync::Lock,
        burst: &Burst,
    ) -> Option<(Decoded, usize)> {
        let mut reader = SymbolReader::new(iq, &self.rrc, lock);
        self.bits.clear();
        let mut lfsr = Lfsr::new();

        if !reader.read_bits(header::HEADER_BITS, &mut self.bits) {
            self.counters.truncated += 1;
            return None;
        }
        lfsr.apply(&mut self.bits[..header::HEADER_BITS]);
        let h = match header::decode(&self.bits[..header::HEADER_BITS]) {
            Ok(h) => h,
            Err(header::HeaderError::LengthInsane(_)) => {
                self.counters.length_insane += 1;
                return None;
            }
            Err(_) => {
                self.counters.header_bad += 1;
                return None;
            }
        };
        self.counters.header_ok += 1;
        if h.corrected {
            self.counters.header_corrected += 1;
        }

        let data_octets = (h.trlen_bits as usize).div_ceil(8);
        let l = block::layout(data_octets);
        if l.total_octets == 0 {
            self.counters.malformed += 1;
            return None;
        }
        let need = header::HEADER_BITS + l.total_octets * 8;
        let done = self.bits.len().min(need);
        if !reader.read_bits(need, &mut self.bits) {
            self.counters.truncated += 1;
            return None;
        }
        // The register carries on from where the header left it: the header and
        // the data are one scrambled run, not two.
        lfsr.apply(&mut self.bits[header::HEADER_BITS..need]);
        debug_assert!(done >= header::HEADER_BITS);

        // Least significant bit first, which is what makes the Reed-Solomon
        // syndromes come out — the check that would say so if it were the other
        // way round.
        self.field.clear();
        for i in 0..l.total_octets {
            let base = header::HEADER_BITS + i * 8;
            let mut o = 0u8;
            for k in 0..8 {
                o |= (self.bits[base + k] & 1) << k;
            }
            self.field.push(o);
        }

        if l.is_multiblock() {
            self.counters.multiblock += 1;
        }
        let fec = match block::decode(&self.field, data_octets, self.order) {
            Ok(f) => f,
            Err(_) => {
                self.counters.rs_fail += 1;
                return None;
            }
        };
        self.counters.rs_ok += 1;
        self.counters.rs_corrected += fec.corrected as u64;
        if l.is_multiblock() {
            self.counters.multiblock_ok += 1;
        }

        let frame_len = (h.trlen_bits as usize) / 8;
        if frame_len < avlc::MIN_LEN || frame_len > fec.data.len() {
            self.counters.malformed += 1;
            return None;
        }
        let raw = fec.data[..frame_len].to_vec();
        let frame = match avlc::parse(&raw) {
            Ok(f) => f,
            Err(avlc::FrameError::BadFcs) => {
                self.counters.fcs_bad += 1;
                return None;
            }
            Err(_) => {
                self.counters.malformed += 1;
                return None;
            }
        };

        let end = reader.pos().max(0.0) as usize + 1;
        Some((
            Decoded {
                frame,
                center_hz: self.center_hz,
                snr_db: burst.snr_db,
                rssi_dbfs: burst.peak_dbfs,
                rs_corrected: fec.corrected,
                evm_deg: reader.evm_deg(),
                freq_err_hz: reader.freq_hz(),
                raw,
            },
            end,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tx::{Noise, Shape, TxParams};
    use sdroxide_types::{Vdl2AddrKind, Vdl2Frame};

    fn a_frame(payload: &[u8]) -> Vec<u8> {
        avlc::build(
            avlc::Address { addr: 0x10_A1_B2, kind: Vdl2AddrKind::GroundAdmin, cr: true },
            avlc::Address { addr: 0x44_0F_31, kind: Vdl2AddrKind::Aircraft, cr: false },
            avlc::control_octet(Vdl2Frame::Ui { pf: false }),
            payload,
        )
    }

    /// Decode one burst straight out of the modulator, with no gate in the way.
    fn decode_direct(rx: &mut ChannelRx, iq: &[Complex32]) -> Vec<Decoded> {
        let burst = Burst {
            iq: iq.to_vec(),
            rate_hz: rx.rate_hz,
            center_hz: rx.center_hz,
            snr_db: 30.0,
            peak_dbfs: -20.0,
        };
        let mut out = Vec::new();
        rx.decode_burst(&burst, &mut out);
        out
    }

    /// The whole chain, on a clean burst. If this fails, nothing else in the
    /// crate matters.
    #[test]
    fn a_clean_burst_decodes() {
        let rate = 96_000.0;
        let frame = a_frame(b"\xff\xff\x01hello there");
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let iq = crate::tx::modulate(&frame, &p, 200.0);

        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let got = decode_direct(&mut rx, &iq);
        assert_eq!(got.len(), 1, "counters: {:?}", rx.counters());
        assert_eq!(got[0].raw, frame);
        assert_eq!(got[0].frame.src.addr, 0x44_0F_31);
        assert_eq!(got[0].frame.dst.addr, 0x10_A1_B2);
        assert_eq!(got[0].rs_corrected, 0);
        assert!(got[0].evm_deg < 5.0, "EVM {}", got[0].evm_deg);
    }

    /// A burst does not arrive on a sample boundary. Every arrival phase has to
    /// decode, or three quarters of real traffic is lost to a bug that a
    /// generator starting on whole samples would never show.
    #[test]
    fn every_sub_sample_arrival_phase_decodes() {
        let rate = 96_000.0;
        let frame = a_frame(b"\xff\xff\x01phase sweep");
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        for step in 0..10 {
            let at = 200.0 + f64::from(step) * 0.1;
            let p = TxParams { sample_rate: rate, ..TxParams::default() };
            let mut iq = vec![Complex32::default(); 200];
            crate::tx::modulate_at(&frame, &p, at, &mut iq);
            iq.resize(iq.len() + 500, Complex32::default());
            let got = decode_direct(&mut rx, &iq);
            assert_eq!(got.len(), 1, "arrival at {at}: {:?}", rx.counters());
            assert_eq!(got[0].raw, frame);
        }
    }

    /// The carrier budget is 656 Hz, and a stock receiver is ten times out. If
    /// the estimate from the synchronisation word were not there, this is the
    /// test that would fail — and on the air it would look like a dead decoder.
    #[test]
    fn a_burst_well_off_frequency_decodes() {
        let rate = 96_000.0;
        let frame = a_frame(b"\xff\xff\x01off frequency");
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        for offset in [-5000.0, -1500.0, -656.0, 0.0, 656.0, 1500.0, 5000.0] {
            let p = TxParams { sample_rate: rate, freq_offset_hz: offset, ..TxParams::default() };
            let iq = crate::tx::modulate(&frame, &p, 200.0);
            let got = decode_direct(&mut rx, &iq);
            assert_eq!(got.len(), 1, "at {offset} Hz: {:?}", rx.counters());
            let err = got[0].freq_err_hz - offset as f32;
            assert!(err.abs() < 200.0, "at {offset} Hz the estimate was off by {err}");
        }
    }

    /// A symbol clock that is not quite the receiver's, over a frame long enough
    /// for the difference to matter.
    #[test]
    fn a_burst_with_a_clock_error_decodes() {
        let rate = 96_000.0;
        let frame = a_frame(&[0x5au8; 200]);
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        for ppm in [-30.0, -10.0, 0.0, 10.0, 30.0] {
            let p = TxParams { sample_rate: rate, clock_ppm: ppm, ..TxParams::default() };
            let iq = crate::tx::modulate(&frame, &p, 200.0);
            let got = decode_direct(&mut rx, &iq);
            assert_eq!(got.len(), 1, "at {ppm} ppm: {:?}", rx.counters());
        }
    }

    /// Either transmit pulse shape. The standard names raised cosine; a
    /// transmitter that splits the root across both ends is the other thing a
    /// receiver meets, and a decoder that only handles one of them works
    /// perfectly on the bench and half the time on the air.
    #[test]
    fn both_transmit_pulse_shapes_decode() {
        let rate = 96_000.0;
        let frame = a_frame(b"\xff\xff\x01shape");
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        for shape in [Shape::Rc, Shape::Rrc] {
            let p = TxParams { sample_rate: rate, shape, ..TxParams::default() };
            let iq = crate::tx::modulate(&frame, &p, 200.0);
            let got = decode_direct(&mut rx, &iq);
            assert_eq!(got.len(), 1, "{shape:?}: {:?}", rx.counters());
        }
    }

    /// A front end with high-side injection mirrors the spectrum, and the
    /// synchronisation word settles which way round it is for free.
    #[test]
    fn a_mirrored_spectrum_decodes() {
        let rate = 96_000.0;
        let frame = a_frame(b"\xff\xff\x01mirror");
        let p = TxParams { sample_rate: rate, inverted: true, ..TxParams::default() };
        let iq = crate::tx::modulate(&frame, &p, 200.0);
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let got = decode_direct(&mut rx, &iq);
        assert_eq!(got.len(), 1, "{:?}", rx.counters());
        assert_eq!(got[0].raw, frame);
    }

    /// Every front-end rate in this tree, since the samples per symbol is
    /// whatever an integer decimation happens to give.
    #[test]
    fn every_channel_rate_a_front_end_produces_decodes() {
        let frame = a_frame(b"\xff\xff\x01rates");
        for rate in [96_000.0, 100_000.0, 101_250.0, 102_400.0, 104_166.7, 125_000.0] {
            let p = TxParams { sample_rate: rate, ..TxParams::default() };
            let iq = crate::tx::modulate(&frame, &p, 200.0);
            let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
            let got = decode_direct(&mut rx, &iq);
            assert_eq!(got.len(), 1, "at {rate} sps: {:?}", rx.counters());
        }
    }

    /// A frame long enough to need more than one Reed-Solomon block — the case
    /// the interleaving hypothesis is about. It has to survive its own
    /// transmitter at least, so that the counter separating multi-block frames
    /// from multi-block successes means something on the air.
    #[test]
    fn a_multiblock_frame_survives_its_own_transmitter() {
        let rate = 96_000.0;
        let frame = a_frame(&(0..600u16).map(|i| (i * 7) as u8).collect::<Vec<u8>>());
        assert!(block::layout(frame.len()).is_multiblock());
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let iq = crate::tx::modulate(&frame, &p, 200.0);
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let got = decode_direct(&mut rx, &iq);
        assert_eq!(got.len(), 1, "{:?}", rx.counters());
        assert_eq!(rx.counters().multiblock, 1);
        assert_eq!(rx.counters().multiblock_ok, 1);
    }

    /// Noise on its own decodes nothing, however long it runs.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 96_000.0;
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let mut n = Noise::new(0xfeed);
        let mut out = Vec::new();
        for _ in 0..20 {
            let mut buf = vec![Complex32::default(); 8192];
            n.add(&mut buf, 0.05);
            rx.push(&buf, &mut out);
        }
        assert!(out.is_empty(), "noise decoded {} frames", out.len());
    }

    /// Through the gate, in noise, at a plausible signal level: the shape a real
    /// channel has.
    #[test]
    fn a_burst_in_noise_decodes_through_the_gate() {
        let rate = 100_000.0;
        let frame = a_frame(b"\xff\xff\x012OE-LWA\x15H1B\x02real enough\x03");
        let p = TxParams { sample_rate: rate, amplitude: 0.5, ..TxParams::default() };
        let burst = crate::tx::modulate(&frame, &p, 50.0);

        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let mut n = Noise::new(0xb00c);
        let mut out = Vec::new();
        let mut quiet = vec![Complex32::default(); 16_384];
        n.add(&mut quiet, 0.01);
        rx.push(&quiet, &mut out);
        let mut sig = burst.clone();
        n.add(&mut sig, 0.01);
        rx.push(&sig, &mut out);
        let mut quiet2 = vec![Complex32::default(); 16_384];
        n.add(&mut quiet2, 0.01);
        rx.push(&quiet2, &mut out);

        assert_eq!(out.len(), 1, "{:?}", rx.counters());
        assert_eq!(out[0].raw, frame);
        assert!(out[0].snr_db > 15.0);
    }

    /// A strong station on the next channel does not produce frames on this
    /// one. The downconverter's own low-pass is designed against its output
    /// rate rather than a 50 kHz raster, so this is the matched filter's job.
    #[test]
    fn a_strong_neighbour_decodes_nothing_here() {
        let rate = 100_000.0;
        let frame = a_frame(b"\xff\xff\x01wrong channel");
        // Thirty decibels stronger, one channel up.
        let p = TxParams {
            sample_rate: rate,
            freq_offset_hz: 50_000.0,
            amplitude: 30.0,
            ..TxParams::default()
        };
        let iq = crate::tx::modulate(&frame, &p, 200.0);
        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let got = decode_direct(&mut rx, &iq);
        assert!(got.is_empty(), "the neighbouring channel decoded here: {:?}", rx.counters());
    }

    /// Two stations colliding: the search resumes after the first transmission
    /// rather than stopping at it.
    #[test]
    fn a_second_station_after_the_first_is_found() {
        let rate = 96_000.0;
        let a = a_frame(b"\xff\xff\x01first");
        let b = a_frame(b"\xff\xff\x01second");
        let p = TxParams { sample_rate: rate, ..TxParams::default() };
        let mut iq = vec![Complex32::default(); 200];
        crate::tx::modulate_at(&a, &p, 200.0, &mut iq);
        let at2 = iq.len() as f64 + 100.0;
        iq.resize(at2 as usize, Complex32::default());
        crate::tx::modulate_at(&b, &p, at2, &mut iq);
        iq.resize(iq.len() + 500, Complex32::default());

        let mut rx = ChannelRx::new(136_975_000.0, rate, 9.0);
        let got = decode_direct(&mut rx, &iq);
        assert_eq!(got.len(), 2, "{:?}", rx.counters());
        assert_eq!(got[0].raw, a);
        assert_eq!(got[1].raw, b);
    }
}
