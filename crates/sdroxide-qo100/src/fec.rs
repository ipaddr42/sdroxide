//! AO-40's *coded* telemetry frame — the other half of what the QO-100 beacon
//! sends, alongside the uncoded frames [`crate::bpsk`] decodes.
//!
//! The beacon alternates the two. An uncoded frame is 514 bytes behind a
//! 32-bit sync word and a CRC, and it either checks out or it does not; a
//! coded frame carries the same kind of payload behind four layers of
//! protection, and will still decode when a fifth of the symbols arrived
//! wrong. On a station whose LNB phase noise leaves the raw bit stream too
//! ragged for a CRC over 514 bytes ever to pass, the coded frame is the one
//! that gets through — which is the whole reason for this module.
//!
//! # The frame
//!
//! 5200 channel symbols, 13.0 s at 400 baud, carrying 256 payload bytes:
//!
//! ```text
//!   256 payload bytes
//!     → two shortened RS(160,128) codewords, alternate bytes to each
//!     → scrambler (all 320 data+parity bytes)
//!     → r=1/2 k=7 convolutional encoder, second output inverted, 6 tail bits
//!     → 80 x 65 block interleaver, written along rows, read down columns
//!     → 65 sync bits, one at the head of each column
//!   = 5200 symbols
//! ```
//!
//! Every layer is doing something specific. The interleaver is why the RS
//! codes can work at all: a fade wipes out a run of consecutive *transmitted*
//! symbols, and reading down columns means those land spread evenly through
//! the convolutional stream instead of in one unrecoverable burst. The
//! scrambler guarantees symbol transitions whatever the payload, so timing
//! recovery does not starve on a run of zeros. The sync bits are distributed
//! one per column rather than gathered into a preamble for the same reason the
//! interleaver exists — a single fade at the wrong moment must not be able to
//! take the whole of sync with it.
//!
//! # Where the numbers come from
//!
//! All of them are from KA9Q's format specification and his reference encoder
//! (`encode_ref.c`), and the constants below name which. They are not
//! negotiable and several are easy to get subtly wrong — the interleaver's
//! fill order and the inverted second convolutional output especially — so
//! [`tests::the_whole_chain_decodes_a_frame_from_the_reference_encoder`]
//! checks this decoder against a frame that implementation actually produced,
//! rather than against a matching encoder written from the same misreading.

use sdroxide_dsp::{Complex32, ConvCode, viterbi_soft};

use crate::bpsk::BAUD;
use crate::rs::{self, Rs};

/// Channel symbols in one coded frame, and how long they take on the air.
pub const FRAME_SYMBOLS: usize = 5200;
pub const FRAME_SECONDS: f64 = FRAME_SYMBOLS as f64 / BAUD;

/// Payload bytes a coded frame carries.
pub const PAYLOAD_BYTES: usize = 256;
/// Payload plus the two codewords' parity: what the convolutional encoder saw.
const CODED_BYTES: usize = PAYLOAD_BYTES + 2 * rs::NROOTS;
/// Zero bits flushing the convolutional encoder back to state 0.
const TAIL_BITS: usize = 6;
/// Symbols the convolutional encoder produced, before sync was mixed in.
const CONV_SYMBOLS: usize = (CODED_BYTES * 8 + TAIL_BITS) * 2;

/// Rows and columns of the block interleaver. Rows × columns is the whole
/// frame; the first row is sync, so `COLUMNS` is also the number of sync bits.
const ROWS: usize = 80;
const COLUMNS: usize = 65;

/// Sync-vector LFSR: `s(x) = x^7 + x^3 + 1`, started at all ones. 65 bits.
const SYNC_POLY: u8 = 0x48;
/// Scrambler LFSR: `h(x) = x^8 + x^7 + x^5 + x^3 + 1`, started at all ones.
const SCRAMBLER_POLY: u8 = 0x95;

/// The convolutional code, in this crate's newest-bit-is-lowest convention:
/// CCSDS `0o171`/`0o133` written the other way round, with the second output
/// inverted as AO-40 sends it.
const CONV: ConvCode = ConvCode { poly_a: 0x4f, poly_b: 0x6d, invert_b: true };

/// How much of the sync vector has to agree before a frame alignment is
/// believed, as a fraction of the summed symbol confidence.
///
/// A 65-bit correlation over 5200 candidate offsets will throw up a best match
/// on pure noise every time it is asked, so the threshold is what separates
/// "the best of a bad lot" from a frame. 0.35 sits well above what noise
/// reaches over this many trials and well below a real frame at the link
/// margins the coded format is designed for — it is meant to hold when a fifth
/// of the sync bits themselves are wrong.
const SYNC_THRESHOLD: f32 = 0.35;

/// Odd parity of `x`.
fn parity(x: u8) -> u8 {
    x.count_ones() as u8 & 1
}

/// The 65-bit sync vector, as the specification generates it.
fn sync_vector() -> [bool; COLUMNS] {
    let mut sr: u8 = 0x7f;
    std::array::from_fn(|_| {
        let bit = sr & 0x40 != 0;
        sr = (sr << 1) | parity(sr & SYNC_POLY);
        bit
    })
}

/// Where the `n`th convolutional symbol sits among the frame's 5200.
///
/// The encoder fills the interleaver along its rows, starting at the second —
/// the first is reserved for sync — and the frame is transmitted down the
/// columns. So symbol `n` is at row `1 + n / COLUMNS`, column `n % COLUMNS`,
/// and reading down columns puts that at `ROWS · column + row`.
///
/// Two consequences worth naming. Sync lands at every 80th transmitted symbol,
/// which is what makes the search below a stride rather than a scan. And the
/// last three cells of the last row are never written: 65 sync + 5132 coded
/// symbols is 5197, three short of the 5200 the matrix holds.
fn interleaved_position(n: usize) -> usize {
    ROWS * (n % COLUMNS) + 1 + n / COLUMNS
}

/// One decoded coded frame, and how hard it was.
#[derive(Debug, Clone, PartialEq)]
pub struct FecFrame {
    /// The 256 payload bytes.
    pub payload: Vec<u8>,
    /// Where in the symbol stream the frame started.
    pub start: usize,
    /// How well the sync vector matched, 0..1 — see [`SYNC_THRESHOLD`].
    pub sync_quality: f32,
    /// Symbol errors each Reed-Solomon codeword had to put right. Near zero
    /// is a comfortable link; near 16 is one frame away from failing.
    pub rs_errors: [usize; 2],
}

/// Where a frame appears to start, and which way round its symbols are.
struct Alignment {
    start: usize,
    /// `-1.0` when the whole stream is inverted. Differential detection fixes
    /// absolute phase but not this: swap I and Q, or receive on the other
    /// sideband, and every bit arrives complemented. The sync correlation
    /// answers it for free — a real frame correlates strongly either way, and
    /// the sign says which.
    polarity: f32,
    quality: f32,
}

/// Find the best frame alignment in `soft`.
fn find_frame(soft: &[f32], sync: &[bool; COLUMNS]) -> Option<Alignment> {
    if soft.len() < FRAME_SYMBOLS {
        return None;
    }
    let mut best: Option<Alignment> = None;
    for start in 0..=(soft.len() - FRAME_SYMBOLS) {
        let mut corr = 0.0f32;
        let mut total = 0.0f32;
        for (i, &want) in sync.iter().enumerate() {
            let v = soft[start + ROWS * i];
            corr += if want { v } else { -v };
            total += v.abs();
        }
        if total <= 0.0 {
            continue;
        }
        // Normalised so the answer is "what fraction of the confidence in
        // these symbols agrees", not "how loud was the signal".
        let quality = (corr / total).abs();
        if best.as_ref().is_none_or(|b| quality > b.quality) {
            best =
                Some(Alignment { start, polarity: if corr >= 0.0 { 1.0 } else { -1.0 }, quality });
        }
    }
    best.filter(|b| b.quality >= SYNC_THRESHOLD)
}

/// Undo the scrambler over `bytes`, in place.
fn descramble(bytes: &mut [u8]) {
    let mut sr: u8 = 0xff;
    for b in bytes.iter_mut() {
        *b ^= sr;
        for _ in 0..8 {
            sr = (sr << 1) | parity(sr & SCRAMBLER_POLY);
        }
    }
}

/// Decode one AO-40 coded frame out of `soft`.
///
/// `soft` is one value per 400-baud channel symbol in transmission order,
/// positive for a received `1`, with the magnitude standing for confidence —
/// what [`crate::rx`] hands back. It must span at least [`FRAME_SYMBOLS`];
/// anything longer is searched for the best frame alignment inside it.
pub fn decode_frame(soft: &[f32]) -> Option<FecFrame> {
    let sync = sync_vector();
    let al = find_frame(soft, &sync)?;
    let frame = &soft[al.start..al.start + FRAME_SYMBOLS];

    // Lift the convolutional symbols back out of the interleaver, undoing the
    // transmitted polarity and the encoder's inverted second output as we go.
    // Inverting that output here rather than inside the decoder keeps the
    // Viterbi's own code description honest about what was transmitted.
    let mut conv = vec![0.0f32; CONV_SYMBOLS];
    for (n, c) in conv.iter_mut().enumerate() {
        *c = al.polarity * frame[interleaved_position(n)];
    }

    let bits = viterbi_soft(&conv, CONV, true);
    if bits.len() < CODED_BYTES * 8 {
        return None;
    }

    // Pack MSB first — the order the encoder fed its bytes in.
    let mut bytes: Vec<u8> = bits[..CODED_BYTES * 8]
        .chunks_exact(8)
        .map(|c| c.iter().fold(0u8, |acc, &b| (acc << 1) | b))
        .collect();
    descramble(&mut bytes);

    // Both the payload bytes and the parity bytes alternate between the two
    // codewords, so each is simply every other byte of its own stream.
    let (data, parity_bytes) = bytes.split_at(PAYLOAD_BYTES);
    let rs = Rs::new();
    let mut payload = vec![0u8; PAYLOAD_BYTES];
    let mut rs_errors = [0usize; 2];
    for j in 0..2 {
        let mut cw = [0u8; rs::N];
        for i in 0..rs::K {
            cw[i] = data[2 * i + j];
        }
        for i in 0..rs::NROOTS {
            cw[rs::K + i] = parity_bytes[2 * i + j];
        }
        rs_errors[j] = rs.decode(&mut cw)?;
        for i in 0..rs::K {
            payload[2 * i + j] = cw[i];
        }
    }

    Some(FecFrame { payload, start: al.start, sync_quality: al.quality, rs_errors })
}

/// Decode a coded frame straight from complex baseband.
///
/// Runs the tracking receiver ([`crate::rx`]) over `iq`, seeded with the
/// spectral tracker's carrier estimate, and tries both Manchester parities
/// through [`decode_frame`]. `iq` needs to span at least one whole frame plus
/// enough for the loops to pull in on — [`FRAME_SECONDS`] and a little.
///
/// Returns the frame and the carrier the receiver settled on, which is a
/// measurement of the station's frequency error in its own right and a better
/// one than the search produces: it is where the loop *is*, not the grid point
/// that happened to decode.
pub fn decode_iq(iq: &[Complex32], rate_hz: f64, carrier_hz: f64) -> Option<(FecFrame, f64)> {
    let mut rx = crate::rx::Rx::new(rate_hz, carrier_hz);
    let mut chips = Vec::new();
    for chunk in iq.chunks(8192) {
        chips.extend(rx.process(chunk));
    }
    (0..2)
        .find_map(|parity| decode_frame(&crate::rx::symbols(&chips, parity)))
        .map(|f| (f, rx.state().carrier_hz))
}

/// The payload as text, for the operator's readout.
///
/// The coded beacon's payload is the same sort of thing the uncoded one
/// carries — status lines meant to be read — so it is rendered the same way,
/// lossily, rather than being hidden behind a "256 bytes decoded".
pub fn payload_text(payload: &[u8]) -> String {
    String::from_utf8_lossy(payload).into_owned()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Build a coded frame the way the satellite does, for tests.
    ///
    /// This mirrors the reference encoder step for step. It is *not* what
    /// validates the decoder — an encoder and decoder written from the same
    /// misunderstanding agree with each other perfectly — it is what lets the
    /// tests put a frame through a channel. The validation is
    /// [`the_whole_chain_decodes_a_frame_from_the_reference_encoder`], which
    /// uses a frame this code had no hand in making.
    pub(crate) fn encode_frame(payload: &[u8; PAYLOAD_BYTES]) -> Vec<bool> {
        let rs = Rs::new();
        let mut bytes = Vec::with_capacity(CODED_BYTES);
        bytes.extend_from_slice(payload);
        let parity: Vec<[u8; rs::NROOTS]> =
            (0..2).map(|j| rs.encode(&std::array::from_fn(|i| payload[2 * i + j]))).collect();
        for i in 0..rs::NROOTS {
            for p in parity.iter() {
                bytes.push(p[i]);
            }
        }

        // Scramble, then convolutionally encode MSB first, then interleave.
        let mut sr: u8 = 0xff;
        let mut conv_in: Vec<u8> = Vec::with_capacity(CODED_BYTES * 8 + TAIL_BITS);
        for &b in &bytes {
            let c = b ^ sr;
            for _ in 0..8 {
                sr = (sr << 1) | parity_of(sr & SCRAMBLER_POLY);
            }
            for k in (0..8).rev() {
                conv_in.push((c >> k) & 1);
            }
        }
        conv_in.extend(std::iter::repeat_n(0u8, TAIL_BITS));

        let mut state = 0usize;
        let mut symbols = Vec::with_capacity(CONV_SYMBOLS);
        for &b in &conv_in {
            let reg = ((state as u32) << 1) | b as u32;
            symbols.push(parity_of((reg & CONV.poly_a) as u8 & 0x7f) == 1);
            symbols.push(parity_of((reg & CONV.poly_b) as u8 & 0x7f) != 1);
            state = (reg as usize) & 0x3f;
        }
        assert_eq!(symbols.len(), CONV_SYMBOLS);

        let sync = sync_vector();
        let mut frame = vec![false; FRAME_SYMBOLS];
        for (i, &s) in sync.iter().enumerate() {
            frame[ROWS * i] = s;
        }
        for (n, &s) in symbols.iter().enumerate() {
            frame[interleaved_position(n)] = s;
        }
        frame
    }

    /// `parity` under another name, so [`encode_frame`] can use it without the
    /// shadowing that `parity_bytes` would cause in `decode_frame`'s scope.
    fn parity_of(x: u8) -> u8 {
        x.count_ones() as u8 & 1
    }

    /// Hard bits as ideal soft values.
    pub(crate) fn soft(bits: &[bool], amp: f32) -> Vec<f32> {
        bits.iter().map(|&b| if b { amp } else { -amp }).collect()
    }

    fn sample_payload() -> [u8; PAYLOAD_BYTES] {
        std::array::from_fn(|i| (i as u8).wrapping_mul(7).wrapping_add(3))
    }

    /// The interleaver's closed form, checked against the pointer walk the
    /// reference encoder actually performs.
    ///
    /// That walk steps forward 80 bits at a time through a 650-byte array,
    /// wrapping to the next bit of the byte each time it runs off the end.
    /// It is not obvious that it comes to `ROWS · (n mod COLUMNS) + 1 + n /
    /// COLUMNS`, and getting it wrong scrambles the frame in a way no other
    /// test here would localise — every symbol would simply be in the wrong
    /// place and nothing would decode.
    #[test]
    fn the_interleaver_matches_the_reference_encoders_pointer_walk() {
        // `encode_ref.c`, `interleave_symbol()`: Bindex starts at 0 and Bmask
        // at 0x40 — bit 1 of byte 0, the cell just after the sync bit.
        let (mut bindex, mut bmask) = (0usize, 0x40u8);
        for n in 0..CONV_SYMBOLS {
            let bit_in_byte = bmask.leading_zeros() as usize;
            assert_eq!(bindex * 8 + bit_in_byte, interleaved_position(n), "symbol {n}");
            bindex += 10; // forward 80 bits
            if bindex >= 650 {
                bindex -= 650;
                bmask >>= 1;
                if bmask == 0 {
                    bmask = 0x80;
                    bindex += 1;
                }
            }
        }
    }

    #[test]
    fn the_sync_vector_is_the_published_one() {
        // KA9Q's specification gives the vector explicitly.
        const SPEC: &str = "11111110000111011110010110010010000001000100110001011101011011000";
        let got: String = sync_vector().iter().map(|&b| if b { '1' } else { '0' }).collect();
        assert_eq!(got, SPEC);
    }

    /// End to end against a frame produced by KA9Q's `encode_ref.c`.
    ///
    /// The bytes below are that program's `Interleaver[]` — the 650 bytes it
    /// says are "ready for transmission ... most significant bit of
    /// Interleaver[0] first" — after being fed the payload `p[i] = (7i+3) mod
    /// 256`. Nothing in this crate had a hand in making them, so a decoder
    /// that reads them back is one that agrees with the format rather than
    /// with itself: interleaver order, sync placement, the inverted second
    /// convolutional output, the scrambler, the Reed-Solomon basis and the
    /// codeword interleaving all have to be right at once for this to pass.
    #[test]
    fn the_whole_chain_decodes_a_frame_from_the_reference_encoder() {
        let hex = include_str!("../tests/ao40_reference_frame.hex").trim();
        let packed: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex"))
            .collect();
        assert_eq!(packed.len(), 650, "one frame is 650 bytes of channel symbols");
        let bits: Vec<bool> =
            (0..FRAME_SYMBOLS).map(|p| packed[p / 8] & (0x80 >> (p % 8)) != 0).collect();

        let frame = decode_frame(&soft(&bits, 1.0)).expect("the reference frame should decode");
        assert_eq!(frame.payload, sample_payload(), "payload");
        assert_eq!(frame.rs_errors, [0, 0], "a clean frame needs no correction");
        assert_eq!(frame.start, 0);
        assert!(frame.sync_quality > 0.99, "sync {}", frame.sync_quality);
    }

    /// The test encoder must agree with the reference encoder bit for bit.
    ///
    /// [`encode_frame`] exists to put frames through channels in these tests,
    /// and everything that uses it would still pass if it and the decoder
    /// shared a mistake. This is what stops that: the same payload through
    /// `encode_ref.c` and through it must give the same 5200 symbols.
    #[test]
    fn the_test_encoder_agrees_with_the_reference_encoder() {
        let hex = include_str!("../tests/ao40_reference_frame.hex").trim();
        let packed: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex"))
            .collect();
        let want: Vec<bool> =
            (0..FRAME_SYMBOLS).map(|p| packed[p / 8] & (0x80 >> (p % 8)) != 0).collect();
        let got = encode_frame(&sample_payload());
        assert_eq!(got.len(), want.len());
        let diff = got.iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(diff, 0, "{diff} of {FRAME_SYMBOLS} symbols differ from the reference");
    }

    /// …and the same frame received upside down, which is what an I/Q swap or
    /// the wrong sideband gives. Differential detection does not fix this.
    #[test]
    fn an_inverted_frame_decodes_just_as_well() {
        let bits = encode_frame(&sample_payload());
        let flipped: Vec<bool> = bits.iter().map(|&b| !b).collect();
        let frame = decode_frame(&soft(&flipped, 1.0)).expect("inverted frames decode");
        assert_eq!(frame.payload, sample_payload());
    }

    #[test]
    fn a_frame_is_found_wherever_it_sits_in_a_longer_stream() {
        let bits = encode_frame(&sample_payload());
        for lead in [1usize, 37, 500] {
            let mut stream = vec![false; lead];
            stream.extend_from_slice(&bits);
            stream.extend(std::iter::repeat_n(false, 9));
            let mut s = soft(&stream, 1.0);
            // The lead-in is silence, not confident zeros.
            s[..lead].fill(0.0);
            let frame = decode_frame(&s).expect("found at any offset");
            assert_eq!(frame.start, lead, "lead {lead}");
            assert_eq!(frame.payload, sample_payload());
        }
    }

    /// The point of the whole format: a frame far too broken for the uncoded
    /// path still decodes.
    ///
    /// One symbol in twelve arrives complemented here. The uncoded frame's
    /// answer to that is a CRC over 514 bytes, which wants a bit error rate
    /// down in the millionths before it passes at all — so at this error rate
    /// the uncoded path yields nothing, ever, and the coded one returns the
    /// payload exactly. That gap is the reason a station with a phase-noisy
    /// LNB can read the coded beacon and not the plain one.
    ///
    /// 8 % is close to the cliff: measured on this decoder it comes back
    /// reliably at 8 % and never at 9 %, which is about where a rate-1/2
    /// convolutional code behind a rate-0.8 Reed-Solomon one gives out on
    /// *hard* symbol flips (the binary channel's own limit for rate 0.4 is
    /// near 14.6 %). Real soft decisions are worth a couple of dB beyond it.
    #[test]
    fn a_frame_with_one_symbol_in_twelve_wrong_still_decodes() {
        let bits = encode_frame(&sample_payload());
        let mut sf = soft(&bits, 1.0);
        let mut st = 0x9e37_79b9_7f4a_7c15u64;
        let mut hits = 0;
        for v in sf.iter_mut() {
            st ^= st << 13;
            st ^= st >> 7;
            st ^= st << 17;
            if (st >> 11) % 100 < 8 {
                *v = -*v;
                hits += 1;
            }
        }
        assert!(hits > FRAME_SYMBOLS / 15, "meant to break {hits} symbols");
        let frame = decode_frame(&sf).expect("the coded format is for exactly this");
        assert_eq!(frame.payload, sample_payload());
        assert!(frame.rs_errors.iter().any(|&e| e > 0), "the RS layer should have had work to do");
    }

    /// Noise must not produce a frame. A 65-bit correlation asked at 5200
    /// offsets will always report a best match; the threshold is what stops
    /// that being mistaken for a decode.
    #[test]
    fn noise_never_yields_a_frame() {
        let mut st = 0x2545_f491_4f6c_dd1du64;
        for trial in 0..8 {
            let noise: Vec<f32> = (0..FRAME_SYMBOLS + 400)
                .map(|_| {
                    st ^= st << 13;
                    st ^= st >> 7;
                    st ^= st << 17;
                    (st >> 11) as f32 / (1u64 << 52) as f32 - 1.0
                })
                .collect();
            assert!(decode_frame(&noise).is_none(), "trial {trial}");
        }
    }
}
