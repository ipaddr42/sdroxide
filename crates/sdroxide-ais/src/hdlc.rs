//! The HDLC layer: NRZI, flag hunting, bit de-stuffing and the frame check
//! sequence.
//!
//! ```text
//!    8         24          8           n            16         8        24
//! ┌───────┬────────────┬───────┬───────────────┬───────────┬───────┬────────┐
//! │ ramp  │  training  │ 0x7E  │  data field   │    FCS    │ 0x7E  │ buffer │
//! └───────┴────────────┴───────┴───────────────┴───────────┴───────┴────────┘
//!                       ◀──── stuffed, and NRZI over the whole packet ────▶
//! ```
//!
//! # The two bit orders, which is the thing to get right
//!
//! AIS uses HDLC's framing and departs from its bit order, and the two halves
//! of this module are where that shows:
//!
//! * The **check sequence** is HDLC's, computed over octets assembled
//!   **least significant bit first** from the received stream — the same
//!   CRC-16/X.25 an AX.25 frame carries, which is why
//!   [`sdroxide_ax25::fcs`] is what checks it rather than a second copy of the
//!   table.
//! * The **data field** is read **most significant bit first** straight off the
//!   received stream, because ITU-R M.1371 states its field tables that way:
//!   the first six bits on the air are the message type, as a number.
//!
//! Getting either backwards produces a decoder that finds frames and reports
//! nonsense, or one that reads sensible-looking fields and rejects every frame
//! — so this module hands out the *bits*, in reception order, and lets
//! [`crate::message`] do the second reading. Nothing here packs a payload
//! octet, and nothing in `message` computes a checksum.
//!
//! # NRZI, and why the receiver's polarity does not matter
//!
//! A zero is a change of level, a one is no change. Nothing in it depends on
//! which level is which, so a spectrum-inverted front end or a swapped I/Q pair
//! decodes identically — see [`crate::demod`].
//!
//! The very first level carries no information, because there is nothing for it
//! to have differed from. It primes the decoder and is not turned into a bit;
//! treating it as a one would put a phantom bit at the head of the stream and
//! shift the first frame by one, and the only symptom would be that nothing
//! ever passes its check sequence.
//!
//! Source: ITU-R M.1371-5 §3.3 (packet structure, bit stuffing, NRZI) and
//! ISO/IEC 13239 for the framing it names.

/// The flag octet, opening and closing.
pub const FLAG: u8 = 0x7E;

/// Shortest data field accepted, in bits.
///
/// Below this a "frame" is a run of noise that happened to contain two flag
/// patterns and pass a sixteen-bit check — which at one chance in 65 536 per
/// candidate is rare, but not rare enough to leave the floor open. The
/// shortest real message is the UTC request at 72 bits.
pub const MIN_DATA_BITS: usize = 40;

/// Longest data field accepted, in bits.
///
/// A message may claim five consecutive slots, which after the packet's own
/// overhead leaves a little under 1200 bits of data.
pub const MAX_DATA_BITS: usize = 1_192;

/// Why a candidate between two flags was not a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reject {
    /// Not a whole number of octets. HDLC guarantees one, so this was noise
    /// that happened to contain the flag pattern.
    Unaligned,
    /// Shorter than [`MIN_DATA_BITS`] or longer than [`MAX_DATA_BITS`].
    Length,
    /// The frame check sequence did not verify.
    BadFcs,
}

/// Flag hunting, de-stuffing and NRZI decode over one burst.
///
/// Fed line levels as the slicer produces them, yields the **data field** of
/// every frame whose check sequence verifies, as bits in reception order with
/// the check sequence already stripped.
#[derive(Debug, Default)]
pub struct Deframer {
    /// The previous line level, for the transition decode.
    last: bool,
    /// Whether a level has been seen at all.
    primed: bool,
    /// The last eight decoded (pre-de-stuffing) bits, newest at the top, for
    /// flag detection.
    window: u8,
    /// Consecutive ones in the decoded stream.
    ones: u8,
    /// True once a flag has been seen, so bits are worth collecting.
    in_frame: bool,
    /// Bits of the frame under construction.
    bits: Vec<bool>,
    /// Candidates refused since the last drain, by reason.
    pub rejects: Vec<Reject>,
}

/// Bits of the closing flag already collected when it is recognised.
///
/// **Seven**, not eight: the flag is only spotted on its last bit, and the
/// branch that acts on it returns before pushing that one. Taking eight off
/// would eat the frame's final data bit, and the only symptom would be that
/// nothing ever passes its check sequence. Those seven are safe to count on —
/// the flag's leading zero could in principle have been swallowed as a stuffed
/// bit, but only after five ones, and the sender's own stuffing guarantees at
/// most four in a row at the end of a frame.
const FLAG_BITS_COLLECTED: usize = 7;

impl Deframer {
    pub fn new() -> Deframer {
        Deframer::default()
    }

    /// Feed one line level from the slicer, appending any completed frame's
    /// data field to `out`.
    pub fn push_level(&mut self, level: bool, out: &mut Vec<Vec<bool>>) {
        let bit = if self.primed { level == self.last } else { true };
        self.last = level;
        if !self.primed {
            self.primed = true;
            return;
        }
        self.push_bit(bit, out);
    }

    /// Feed one already-NRZI-decoded bit. Used by the tests and by anything
    /// that recovers data bits directly.
    pub fn push_bit(&mut self, bit: bool, out: &mut Vec<Vec<bool>>) {
        self.window = (self.window >> 1) | if bit { 0x80 } else { 0 };

        if self.window == FLAG {
            if self.in_frame {
                let n = self.bits.len().saturating_sub(FLAG_BITS_COLLECTED);
                self.bits.truncate(n);
                self.finish(out);
            }
            self.in_frame = true;
            self.bits.clear();
            self.ones = 0;
            return;
        }

        if !self.in_frame {
            return;
        }

        if bit {
            self.ones += 1;
            // Seven ones in a row cannot happen in stuffed data and is not a
            // flag: the line is idle or the transmission has ended. Abandon
            // quietly — this is the normal end of a slot, not an error.
            if self.ones > 6 {
                self.in_frame = false;
                self.bits.clear();
                self.ones = 0;
                return;
            }
        } else {
            // A zero after exactly five ones was stuffed in by the sender and
            // is not data. Any other zero is.
            let stuffed = self.ones == 5;
            self.ones = 0;
            if stuffed {
                return;
            }
        }
        self.bits.push(bit);
        if self.bits.len() > MAX_DATA_BITS + 16 {
            self.rejects.push(Reject::Length);
            self.in_frame = false;
            self.bits.clear();
        }
    }

    /// Judge the bits collected between two flags.
    fn finish(&mut self, out: &mut Vec<Vec<bool>>) {
        let mut bits = std::mem::take(&mut self.bits);
        if bits.is_empty() {
            return;
        }
        if !bits.len().is_multiple_of(8) {
            self.rejects.push(Reject::Unaligned);
            return;
        }
        if bits.len() < MIN_DATA_BITS + 16 || bits.len() > MAX_DATA_BITS + 16 {
            self.rejects.push(Reject::Length);
            return;
        }
        if !sdroxide_ax25::fcs::check(&octets_lsb_first(&bits)) {
            self.rejects.push(Reject::BadFcs);
            return;
        }
        bits.truncate(bits.len() - 16);
        out.push(bits);
    }
}

/// Pack a bit stream into octets, least significant bit first — the order
/// HDLC's check sequence is computed over.
///
/// Not the order the data field is read in; see the module note.
pub fn octets_lsb_first(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |b, (i, &v)| if v { b | 1 << i } else { b }))
        .collect()
}

/// Spread octets back into a bit stream, least significant bit first. The
/// inverse of [`octets_lsb_first`], and what a transmitter needs.
pub fn bits_lsb_first(octets: &[u8]) -> Vec<bool> {
    octets.iter().flat_map(|&b| (0..8).map(move |i| b >> i & 1 != 0)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build the bit stream a transmitter would put on the line: flags, a
    /// stuffed body, and NRZI over the lot.
    fn framed(data_bits: &[bool]) -> Vec<bool> {
        let mut bits = Vec::new();
        let push_flag = |bits: &mut Vec<bool>| {
            for i in 0..8 {
                bits.push(FLAG >> i & 1 != 0);
            }
        };
        // A body is the data field plus the check sequence over it.
        let mut body = data_bits.to_vec();
        let fcs = sdroxide_ax25::fcs::fcs(&octets_lsb_first(&body));
        body.extend(bits_lsb_first(&fcs));

        push_flag(&mut bits);
        let mut ones = 0;
        for &b in &body {
            bits.push(b);
            if b {
                ones += 1;
                if ones == 5 {
                    bits.push(false);
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
        push_flag(&mut bits);

        // NRZI: a zero is a change of level.
        let mut level = false;
        let mut line = vec![level];
        for &b in &bits {
            if !b {
                level = !level;
            }
            line.push(level);
        }
        line
    }

    fn data(n: usize, seed: u8) -> Vec<bool> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(97).wrapping_add(29);
                s & 0x40 != 0
            })
            .collect()
    }

    /// A frame goes round the loop, check sequence and all.
    #[test]
    fn a_frame_survives_the_round_trip() {
        let d = data(168, 7);
        let mut rx = Deframer::new();
        let mut out = Vec::new();
        for lvl in framed(&d) {
            rx.push_level(lvl, &mut out);
        }
        assert_eq!(out.len(), 1, "rejects: {:?}", rx.rejects);
        assert_eq!(out[0], d);
    }

    /// Bit stuffing has to survive a body that would otherwise contain a flag,
    /// and one with a long run of ones in it — the two things it exists for.
    #[test]
    fn stuffing_survives_a_flag_pattern_in_the_data() {
        let mut d = data(160, 3);
        // Eight ones, then the flag pattern itself, as data.
        for (i, v) in [true; 8].iter().enumerate() {
            d[i] = *v;
        }
        for (i, v) in [false, true, true, true, true, true, true, false].iter().enumerate() {
            d[16 + i] = *v;
        }
        let mut rx = Deframer::new();
        let mut out = Vec::new();
        for lvl in framed(&d) {
            rx.push_level(lvl, &mut out);
        }
        assert_eq!(out.len(), 1, "rejects: {:?}", rx.rejects);
        assert_eq!(out[0], d);
    }

    /// The polarity of the line is not information. A receiver that inverts
    /// everything — a swapped I/Q pair, an inverting front end — decodes the
    /// same frame, which is the whole reason AIS is NRZI.
    #[test]
    fn an_inverted_line_decodes_the_same_frame() {
        let d = data(168, 11);
        let line = framed(&d);
        let mut a = (Deframer::new(), Vec::new());
        let mut b = (Deframer::new(), Vec::new());
        for lvl in line {
            a.0.push_level(lvl, &mut a.1);
            b.0.push_level(!lvl, &mut b.1);
        }
        assert_eq!(a.1, b.1);
        assert_eq!(a.1.len(), 1);
    }

    /// One bit flipped in the body must be caught. A frame that gets through
    /// damaged is a ship reported in the wrong place, which is worse than one
    /// not reported at all.
    #[test]
    fn a_damaged_frame_is_refused() {
        let d = data(168, 5);
        let mut line = framed(&d);
        // Flip a level in the middle of the body; NRZI turns one flipped level
        // into two flipped bits, which is what a real error does too.
        let mid = line.len() / 2;
        line[mid] = !line[mid];
        let mut rx = Deframer::new();
        let mut out = Vec::new();
        for lvl in line {
            rx.push_level(lvl, &mut out);
        }
        assert!(out.is_empty(), "a corrupted frame was accepted");
        assert!(rx.rejects.contains(&Reject::BadFcs), "{:?}", rx.rejects);
    }

    /// The two packings are inverses, which is what lets a test transmitter and
    /// the check sequence agree about what an octet is.
    #[test]
    fn the_octet_packing_round_trips() {
        let bytes = [0x00u8, 0x7E, 0xFF, 0x5A, 0xA5];
        assert_eq!(octets_lsb_first(&bits_lsb_first(&bytes)), bytes);
        // ...and it really is least-significant-bit-first: a lone 1 bit at the
        // head of the stream is bit 0 of the first octet.
        assert_eq!(octets_lsb_first(&[true, false, false, false, false, false, false, false]), [1]);
    }
}
