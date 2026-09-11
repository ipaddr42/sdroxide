//! The HDLC frame the AVLC frame arrives inside: two `0x7E` flags and bit
//! stuffing.
//!
//! ```text
//!   8 bits        trlen − 16 bits         8 bits      0..7 bits
//! ┌────────┬──────────────────────────┬──────────┬─────────────┐
//! │  0x7E  │ the AVLC frame, stuffed  │   0x7E   │ zero padding│
//! └────────┴──────────────────────────┴──────────┴─────────────┘
//!  ◀──────── the transmission header's length field ─────────▶
//! ```
//!
//! # Why this exists at all, when the length is already known
//!
//! It should not have to. The transmission header says exactly how long the
//! data field is, so nothing has to hunt for a frame boundary — and this
//! decoder was written on the strength of that, taking the data field to be the
//! AVLC frame itself. On the air it is not: every transmission in a 24-second
//! recording of six channels carries the flags, and the great majority carry at
//! least one stuffed bit as well. Without this module the frame is one octet
//! out and every check sequence fails, which is what issue #265 was.
//!
//! The length field settles what the flags would otherwise have to: it is the
//! length of the field *including* both flags and every stuffed bit, which is
//! why it is stated in bits rather than octets — a stuffed frame is not a whole
//! number of octets. `ceil(trlen / 8)` is what the Reed-Solomon layer above
//! protects, and the last octet is padded with zeros (measured: every frame in
//! that recording, one to seven bits of it).
//!
//! # The stuffing
//!
//! A zero is inserted after every five consecutive ones, so that `0111_1110`
//! can only ever be a flag. Removing them again is the same rule read
//! backwards, with one wrinkle worth stating: a *sixth* one where the stuffed
//! zero should be is not data. It is the closing flag if a zero follows, and an
//! abort otherwise, and neither may be silently absorbed into the frame.
//!
//! # Why not `sdroxide_ax25::hdlc`
//!
//! That module is the same stuffing rule and a different job. It is a
//! *streaming* deframer for a modem that never stops: it hunts for flags in a
//! continuous bit stream, decodes NRZI, bounds the frame by AX.25's own limits
//! and checks and strips the frame check sequence itself. Here the field's
//! length is already known to the bit, there is exactly one frame in it, there
//! is no NRZI, and the caller needs the check sequence left on and needs "this
//! did not unwrap" told apart from "it unwrapped and the check failed" —
//! because it runs this twice on one transmission, over two readings of the
//! same octets. The one thing genuinely shared is the check sequence itself,
//! and `crate::avlc` already takes that from `sdroxide_ax25::fcs` rather than
//! writing it again.
//!
//! Source: ISO/IEC 13239 (HDLC) as ETSI EN 301 841-1 §5 applies it.

/// The flag octet, opening and closing.
pub const FLAG: u8 = 0x7E;

/// The flag as the eight bits it goes out as. Symmetrical, so which end of the
/// octet goes first does not arise here — but the frame between the flags is
/// least-significant-bit first, like every other HDLC descendant.
const FLAG_BITS: [u8; 8] = [0, 1, 1, 1, 1, 1, 1, 0];

/// Ones in a row before the transmitter inserts a zero.
const STUFF_AFTER: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HdlcError {
    /// The field does not begin with a flag.
    NoOpeningFlag,
    /// It ran out before one closed the frame.
    NoClosingFlag,
    /// Seven ones or more: the transmitter abandoned the frame part way
    /// through, and what came before it is not a frame.
    Abort,
    /// The destuffed frame is not a whole number of octets.
    Ragged,
}

fn is_flag(bits: &[u8]) -> bool {
    bits.len() >= 8 && bits[..8] == FLAG_BITS
}

/// Take the AVLC frame out of a data field: strip the flags, remove the stuffed
/// zeros, and pack what is left into octets.
///
/// `bits` is the data field in transmission order, one entry per bit — exactly
/// `trlen_bits` of them, since the padding past that is not part of the field.
pub fn unframe(bits: &[u8]) -> Result<Vec<u8>, HdlcError> {
    if !is_flag(bits) {
        return Err(HdlcError::NoOpeningFlag);
    }
    let mut i = 8;
    // Repeated flags are legal fill between frames, and cost nothing to allow.
    while is_flag(&bits[i.min(bits.len())..]) {
        i += 8;
    }
    let mut out: Vec<u8> = Vec::with_capacity(bits.len());
    let mut ones = 0u32;
    loop {
        // The stuffing is what makes this test safe: after it, `0111_1110`
        // cannot occur inside the frame, so the first one found is the closing
        // flag and nothing else.
        if is_flag(&bits[i.min(bits.len())..]) {
            break;
        }
        let Some(&b) = bits.get(i) else {
            return Err(HdlcError::NoClosingFlag);
        };
        if ones == STUFF_AFTER {
            // Only a zero can follow five ones. A one is six in a row, and the
            // flag that would have been was already taken above — so this is an
            // abort.
            if b != 0 {
                return Err(HdlcError::Abort);
            }
            i += 1;
            ones = 0;
            continue;
        }
        out.push(b);
        ones = if b == 1 { ones + 1 } else { 0 };
        i += 1;
    }
    if !out.len().is_multiple_of(8) {
        return Err(HdlcError::Ragged);
    }
    Ok(out
        .chunks_exact(8)
        .map(|c| c.iter().enumerate().fold(0u8, |o, (k, &b)| o | ((b & 1) << k)))
        .collect())
}

/// The transmitter's half: wrap `frame` in flags and stuff it.
///
/// Returns the bit stream, whose length is what the transmission header's
/// length field states. The caller pads it to a whole octet with zeros.
pub fn frame_bits(frame: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(frame.len() * 9 + 16);
    out.extend_from_slice(&FLAG_BITS);
    let mut ones = 0u32;
    for &o in frame {
        for k in 0..8 {
            let b = (o >> k) & 1;
            out.push(b);
            if b == 1 {
                ones += 1;
                if ones == STUFF_AFTER {
                    out.push(0);
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    }
    out.extend_from_slice(&FLAG_BITS);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The round trip, over the runs of ones that make stuffing happen at all.
    #[test]
    fn a_frame_survives_being_stuffed_and_unstuffed() {
        for frame in [
            vec![0x00, 0x01, 0x02],
            vec![0xff; 12],
            vec![0x7e, 0x7e, 0x7e],
            vec![0xf8, 0x1f, 0xff, 0x00, 0x3e],
            (0..=255u8).collect(),
        ] {
            let bits = frame_bits(&frame);
            assert_eq!(unframe(&bits), Ok(frame.clone()), "{frame:02x?}");
        }
    }

    /// Nothing between the flags may look like one, which is the whole reason
    /// the stuffing exists — and the reason the closing flag can be found by
    /// looking rather than by counting.
    #[test]
    fn no_flag_can_appear_inside_a_stuffed_frame() {
        let bits = frame_bits(&[0x7e, 0xfc, 0x3f, 0x7e, 0xff]);
        for i in 8..bits.len() - 8 {
            assert!(!is_flag(&bits[i..]), "a flag at {i} inside the frame");
        }
        // ...and the run of ones never reaches six anywhere.
        let mut ones = 0;
        for &b in &bits[8..bits.len() - 8] {
            ones = if b == 1 { ones + 1 } else { 0 };
            assert!(ones <= 5);
        }
    }

    /// A field that never closes, and one abandoned part way through, are
    /// refused rather than truncated into a short frame.
    #[test]
    fn an_unclosed_or_aborted_frame_is_refused() {
        let mut bits = frame_bits(&[0x11, 0x22, 0x33]);
        bits.truncate(bits.len() - 8);
        assert_eq!(unframe(&bits), Err(HdlcError::NoClosingFlag));

        let mut bits = frame_bits(&[0x11, 0x22, 0x33]);
        // Seven ones in the middle: the abort sequence.
        let at = 16;
        for k in 0..7 {
            bits[at + k] = 1;
        }
        assert_eq!(unframe(&bits), Err(HdlcError::Abort));

        assert_eq!(unframe(&[0, 0, 0, 0, 0, 0, 0, 0]), Err(HdlcError::NoOpeningFlag));
        assert_eq!(unframe(&[]), Err(HdlcError::NoOpeningFlag));
    }

    /// A frame that is not a whole number of octets never left a transmitter,
    /// so it is a decode that has gone wrong rather than a short frame.
    #[test]
    fn a_ragged_frame_is_refused() {
        let mut bits = frame_bits(&[0x11, 0x22, 0x33]);
        bits.insert(9, 0);
        assert_eq!(unframe(&bits), Err(HdlcError::Ragged));
    }

    /// The length the header states is the length of *this*, flags and stuffing
    /// included — which is why it is in bits.
    #[test]
    fn the_stated_length_counts_the_flags_and_the_stuffing() {
        // No run of five ones anywhere, so nothing is stuffed: two flags and
        // the frame.
        let plain = [0x14u8, 0x62, 0x04, 0x58, 0x52, 0x40, 0x20, 0x07, 0xe1, 0x5e, 0x6a];
        assert_eq!(frame_bits(&plain).len(), 16 + plain.len() * 8);
        // That is the 104-bit field measured off the air (issue #265): eleven
        // octets of frame between two flags, 13 octets in all.
        assert_eq!(frame_bits(&plain).len(), 104);
        // ...and one that does stuff is longer than its octets.
        assert!(frame_bits(&[0xff; 11]).len() > 16 + 11 * 8);
    }
}
