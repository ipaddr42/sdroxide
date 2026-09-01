//! The VDL2 transmission header: twenty-five bits that say how long the rest of
//! the burst is, protected by a code that can repair one of them.
//!
//! The header follows the synchronisation word and precedes the data. It is
//! three reserved bits, a seventeen-bit transmission length, and five parity
//! bits — twenty-five in all, which is deliberately *not* a multiple of the
//! three bits a D8PSK symbol carries. The data field therefore begins two bits
//! into the ninth symbol after the sync word, and a decoder that rounds up to a
//! symbol boundary reads a header that checks out perfectly and a data field
//! that never does.
//!
//! # The code, and why its shape is derived here rather than written down
//!
//! The standard publishes five parity rows. Everything else about the code —
//! that it corrects one bit, which word bit each syndrome names, which
//! syndromes name nothing, and even which end of the word the first bit on air
//! belongs at — is a *consequence* of those rows, and [`COLUMNS`] computes it.
//!
//! That last one matters. The five parity columns turn out to have weight one
//! and to sit at word bits 0..4; since the parity field is last on the air, the
//! word has to be accumulated most-significant-bit first. Had the accumulation
//! been the other way the code would still "work" — every syndrome would still
//! be computable — and it would correct the wrong bit every time. Deriving the
//! column positions and asserting them is what stops that being a guess.
//!
//! # Six syndromes name nothing
//!
//! Twenty-five of the thirty-one nonzero syndromes name a bit. The other six are
//! double errors the code can detect and cannot repair. Turning one of them into
//! a bit flip would be inventing a length out of noise, so the header is
//! rejected instead.
//!
//! Source: ETSI EN 301 841-1, the VDL Mode 2 transmission header.

/// Bits in the header, before the D8PSK symbols are packed.
pub const HEADER_BITS: usize = 25;
/// Width of the transmission-length field.
pub const TRLEN_BITS: usize = 17;
/// Width of the parity field.
pub const FEC_BITS: usize = 5;
/// Width of the reserved field, which the standard does not define a use for.
pub const RESERVED_BITS: usize = HEADER_BITS - TRLEN_BITS - FEC_BITS;

/// The parity rows. Row `i` produces syndrome bit `i` as the parity of the word
/// masked by it.
pub const H: [u32; FEC_BITS] = [
    0b0000000011111111111110000,
    0b0011111100001111111101000,
    0b1100011100110000111100100,
    0b1101101101010011001100010,
    0b0110100111100101010100001,
];

/// The longest transmission the standard allows, in data bits.
pub const TRLEN_MAX_BITS: u32 = 0x3FFF;

/// The longest transmission accepted from a header that needed repairing.
///
/// A single-bit correction may itself be a miscorrection — the code cannot tell
/// one bad bit from two — and a wrong bit high in the length field asks the
/// decoder to collect sixteen thousand bits that were never transmitted, which
/// costs it the burst and everything that arrives during it. Frames past this
/// length are vanishingly rare, so capping a repaired header here turns an
/// expensive miscorrection into a cheap rejection.
pub const TRLEN_MAX_BITS_CORRECTED: u32 = 0x1FFF;

/// Syndrome contributed by each bit of the word, derived from [`H`].
pub const COLUMNS: [u8; HEADER_BITS] = columns();

const fn columns() -> [u8; HEADER_BITS] {
    let mut cols = [0u8; HEADER_BITS];
    let mut j = 0;
    while j < HEADER_BITS {
        let mut c = 0u8;
        let mut i = 0;
        while i < FEC_BITS {
            if (H[i] >> j) & 1 == 1 {
                c |= 1 << i;
            }
            i += 1;
        }
        cols[j] = c;
        j += 1;
    }
    cols
}

/// Syndrome to the word bit it names, or [`NO_BIT`] when it names none.
const SYNDROME_TO_BIT: [u8; 32] = syndrome_to_bit();
/// A syndrome that names no single bit: a detected, uncorrectable error.
const NO_BIT: u8 = 0xff;

const fn syndrome_to_bit() -> [u8; 32] {
    let mut t = [NO_BIT; 32];
    let mut j = 0;
    while j < HEADER_BITS {
        // A zero column would mean a word bit the code cannot see at all, and
        // would claim syndrome zero. `the_code_corrects_exactly_one_bit` proves
        // there is none; this keeps the table honest if the rows ever change.
        if COLUMNS[j] != 0 {
            t[COLUMNS[j] as usize] = j as u8;
        }
        j += 1;
    }
    t
}

/// A header that checked out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Length of the data field in *bits*, not counting the Reed-Solomon parity
    /// that follows it.
    pub trlen_bits: u32,
    /// The three reserved bits, kept because a decoder that discards them
    /// cannot report a transmitter using them.
    pub reserved: u8,
    /// The code had to repair a bit. Everything downstream is a little less
    /// trustworthy, and the length limit is tighter.
    pub corrected: bool,
}

/// Why a header was not believed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderError {
    /// Not twenty-five bits.
    Short,
    /// The syndrome named no single bit: two or more errors.
    Uncorrectable,
    /// The length is longer than the standard allows, or longer than a repaired
    /// header is trusted for.
    LengthInsane(u32),
}

/// Reverse the low `n` bits of `x`.
pub fn reverse_bits(x: u32, n: usize) -> u32 {
    let mut out = 0;
    for i in 0..n {
        out |= ((x >> i) & 1) << (n - 1 - i);
    }
    out
}

/// The five-bit syndrome of a twenty-five-bit header word.
pub fn syndrome(word: u32) -> u8 {
    let mut s = 0u8;
    for (i, h) in H.iter().enumerate() {
        if (word & h).count_ones() & 1 == 1 {
            s |= 1 << i;
        }
    }
    s
}

/// Decode a header from its twenty-five descrambled bits, in the order they
/// arrived — one entry per bit, least significant bit of each entry used.
pub fn decode(bits: &[u8]) -> Result<Header, HeaderError> {
    if bits.len() != HEADER_BITS {
        return Err(HeaderError::Short);
    }
    // Most significant first: the first bit on the air is the top of the word,
    // which is what puts the parity field — last on the air — at bits 0..4.
    let mut word = 0u32;
    for &b in bits {
        word = (word << 1) | u32::from(b & 1);
    }

    let mut corrected = false;
    let s = syndrome(word);
    if s != 0 {
        let j = SYNDROME_TO_BIT[s as usize];
        if j == NO_BIT {
            return Err(HeaderError::Uncorrectable);
        }
        word ^= 1 << j;
        corrected = true;
    }

    let trlen = reverse_bits((word >> FEC_BITS) & ones(TRLEN_BITS), TRLEN_BITS);
    let limit = if corrected { TRLEN_MAX_BITS_CORRECTED } else { TRLEN_MAX_BITS };
    if trlen > limit {
        return Err(HeaderError::LengthInsane(trlen));
    }
    let reserved = ((word >> (FEC_BITS + TRLEN_BITS)) & ones(RESERVED_BITS)) as u8;
    Ok(Header { trlen_bits: trlen, reserved, corrected })
}

/// Build the twenty-five header bits for a transmission, in on-air order.
///
/// The transmitter's half, used by `crate::tx` and by the tests. Kept beside
/// [`decode`] so the two cannot disagree about which end of the word is which.
pub fn encode(trlen_bits: u32, reserved: u8) -> [u8; HEADER_BITS] {
    let mut word = (u32::from(reserved) & ones(RESERVED_BITS)) << (FEC_BITS + TRLEN_BITS)
        | (reverse_bits(trlen_bits & ones(TRLEN_BITS), TRLEN_BITS) << FEC_BITS);
    // The parity columns have weight one, so each syndrome bit is fixed by the
    // one word bit that produces it and nothing else.
    let s = syndrome(word);
    for i in 0..FEC_BITS {
        if s & (1 << i) != 0 {
            let j = SYNDROME_TO_BIT[1usize << i];
            debug_assert_ne!(j, NO_BIT, "parity column {i} is not weight one");
            word ^= 1 << j;
        }
    }
    debug_assert_eq!(syndrome(word), 0);

    let mut bits = [0u8; HEADER_BITS];
    for (i, b) in bits.iter_mut().enumerate() {
        *b = ((word >> (HEADER_BITS - 1 - i)) & 1) as u8;
    }
    bits
}

fn ones(n: usize) -> u32 {
    if n >= 32 { u32::MAX } else { (1u32 << n) - 1 }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The code's columns are distinct and nonzero, which is what makes it a
    /// single-error-correcting code rather than a checksum.
    ///
    /// Distinct: no two bits produce the same syndrome, so a syndrome names one
    /// bit unambiguously. Nonzero: no bit is invisible to the code.
    #[test]
    fn the_code_corrects_exactly_one_bit() {
        for (j, &c) in COLUMNS.iter().enumerate() {
            assert_ne!(c, 0, "bit {j} contributes to no syndrome");
        }
        for (a, &ca) in COLUMNS.iter().enumerate() {
            for (b, &cb) in COLUMNS.iter().enumerate().skip(a + 1) {
                assert_ne!(ca, cb, "bits {a} and {b} share a syndrome");
            }
        }
    }

    /// The five parity columns have weight one and sit at word bits 0..4.
    ///
    /// This is the fact that pins the packing direction. The parity field is
    /// last on the air; the only accumulation that puts it at the bottom of the
    /// word is most-significant-bit first. Get it the other way round and every
    /// syndrome still computes and every correction lands on the wrong bit.
    #[test]
    fn the_parity_bits_are_the_bottom_five_of_the_word() {
        for (i, &c) in COLUMNS.iter().take(FEC_BITS).enumerate() {
            assert_eq!(c, 1 << (FEC_BITS - 1 - i), "column {i}");
        }
    }

    /// Six of the thirty-one nonzero syndromes name no bit, and a header
    /// carrying one is rejected rather than repaired.
    #[test]
    fn the_unused_syndromes_are_refusals_not_repairs() {
        let unused: Vec<usize> = (1..32).filter(|&s| SYNDROME_TO_BIT[s] == NO_BIT).collect();
        assert_eq!(unused, vec![5, 9, 20, 22, 24, 29]);
        assert_eq!(SYNDROME_TO_BIT[0], NO_BIT, "syndrome zero names no error at all");
    }

    /// A clean header round-trips at every length the standard allows, and one
    /// flipped bit anywhere in it is repaired to the same answer.
    ///
    /// The flip sweep stops at [`TRLEN_MAX_BITS_CORRECTED`] on purpose: past
    /// that a repaired header is rejected by policy rather than by the code,
    /// which is the subject of its own test below.
    #[test]
    fn one_flipped_bit_is_repaired() {
        for trlen in [0u32, 1, 11 * 8, 0x1234, TRLEN_MAX_BITS] {
            let clean = encode(trlen, 0);
            let h = decode(&clean).expect("clean header");
            assert_eq!(h.trlen_bits, trlen);
            assert!(!h.corrected);
            if trlen > TRLEN_MAX_BITS_CORRECTED {
                continue;
            }

            for j in 0..HEADER_BITS {
                let mut bad = clean;
                bad[j] ^= 1;
                let h = decode(&bad).unwrap_or_else(|e| panic!("bit {j} of {trlen}: {e:?}"));
                assert_eq!(h.trlen_bits, trlen, "bit {j} repaired to the wrong length");
                assert!(h.corrected);
            }
        }
    }

    /// Two flipped bits either produce a detected failure or a wrong answer —
    /// but never a panic, and never a "clean" header, which would be the code
    /// claiming a confidence it does not have.
    #[test]
    fn two_flipped_bits_never_look_clean() {
        let clean = encode(0x0777, 0);
        for a in 0..HEADER_BITS {
            for b in (a + 1)..HEADER_BITS {
                let mut bad = clean;
                bad[a] ^= 1;
                bad[b] ^= 1;
                if let Ok(h) = decode(&bad) {
                    assert!(h.corrected, "a double error decoded as a clean header");
                }
            }
        }
    }

    /// The reserved bits survive the round trip, so a transmitter using them
    /// can be reported rather than silently ignored.
    #[test]
    fn the_reserved_bits_are_kept() {
        for r in 0..8u8 {
            let h = decode(&encode(96, r)).expect("clean");
            assert_eq!(h.reserved, r);
        }
    }

    /// A repaired header is held to the tighter limit; an unrepaired one is not.
    #[test]
    fn a_repaired_header_is_not_trusted_with_a_long_length() {
        let long = TRLEN_MAX_BITS_CORRECTED + 8;
        let clean = encode(long, 0);
        assert_eq!(decode(&clean).expect("clean").trlen_bits, long);

        // Flip a parity bit: the length is untouched but the header needed
        // repairing, so the tighter limit applies.
        let mut bad = clean;
        bad[HEADER_BITS - 1] ^= 1;
        assert_eq!(decode(&bad), Err(HeaderError::LengthInsane(long)));
    }

    /// The header is not a whole number of symbols, which is the arithmetic the
    /// data field's start depends on.
    #[test]
    fn the_header_does_not_end_on_a_symbol_boundary() {
        assert_eq!(HEADER_BITS % 3, 1, "the data field starts two bits into a symbol");
        assert_eq!(HEADER_BITS.div_ceil(3), 9);
    }

    /// Bit reversal is its own inverse over the field's width.
    #[test]
    fn reversing_twice_is_the_identity() {
        for x in [0u32, 1, 0x1FFFF, 0x0AAAA, 12345] {
            assert_eq!(reverse_bits(reverse_bits(x, TRLEN_BITS), TRLEN_BITS), x);
        }
        assert_eq!(reverse_bits(1, TRLEN_BITS), 1 << (TRLEN_BITS - 1));
    }
}
