//! The VDL2 bit scrambler.
//!
//! A 15-bit linear feedback shift register with the feedback polynomial
//! x^15 + x + 1, seeded to `0x6959` at the first bit of the header and run
//! straight through to the last bit of the data field. Its output is XORed onto
//! the transmitted bits, which is what stops a long run of identical symbols
//! putting a line in the spectrum and starving the receiver's timing recovery
//! of transitions.
//!
//! # One register per burst, not one per field
//!
//! The header and the data are two fields with two different error-correcting
//! codes, and it is tempting to treat them as two things to descramble. They are
//! not: the register runs continuously across the boundary, so the first data
//! bit is XORed with the twenty-sixth output of the *same* sequence. Restarting
//! it at the boundary leaves a header that decodes and a data field that never
//! does.
//!
//! # Descrambling is the same operation as scrambling
//!
//! The register is free-running — its state does not depend on the data — so
//! XORing twice restores the original. That also means an error in the received
//! bits is *not* multiplied: one wrong bit in, one wrong bit out, which is what
//! lets the Reed-Solomon layer above work in the first place. A self
//! synchronising scrambler, the kind `sdroxide-dsp`'s G3RUH modem uses, would
//! have tripled every error before the FEC saw it.
//!
//! Source: ETSI EN 301 841-1, the VDL Mode 2 physical layer.

/// The register's state at the first header bit.
pub const SEED: u16 = 0x6959;

/// The free-running scrambler.
#[derive(Debug, Clone)]
pub struct Lfsr(u16);

impl Default for Lfsr {
    fn default() -> Self {
        Lfsr::new()
    }
}

impl Lfsr {
    /// A register seeded for the start of a burst.
    pub fn new() -> Lfsr {
        Lfsr(SEED)
    }

    /// The next bit of the keystream.
    pub fn next_bit(&mut self) -> u8 {
        // x^15 + x + 1, shifting right: the new bit is the XOR of the two taps
        // and enters at the top.
        let bit = (self.0 ^ (self.0 >> 14)) & 1;
        self.0 = (self.0 >> 1) | (bit << 14);
        bit as u8
    }

    /// XOR the keystream onto `bits`, one entry per bit, advancing the register.
    ///
    /// Called twice on a burst — once for the header, once for the data — with
    /// the *same* register, which is the whole point of it being a method
    /// rather than a free function over the buffer.
    pub fn apply(&mut self, bits: &mut [u8]) {
        for b in bits.iter_mut() {
            *b ^= self.next_bit();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// x^15 + x + 1 is primitive, so the keystream is a maximal-length sequence:
    /// period 2^15 - 1, and exactly one more one than zero in it.
    ///
    /// This is the test that matters. A round trip — scramble, descramble,
    /// compare — passes just as happily with the taps in the wrong places or
    /// the register shifting the wrong way, and the failure would show up only
    /// as "nothing ever decodes on the air". This checks a property of the
    /// polynomial itself, which nothing about this implementation can fake.
    #[test]
    fn the_keystream_is_a_maximal_length_sequence() {
        let mut l = Lfsr::new();
        let mut ones = 0usize;
        let mut first: Option<u16> = None;
        let mut period = 0usize;
        for i in 0..70_000usize {
            if l.0 == SEED && i > 0 && period == 0 {
                period = i;
            }
            if first.is_none() {
                first = Some(l.0);
            }
            if i < 32_767 {
                ones += usize::from(l.next_bit());
            } else {
                l.next_bit();
            }
        }
        assert_eq!(period, 32_767, "not a maximal-length sequence");
        assert_eq!(ones, 16_384, "an m-sequence has one more one than zero");
    }

    /// The register never reaches the all-zeros state, which would lock it up.
    #[test]
    fn the_register_never_dies() {
        let mut l = Lfsr::new();
        for _ in 0..32_767 {
            assert_ne!(l.0, 0);
            l.next_bit();
        }
    }

    /// Descrambling is scrambling, and one bad bit in is one bad bit out —
    /// which is what the Reed-Solomon layer above is counting on.
    #[test]
    fn an_error_is_not_multiplied() {
        let plain: Vec<u8> = (0..200u16).map(|i| (i % 2) as u8).collect();
        let mut wire = plain.clone();
        Lfsr::new().apply(&mut wire);
        wire[73] ^= 1;
        let mut back = wire.clone();
        Lfsr::new().apply(&mut back);
        let wrong: Vec<usize> = (0..plain.len()).filter(|&i| plain[i] != back[i]).collect();
        assert_eq!(wrong, vec![73]);
    }

    /// The register runs across the header/data boundary rather than restarting.
    #[test]
    fn one_register_spans_the_whole_burst() {
        let mut whole: Vec<u8> = vec![0; 60];
        Lfsr::new().apply(&mut whole);

        let mut l = Lfsr::new();
        let mut header = vec![0u8; 25];
        let mut data = vec![0u8; 35];
        l.apply(&mut header);
        l.apply(&mut data);

        assert_eq!(&whole[..25], &header[..]);
        assert_eq!(&whole[25..], &data[..], "the register restarted at the boundary");
    }
}
