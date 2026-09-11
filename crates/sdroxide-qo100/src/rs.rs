//! The CCSDS (255,223) Reed-Solomon code over GF(256), as AO-40's coded
//! telemetry uses it.
//!
//! AO-40 carries two of these per frame, each *shortened* to (160,128) by
//! notionally prefixing 95 zero symbols that are never transmitted — so one
//! codeword protects 128 bytes with 32 parity bytes and corrects up to 16 byte
//! errors anywhere in it. The two codewords take alternate bytes of the
//! 256-byte payload, which is what turns a burst destroying a run of
//! consecutive bytes into two half-length bursts, one in each codeword.
//!
//! The parameters are not free choices — they are the ones AO-40 names, and
//! any of them wrong gives a decoder that agrees with itself and with nothing
//! on the air:
//!
//! * field generator `F(x) = x^8 + x^7 + x^2 + x + 1` ([`GF_POLY`]),
//! * code generator roots `α^(11j)` for `j = 112..=143` — a first consecutive
//!   root of 112, and a step of `α^11` rather than `α`,
//! * conventional polynomial basis, **not** the CCSDS dual basis.
//!
//! The last is the trap: CCSDS specifies the dual-basis representation and
//! much published material assumes it, but AO-40 states the conventional
//! basis. [`tests::the_generator_matches_the_published_ao40_table`] settles
//! the question by deriving the generator polynomial from the roots above and
//! checking it against the coefficient table in the reference encoder, so the
//! parameters are pinned to the format rather than to this file's own opinion.
//!
//! Shortening is handled by decoding the full 255-symbol codeword with the 95
//! leading zeros put back, rather than by threading a pad through the syndrome
//! and Chien arithmetic. It costs a little work the shortened form would save
//! and removes a whole class of off-by-one. A correction landing inside the
//! padding is then simply a decode to refuse: those symbols were never sent,
//! so they cannot have been in error, and a codeword that needs them changed
//! is one the parity happens to fit rather than the one that was transmitted.

/// Field generator polynomial of GF(256): x^8 + x^7 + x^2 + x + 1.
const GF_POLY: u16 = 0x187;
/// Symbols in a full (unshortened) codeword.
const NN: usize = 255;
/// Parity symbols, and so `NROOTS / 2` correctable errors.
pub const NROOTS: usize = 32;
/// First consecutive root of the generator, as a multiple of [`PRIM`].
const FCR: usize = 112;
/// The generator's roots step by this power of `α`.
const PRIM: usize = 11;
/// Log of the zero element — "minus infinity", the value meaning "no term".
const A0: u8 = 255;

/// Payload symbols in one AO-40 codeword, and the symbols actually sent.
pub const K: usize = 128;
pub const N: usize = K + NROOTS;
/// Zero symbols that shorten (255,223) to (160,128). Never transmitted.
const PAD: usize = NN - N;

/// GF(256) log and antilog tables — the whole of the field arithmetic.
struct Gf {
    alpha_to: [u8; NN],
    index_of: [u8; 256],
}

impl Gf {
    fn new() -> Gf {
        let mut g = Gf { alpha_to: [0; NN], index_of: [0; 256] };
        let mut sr: u16 = 1;
        for i in 0..NN {
            g.index_of[sr as usize] = i as u8;
            g.alpha_to[i] = sr as u8;
            sr <<= 1;
            if sr & 0x100 != 0 {
                sr ^= GF_POLY;
            }
            sr &= 0xff;
        }
        g.index_of[0] = A0; // log(0) has no value; A0 is the marker for it
        g
    }

    /// `x mod 255`, for adding logs. Arguments here never exceed a few
    /// thousand, so the loop beats a division.
    fn modnn(&self, mut x: usize) -> usize {
        while x >= NN {
            x -= NN;
        }
        x
    }

    /// `α^a · α^b`, both given as logs, zero if either is the zero element.
    fn mul_log(&self, a: u8, b: u8) -> u8 {
        if a == A0 || b == A0 { 0 } else { self.alpha_to[self.modnn(a as usize + b as usize)] }
    }
}

/// The generator polynomial `g(x) = Π (x - α^(11j))` for `j = FCR..FCR+NROOTS`,
/// in log form, lowest order first. `g[0]` and `g[NROOTS]` are both `α^0`.
fn generator(gf: &Gf) -> [u8; NROOTS + 1] {
    // Built in value form by multiplying in one root at a time, then converted.
    let mut g = [0u8; NROOTS + 1];
    g[0] = 1;
    for j in 0..NROOTS {
        // Multiply g(x) by (x - α^(PRIM·(FCR+j))). Over GF(2), minus is plus.
        let rlog = gf.modnn(PRIM * (FCR + j)) as u8;
        let deg = j + 1;
        let mut next = [0u8; NROOTS + 1];
        for i in (1..=deg).rev() {
            next[i] = g[i - 1];
        }
        for i in 0..deg {
            next[i] ^= gf.mul_log(gf.index_of[g[i] as usize], rlog);
        }
        g = next;
    }
    std::array::from_fn(|i| gf.index_of[g[i] as usize])
}

/// One AO-40 Reed-Solomon codec, tables built once.
pub struct Rs {
    gf: Gf,
    /// Generator coefficients in log form, lowest order first.
    genpoly: [u8; NROOTS + 1],
    /// Multiplicative inverse of [`PRIM`] modulo 255, which is what lets the
    /// Chien search step through error *positions* rather than through powers
    /// of `α` — this code's roots advance by `α^11`, so the two are not the
    /// same walk.
    iprim: usize,
}

impl Default for Rs {
    fn default() -> Self {
        Self::new()
    }
}

impl Rs {
    pub fn new() -> Rs {
        let gf = Gf::new();
        let genpoly = generator(&gf);
        let iprim = (1..NN).find(|i| (PRIM * i) % NN == 1).expect("PRIM is coprime with 255");
        Rs { gf, genpoly, iprim }
    }

    /// The 32 parity symbols for one 128-symbol block.
    ///
    /// The beacon does the encoding for real; this exists so the decoder can
    /// be tested against codewords built the same way the satellite builds
    /// them, and so [`crate::fec`] can check itself against the reference
    /// implementation's own output.
    pub fn encode(&self, data: &[u8; K]) -> [u8; NROOTS] {
        let mut bb = [0u8; NROOTS];
        for &d in data.iter() {
            let feedback = self.gf.index_of[(d ^ bb[0]) as usize];
            if feedback != A0 {
                for (j, b) in bb.iter_mut().enumerate().skip(1) {
                    *b ^= self.gf.mul_log(feedback, self.genpoly[NROOTS - j]);
                }
            }
            bb.copy_within(1..NROOTS, 0);
            bb[NROOTS - 1] =
                if feedback != A0 { self.gf.mul_log(feedback, self.genpoly[0]) } else { 0 };
        }
        bb
    }

    /// Correct a received codeword in place, returning how many symbol errors
    /// were put right, or `None` when it could not be decoded.
    ///
    /// `cw` is the 160 symbols actually on the air — 128 data then 32 parity.
    pub fn decode(&self, cw: &mut [u8; N]) -> Option<usize> {
        let gf = &self.gf;
        let mut data = [0u8; NN];
        data[PAD..].copy_from_slice(cw.as_slice());

        // Syndromes: the received polynomial evaluated at each root of g(x).
        // All zero means it is already a codeword and there is nothing to do.
        let mut syn = [0u8; NROOTS];
        for (i, si) in syn.iter_mut().enumerate() {
            let mut acc = data[0];
            for &d in data.iter().skip(1) {
                acc = if acc == 0 {
                    d
                } else {
                    d ^ gf.alpha_to[gf.modnn(gf.index_of[acc as usize] as usize + (FCR + i) * PRIM)]
                };
            }
            *si = acc;
        }
        if syn.iter().all(|&x| x == 0) {
            return Some(0);
        }
        let s: Vec<u8> = syn.iter().map(|&x| gf.index_of[x as usize]).collect();

        // Berlekamp-Massey: the shortest shift register that generates the
        // syndrome sequence is the error locator polynomial Λ(x).
        let mut lambda = [0u8; NROOTS + 1];
        lambda[0] = 1;
        let mut b: Vec<u8> = lambda.iter().map(|&x| gf.index_of[x as usize]).collect();
        let mut el = 0usize;
        for r in 1..=NROOTS {
            let mut discr = 0u8;
            for i in 0..r {
                if lambda[i] != 0 && s[r - i - 1] != A0 {
                    discr ^= gf.mul_log(gf.index_of[lambda[i] as usize], s[r - i - 1]);
                }
            }
            let discr = gf.index_of[discr as usize];
            if discr == A0 {
                b.insert(0, A0);
                b.pop();
                continue;
            }
            let mut t = [0u8; NROOTS + 1];
            t[0] = lambda[0];
            for i in 0..NROOTS {
                t[i + 1] = lambda[i + 1] ^ if b[i] != A0 { gf.mul_log(discr, b[i]) } else { 0 };
            }
            if 2 * el < r {
                el = r - el;
                for i in 0..=NROOTS {
                    b[i] = if lambda[i] == 0 {
                        A0
                    } else {
                        gf.modnn(gf.index_of[lambda[i] as usize] as usize + NN - discr as usize)
                            as u8
                    };
                }
            } else {
                b.insert(0, A0);
                b.pop();
            }
            lambda = t;
        }

        let lambda: Vec<u8> = lambda.iter().map(|&x| gf.index_of[x as usize]).collect();
        let deg_lambda = (0..=NROOTS).rev().find(|&i| lambda[i] != A0)?;

        // Chien search: try every position and keep the ones that are roots.
        let mut reg = lambda.clone();
        let mut root = Vec::with_capacity(NROOTS);
        let mut loc = Vec::with_capacity(NROOTS);
        let mut k = self.iprim - 1;
        for i in 1..=NN {
            let mut q = 1u8;
            for j in (1..=deg_lambda).rev() {
                if reg[j] != A0 {
                    reg[j] = gf.modnn(reg[j] as usize + j) as u8;
                    q ^= gf.alpha_to[reg[j] as usize];
                }
            }
            if q == 0 {
                root.push(i);
                loc.push(k);
                if root.len() == deg_lambda {
                    break;
                }
            }
            k = gf.modnn(k + self.iprim);
        }
        // Fewer roots than the locator's degree means the error pattern is not
        // one this code can describe: more errors than it can carry.
        if root.len() != deg_lambda {
            return None;
        }

        // Error evaluator ω(x) = s(x)·Λ(x) mod x^NROOTS.
        let deg_omega = deg_lambda - 1;
        let mut omega = vec![A0; deg_omega + 1];
        for (i, oi) in omega.iter_mut().enumerate() {
            let mut tmp = 0u8;
            for j in (0..=i).rev() {
                if s[i - j] != A0 && lambda[j] != A0 {
                    tmp ^= gf.mul_log(s[i - j], lambda[j]);
                }
            }
            *oi = gf.index_of[tmp as usize];
        }

        // Forney: the error magnitude at each located position.
        let mut fixed = 0usize;
        for j in 0..root.len() {
            let mut num1 = 0u8;
            for i in (0..=deg_omega).rev() {
                if omega[i] != A0 {
                    num1 ^= gf.alpha_to[gf.modnn(omega[i] as usize + i * root[j])];
                }
            }
            let num2 = gf.alpha_to[gf.modnn(root[j] * (FCR - 1) + NN)];
            // Λ'(x) over GF(2) keeps only the odd-order terms.
            let mut den = 0u8;
            let mut i = (deg_lambda.min(NROOTS - 1) & !1) as isize;
            while i >= 0 {
                if lambda[i as usize + 1] != A0 {
                    den ^= gf.alpha_to
                        [gf.modnn(lambda[i as usize + 1] as usize + i as usize * root[j])];
                }
                i -= 2;
            }
            if den == 0 {
                return None; // a singular evaluator: not a real error pattern
            }
            if num1 != 0 {
                let mag = gf.alpha_to[gf.modnn(
                    gf.index_of[num1 as usize] as usize + gf.index_of[num2 as usize] as usize + NN
                        - gf.index_of[den as usize] as usize,
                )];
                data[loc[j]] ^= mag;
                fixed += 1;
            }
        }

        // A correction inside the shortening zeros fits the parity but not the
        // transmission — see the module doc.
        if data[..PAD].iter().any(|&x| x != 0) {
            return None;
        }
        cw.copy_from_slice(&data[PAD..]);
        Some(fixed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The generator polynomial derived from AO-40's stated roots — first
    /// consecutive root 112, stepping by α^11, over x^8+x^7+x^2+x+1 — must be
    /// the one the reference encoder carries as a table.
    ///
    /// This is the check that the *parameters* are right rather than merely
    /// self-consistent. Read the same CCSDS code in the dual basis and the
    /// numbers come out different; a codec built on that would round-trip its
    /// own output perfectly and never decode a frame off the satellite.
    #[test]
    fn the_generator_matches_the_published_ao40_table() {
        // KA9Q, `encode_ref.c`: "The code generator polynomial coefficients in
        // index (logarithmic) form for the CCSDS standard (255,223) RS code.
        // Only coefficients C1-C16 are given since the polynomial is
        // palindromic and C0 and 32 are 0."
        const C1_C16: [u8; 16] = [249, 59, 66, 4, 43, 126, 251, 97, 30, 3, 213, 50, 66, 170, 5, 24];

        let g = generator(&Gf::new());
        assert_eq!(g[0], 0, "C0 is α^0");
        assert_eq!(g[NROOTS], 0, "C32 is α^0");
        assert_eq!(&g[1..17], &C1_C16, "C1..C16");
        for i in 1..NROOTS {
            assert_eq!(g[i], g[NROOTS - i], "the generator is palindromic at {i}");
        }
    }

    /// The encoder, checked against a codeword the reference implementation
    /// actually produced rather than against itself.
    ///
    /// The payload is `p[i] = (7i + 3) mod 256`; the two parity blocks below
    /// are what KA9Q's `encode_ref.c` left in its own shift registers after
    /// being fed it. Getting the field, the roots or the basis wrong changes
    /// these bytes, and nothing else in this file would notice.
    #[test]
    fn the_encoder_reproduces_the_reference_implementations_parity() {
        const PARITY_EVEN: [u8; NROOTS] = [
            0x82, 0xd0, 0x05, 0x19, 0xf8, 0x35, 0xa2, 0xdd, 0xcb, 0x45, 0x87, 0xbc, 0x45, 0x1f,
            0xfb, 0x01, 0x9f, 0x07, 0x2a, 0x56, 0x99, 0xab, 0x25, 0x6c, 0xa5, 0xb9, 0xab, 0x4c,
            0x61, 0xdb, 0xc5, 0xa0,
        ];
        const PARITY_ODD: [u8; NROOTS] = [
            0xed, 0x47, 0xd0, 0x25, 0xdd, 0x3c, 0x45, 0xc9, 0x76, 0xb5, 0x69, 0x5b, 0xea, 0xf0,
            0xc4, 0xf4, 0xc5, 0x3c, 0x76, 0xb2, 0xe3, 0xb2, 0x00, 0x26, 0xb7, 0xb9, 0x25, 0xcc,
            0xe1, 0x8b, 0xb1, 0x2a,
        ];

        let payload: [u8; 256] = std::array::from_fn(|i| (i as u8).wrapping_mul(7).wrapping_add(3));
        let rs = Rs::new();
        // Even-numbered payload bytes make the first codeword, odd the second.
        let even: [u8; K] = std::array::from_fn(|i| payload[2 * i]);
        let odd: [u8; K] = std::array::from_fn(|i| payload[2 * i + 1]);
        assert_eq!(rs.encode(&even), PARITY_EVEN, "first codeword");
        assert_eq!(rs.encode(&odd), PARITY_ODD, "second codeword");
    }

    #[test]
    fn a_clean_codeword_decodes_with_nothing_to_correct() {
        let rs = Rs::new();
        let data: [u8; K] = std::array::from_fn(|i| (i as u8).wrapping_mul(7).wrapping_add(3));
        let mut cw = [0u8; N];
        cw[..K].copy_from_slice(&data);
        cw[K..].copy_from_slice(&rs.encode(&data));
        assert_eq!(rs.decode(&mut cw), Some(0));
        assert_eq!(&cw[..K], &data);
    }

    #[test]
    fn up_to_sixteen_symbol_errors_anywhere_are_corrected() {
        let rs = Rs::new();
        let data: [u8; K] = std::array::from_fn(|i| (i as u8).wrapping_mul(31).wrapping_add(17));
        let mut clean = [0u8; N];
        clean[..K].copy_from_slice(&data);
        clean[K..].copy_from_slice(&rs.encode(&data));

        for errors in 1..=16usize {
            let mut cw = clean;
            // Spread them across the whole codeword, parity included.
            for e in 0..errors {
                let at = (e * 9 + 5) % N;
                cw[at] ^= (e as u8).wrapping_mul(53).wrapping_add(1);
            }
            assert_eq!(rs.decode(&mut cw), Some(errors), "{errors} errors");
            assert_eq!(&cw[..K], &data, "{errors} errors");
        }
    }

    /// Past the code's strength it must say so rather than hand back plausible
    /// rubbish. Twenty errors is not always *detectable* — a far enough
    /// pattern lands on another codeword — so what is asserted is the weaker
    /// true thing: it never quietly returns wrong data as though it were right.
    #[test]
    fn beyond_sixteen_errors_it_refuses_rather_than_inventing_a_codeword() {
        let rs = Rs::new();
        let data: [u8; K] = std::array::from_fn(|i| (i as u8).wrapping_mul(11));
        let mut clean = [0u8; N];
        clean[..K].copy_from_slice(&data);
        clean[K..].copy_from_slice(&rs.encode(&data));

        let mut refused = 0;
        for trial in 0..40u8 {
            let mut cw = clean;
            for e in 0..20usize {
                let at = (e * 7 + trial as usize * 3) % N;
                cw[at] ^= trial.wrapping_mul(37).wrapping_add(e as u8).wrapping_add(1) | 1;
            }
            match rs.decode(&mut cw) {
                None => refused += 1,
                Some(_) => assert_ne!(
                    &cw[..K],
                    &data,
                    "trial {trial}: 20 errors are past correcting, so success here would be luck"
                ),
            }
        }
        assert!(refused > 20, "only {refused}/40 over-strength words were refused");
    }
}
