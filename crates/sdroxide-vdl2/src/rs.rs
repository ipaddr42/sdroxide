//! The Reed-Solomon code VDL2 protects its data field with: RS(255, 249) over
//! GF(256), shortened, with two, four or six parity symbols depending on how
//! long the block is.
//!
//! # Why this is written here rather than taken from a crate
//!
//! The parameters. The field polynomial is x^8 + x^7 + x^2 + x + 1 (`0x187`)
//! and the generator's roots start at α^120, which is unusual: the
//! Reed-Solomon crates on crates.io are written for the QR-code and CCSDS
//! parameter sets and hard-code a first consecutive root of 0 or 1 and the
//! field polynomial `0x11d`. A first root of 120 is also close enough to
//! CCSDS's 112 to be exactly the sort of thing that gets copied by mistake, so
//! [`generator`] derives the polynomial from the three published numbers and a
//! test asserts the coefficients that come out.
//!
//! # Shortening, and the correction that must not happen
//!
//! A block shorter than 255 symbols is the same code with high-order zeros in
//! front that are never transmitted. The syndromes do not need to know: leading
//! zeros contribute nothing to a Horner evaluation, so the sum over the
//! transmitted symbols is already right.
//!
//! The error locations do need to know. A root of the error locator that lands
//! in the virtual padding names a symbol that was never sent, which cannot
//! happen in a real codeword — it is the signature of a *miscorrection*, a
//! syndrome pattern that looks like a small error set and is not. Some
//! implementations skip such a root and apply the rest; this one rejects the
//! block. Applying half of a miscorrection produces a codeword that is wrong in
//! a way nothing downstream can see, and with only two parity symbols on a
//! short block, miscorrections are not rare.
//!
//! Sources: ETSI EN 301 841-1 for the parameters; the decoder is
//! Berlekamp-Massey, Chien and Forney as they are given in any coding text.

/// The field polynomial, x^8 + x^7 + x^2 + x + 1.
pub const POLY: u16 = 0x187;
/// The first consecutive root of the generator: α^120.
pub const FCR: usize = 120;
/// The primitive element's step between roots.
pub const PRIM: usize = 1;
/// The unshortened code length.
pub const N: usize = 255;
/// The unshortened code's data length.
pub const K: usize = 249;

/// α^i for i in 0..510, so a product of two logarithms indexes directly.
const EXP: [u8; 510] = exp_table();
/// The base-α logarithm. `LOG[0]` is meaningless and never read.
const LOG: [u8; 256] = log_table();

const fn exp_table() -> [u8; 510] {
    let mut t = [0u8; 510];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        t[i] = x as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= POLY;
        }
        i += 1;
    }
    let mut i = 255;
    while i < 510 {
        t[i] = t[i - 255];
        i += 1;
    }
    t
}

const fn log_table() -> [u8; 256] {
    let mut t = [0u8; 256];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        t[x as usize] = i as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= POLY;
        }
        i += 1;
    }
    t
}

/// α raised to `i`, for any `i`.
#[inline]
pub fn alpha(i: usize) -> u8 {
    EXP[i % 255]
}

#[inline]
fn mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 { 0 } else { EXP[LOG[a as usize] as usize + LOG[b as usize] as usize] }
}

#[inline]
fn div(a: u8, b: u8) -> u8 {
    debug_assert_ne!(b, 0, "division by zero in GF(256)");
    if a == 0 { 0 } else { EXP[LOG[a as usize] as usize + 255 - LOG[b as usize] as usize] }
}

/// Evaluate a polynomial given lowest coefficient first, at `x`.
fn eval(coeffs: &[u8], x: u8) -> u8 {
    let mut acc = 0u8;
    for &c in coeffs.iter().rev() {
        acc = mul(acc, x) ^ c;
    }
    acc
}

/// The generator polynomial for `nroots` parity symbols, lowest coefficient
/// first — so the last entry is the leading 1.
///
/// Derived from [`POLY`], [`FCR`] and [`PRIM`] rather than tabulated, because
/// the table is the thing most likely to be wrong and a derivation can be
/// checked against the roots it is supposed to have.
pub fn generator(nroots: usize) -> Vec<u8> {
    let mut g = vec![0u8; nroots + 1];
    g[0] = 1;
    for i in 0..nroots {
        let root = alpha(FCR + i * PRIM);
        // Multiply g by (x + root). In GF(2) subtraction is addition, so the
        // monic factor for a root r is (x + r).
        for k in (1..=(i + 1)).rev() {
            g[k] = g[k - 1] ^ mul(root, g[k]);
        }
        g[0] = mul(root, g[0]);
    }
    g
}

/// Why a block could not be repaired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RsError {
    /// The block is not a length this code can describe.
    BadLength,
    /// More errors than the parity can locate, or a locator root that names a
    /// symbol outside the block.
    Uncorrectable,
}

/// Repair one shortened block in place: `data` symbols then `nroots` parity
/// symbols, all in transmission order. Returns how many symbols were changed.
///
/// `nroots == 0` is a real case — VDL2 gives a final block of one or two octets
/// no parity at all — and means "nothing to check", not "degenerate".
pub fn decode_block(block: &mut [u8], nroots: usize) -> Result<usize, RsError> {
    if nroots == 0 {
        return Ok(0);
    }
    let len = block.len();
    if len <= nroots || len > N {
        return Err(RsError::BadLength);
    }
    let pad = N - len;

    // Syndromes. Horner over the transmitted symbols; the virtual leading zeros
    // of a shortened block contribute nothing, so they need no handling here.
    let mut s = vec![0u8; nroots];
    let mut clean = true;
    for (i, si) in s.iter_mut().enumerate() {
        let r = alpha(FCR + i * PRIM);
        let mut acc = 0u8;
        for &b in block.iter() {
            acc = mul(acc, r) ^ b;
        }
        *si = acc;
        clean &= acc == 0;
    }
    if clean {
        return Ok(0);
    }

    // Berlekamp-Massey, in value form. `nroots` is at most six, so the extra
    // multiplies against an index-form implementation are not worth the
    // opportunity to get a logarithm the wrong way round.
    let mut lambda = vec![0u8; nroots + 1];
    lambda[0] = 1;
    let mut prev = lambda.clone();
    let mut l = 0usize;
    let mut m = 1usize;
    let mut prev_d = 1u8;
    for n in 0..nroots {
        let mut d = s[n];
        for i in 1..=l {
            d ^= mul(lambda[i], s[n - i]);
        }
        if d == 0 {
            m += 1;
        } else {
            let coef = div(d, prev_d);
            let updated: Vec<u8> = lambda.clone();
            for i in m..lambda.len() {
                lambda[i] ^= mul(coef, prev[i - m]);
            }
            if 2 * l <= n {
                l = n + 1 - l;
                prev = updated;
                prev_d = d;
                m = 1;
            } else {
                m += 1;
            }
        }
    }

    let deg_lambda = lambda.iter().rposition(|&c| c != 0).unwrap_or(0);
    if l > nroots / 2 || deg_lambda != l {
        return Err(RsError::Uncorrectable);
    }

    // Chien search. `q = Λ(α^i)`; a root at α^i puts the error at exponent
    // N - i, which is symbol `i - 1 - pad` of the transmitted block.
    let mut roots: Vec<(usize, usize)> = Vec::with_capacity(deg_lambda);
    for i in 1..=N {
        if eval(&lambda, alpha(i)) != 0 {
            continue;
        }
        let loc = i - 1;
        if loc < pad {
            // A root in the padding names a symbol that was never transmitted.
            // Real errors cannot land there, so this is a miscorrection.
            return Err(RsError::Uncorrectable);
        }
        let idx = loc - pad;
        if idx >= len {
            return Err(RsError::Uncorrectable);
        }
        roots.push((i, idx));
    }
    if roots.len() != deg_lambda {
        // Fewer distinct roots than the locator's degree: the locator does not
        // factor over the field, so it did not come from a correctable error.
        return Err(RsError::Uncorrectable);
    }

    // Ω(x) = S(x)·Λ(x) mod x^nroots.
    let mut omega = vec![0u8; deg_lambda];
    for (i, oi) in omega.iter_mut().enumerate() {
        let mut acc = 0u8;
        for j in 0..=i.min(deg_lambda) {
            acc ^= mul(s[i - j], lambda[j]);
        }
        *oi = acc;
    }

    // Λ'(x), which in characteristic two keeps only the odd-power terms.
    let mut lambda_pr = vec![0u8; deg_lambda];
    for i in (1..=deg_lambda).step_by(2) {
        lambda_pr[i - 1] = lambda[i];
    }

    let mut corrected = 0usize;
    for &(i, idx) in &roots {
        let xinv = alpha(i);
        let num = mul(eval(&omega, xinv), alpha(i * (FCR - 1) + N));
        let den = eval(&lambda_pr, xinv);
        if den == 0 {
            return Err(RsError::Uncorrectable);
        }
        let e = div(num, den);
        if e != 0 {
            block[idx] ^= e;
            corrected += 1;
        }
    }

    // Nothing proves a repair like the syndromes coming out zero afterwards.
    // Six extra Horner passes on a block that was already broken, and it turns
    // every miscorrection this code can make into a clean refusal.
    for i in 0..nroots {
        let r = alpha(FCR + i * PRIM);
        let mut acc = 0u8;
        for &b in block.iter() {
            acc = mul(acc, r) ^ b;
        }
        if acc != 0 {
            return Err(RsError::Uncorrectable);
        }
    }
    Ok(corrected)
}

/// The transmitter's half: `data` followed by `nroots` parity symbols.
///
/// Here rather than in a test so that `crate::tx` and the decoder cannot end up
/// disagreeing about the parameters — the failure that a round-trip test is
/// famously blind to.
pub fn encode_block(data: &[u8], nroots: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + nroots);
    out.extend_from_slice(data);
    if nroots == 0 {
        return out;
    }
    let g = generator(nroots);
    let mut par = vec![0u8; nroots];
    for &d in data {
        let fb = d ^ par[0];
        if fb != 0 {
            for j in 1..nroots {
                par[j] ^= mul(fb, g[nroots - j]);
            }
        }
        par.rotate_left(1);
        par[nroots - 1] = if fb != 0 { mul(fb, g[0]) } else { 0 };
    }
    out.extend_from_slice(&par);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic generator, because there is no `rand` in this tree and a
    /// test that fails one run in fifty is worse than no test.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
        fn byte(&mut self) -> u8 {
            (self.next() >> 24) as u8
        }
    }

    /// `0x187` really is primitive, so the 255 nonzero elements are α's powers
    /// and every logarithm in this file means what it says.
    ///
    /// A property of the polynomial, checkable without knowing anything about
    /// VDL2 — and the thing a round-trip test cannot see, because a non-
    /// primitive polynomial still gives a consistent (if much smaller) group.
    #[test]
    fn the_field_polynomial_is_primitive() {
        let mut seen = [false; 256];
        for (i, &x) in EXP.iter().take(255).enumerate() {
            assert_ne!(x, 0, "alpha^{i} is zero");
            assert!(!seen[x as usize], "alpha^{i} repeats an earlier power");
            seen[x as usize] = true;
            assert_eq!(LOG[x as usize] as usize, i);
        }
        assert_eq!(EXP[255], 1, "alpha^255 is not 1");
    }

    /// The generator polynomials the three published parameters produce.
    ///
    /// These are the numbers to compare against the table printed in
    /// EN 301 841-1: they depend on the field polynomial, the first consecutive
    /// root and the primitive step together, so agreeing with the standard here
    /// is an external check on all three at once — and in particular on
    /// `FCR = 120`, which is the value most likely to be wrong.
    #[test]
    fn the_generator_polynomials_are_what_the_parameters_give() {
        assert_eq!(generator(2), vec![0x66, 0xA4, 0x01]);
        assert_eq!(generator(4), vec![0xD2, 0xB4, 0xE8, 0xBD, 0x01]);
        assert_eq!(generator(6), vec![0x17, 0x82, 0xD9, 0x3E, 0x63, 0xD9, 0x01]);
    }

    /// ...and its roots really are α^120 upwards, which is the same fact stated
    /// where a mistyped coefficient above cannot hide it.
    #[test]
    fn the_generators_roots_start_at_alpha_120() {
        for nroots in [2usize, 4, 6] {
            let g = generator(nroots);
            for i in 0..nroots {
                assert_eq!(eval(&g, alpha(FCR + i * PRIM)), 0, "{nroots} roots, root {i}");
            }
            // ...and not one further along, which a generator built with the
            // wrong first root would still satisfy above.
            assert_ne!(eval(&g, alpha(FCR + nroots * PRIM)), 0, "{nroots} roots has one too many");
            assert_ne!(eval(&g, alpha(FCR - 1)), 0, "{nroots} roots starts too early");
        }
    }

    /// A clean block decodes to itself with nothing corrected.
    #[test]
    fn a_clean_block_is_left_alone() {
        let mut rng = Rng(0x5eed_1234);
        for &(k, nroots) in &[(11usize, 2usize), (30, 2), (31, 4), (68, 6), (249, 6)] {
            let data: Vec<u8> = (0..k).map(|_| rng.byte()).collect();
            let mut block = encode_block(&data, nroots);
            assert_eq!(decode_block(&mut block, nroots), Ok(0));
            assert_eq!(&block[..k], &data[..]);
        }
    }

    /// Correction reaches exactly `nroots / 2` symbols, and one more is refused
    /// rather than silently mangled.
    #[test]
    fn correction_reaches_the_limit_and_stops() {
        let mut rng = Rng(0xc0ffee);
        for &nroots in &[2usize, 4, 6] {
            let t = nroots / 2;
            let k = 100;
            for _ in 0..200 {
                let data: Vec<u8> = (0..k).map(|_| rng.byte()).collect();
                let clean = encode_block(&data, nroots);

                let mut block = clean.clone();
                let mut hit = Vec::new();
                while hit.len() < t {
                    let p = rng.below(block.len());
                    if !hit.contains(&p) {
                        hit.push(p);
                        let mut e = rng.byte();
                        if e == 0 {
                            e = 1;
                        }
                        block[p] ^= e;
                    }
                }
                assert_eq!(decode_block(&mut block, nroots), Ok(t), "{nroots} roots, {t} errors");
                assert_eq!(block, clean);

                // One past the limit. It may be refused, and it may decode to
                // something else — but it must never claim to have found
                // nothing wrong, because that is the answer a caller believes.
                let mut block = clean.clone();
                let mut hit = Vec::new();
                while hit.len() < t + 1 {
                    let p = rng.below(block.len());
                    if !hit.contains(&p) {
                        hit.push(p);
                        let mut e = rng.byte();
                        if e == 0 {
                            e = 1;
                        }
                        block[p] ^= e;
                    }
                }
                if let Ok(n) = decode_block(&mut block, nroots) {
                    assert_ne!(n, 0, "{t}+1 errors decoded as a clean block");
                }
            }
        }
    }

    /// A shortened block decodes to the same answer as the same data padded out
    /// to the full 255 symbols.
    ///
    /// The check on the shortening arithmetic: if the syndromes or the error
    /// locations were being computed against the wrong length, these two would
    /// disagree.
    #[test]
    fn shortening_agrees_with_the_unshortened_code() {
        let mut rng = Rng(0xabcd_1111);
        let nroots = 6;
        for &k in &[11usize, 40, 100, 249] {
            let data: Vec<u8> = (0..k).map(|_| rng.byte()).collect();
            let short = encode_block(&data, nroots);

            let mut padded = vec![0u8; K - k];
            padded.extend_from_slice(&data);
            let full = encode_block(&padded, nroots);
            assert_eq!(&full[K - k..], &short[..], "shortened parity differs from padded");

            let mut a = short.clone();
            a[3] ^= 0x5a;
            a[k] ^= 0x11;
            let mut b = full.clone();
            b[K - k + 3] ^= 0x5a;
            b[K] ^= 0x11;
            let ra = decode_block(&mut a, nroots);
            let rb = decode_block(&mut b, nroots);
            assert_eq!(ra, rb);
            assert_eq!(&a[..], &b[K - k..]);
        }
    }

    /// A block with no parity at all is passed through, not refused. VDL2's
    /// final block really can be one or two octets with nothing behind them.
    #[test]
    fn a_block_without_parity_is_not_an_error() {
        let mut block = [0x12u8, 0x34];
        assert_eq!(decode_block(&mut block, 0), Ok(0));
        assert_eq!(block, [0x12, 0x34]);
        assert_eq!(encode_block(&[0x12, 0x34], 0), vec![0x12, 0x34]);
    }

    /// Noise is refused far more often than it is "repaired", and never
    /// repaired into a block that fails its own syndromes.
    #[test]
    fn random_noise_is_mostly_refused() {
        let mut rng = Rng(0x9999_4242);
        let nroots = 6;
        let mut accepted = 0;
        for _ in 0..2000 {
            let mut block: Vec<u8> = (0..106).map(|_| rng.byte()).collect();
            if decode_block(&mut block, nroots).is_ok() {
                accepted += 1;
                // Whatever it produced has to satisfy the code, or the final
                // syndrome check in `decode_block` is not doing its job.
                assert_eq!(decode_block(&mut block.clone(), nroots), Ok(0));
            }
        }
        // With six parity symbols the miscorrection probability is small; this
        // is a loose bound that a broken decoder blows through.
        assert!(accepted < 200, "{accepted} random blocks in 2000 decoded");
    }
}
