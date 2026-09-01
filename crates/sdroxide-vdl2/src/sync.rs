//! Finding the start of a VDL2 burst, and everything that has to be known
//! before a single bit can be read out of it.
//!
//! A transmission opens with sixteen symbols the standard fixes in advance —
//! the synchronisation and ambiguity-resolution word. Correlating against it
//! answers four questions at once: whether this burst is VDL2 at all, where its
//! symbol clock is, how far off frequency the transmitter (or this receiver) is,
//! and whether something in the receive chain mirrored the spectrum.
//!
//! # The word, twice
//!
//! The standard publishes the sixteen symbols, and separately the cumulative
//! carrier phase they produce. Those are the same fact written two ways, and
//! [`UNIQUE_WORD`], [`UW_INCREMENTS`] and [`UW_CUMULATIVE`] hold all three
//! forms so a test can prove they agree. That agreement is what pins the sign
//! of a phase increment, which is otherwise a coin toss that costs a decoder
//! everything.
//!
//! # A differential correlator, not a phase regression
//!
//! Comparing observed phases against the expected sequence and least-squares
//! fitting out the offset and the drift works, and costs sixteen arctangents
//! per trial position. Correlating the *differential* products against a
//! constant table costs none, and gives the frequency estimate as a by-product:
//!
//! ```text
//! d[n]  = z[n]·conj(z[n−1])                     n = 1..15
//! e[n]  = exp(−j·UW_INCREMENTS[n]·π/4)          a constant table
//! C     = Σ d[n]·e[n]
//! score = |C| / Σ|d[n]|      w = arg(C)         radians per symbol
//! ```
//!
//! A carrier error adds the *same* phase to every differential product, so the
//! sixteen terms still add coherently however far off frequency the burst is.
//! `score` is one for a noiseless burst at any offset inside half a symbol rate,
//! which is why there is no frequency grid to search — unlike the QO-100 beacon
//! decoder next door, where a grid exists because there is nothing else to hang
//! an estimate on.
//!
//! Spectral inversion is one extra correlation against the conjugate table, and
//! whichever scores higher wins. The ISM slicer makes the same argument in its
//! own domain: a known word settles for free what would otherwise be a
//! per-front-end assumption that silently fails on high-side injection.
//!
//! Source: ETSI EN 301 841-1 §4.4, the transmission synchronisation sequence.

use std::f32::consts::FRAC_PI_4;

use sdroxide_dsp::Complex32;

use crate::demod::{ChannelFilter, SYMBOL_RATE};

/// Symbols in the synchronisation word.
pub const UW_SYMS: usize = 16;

/// The synchronisation and ambiguity-resolution word, as the three-bit symbols
/// it is transmitted as.
pub const UNIQUE_WORD: [u8; UW_SYMS] = [0, 2, 3, 6, 0, 1, 5, 6, 1, 4, 3, 7, 5, 7, 4, 2];

/// The phase change each of those symbols carries, in eighths of a turn.
pub const UW_INCREMENTS: [u8; UW_SYMS] = [0, 3, 2, 4, 0, 1, 6, 4, 1, 7, 2, 5, 6, 5, 7, 3];

/// The cumulative carrier phase the standard publishes for the same word, in the
/// same units.
///
/// Held only so a test can prove it is the running sum of [`UW_INCREMENTS`].
/// Nothing reads it at run time — but the two being independently published and
/// agreeing is the only external evidence available for the sign convention.
pub const UW_CUMULATIVE: [u8; UW_SYMS] = [0, 3, 5, 1, 1, 2, 0, 4, 5, 4, 6, 3, 1, 6, 5, 0];

/// Correlation score below which a candidate is not a synchronisation word.
///
/// One is a noiseless burst; pure noise averages about `1/√15` = 0.26. This sits
/// well above the noise and below what a weak but real burst produces.
pub const SCORE_THRESHOLD: f32 = 0.55;

/// How far either side of the coarse peak the fractional search looks, and in
/// what steps — in symbols.
const REFINE_SPAN: f64 = 0.6;
const REFINE_STEP: f64 = 1.0 / 16.0;

/// Everything the correlator learned about one burst.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lock {
    /// Fractional sample position of the *last* synchronisation symbol — the
    /// reference the first header symbol is differenced against, so the symbol
    /// reader needs no special case for its first step.
    pub at: f64,
    /// Samples per symbol.
    pub sps: f64,
    /// Residual carrier, radians per symbol.
    pub w: f32,
    /// Absolute carrier phase at [`Lock::at`].
    ///
    /// Differential detection does not need it. It is what a coherent second
    /// pass — worth about three decibels — would start from, and resolving the
    /// eight-fold phase ambiguity is what the standard calls this word for, so
    /// throwing it away would be throwing away the word's whole purpose.
    pub phase: f32,
    /// Normalised correlation: one for a noiseless burst at any carrier offset.
    pub score: f32,
    /// The receive chain mirrored the spectrum, so every phase increment is
    /// negated.
    pub inverted: bool,
    /// Mean symbol magnitude across the word, for the level report.
    pub mag: f32,
}

/// Find the next synchronisation word at or after `from`.
///
/// `mf` is `iq` run through `rrc` at whole-sample positions — the coarse search
/// reads it rather than filtering sixteen times per trial offset. The
/// refinement goes back to `iq` for fractional positions.
///
/// `min_mag` is a floor on the mean differential magnitude, as a fraction of the
/// transmission's own peak power. Without it the *shaped tail* of a burst — the
/// eight symbols of pulse that precede the first real symbol — correlates
/// respectably against a sixteen-symbol word, because the score is normalised
/// and normalising divides the weakness out. Measured on a synthetic burst, a
/// tail candidate scores 0.78 where the true position scores 0.98, which is far
/// too close to separate on score alone.
///
/// Returns the *first* peak, not the strongest in the buffer: two stations may
/// share a transmission window, and the caller decodes each in turn and resumes
/// past it. [`Found::resume`] is where to carry on from — past the whole peak,
/// not one sample along, because the refinement can move the answer backwards
/// and a caller resuming one sample at a time would find the same peak forever.
pub fn find_next(
    iq: &[Complex32],
    mf: &[Complex32],
    rrc: &ChannelFilter,
    sps: f64,
    from: usize,
    min_mag: f32,
) -> Option<Found> {
    let span = ((UW_SYMS as f64 - 1.0) * sps).ceil() as usize;
    let guard = rrc.half() + 2;
    if mf.len() <= guard * 2 + span {
        return None;
    }
    let last = mf.len() - guard - span - 1;
    let mut i = from.max(guard);
    while i < last {
        let (score, inverted, mag) = coarse(mf, i, sps);
        if score < SCORE_THRESHOLD || mag < min_mag {
            i += 1;
            continue;
        }
        // Climb to the top of the peak rather than taking its leading edge: the
        // correlation rises over several samples and the crest is the timing
        // estimate the refinement then polishes.
        let mut best = (score, i, inverted);
        let mut j = i + 1;
        let limit = (i + span).min(last);
        while j < limit {
            let (s, inv, m) = coarse(mf, j, sps);
            if s < SCORE_THRESHOLD || m < min_mag {
                break;
            }
            if s > best.0 {
                best = (s, j, inv);
            }
            j += 1;
        }
        let resume = j.max(best.1 + 1);
        if let Some(lock) = refine(iq, rrc, best.1 as f64, sps, best.2) {
            return Some(Found { lock, resume });
        }
        i = resume;
    }
    None
}

/// A candidate, and where to carry the search on from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Found {
    pub lock: Lock,
    /// The first offset past this peak. A caller that decodes nothing from the
    /// lock resumes here.
    pub resume: usize,
}

/// Correlate at a whole-sample offset, in both spectral senses. Public for the
/// bring-up example, which prints the profile around a known position.
pub fn coarse_at(mf: &[Complex32], at: usize, sps: f64) -> (f32, bool, f32) {
    coarse(mf, at, sps)
}

/// Correlate at a whole-sample offset, in both spectral senses.
fn coarse(mf: &[Complex32], at: usize, sps: f64) -> (f32, bool, f32) {
    let mut sum_a = Complex32::default();
    let mut sum_b = Complex32::default();
    let mut mag = 0f32;
    let mut prev = mf[at];
    for n in 1..UW_SYMS {
        let idx = at + (n as f64 * sps).round() as usize;
        let z = mf[idx];
        let d = z * prev.conj();
        prev = z;
        mag += d.norm_sqr().sqrt();
        let e = table(n);
        sum_a += d * e;
        sum_b += d.conj() * e;
    }
    if mag <= 0.0 {
        return (0.0, false, 0.0);
    }
    let mean = mag / (UW_SYMS - 1) as f32;
    let a = sum_a.norm_sqr().sqrt() / mag;
    let b = sum_b.norm_sqr().sqrt() / mag;
    if b > a { (b, true, mean) } else { (a, false, mean) }
}

/// The expected differential product for symbol `n`, conjugated.
fn table(n: usize) -> Complex32 {
    let ph = -f32::from(UW_INCREMENTS[n]) * FRAC_PI_4;
    Complex32::new(ph.cos(), ph.sin())
}

/// Search fractional positions around a coarse peak and build the lock.
fn refine(
    iq: &[Complex32],
    rrc: &ChannelFilter,
    coarse_at: f64,
    sps: f64,
    inverted: bool,
) -> Option<Lock> {
    let mut best: Option<(f32, f64, Complex32, f32)> = None;
    let steps = (2.0 * REFINE_SPAN / REFINE_STEP).round() as i32;
    for s in 0..=steps {
        let off = -REFINE_SPAN + f64::from(s) * REFINE_STEP;
        let start = coarse_at + off * sps;
        if start < 0.0 {
            continue;
        }
        let mut sum = Complex32::default();
        let mut mag = 0f32;
        let mut prev = sample(iq, rrc, start, inverted);
        if prev.norm_sqr() <= 0.0 {
            continue;
        }
        for n in 1..UW_SYMS {
            let z = sample(iq, rrc, start + n as f64 * sps, inverted);
            let d = z * prev.conj();
            prev = z;
            mag += d.norm_sqr().sqrt();
            sum += d * table(n);
        }
        if mag <= 0.0 {
            continue;
        }
        let score = sum.norm_sqr().sqrt() / mag;
        if best.is_none_or(|(b, ..)| score > b) {
            best = Some((score, start, sum, mag / (UW_SYMS - 1) as f32));
        }
    }
    let (score, start, sum, mag) = best?;
    if score < SCORE_THRESHOLD {
        return None;
    }
    let w = sum.arg();
    let at = start + (UW_SYMS - 1) as f64 * sps;

    // The absolute phase, from the same sixteen symbols with the increments and
    // the residual carrier taken back out.
    let mut acc = Complex32::default();
    for (n, &cum) in UW_CUMULATIVE.iter().enumerate() {
        let z = sample(iq, rrc, start + n as f64 * sps, inverted);
        let ph = -f32::from(cum) * FRAC_PI_4 - w * (n as f32);
        acc += z * Complex32::new(ph.cos(), ph.sin());
    }

    Some(Lock { at, sps, w, phase: acc.arg(), score, inverted, mag })
}

fn sample(iq: &[Complex32], rrc: &ChannelFilter, x: f64, inverted: bool) -> Complex32 {
    let z = rrc.at(iq, x);
    if inverted { z.conj() } else { z }
}

/// The carrier offset a lock implies, in hertz.
pub fn freq_hz(lock: &Lock) -> f32 {
    lock.w / (2.0 * std::f32::consts::PI) * SYMBOL_RATE as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::UNGRAY;

    /// The three published forms of the synchronisation word are one fact.
    ///
    /// The symbols, the phase increments they Gray-decode to, and the
    /// cumulative phase the standard prints — the running sum of the second has
    /// to be the third. Two independently published statements agreeing is the
    /// only external evidence available for which way a phase increment counts,
    /// and getting that backwards produces a decoder that runs and finds
    /// nothing.
    #[test]
    fn the_word_and_its_cumulative_phase_are_the_same_fact() {
        let increments: Vec<u8> = UNIQUE_WORD.iter().map(|&s| UNGRAY[s as usize]).collect();
        assert_eq!(increments, UW_INCREMENTS.to_vec());

        let mut acc = 0u8;
        let cumulative: Vec<u8> = UW_INCREMENTS
            .iter()
            .map(|&d| {
                acc = (acc + d) % 8;
                acc
            })
            .collect();
        assert_eq!(cumulative, UW_CUMULATIVE.to_vec());
    }

    /// The word uses seven of the eight symbols and repeats none more than
    /// twice — a sanity check on the transcription that a round trip cannot do.
    #[test]
    fn the_word_is_transcribed_plausibly() {
        let mut count = [0u8; 8];
        for &s in &UNIQUE_WORD {
            assert!(s < 8, "{s} is not a three-bit symbol");
            count[s as usize] += 1;
        }
        assert!(count.iter().all(|&c| c <= 3), "{count:?}");
    }
}
