//! DeepFilterNet3 audio noise reduction — the strongest of the five engines.
//!
//! Two stages the others do not have: an ERB-band gain like RNNoise's, and then
//! a learned complex *deep filter* over the low bins, which restores phase and
//! recovers speech RNNoise has already given up on. Pure-Rust inference through
//! [tract] — no ONNX Runtime, no C — with the model embedded in the binary.
//!
//! DFN3 is fixed at **48 kHz** and **480-sample** hops, so the streaming,
//! framing and resampling all belong to [`crate::frame48::Frame48`], shared with
//! [`crate::NeuralNr`]. Unlike RNNoise it works in `[-1, 1]`, so there is no
//! scaling step.
//!
//! Intensity is `atten_lim_db`, the network's own cap on how much it may take
//! out of any band, rather than a wet/dry blend: a capped mask still tracks the
//! speech, where a dry blend at 45 % puts 45 % of the noise back with it.
//!
//! Construction is expensive — an 8 MB archive unpacked and three graphs
//! optimised — and fallible, so callers build it lazily and report the error
//! rather than retrying. See [`DeepFilterNr::new`].
//!
//! [tract]: https://github.com/sonos/tract

use std::panic::{AssertUnwindSafe, catch_unwind};

use df::tract::{DfParams, DfTract, RuntimeParams};
use ndarray::Array2;

use crate::frame48::Frame48;

pub struct DeepFilterNr {
    df: DfTract,
    /// Resampling, framing and the one-in/one-out queue.
    shell: Frame48,
    /// `[1, hop]` scratch handed to the network, and what it hands back.
    noisy: Array2<f32>,
    enh: Array2<f32>,
    lsnr_db: f32,
}

impl DeepFilterNr {
    /// Load the embedded DeepFilterNet3 model.
    ///
    /// Expensive and fallible. `DfTract::new` reports a bad graph as an error,
    /// but `DfParams::default` *panics* on a malformed archive, so both are run
    /// under [`catch_unwind`]: this is built on the engine thread, and a panic
    /// there would take the audio down with it rather than costing one engine.
    pub fn new() -> Result<Self, String> {
        let rp = RuntimeParams::default_with_ch(1);
        let built = catch_unwind(AssertUnwindSafe(|| DfTract::new(DfParams::default(), &rp)));
        let df = match built {
            Ok(Ok(df)) => df,
            Ok(Err(e)) => return Err(e.to_string()),
            Err(_) => return Err("the embedded model could not be unpacked".to_string()),
        };
        if df.ch != 1 {
            return Err(format!("expected a mono model, got {} channels", df.ch));
        }
        let hop = df.hop_size;
        Ok(DeepFilterNr {
            shell: Frame48::new(hop),
            noisy: Array2::zeros((1, hop)),
            enh: Array2::zeros((1, hop)),
            lsnr_db: 0.0,
            df,
        })
    }

    /// Cap on the attenuation the network may apply, in dB. Higher removes more;
    /// 100 dB is effectively unlimited.
    pub fn set_atten_lim_db(&mut self, db: f32) {
        self.df.set_atten_lim(db);
    }

    /// Point the denoiser at a new input sample rate, rebuilding the resamplers
    /// and clearing state if it changed. Cheap no-op when the rate is unchanged.
    pub fn set_rate(&mut self, in_rate: f64) {
        if self.shell.set_rate(in_rate) {
            let _ = self.df.init();
        }
    }

    /// Clear all streaming state, including the network's.
    pub fn reset(&mut self) {
        self.shell.reset();
        let _ = self.df.init();
    }

    /// The network's own local-SNR estimate for the last frame it saw, in dB.
    pub fn lsnr_db(&self) -> f32 {
        self.lsnr_db
    }

    /// Denoise `audio` in place. Same length in and out; introduces a priming
    /// latency (zero-filled) of roughly one frame plus the resampler delay.
    pub fn process(&mut self, audio: &mut [f32]) {
        // Destructured so the closure borrows only the fields it touches.
        let Self { df, shell, noisy, enh, lsnr_db } = self;
        shell.process(audio, |frame, den| {
            noisy.as_slice_mut().expect("contiguous").copy_from_slice(frame);
            match df.process(noisy.view(), enh.view_mut()) {
                Ok(lsnr) => {
                    *lsnr_db = lsnr;
                    den.extend_from_slice(enh.as_slice().expect("contiguous"));
                }
                // One bad frame is not worth dropping the receiver over: pass it
                // through undenoised, keeping the sample count exact.
                Err(_) => den.extend_from_slice(frame),
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unpacking the model takes long enough that four tests each building their
    /// own is worth avoiding.
    fn nr() -> DeepFilterNr {
        DeepFilterNr::new().expect("the embedded DeepFilterNet3 model should load")
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|&s| s * s).sum::<f32>() / x.len().max(1) as f32).sqrt()
    }

    fn noise(n: usize, amp: f32, seed: u64) -> Vec<f32> {
        let mut r = seed;
        (0..n)
            .map(|_| {
                r ^= r << 13;
                r ^= r >> 7;
                r ^= r << 17;
                (((r >> 40) as i32) as f32 / (1 << 23) as f32 - 1.0) * amp
            })
            .collect()
    }

    #[test]
    fn preserves_sample_count() {
        let mut nr = nr();
        // Odd block sizes at 48 kHz (identity resample path).
        for &len in &[512usize, 300, 480, 1024, 137] {
            let mut buf = vec![0.1f32; len];
            nr.process(&mut buf);
            assert_eq!(buf.len(), len);
        }
    }

    #[test]
    fn reduces_broadband_noise_at_full_strength() {
        let mut nr = nr();
        nr.set_rate(48_000.0);
        nr.set_atten_lim_db(100.0);
        let mut buf = noise(96_000, 0.2, 0x51DE);
        let before = rms(&buf);
        nr.process(&mut buf);
        // Skip priming latency.
        let after = rms(&buf[480 * 8..]);
        assert!(after < 0.9 * before, "noise not reduced: {before} -> {after}");
    }

    #[test]
    fn a_lower_limit_attenuates_less() {
        // `atten_lim_db` has no dry path to compare against, so the meaningful
        // assertion is relative: the same noise through a 6 dB cap must come out
        // louder than through a 24 dB one.
        let run = |db: f32| {
            let mut nr = nr();
            nr.set_atten_lim_db(db);
            let mut buf = noise(96_000, 0.2, 0xD00D);
            nr.process(&mut buf);
            rms(&buf[480 * 8..])
        };
        let (low, high) = (run(6.0), run(24.0));
        assert!(low > high, "6 dB cap was not gentler than 24 dB: {low} vs {high}");
    }

    /// How much of one core DeepFilterNet needs to keep up, as a fraction of
    /// real time. Ignored by default — it is a measurement, not an assertion,
    /// and it is only meaningful in a release build:
    ///
    /// ```text
    /// cargo test --release -p sdroxide-dsp -- --ignored --nocapture realtime_cost
    /// ```
    ///
    /// Anything approaching 1.0 means this host cannot run DFNR on the engine
    /// thread without dropping audio. Worth re-running on aarch64, which is a
    /// release target and where it will bite first.
    #[test]
    #[ignore = "a measurement, not an assertion; needs --release to mean anything"]
    fn realtime_cost() {
        let mut nr = nr();
        nr.set_rate(48_000.0);
        nr.set_atten_lim_db(100.0);
        let seconds = 10;
        let mut buf = noise(48_000 * seconds, 0.2, 0xC057);
        // One 1024-sample block at a time, as the engine feeds it.
        let start = std::time::Instant::now();
        for block in buf.chunks_mut(1024) {
            nr.process(block);
        }
        let elapsed = start.elapsed();
        let fraction = elapsed.as_secs_f64() / seconds as f64;
        println!(
            "DeepFilterNet3: {fraction:.3}x real time ({elapsed:.2?} for {seconds}s of audio)"
        );
    }

    #[test]
    fn resamples_non_48k_rate() {
        // A non-48 kHz rate exercises the up/down resampler path and must still
        // return the same number of samples.
        let mut nr = nr();
        nr.set_rate(44_100.0);
        let mut buf = noise(44_100, 0.15, 0xBEE5);
        nr.process(&mut buf);
        assert_eq!(buf.len(), 44_100);
    }
}
