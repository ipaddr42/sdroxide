//! The CW skimmer core: a streaming STFT over a wide complex-baseband window,
//! per-bin on/off-keying envelope detection with an adaptive noise floor, and
//! light signal tracking. Produces one [`SkimmerSpot`] per tracked CW signal.
//!
//! Finding the signals and reading them are two separate jobs here, and only the
//! second one changed when DeepCW arrived. The STFT below still decides where
//! the stations are, how strong they are and what frequency to put the marker
//! on — all of which it does from bin power alone, and none of which a character
//! model has an opinion about. The text comes from [`crate::deep`], which lifts
//! each station's slice out of a second transform at the model's own resolution
//! and hands it to a pool of decoders.

use std::sync::Arc;

use rustfft::{Fft, FftPlanner};
use sdroxide_deepcw::Pool;
use sdroxide_dsp::{Complex32 as C32, CwDecoder};
use sdroxide_types::{CwSkimmerDecoder, SkimmerKind, SkimmerSettings, SkimmerSpot, is_cw_segment};

use crate::callsign::find_callsign;
use crate::deep::DeepFront;

// A 4096-pt window over the ~200 kHz skim rate is ~20 ms / ~49 Hz per bin —
// close to a CW signal's own bandwidth, so the carrier lands in one bin with
// good SNR, while the window is still shorter than a dit at moderate speeds.
const FFT_SIZE: usize = 4096;
/// Hop between analysis frames (75% overlap). Frame time = HOP / skim_rate
/// (~5 ms at 200 kHz) — good keying resolution for CW.
const HOP: usize = 1024;
/// Frames of noise-floor priming before detection starts.
const WARMUP: u32 = 40;

/// Detection threshold above the per-bin noise floor (power ratio; ~10 dB).
/// High enough that random noise rarely crosses it across thousands of bins.
///
/// This only decides where the *signals* are — which bins are worth tracking,
/// where the spot marker goes, and whether a track is still alive. Whether the
/// key was down at any given moment is [`CwDecoder`]'s business, and it decides
/// it by fitting a threshold of its own to each track's envelope.
const ON_RATIO: f32 = 10.0;
/// Bins per region for the median noise-floor estimate (~3 kHz at 47 Hz/bin).
/// The median of a region is a noise bin as long as CW signals stay sparse in
/// it, so a signal — however strong or persistent — can't inflate the floor.
const REGION_BINS: usize = 64;
/// Frames between noise-floor updates.
///
/// One full window of fresh samples: at 75% overlap, consecutive frames share
/// three quarters of their input, so re-running the region medians every frame
/// mostly re-reads noise it has already measured. Updating once per
/// `FFT_SIZE / HOP` frames costs a quarter as much and draws each estimate from
/// independent samples; [`sdroxide_dsp::noisefloor::stride_alpha`] keeps the floor's time
/// constant in seconds unchanged.
const FLOOR_EVERY: u32 = (FFT_SIZE / HOP) as u32;
/// A peak must be the strongest bin within ±this window to count — enforces a
/// minimum signal spacing and rejects a strong signal's own leakage sidelobes.
const PEAK_SPACING: usize = 8; // ~±390 Hz at 49 Hz/bin
/// Bins within this of a track's center are "the same signal" (spawn tolerance).
const TRACK_TOL: i64 = 3;
/// Guard band around the front end's DC spike to ignore, in bins. Where that
/// lands in this window is [`CwSkimmer::dc_off_hz`] — the window centre only
/// while the two coincide.
const DC_GUARD: i64 = 3;
/// Frames a track must be detected before it's reported (rejects noise blips;
/// one dit at 20 WPM is ~12 frames at this hop).
const MIN_HITS: u32 = 8;
/// Fraction of its life a track must have spent keyed before the model is asked
/// to read it.
///
/// [`MIN_HITS`] counts frames over a track's whole life and never decays, so a
/// noise bin only has to cross the threshold eight times — spread over as many
/// seconds as it likes — to look confirmed, earn the long pruning timer, and
/// then be handed to the model every [`crate::deep`] interval for as long as it
/// survives. Measured on a quiet band that was 97% of every decode.
///
/// Duty cycle separates the two cleanly. Morse is about 40% key-down while a
/// station is actually sending, and a bin that crosses the detection threshold
/// on fluctuation alone sits near zero. The gate is an order of magnitude below
/// any real fist so that a station pausing between overs keeps its slot.
const MIN_DUTY: f32 = 0.04;
/// Track pruning (ms). A blip that never became a signal goes quickly; anything
/// that did is kept long enough to be worth keeping.
///
/// The middle case exists because the model needs several seconds of a station
/// before it will say anything at all. A track that has been keying but has no
/// text yet is not a failure, it is one that has not been read yet, and pruning
/// it on the old empty-track timer would drop every station that paused inside
/// its first few seconds — before it had ever had a chance to decode.
const PRUNE_EMPTY_MS: f64 = 1200.0;
const PRUNE_PENDING_MS: f64 = 9000.0;
const PRUNE_DECODED_MS: f64 = 8000.0;
/// A track counts as "active" (currently keying) within this of its last mark.
const ACTIVE_MS: f64 = 1500.0;
/// How recently a track must have been keyed for [`crate::deep`] to be told it
/// is still sending.
///
/// Much shorter than [`ACTIVE_MS`], which is a display property — how long a
/// spot keeps its "active" marker — and far too slow to say where a
/// transmission ended. This answers a different question, so it debounces
/// barely: just enough to bridge the gap between two elements of one character.
/// Deciding that a station has actually stopped is [`crate::deep`]'s job and it
/// waits its own interval before doing so; stacking that on top of a
/// display-length debounce would put seconds of silence on the end of every
/// window the model reads.
const DEEP_ACTIVE_MS: f64 = 200.0;
/// Bound on simultaneous tracks.
const MAX_TRACKS: usize = 256;
/// Rolling decoded text kept per track.
const MAX_TEXT: usize = 64;
/// How often the timing decoder re-fits each track's envelope.
///
/// Far slower than the CW panel's own decoder, which re-fits ten times a second
/// for one signal. Here there may be a couple of hundred tracks and the fit is a
/// search over threshold, speed and spacing, so it is worth half a second of
/// latency to run it a fifth as often.
const TRACK_HOP_S: f32 = 0.5;
/// Speed range the timing decoder's spots must fall in to be reported.
///
/// Tighter than the neural decoder's, because this speed is *fitted* — a figure
/// outside this range means the fit found something that was not Morse, whereas
/// the model's speed is inferred from how fast text arrived and reads low for a
/// station that pauses.
const TIMING_WPM: std::ops::RangeInclusive<u16> = 8..=45;
/// Speed range for the neural decoder's spots. Wide on purpose — see above.
const NEURAL_WPM: std::ops::RangeInclusive<u16> = 5..=60;

/// Spin up the model and its threads, or report why not and carry on without.
///
/// A skimmer that finds and marks signals but cannot read them is still worth
/// having, so a model that will not load is a downgrade rather than a failure.
fn build_deep(skim_rate: f64, max_tracks: usize) -> Option<(DeepFront, Pool)> {
    // Inference is the only thing here worth spreading, and it is not worth the
    // whole machine: the skimmer shares it with a receiver. The pool bounds both
    // how many windows are in the model at once and how many cores they may use
    // between them — see `Decoder::with_thread_pool`.
    let threads = std::thread::available_parallelism().map_or(2, |n| n.get() / 2).clamp(1, 4);
    match Pool::new(threads) {
        Ok(pool) => Some((DeepFront::new(skim_rate, FFT_SIZE, max_tracks), pool)),
        Err(e) => {
            tracing::error!("DeepCW unavailable, the CW skimmer will mark signals only: {e}");
            None
        }
    }
}

/// Which bins of a `rate`-wide window on `center_hz` sit in a CW sub-segment,
/// indexed the way the transform is (bin 0 at DC, the top half negative).
///
/// All false where the window is nowhere near an amateur CW allocation, which
/// is the whole of VHF broadcast, the whole of the air band, and most of what
/// anyone points an RTL-SDR at.
fn cw_bin_mask(rate: f64, center_hz: f64) -> Vec<bool> {
    let bin_hz = rate / FFT_SIZE as f64;
    (0..FFT_SIZE)
        .map(|k| {
            let off = if k <= FFT_SIZE / 2 { k as i64 } else { k as i64 - FFT_SIZE as i64 };
            is_cw_segment(center_hz + off as f64 * bin_hz)
        })
        .collect()
}

struct Track {
    id: u64,
    bin: i64, // signed offset bin from DC (negative = below center)
    /// Whatever the model last made of this station, and the speed implied by
    /// how fast the text arrived. Both are replaced wholesale each time a decode
    /// comes back rather than accumulated, because the model re-reads its whole
    /// window every round and may revise what it said last time.
    text: String,
    wpm: u16,
    last_on_ms: f64,
    snr_db: i16,
    /// Frames this track has been keyed (for confirmation).
    hits: u32,
    /// Frames since it was spawned, so [`MIN_DUTY`] has a denominator.
    frames: u32,
    /// Set once this track has looked like a keyed signal, and never cleared.
    ///
    /// It has to latch. Duty cycle can only fall once a station stops sending,
    /// and dropping a track out of the list handed to [`crate::deep`] destroys
    /// its buffered spectrogram — so a marginal signal whose duty wanders across
    /// the threshold would have its context thrown away and rebuilt from nothing
    /// each time, which reads as a station that keeps forgetting the first half
    /// of what it just sent. What the test is for is keeping noise out, and noise
    /// never crosses it in the first place.
    confirmed: bool,
    /// The envelope-timing decoder, when that is the one in use.
    ///
    /// Boxed because it carries ten `Vec`s and there may be a couple of hundred
    /// tracks. `None` under the neural decoder, and because switching decoders
    /// resets every track there is never a mixed population.
    dec: Option<Box<CwDecoder>>,
    /// Time-smoothed power at [bin-1, bin, bin+1], accumulated over keyed-on
    /// frames. Quadratic interpolation over these three resolves the carrier to
    /// a fraction of a bin, so the spot marker lands on the signal instead of
    /// snapping to the 49 Hz grid. Smoothing is essential: a single frame of a
    /// keying CW signal is spiky and would bias the estimate.
    pk: [f32; 3],
}

impl Track {
    /// Whether this looks like a keyed CW signal rather than a bin of noise that
    /// has crossed the threshold a few times over a long life. Only these are
    /// worth a decoder slot — see [`MIN_DUTY`] and [`Track::confirmed`].
    fn keying(&self) -> bool {
        self.confirmed
            || (self.hits >= MIN_HITS && self.hits as f32 >= self.frames as f32 * MIN_DUTY)
    }
}

pub struct CwSkimmer {
    skim_rate: f64,
    skim_center_hz: f64,
    /// The operator's visible waterfall window in absolute Hz, or `None` for
    /// the whole skim window. See [`CwSkimmer::set_view`].
    view: Option<(f64, f64)>,
    /// Where the front end's own DC spike sits in this window, as an offset in
    /// Hz from its centre. See [`CwSkimmer::set_dc_offset_hz`].
    dc_off_hz: f64,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    inbuf: Vec<C32>,
    /// Read cursor into `inbuf`; consumed samples are compacted out rarely so
    /// the STFT never memmoves the whole buffer every frame.
    read_pos: usize,
    scratch: Vec<C32>,
    power: Vec<f32>,
    /// Per-bin noise floor (the region median scaled to a mean), time-smoothed.
    noise: Vec<f32>,
    /// Reused scratch for the per-region median (one region's power values).
    /// Scratch for the per-region median floor estimate (one region's power, as
    /// bit patterns — see [`sdroxide_dsp::noisefloor::update_regions`]).
    med_scratch: Vec<u32>,
    /// Reused per-frame scratch (avoids re-allocating every frame).
    cands: Vec<(f32, i64)>,
    centers: Vec<i64>,
    frame_ms: f32,
    frames: u32,
    now_ms: f64,
    tracks: Vec<Track>,
    next_id: u64,
    /// The model's front end, and the threads that run it.
    ///
    /// `None` under the timing decoder — in which case none of it is built, so
    /// the wide transform, the worker threads and the 15 MB of weights are not
    /// paid for at all — and also if the model would not load, in which case the
    /// skimmer still finds and marks signals but reports no text for them.
    deep: Option<(DeepFront, Pool)>,
    /// Which decoder reads the signals, and how many the neural one may read.
    decoder: CwSkimmerDecoder,
    slots: usize,
    /// Centers seen last frame, so a track spawns only on a peak that persists
    /// (a single-frame noise blip never becomes a track).
    prev_centers: Vec<i64>,
    /// Which FFT bins of this window fall in a CW sub-segment of the band plan,
    /// by bin index. Rebuilt whenever the window is placed.
    ///
    /// The engine throws away every spot outside one
    /// (`Engine::poll_skimmer`), so a track anywhere else can only ever produce
    /// a result nobody is shown — while costing a DeepCW inference every few
    /// seconds for as long as it lives. On a broadcast FM station, which has no
    /// CW segment anywhere near it, *every* track was of that kind: measured on
    /// an RTL-SDR at 2.4 Msps, the model and its four inference threads were
    /// two thirds of everything the program spent, all of it discarded.
    ///
    /// A mask rather than a test per candidate because the candidate loop runs
    /// over the whole transform two hundred times a second, and a band-plan
    /// lookup is a search through the region's segment table.
    cw_bins: Vec<bool>,
    /// Whether any bin of this window is in one at all. False stands the
    /// skimmer down entirely — no transform, no detection — because there is
    /// nowhere in the window a CW spot could come from.
    any_cw: bool,
    /// Windows the pool took, and windows it refused, for the cost harness.
    #[cfg(test)]
    submits: u64,
    #[cfg(test)]
    refusals: u64,
    /// Decodes that came back with no text at all.
    #[cfg(test)]
    empty: u64,
}

impl CwSkimmer {
    /// A skimmer with the default decoder settings. Used by the tests; the
    /// engine goes through [`CwSkimmer::with_config`].
    pub fn new(skim_rate: f64, skim_center_hz: f64) -> Self {
        Self::with_config(skim_rate, skim_center_hz, &SkimmerSettings::default())
    }

    pub fn with_config(skim_rate: f64, skim_center_hz: f64, cfg: &SkimmerSettings) -> Self {
        let fft = FftPlanner::<f32>::new().plan_fft_forward(FFT_SIZE);
        // Hann window (reduces spectral leakage between adjacent CW signals).
        let window: Vec<f32> = (0..FFT_SIZE)
            .map(|i| {
                let x = std::f32::consts::PI * i as f32 / (FFT_SIZE as f32 - 1.0);
                x.sin().powi(2)
            })
            .collect();
        CwSkimmer {
            skim_rate,
            skim_center_hz,
            view: None,
            dc_off_hz: 0.0,
            fft,
            window,
            inbuf: Vec::with_capacity(FFT_SIZE * 4),
            read_pos: 0,
            scratch: vec![C32::default(); FFT_SIZE],
            power: vec![0.0; FFT_SIZE],
            noise: vec![0.0; FFT_SIZE],
            med_scratch: Vec::with_capacity(REGION_BINS),
            cands: Vec::with_capacity(512),
            centers: Vec::with_capacity(256),
            frame_ms: (HOP as f64 / skim_rate * 1000.0) as f32,
            frames: 0,
            now_ms: 0.0,
            tracks: Vec::new(),
            next_id: 1,
            deep: match cfg.cw_decoder {
                CwSkimmerDecoder::Neural => build_deep(skim_rate, cfg.cw_slots as usize),
                CwSkimmerDecoder::Timing => None,
            },
            decoder: cfg.cw_decoder,
            slots: cfg.cw_slots as usize,
            prev_centers: Vec::new(),
            any_cw: false, // set below, from the mask
            cw_bins: cw_bin_mask(skim_rate, skim_center_hz),
            #[cfg(test)]
            submits: 0,
            #[cfg(test)]
            refusals: 0,
            #[cfg(test)]
            empty: 0,
        }
        .with_cw_mask_applied()
    }

    /// Finish construction: the "is there any CW here at all" flag is read off
    /// the mask, so the two cannot disagree.
    fn with_cw_mask_applied(mut self) -> Self {
        self.any_cw = self.cw_bins.iter().any(|&b| b);
        self
    }

    /// Apply new settings to a running skimmer.
    ///
    /// Changing decoder tears the old one down and starts from a clean band. The
    /// two write a track's text in incompatible ways — the timing decoder appends
    /// characters as it completes them, the model replaces the whole transcript
    /// each time it reads the window — so carrying half of one into the other
    /// produces text that misrepresents what was heard. Losing a few seconds of
    /// re-acquisition is the right price for a setting the operator just changed.
    pub fn set_config(&mut self, cfg: &SkimmerSettings) {
        let slots = (cfg.cw_slots as usize).max(1);
        if cfg.cw_decoder != self.decoder {
            self.decoder = cfg.cw_decoder;
            self.slots = slots;
            // Dropping the pool closes its job channel; each worker's `recv`
            // fails and the thread returns. A decode already in flight finishes
            // and finds nowhere to send its result, which is exactly right.
            self.deep = match self.decoder {
                CwSkimmerDecoder::Neural => build_deep(self.skim_rate, slots),
                CwSkimmerDecoder::Timing => None,
            };
            self.reset();
            return;
        }
        if slots != self.slots {
            self.slots = slots;
            if let Some((front, _)) = self.deep.as_mut() {
                front.set_max_tracks(slots);
            }
        }
    }

    pub fn set_center(&mut self, center_hz: f64) {
        if (center_hz - self.skim_center_hz).abs() > 1.0 {
            self.skim_center_hz = center_hz;
            self.cw_bins = cw_bin_mask(self.skim_rate, center_hz);
            self.any_cw = self.cw_bins.iter().any(|&b| b);
            self.reset();
        }
    }

    /// Limit the skimmer to the operator's visible waterfall window.
    ///
    /// The skim window is a fixed 192 kHz slice of the front end, but the
    /// operator is usually looking at a fraction of it. Reading Morse off a
    /// station nobody can see costs a DeepCW inference every few seconds and
    /// puts its spot somewhere off-screen, so signals outside the window are
    /// neither tracked nor decoded. Tracks already outside it are dropped —
    /// which also releases their model buffers, since the decoder is fed from
    /// `tracks` — so panning away stops the work rather than merely hiding it.
    ///
    /// `None` restores the whole skim window, which is what a headless server
    /// with nobody watching gets.
    pub fn set_view(&mut self, view: Option<(f64, f64)>) {
        if self.view == view {
            return;
        }
        self.view = view;
        let Some((lo, hi)) = view else { return };
        let bin_hz = self.skim_rate / FFT_SIZE as f64;
        let center = self.skim_center_hz;
        let visible = |bin: i64| {
            let hz = center + bin as f64 * bin_hz;
            hz >= lo && hz <= hi
        };
        self.tracks.retain(|t| visible(t.bin));
        self.prev_centers.retain(|&c| visible(c));
    }

    /// Where the front end's own DC spike falls in this window, as an offset in
    /// Hz from the window centre.
    ///
    /// Zero while the window sits on the hardware centre, which is where it
    /// starts. Once it is placed on the operator's view instead the spike is
    /// somewhere else inside the window — and the guard has to travel with it,
    /// or it blinds three bins in the middle of the screen, which on a
    /// panadapter centred on the dial is exactly the signal being listened to.
    pub fn set_dc_offset_hz(&mut self, off_hz: f64) {
        self.dc_off_hz = off_hz;
    }

    /// Whether a bin offset from the skim center is on the front end's DC
    /// spike. A spike outside this window simply never matches.
    fn at_dc(&self, off: i64) -> bool {
        let bin = (self.dc_off_hz / (self.skim_rate / FFT_SIZE as f64)).round() as i64;
        (off - bin).abs() < DC_GUARD
    }

    /// Whether a bin offset from the skim center is on screen.
    fn in_view(&self, off: i64) -> bool {
        let Some((lo, hi)) = self.view else { return true };
        let hz = self.skim_center_hz + off as f64 * self.skim_rate / FFT_SIZE as f64;
        hz >= lo && hz <= hi
    }

    /// Forget every track and re-prime the noise floor (band moved, or the
    /// skimmer was switched off and its state is now stale).
    pub fn reset(&mut self) {
        self.tracks.clear();
        self.inbuf.clear();
        self.read_pos = 0;
        self.frames = 0;
        if let Some((front, _)) = self.deep.as_mut() {
            front.reset();
        }
    }

    /// Feed a block of complex baseband IQ (skim-rate, centered on skim_center).
    pub fn process(&mut self, iq: &[C32]) {
        // Nowhere in this window can a CW spot come from, so there is nothing
        // for the transform to find — see [`CwSkimmer::cw_bins`]. Ahead of the
        // buffering as well as the analysis: a stand-down that kept filling
        // `inbuf` would hand the band back as a backlog the moment the operator
        // tuned onto an amateur band.
        if !self.any_cw {
            return;
        }
        self.inbuf.extend_from_slice(iq);
        while self.read_pos + FFT_SIZE <= self.inbuf.len() {
            let base = self.read_pos;
            for i in 0..FFT_SIZE {
                self.scratch[i] = self.inbuf[base + i] * self.window[i];
            }
            self.fft.process(&mut self.scratch);
            self.on_frame();
            self.read_pos += HOP;
            self.frames = self.frames.saturating_add(1);
            self.now_ms += self.frame_ms as f64;
        }
        // Compact only once the consumed prefix is large, so the memmove is
        // amortized O(1)/sample instead of shifting the buffer every frame.
        if self.read_pos >= FFT_SIZE {
            self.inbuf.drain(..self.read_pos);
            self.read_pos = 0;
        }
        self.run_deep(iq);
    }

    /// Keep the model fed and collect whatever it has finished.
    fn run_deep(&mut self, iq: &[C32]) {
        let Some((front, pool)) = self.deep.as_mut() else { return };

        // Only tracks that are actually being keyed, strongest first: when there
        // are more stations than slots the ones that get read should be the ones
        // most likely to be readable, and a bin of noise is never one of them.
        let now = self.now_ms;
        let mut live: Vec<(u64, i64, i16, bool)> = self
            .tracks
            .iter()
            .filter(|t| t.keying())
            .map(|t| (t.id, t.bin, t.snr_db, now - t.last_on_ms < DEEP_ACTIVE_MS))
            .collect();
        live.sort_unstable_by_key(|t| std::cmp::Reverse(t.2));
        let live: Vec<(u64, i64, bool)> =
            live.into_iter().map(|(id, bin, _, active)| (id, bin, active)).collect();
        front.sync(&live);
        front.process(iq);

        #[cfg(test)]
        let (mut submits, mut refusals) = (0u64, 0u64);
        for id in front.due() {
            // Built one at a time, and only for a job the pool will actually
            // take: see the note on `DeepFront::window`.
            let Some(window) = front.window(id) else { continue };
            // A refused job is left unmarked and comes back next round.
            if !pool.submit(id, window) {
                #[cfg(test)]
                {
                    refusals += 1;
                }
                break;
            }
            #[cfg(test)]
            {
                submits += 1;
            }
            front.mark_submitted(id);
        }
        #[cfg(test)]
        {
            self.submits += submits;
            self.refusals += refusals;
        }

        for (id, decoded) in pool.poll() {
            front.complete(id);
            let decoded = match decoded {
                Ok(d) => d,
                Err(e) => {
                    tracing::warn!("DeepCW: {e}");
                    continue;
                }
            };
            let Some(track) = self.tracks.iter_mut().find(|t| t.id == id) else { continue };
            let text = decoded.normalized();
            if text.is_empty() {
                #[cfg(test)]
                {
                    self.empty += 1;
                }
                continue;
            }
            if let Some(wpm) = decoded.wpm() {
                track.wpm = wpm.round().clamp(0.0, 99.0) as u16;
            }
            // Keep the tail: a spot is a rolling read-out, not a transcript.
            track.text =
                match text.char_indices().nth(text.chars().count().saturating_sub(MAX_TEXT)) {
                    Some((cut, _)) => text[cut..].to_string(),
                    None => text,
                };
        }
    }

    fn on_frame(&mut self) {
        let n = FFT_SIZE;
        for k in 0..n {
            self.power[k] = self.scratch[k].norm_sqr();
        }

        // Per-bin noise floor from a per-region median. The old approach was a
        // single floor-gated EMA updated whenever `power <= floor * ON_RATIO` — a
        // positive-feedback trap: once a strong, near-continuous signal's floor
        // crept above `signal / ON_RATIO` (primed high while it was keying, or
        // riding a high ambient noise floor), the signal stopped exceeding the
        // key-on threshold and was folded straight back into its own floor, so it
        // never decoded — the "strong signal in high noise won't decode while
        // weaker ones do" failure.
        //
        // Fix: estimate the floor from the MEDIAN of each ~64-bin region rather
        // than per-bin history. CW signals are narrow and sparse, so the median
        // of a region is always a noise bin — a signal, however strong or
        // persistent, cannot pull the median up and trap the floor. Scale the
        // median to a mean (thresholds expect the mean) and smooth over time.
        if self.frames % FLOOR_EVERY == 0 {
            let smooth = if self.frames < WARMUP { 0.3 } else { 0.05 };
            let mut med = std::mem::take(&mut self.med_scratch);
            sdroxide_dsp::noisefloor::update_regions(
                &mut self.noise,
                &self.power,
                &mut med,
                REGION_BINS,
                sdroxide_dsp::noisefloor::stride_alpha(smooth, FLOOR_EVERY),
                false,
            );
            self.med_scratch = med;
        }
        if self.frames < WARMUP {
            return; // priming only — no detection yet
        }

        // Signal centers via non-max suppression: collect above-threshold bins,
        // then take them strongest-first, suppressing anything within
        // ±PEAK_SPACING of an already-taken peak. This yields exactly one center
        // per signal (a plateau of near-equal bins can't spawn duplicates) and
        // rejects a strong signal's own leakage sidelobes.
        let mut cands = std::mem::take(&mut self.cands);
        cands.clear();
        for k in 0..n {
            let off = self.offset_bin(k);
            // `cw_bins` first: it is the cheapest of the three and the one that
            // rejects the most — off an amateur band it rejects everything.
            if !self.cw_bins[k] || self.at_dc(off) || !self.in_view(off) {
                continue;
            }
            let p = self.power[k];
            if p > self.noise[k] * ON_RATIO {
                cands.push((p, off));
            }
        }
        cands.sort_unstable_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let spacing = PEAK_SPACING as i64;
        let mut centers = std::mem::take(&mut self.centers);
        centers.clear();
        for &(_, off) in cands.iter() {
            if centers.iter().all(|&c| (c - off).abs() > spacing) {
                centers.push(off);
            }
        }

        // Spawn a fixed-bin track for each center that has no track nearby and
        // that persisted from the previous frame (a single-frame noise blip
        // never becomes one). Tracks never move — CW carriers are stable to well
        // under a bin, so a fixed bin gives clean per-signal envelopes and can't
        // drift onto a neighbour's noise.
        for &off in &centers {
            let near = self.tracks.iter().any(|t| (t.bin - off).abs() <= spacing);
            let persisted = self.prev_centers.iter().any(|&c| (c - off).abs() <= TRACK_TOL);
            if !near && persisted && self.tracks.len() < MAX_TRACKS {
                let id = self.next_id;
                self.next_id += 1;
                let k = self.bin_index(off);
                let km = self.bin_index(off - 1);
                let kp = self.bin_index(off + 1);
                self.tracks.push(Track {
                    id,
                    bin: off,
                    text: String::new(),
                    wpm: 0,
                    last_on_ms: self.now_ms,
                    snr_db: 0,
                    hits: 0,
                    frames: 0,
                    confirmed: false,
                    dec: (self.decoder == CwSkimmerDecoder::Timing).then(|| {
                        let mut d = CwDecoder::new(1000.0 / self.frame_ms);
                        d.set_hop_s(TRACK_HOP_S);
                        Box::new(d)
                    }),
                    pk: [self.power[km], self.power[k], self.power[kp]],
                });
            }
        }
        // This frame's centers become next frame's `prev_centers`; the old
        // buffers are kept for reuse (no per-frame allocation).
        std::mem::swap(&mut self.prev_centers, &mut centers);
        self.centers = centers;
        self.cands = cands;

        // Advance every track: hand its bin's magnitude to the decoder, and keep
        // the bookkeeping the *spot* needs — is anything there, how strong, and
        // exactly which frequency — from a plain threshold on the bin.
        let now = self.now_ms;
        for t in self.tracks.iter_mut() {
            let k = (t.bin.rem_euclid(n as i64)) as usize;
            let floor = self.noise[k].max(1e-12);
            let p = self.power[k];
            t.frames = t.frames.saturating_add(1);
            t.confirmed |= t.keying();
            if p > floor * ON_RATIO {
                t.hits = t.hits.saturating_add(1);
                t.last_on_ms = now;
                t.snr_db = (10.0 * (p / floor).log10()).round().clamp(-30.0, 60.0) as i16;
                // Accumulate the smoothed 3-bin peak shape while keyed.
                let km = ((t.bin - 1).rem_euclid(n as i64)) as usize;
                let kp = ((t.bin + 1).rem_euclid(n as i64)) as usize;
                t.pk[0] = 0.9 * t.pk[0] + 0.1 * self.power[km];
                t.pk[1] = 0.9 * t.pk[1] + 0.1 * self.power[k];
                t.pk[2] = 0.9 * t.pk[2] + 0.1 * self.power[kp];
            }
            // Amplitude, not power: the decoder's thresholds and its noise model
            // are an envelope's. Fed on every frame, keyed or not — it fits a
            // threshold of its own and the silences are half of what it fits.
            if let Some(dec) = t.dec.as_mut() {
                let fresh = dec.push(&[p.sqrt()]);
                if !fresh.is_empty() {
                    t.text.push_str(&fresh);
                    let over = t.text.chars().count().saturating_sub(MAX_TEXT);
                    if over > 0 {
                        let cut = t.text.char_indices().nth(over).map_or(0, |(i, _)| i);
                        t.text.drain(..cut);
                    }
                    t.wpm = dec.wpm().round().clamp(0.0, 99.0) as u16;
                }
            }
        }

        // Prune stale tracks.
        self.tracks.retain(|t| {
            let age = now - t.last_on_ms;
            let keep = match (t.text.is_empty(), t.hits >= MIN_HITS) {
                (false, _) => PRUNE_DECODED_MS,
                (true, true) => PRUNE_PENDING_MS,
                (true, false) => PRUNE_EMPTY_MS,
            };
            age < keep
        });
    }

    /// Snapshot the current tracked signals worth reporting. Filters out noise
    /// tracks: a spot needs a confirmed track, a plausible speed, and text with
    /// real content (a callsign, or several non-trivial characters — random
    /// noise mostly decodes to strings of E/I/T).
    pub fn spots(&self) -> Vec<SkimmerSpot> {
        let bin_hz = self.skim_rate / FFT_SIZE as f64;
        self.tracks
            .iter()
            .filter_map(|t| {
                if t.hits < MIN_HITS {
                    return None;
                }
                let text = t.text.trim_end();
                // A wider range than a timing decoder would need: this speed is
                // measured from how fast text arrived, so a station that pauses
                // reads slower than its fist.
                let plausible = match self.decoder {
                    CwSkimmerDecoder::Neural => NEURAL_WPM,
                    CwSkimmerDecoder::Timing => TIMING_WPM,
                };
                if !plausible.contains(&t.wpm) || text.is_empty() {
                    return None;
                }
                // Quadratic peak interpolation over the smoothed 3-bin shape
                // gives a sub-bin carrier offset in [-0.5, 0.5].
                let [a, b, c] = t.pk;
                let denom = a - 2.0 * b + c;
                let delta =
                    if denom < 0.0 { (0.5 * (a - c) / denom).clamp(-0.5, 0.5) } else { 0.0 };
                let callsign = find_callsign(text);
                let meaty = text
                    .chars()
                    .filter(|c| c.is_ascii_alphanumeric() && !matches!(c, 'E' | 'I' | 'T'))
                    .count();
                if callsign.is_none() && meaty < 3 {
                    return None;
                }
                Some(SkimmerSpot {
                    id: t.id,
                    kind: SkimmerKind::Cw,
                    freq_hz: self.skim_center_hz + (t.bin as f64 + delta as f64) * bin_hz,
                    callsign,
                    text: text.to_string(),
                    snr_db: t.snr_db,
                    wpm: t.wpm,
                    active: (self.now_ms - t.last_on_ms) < ACTIVE_MS,
                })
            })
            .collect()
    }

    #[cfg(test)]
    pub fn debug_dump(&self) {
        for t in &self.tracks {
            if t.hits >= 3 {
                eprintln!("bin{} hits{} wpm{} text={:?}", t.bin, t.hits, t.wpm, t.text);
            }
        }
    }

    /// Signed offset (in bins) of FFT bin `k` from DC.
    fn offset_bin(&self, k: usize) -> i64 {
        let n = FFT_SIZE as i64;
        let k = k as i64;
        if k <= n / 2 { k } else { k - n }
    }

    /// FFT bin index for a signed offset bin.
    fn bin_index(&self, off: i64) -> usize {
        let n = FFT_SIZE as i64;
        (off.rem_euclid(n)) as usize
    }
}

#[cfg(test)]
mod tests {
    use sdroxide_dsp::CwTx;

    use super::*;

    /// Build skim IQ: a keyed CW tone at `off_hz` plus noise of amplitude
    /// `noise` per component (0.02 = light; a large value = a high noise floor).
    ///
    /// The keying comes from the shared [`CwTx`], so the skimmer is exercised
    /// against exactly the envelope shape the rest of the tree agrees is CW —
    /// raised-cosine edges and all — rather than against square keying only
    /// this test knows how to make.
    fn synth(text: &str, off_hz: f64, wpm: f32, rate: f64, noise: f32) -> Vec<C32> {
        // Generate the sidetone at a rate where a dit is plenty of samples,
        // then take its envelope and re-key a complex carrier at the skim rate.
        //
        // Both constants below matter more than they look. The envelope is
        // recovered as the peak over one cycle of the sidetone, so the sidetone
        // frequency *is* the envelope's sample rate, and that staircase is then
        // resampled up to the skim rate. Hold each step and the staircase images
        // the envelope around every multiple of that rate: at 1 kHz it planted a
        // comb of keying-synchronous sidebands either side of the carrier, 20 dB
        // over the noise, and the skimmer — correctly — tracked every one of them
        // as a separate station. A cost measurement taken against that is
        // measuring the generator. Sampling the envelope at 8 kHz and
        // interpolating between the samples puts the first image far outside the
        // envelope's own bandwidth, where there is nothing left to image.
        const AUDIO: f64 = 96_000.0;
        const TONE: f64 = 8_000.0;
        let mut tx = CwTx::new(AUDIO, TONE, wpm);
        tx.push_text(text);
        let mut env: Vec<f32> = Vec::new();
        while !tx.drained() {
            let mut blk = [0.0f32; 512];
            tx.next_block(&mut blk);
            env.extend_from_slice(&blk);
        }
        let per_cycle = (AUDIO / TONE) as usize;
        let key: Vec<f32> =
            env.chunks(per_cycle).map(|c| c.iter().fold(0.0f32, |a, b| a.max(b.abs()))).collect();
        let key_rate = TONE;

        let lead = (key_rate * 0.25) as usize;
        // Enough quiet band after the station stops for the skimmer to notice
        // that it has, and to take the decode that reads the end of the
        // transmission. Cut this short and the last thing a station sends —
        // which is its callsign — is never read.
        let tail = (key_rate * 3.0) as usize;

        let mut phase = 0.0f64;
        let dphi = 2.0 * std::f64::consts::PI * off_hz / rate;
        let mut seed = 0x1234_5678u32;
        let mut rng = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let spk = rate / key_rate; // skim samples per key-envelope sample
        let n = ((lead + key.len() + tail) as f64 * spk) as usize;
        let mut iq = Vec::with_capacity(n);
        for j in 0..n {
            // Linear interpolation between envelope samples, so the carrier is
            // keyed by a ramp rather than by a step.
            let at = j as f64 / spk - lead as f64;
            let amp = if at < 0.0 {
                0.0
            } else {
                let i0 = at as usize;
                let f = (at - i0 as f64) as f32;
                let a = key.get(i0).copied().unwrap_or(0.0);
                let b = key.get(i0 + 1).copied().unwrap_or(0.0);
                a + (b - a) * f
            };
            phase += dphi;
            iq.push(C32::new(
                amp * phase.cos() as f32 + noise * rng(),
                amp * phase.sin() as f32 + noise * rng(),
            ));
        }
        iq
    }

    /// Repeat `text` until it renders to at least `seconds` of IQ, then trim to
    /// exactly that. A cost measurement wants a station in steady state, not one
    /// that says its piece and falls silent.
    fn synth_seconds(
        text: &str,
        off_hz: f64,
        wpm: f32,
        rate: f64,
        noise: f32,
        seconds: f64,
    ) -> Vec<C32> {
        let want = (rate * seconds) as usize;
        let once = synth(text, off_hz, wpm, rate, 0.0).len().max(1);
        let mut msg = String::new();
        for _ in 0..want.div_ceil(once) + 1 {
            msg.push_str(text);
            msg.push(' ');
        }
        let mut iq = synth(&msg, off_hz, wpm, rate, noise);
        iq.resize(want, C32::default());
        iq
    }

    /// CPU consumed by this process and every thread in it, in seconds.
    ///
    /// Wall time cannot answer the question being asked here: with rten's global
    /// pool, 25 ms of inference on one core and 25 ms on sixteen look identical
    /// from the outside and differ by 16× in what they cost the operator's
    /// machine. `utime + stime` from `/proc/self/stat` (fields 14 and 15,
    /// USER_HZ units) counts every core the process actually used.
    #[cfg(target_os = "linux")]
    fn cpu_seconds() -> f64 {
        let stat = std::fs::read_to_string("/proc/self/stat").unwrap_or_default();
        // The comm field is parenthesised and may itself contain spaces, so the
        // fields after it are found from the last ')' rather than by splitting.
        let after = stat.rfind(')').map_or("", |i| &stat[i + 2..]);
        let f: Vec<&str> = after.split_whitespace().collect();
        let at = |i: usize| -> f64 { f.get(i).and_then(|v| v.parse().ok()).unwrap_or(0.0) };
        // Index 0 here is field 3 (state), so utime is 11 and stime is 12.
        (at(11) + at(12)) / 100.0
    }

    #[cfg(not(target_os = "linux"))]
    fn cpu_seconds() -> f64 {
        f64::NAN
    }

    /// What the CW skimmer costs, in cores, to keep up with real time.
    ///
    /// This is the number the CPU-load reports are about, and the one to re-run
    /// after each change. It has to be paced: tracks come due on the *audio*
    /// clock, so feeding a minute of IQ as fast as the machine will take it
    /// leaves the pool permanently saturated, refusing most of the work and
    /// flattering the result. Fed at real time, the refusal count is itself a
    /// finding.
    #[test]
    #[ignore = "a measurement, not an assertion; needs --release to mean anything"]
    fn skimmer_realtime_cost() {
        const SECONDS: f64 = 30.0;
        // The engine hands the skimmer one DDC'd block per 16,384 device
        // samples — about 1,638 skim samples at a typical 10× decimation. That
        // sets how often `run_deep` runs, so the harness has to match it.
        const CHUNK: usize = 1_638;
        const MSG: &str = "CQ TEST DE W1AW W1AW K";

        let rate = 192_000.0;
        let center = 14_030_000.0;
        println!(
            "\n{:>8}{:>5}{:>7}{:>10}{:>10}{:>9}{:>9}{:>9}{:>7}{:>7}{:>7}",
            "decoder",
            "sigs",
            "duty",
            "cores",
            "cpu-s",
            "submits",
            "empty",
            "windows",
            "trk",
            "conf",
            "spots"
        );
        // The last case is the one a real band looks like: stations send an over
        // and then listen. Everything above it transmits without pause, which is
        // the worst case for cost and the one that says nothing about how much
        // the activity gate is worth.
        use CwSkimmerDecoder::{Neural, Timing};
        for (k, over_s, decoder) in [
            (0usize, 0.0, Neural),
            (1, 0.0, Neural),
            (8, 0.0, Neural),
            (32, 0.0, Neural),
            (32, 8.0, Neural),
            (32, 0.0, Timing),
            (32, 8.0, Timing),
        ] {
            let mut mixed = vec![C32::default(); (rate * SECONDS) as usize];
            for i in 0..k {
                // Spread them across the window, well clear of DC, at a mix of
                // speeds and levels so the detector sees a realistic band.
                let off = -18_000.0 + i as f64 * 1_300.0;
                let wpm = 18.0 + (i % 5) as f32 * 4.0;
                let amp = 1.0 / (1.0 + (i % 4) as f32);
                let mut sig = synth_seconds(MSG, off, wpm, rate, 0.0, SECONDS);
                if over_s > 0.0 {
                    // Key for `over_s`, listen for `over_s`, staggered per station
                    // so they are not all sending at once.
                    let period = (rate * over_s * 2.0) as usize;
                    let skew = period / (k.max(1)) * i;
                    for (j, s) in sig.iter_mut().enumerate() {
                        if (j + skew) % period >= period / 2 {
                            *s = C32::default();
                        }
                    }
                }
                for (m, s) in mixed.iter_mut().zip(sig.iter()) {
                    *m += *s * amp;
                }
            }
            let mut seed = 0xC0FF_EE01u32;
            for m in mixed.iter_mut() {
                let mut r = || {
                    seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
                };
                *m += C32::new(0.02 * r(), 0.02 * r());
            }

            let mut sk = skimmer(rate, center, decoder);
            let wall = std::time::Instant::now();
            let cpu0 = cpu_seconds();
            let mut fed = 0usize;
            let (mut peak_trk, mut peak_conf) = (0usize, 0usize);
            for chunk in mixed.chunks(CHUNK) {
                sk.process(chunk);
                peak_trk = peak_trk.max(sk.tracks.len());
                peak_conf = peak_conf.max(sk.tracks.iter().filter(|t| t.keying()).count());
                fed += chunk.len();
                let audio = fed as f64 / rate;
                let elapsed = wall.elapsed().as_secs_f64();
                if audio > elapsed {
                    std::thread::sleep(std::time::Duration::from_secs_f64(audio - elapsed));
                }
            }
            settle(&mut sk);
            let cpu = cpu_seconds() - cpu0;
            let windows = sk.deep.as_ref().map_or(0, |(front, _)| front.windows_built());
            println!(
                "{:>8}{k:>5}{:>7}{:>10.2}{cpu:>10.1}{:>9}{:>9}{windows:>9}{peak_trk:>7}{peak_conf:>7}{:>7}",
                decoder.label(),
                if over_s > 0.0 { "50%" } else { "100%" },
                cpu / SECONDS,
                sk.submits,
                sk.empty,
                sk.spots().len()
            );
        }
    }

    /// Let the model catch up. Decoding happens on a pool of threads now, so a
    /// test that stops feeding IQ has to wait for what is still in flight.
    /// Feed a clip the way the engine does, letting the decoders keep up.
    ///
    /// They run on threads, so pushing a clip through faster than they can read
    /// it leaves the pool refusing windows — and *which* ones it refused is what
    /// decides each station's final text, so the answer comes out different from
    /// run to run. Waiting between blocks makes it deterministic and puts every
    /// decode at the clock time it would really have happened at.
    fn feed(sk: &mut CwSkimmer, iq: &[C32]) {
        for chunk in iq.chunks(8192) {
            sk.process(chunk);
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
            while std::time::Instant::now() < deadline
                && sk.deep.as_ref().is_some_and(|(_, pool)| pool.in_flight() > 0)
            {
                std::thread::sleep(std::time::Duration::from_millis(2));
                sk.process(&[]);
            }
        }
    }

    /// Drain whatever is still in flight once the clip has run out.
    fn settle(sk: &mut CwSkimmer) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(60);
        while std::time::Instant::now() < deadline {
            sk.process(&[]);
            let done = match sk.deep.as_ref() {
                Some((front, pool)) => pool.in_flight() == 0 && front.due().is_empty(),
                None => true,
            };
            if done {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    /// A skimmer running the named decoder, everything else default.
    fn skimmer(rate: f64, center: f64, decoder: CwSkimmerDecoder) -> CwSkimmer {
        let cfg = SkimmerSettings { cw_decoder: decoder, ..SkimmerSettings::default() };
        CwSkimmer::with_config(rate, center, &cfg)
    }

    /// Both decoders read a clean signal and pull the callsign out of it.
    ///
    /// The timing decoder is the fallback for hardware that cannot spare the
    /// cores for a Conformer per station, so it has to keep working: it is the
    /// same four end-to-end cases as before DeepCW arrived, and they are run
    /// against both.
    #[test]
    fn both_decoders_read_a_clean_signal() {
        let rate = 192_000.0;
        let center = 14_020_000.0;
        let off = 5_000.0;
        for decoder in CwSkimmerDecoder::ALL {
            let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);
            let mut sk = skimmer(rate, center, decoder);
            feed(&mut sk, &iq);
            settle(&mut sk);
            eprintln!("--- {} ---", decoder.label());
            sk.debug_dump();
            let spots = sk.spots();
            let hit = spots.iter().find(|s| s.callsign.as_deref() == Some("W1AW"));
            assert!(hit.is_some(), "{}: W1AW not spotted; got {spots:?}", decoder.label());
            let err = hit.unwrap().freq_hz - (center + off);
            assert!(err.abs() < 100.0, "{}: spotted {err:+.0} Hz off", decoder.label());
        }
    }

    /// The timing decoder keeps several stations apart too.
    ///
    /// Levelled more evenly than the neural test's 25 dB spread: reading a
    /// signal a few dB out of the noise is what the model is *for*, and this
    /// decoder needs about 8 dB. What is under test here is that one decoder per
    /// track still fits each station's own envelope, which is the property the
    /// whole approach rests on.
    #[test]
    fn the_timing_decoder_reads_several_stations() {
        let rate = 192_000.0;
        let center = 14_030_000.0;
        let want = [
            ("CQ TEST DE W1AW W1AW K", 6_000.0, 18.0, 1.0),
            ("CQ CQ DE K5ZZ K5ZZ K", -2_500.0, 24.0, 0.6),
            ("CQ DE VK3XY VK3XY K", 12_000.0, 22.0, 0.8),
        ];
        let mut mixed: Vec<C32> = Vec::new();
        for (text, off, wpm, amp) in want {
            let sig = synth(text, off, wpm, rate, 0.0);
            if mixed.len() < sig.len() {
                mixed.resize(sig.len(), C32::default());
            }
            for (m, s) in mixed.iter_mut().zip(sig.iter()) {
                *m += *s * amp;
            }
        }
        let mut seed = 0xC0FF_EE01u32;
        for m in mixed.iter_mut() {
            let mut r = || {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
            };
            *m += C32::new(0.02 * r(), 0.02 * r());
        }

        let mut sk = skimmer(rate, center, CwSkimmerDecoder::Timing);
        feed(&mut sk, &mixed);
        settle(&mut sk);
        sk.debug_dump();
        let spots = sk.spots();
        for (call, off) in [("W1AW", 6_000.0), ("K5ZZ", -2_500.0), ("VK3XY", 12_000.0)] {
            let hit = spots
                .iter()
                .find(|s| s.callsign.as_deref() == Some(call))
                .unwrap_or_else(|| panic!("{call} not spotted; got {spots:?}"));
            let err = hit.freq_hz - (center + off);
            assert!(err.abs() < 60.0, "{call} spotted {err:+.0} Hz off");
        }
    }

    /// The blind spot belongs to the front end's DC spike, not to the middle of
    /// the window.
    ///
    /// The two used to be the same place, because the window was pinned to the
    /// hardware centre. Now that it follows the operator's waterfall they are
    /// not — and a guard left in the middle of the window would blind three bins
    /// in the middle of the screen, which on a panadapter centred on the dial is
    /// exactly the signal being listened to.
    #[test]
    fn the_dc_guard_follows_the_front_end_not_the_window() {
        let rate = 192_000.0;
        let center = 14_030_000.0;
        let iq = synth("CQ DE W1AW", 0.0, 20.0, rate, 0.02);
        // Within a bin: the interpolated carrier lands sub-hertz from the centre,
        // where the nearest track the notch can leave behind is three bins out.
        let on_center = |s: &SkimmerSpot| (s.freq_hz - center).abs() < 50.0;

        // The front end is 40 kHz down the window, so that is where the notch
        // belongs and the station in the middle is read normally.
        let mut sk = skimmer(rate, center, CwSkimmerDecoder::Timing);
        sk.set_dc_offset_hz(-40_000.0);
        feed(&mut sk, &iq);
        settle(&mut sk);
        let spots = sk.spots();
        assert!(
            spots.iter().any(|s| on_center(s) && s.callsign.as_deref() == Some("W1AW")),
            "the station on the window centre was not decoded: {spots:?}"
        );

        // ...and with the spike declared in the middle — a window still sitting
        // on the hardware centre, which is where one with nobody watching stays
        // — the same station is inside the notch and is not read at all.
        let mut pinned = skimmer(rate, center, CwSkimmerDecoder::Timing);
        feed(&mut pinned, &iq);
        settle(&mut pinned);
        let spots = pinned.spots();
        assert!(
            !spots.iter().any(on_center),
            "the guard did not notch the window centre at all: {spots:?}"
        );
    }

    #[test]
    /// A station off the edge of the waterfall is not tracked at all.
    ///
    /// The point is the decoder time, not where the box lands: an off-screen
    /// track costs a DeepCW inference every few seconds for a spot the operator
    /// cannot see.
    #[test]
    fn only_signals_inside_the_view_are_tracked() {
        let rate = 192_000.0;
        let center = 14_020_000.0;
        let (seen, hidden) = (5_000.0, -60_000.0);
        let mut iq = synth("CQ DE W1AW", seen, 20.0, rate, 0.02);
        for (a, b) in iq.iter_mut().zip(synth("CQ DE K2ABC", hidden, 20.0, rate, 0.0).iter()) {
            *a += *b;
        }

        let mut sk = skimmer(rate, center, CwSkimmerDecoder::Timing);
        sk.set_view(Some((center + seen - 2_000.0, center + seen + 2_000.0)));
        feed(&mut sk, &iq);
        settle(&mut sk);
        let spots = sk.spots();
        assert!(!spots.is_empty(), "the visible station was not decoded");
        for s in &spots {
            assert!(
                (s.freq_hz - (center + hidden)).abs() > 1_000.0,
                "spotted an off-screen station at {}",
                s.freq_hz
            );
        }
    }

    /// Panning away from a station stops the work, rather than just hiding it.
    #[test]
    fn panning_away_drops_the_tracks_it_leaves_behind() {
        let rate = 192_000.0;
        let center = 14_020_000.0;
        let off = 5_000.0;
        let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);

        let mut sk = skimmer(rate, center, CwSkimmerDecoder::Timing);
        feed(&mut sk, &iq);
        settle(&mut sk);
        assert!(!sk.spots().is_empty(), "nothing was tracked, so nothing to drop");

        sk.set_view(Some((center + 50_000.0, center + 90_000.0)));
        assert!(sk.spots().is_empty(), "a track survived the pan away from it");
    }

    /// A window with no CW allocation in it tracks nothing, however loud the
    /// keying.
    ///
    /// The engine discards every CW spot outside a CW sub-segment, so a track
    /// anywhere else was decoder time spent on a result nobody is ever shown.
    /// On the broadcast FM band — where a great many RTL-SDRs live — that was
    /// every track there was.
    #[test]
    fn a_window_with_no_cw_allocation_tracks_nothing() {
        let rate = 192_000.0;
        let off = 5_000.0;
        let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);

        // The same signal on 20 m, to show the test is of the band and not of
        // the signal.
        let mut ham = skimmer(rate, 14_020_000.0, CwSkimmerDecoder::Timing);
        feed(&mut ham, &iq);
        settle(&mut ham);
        assert!(!ham.spots().is_empty(), "the control case decoded nothing");

        let mut fm = skimmer(rate, 96_300_000.0, CwSkimmerDecoder::Timing);
        feed(&mut fm, &iq);
        settle(&mut fm);
        assert!(fm.spots().is_empty(), "tracked a station on the broadcast FM band");
    }

    /// And the mask travels with the window: a skimmer moved onto a CW segment
    /// starts working, one moved off it stops.
    #[test]
    fn moving_the_window_onto_a_cw_segment_starts_the_tracking() {
        let rate = 192_000.0;
        let off = 5_000.0;
        let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);

        let mut sk = skimmer(rate, 96_300_000.0, CwSkimmerDecoder::Timing);
        feed(&mut sk, &iq);
        settle(&mut sk);
        assert!(sk.spots().is_empty(), "tracked a station off any CW allocation");

        sk.set_center(14_020_000.0);
        feed(&mut sk, &iq);
        settle(&mut sk);
        assert!(!sk.spots().is_empty(), "nothing was tracked after moving onto 20 m");
    }

    /// Clearing the view goes back to skimming the whole window — what a
    /// headless server with nobody watching gets.
    #[test]
    fn clearing_the_view_restores_the_whole_window() {
        let rate = 192_000.0;
        let center = 14_020_000.0;
        let off = 5_000.0;
        let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);

        let mut sk = skimmer(rate, center, CwSkimmerDecoder::Timing);
        sk.set_view(Some((center + 50_000.0, center + 90_000.0)));
        sk.set_view(None);
        feed(&mut sk, &iq);
        settle(&mut sk);
        assert!(!sk.spots().is_empty(), "no spots with the view cleared");
    }

    #[test]
    fn decodes_a_single_cw_tone() {
        let rate = 192_000.0;
        let center = 14_020_000.0;
        let off = 5_000.0;
        let iq = synth("CQ DE W1AW", off, 20.0, rate, 0.02);

        let mut sk = CwSkimmer::new(rate, center);
        feed(&mut sk, &iq);
        settle(&mut sk);
        sk.debug_dump();
        let spots = sk.spots();
        assert!(!spots.is_empty(), "no spots decoded");
        // The spot should sit within a couple bins of the true frequency.
        let s = spots
            .iter()
            .min_by(|a, b| {
                (a.freq_hz - (center + off))
                    .abs()
                    .partial_cmp(&(b.freq_hz - (center + off)).abs())
                    .unwrap()
            })
            .unwrap();
        assert!(
            (s.freq_hz - (center + off)).abs() < 100.0,
            "freq off: {} vs {}",
            s.freq_hz,
            center + off
        );
        assert!(s.text.contains("W1AW"), "text: {:?}", s.text);
        assert_eq!(s.callsign.as_deref(), Some("W1AW"));
    }

    #[test]
    fn decodes_a_strong_signal_in_a_high_noise_floor() {
        // Regression for the noise-floor trap: a prominent, near-continuously
        // keyed CW signal riding a HIGH noise floor used to stop decoding (the
        // old floor-gated EMA folded it into its own floor), while weaker signals
        // kept working. The region-median floor can't be inflated by the signal.
        let rate = 192_000.0;
        let center = 7_020_000.0;
        let off = -4_000.0;
        // A high broadband noise floor (15× the light-noise case) with a strong
        // but not overwhelming carrier.
        let iq = synth("CQ TEST DE W1AW W1AW", off, 22.0, rate, 0.3);

        let mut sk = CwSkimmer::new(rate, center);
        feed(&mut sk, &iq);
        settle(&mut sk);
        let spots = sk.spots();
        let hit = spots.iter().find(|s| s.text.contains("W1AW"));
        assert!(hit.is_some(), "strong signal in high noise did not decode: {spots:?}");
    }

    /// A crowded window: several stations at once, at different speeds and 25 dB
    /// apart in level, on a common noise floor.
    ///
    /// This is what a skimmer is for and the case a single global key-on
    /// threshold cannot serve — the level that reads the loud station's keying
    /// is above the weak one's marks entirely. Each track fits its own.
    #[test]
    fn decodes_several_stations_at_once() {
        let rate = 192_000.0;
        let center = 14_030_000.0;
        let want = [
            ("CQ TEST DE W1AW W1AW K", 6_000.0, 18.0, 1.0),
            ("CQ CQ DE K5ZZ K5ZZ K", -2_500.0, 30.0, 0.06),
            ("CQ DE VK3XY VK3XY K", 12_000.0, 24.0, 0.3),
        ];
        let mut mixed: Vec<C32> = Vec::new();
        for (text, off, wpm, amp) in want {
            let sig = synth(text, off, wpm, rate, 0.0);
            if mixed.len() < sig.len() {
                mixed.resize(sig.len(), C32::default());
            }
            for (m, s) in mixed.iter_mut().zip(sig.iter()) {
                *m += *s * amp;
            }
        }
        // One noise floor under all of them.
        let mut seed = 0xC0FF_EE01u32;
        for m in mixed.iter_mut() {
            let mut r = || {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
            };
            *m += C32::new(0.02 * r(), 0.02 * r());
        }

        let mut sk = CwSkimmer::new(rate, center);
        feed(&mut sk, &mixed);
        settle(&mut sk);
        sk.debug_dump();
        let spots = sk.spots();
        for (_, off, wpm, _) in want {
            let call = ["W1AW", "K5ZZ", "VK3XY"]
                [want.iter().position(|w| w.1 == off).expect("offset is one of ours")];
            let hit = spots
                .iter()
                .find(|s| s.callsign.as_deref() == Some(call))
                .unwrap_or_else(|| panic!("{call} not spotted; got {spots:?}"));
            let err = hit.freq_hz - (center + off);
            assert!(err.abs() < 60.0, "{call} spotted {err:+.0} Hz off");
            assert!(
                (hit.wpm as f32 - wpm).abs() < 4.0,
                "{call} read as {} WPM, sent at {wpm}",
                hit.wpm
            );
        }
    }

    /// End-to-end frequency accuracy through the *real* engine path: device-rate
    /// IQ → skim DDC → CwSkimmer. Catches any bin/decimation mismatch that would
    /// mistune the spot marker against the waterfall.
    #[test]
    fn frequency_is_accurate_through_the_ddc() {
        let dev_rate = 2_000_000.0;
        let center = 14_025_000.0;
        for off in [2_000.0f64, -3_500.0, 7_000.0] {
            let iq_dev = synth("CQ DE W1AW", off, 24.0, dev_rate, 0.02);
            let mut ddc = sdroxide_dsp::Ddc::new(dev_rate, 192_000.0);
            let skim_rate = ddc.out_rate();
            let mut sk = CwSkimmer::new(skim_rate, center);
            let mut buf = Vec::new();
            for chunk in iq_dev.chunks(16_384) {
                buf.clear();
                ddc.process(chunk, &mut buf);
                sk.process(&buf);
            }
            settle(&mut sk);
            let spots = sk.spots();
            assert!(!spots.is_empty(), "off {off}: no spots");
            let want = center + off;
            let s = spots
                .iter()
                .min_by(|a, b| {
                    (a.freq_hz - want).abs().partial_cmp(&(b.freq_hz - want).abs()).unwrap()
                })
                .unwrap();
            let err = s.freq_hz - want;
            eprintln!("off {off:+}: got {} want {} err {err:+.1} Hz", s.freq_hz, want);
            assert!(err.abs() < 20.0, "off {off}: freq error {err:+.1} Hz");
        }
    }
}
