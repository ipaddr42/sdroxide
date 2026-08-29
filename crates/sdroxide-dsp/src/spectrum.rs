use std::sync::Arc;

use rustfft::{Fft, FftPlanner};
use sdroxide_types::SpectrumFrame;

use crate::{Complex32, window::blackman_harris};

/// Overlapped windowed-FFT power spectrum with exponential averaging.
///
/// Feed raw IQ with [`process`](Self::process); it runs an FFT every
/// `fft_size / 2` samples (50 % overlap) and folds the result into a running
/// power average. Emit display frames at any rate with
/// [`make_frame`](Self::make_frame).
pub struct SpectrumAnalyzer {
    fft: Arc<dyn Fft<f32>>,
    fft_size: usize,
    hop: usize,
    window: Vec<f32>,
    /// Normalization so a full-scale coherent sine reads ~0 dBFS.
    coherent_gain: f32,

    pending: Vec<Complex32>,
    scratch: Vec<Complex32>,
    work: Vec<Complex32>,

    /// Averaged linear power per bin, natural FFT order.
    avg_power: Vec<f32>,
    /// Peak of [`Self::avg_power`] since the last waterfall row was taken —
    /// empty while nobody is clocking rows.
    ///
    /// The waterfall's time resolution is bounded by how often rows are drawn,
    /// not by how often transforms land: a front end streaming megahertz runs
    /// hundreds of transforms a second and a screen draws at most ninety. With
    /// only the latest one kept, everything between two rows is discarded, and
    /// a CW dot shorter than a row interval can fall between two frames and
    /// never be seen at all. Holding the maximum instead means a row is
    /// "the strongest thing that happened in this slice of time", which is
    /// what a waterfall is *for*.
    hold: Vec<f32>,
    /// Whether [`Self::make_frame`] pools from [`Self::hold`] instead of
    /// [`Self::avg_power`]. Set for the length of one row build — see
    /// [`Self::set_read_hold`].
    read_hold: bool,
    /// Whether the last [`Self::make_frame`] was a row build — see
    /// [`Self::took_row`].
    hold_read: bool,
    alpha: f32,
    primed: bool,
    peak_abs: f32,
    /// Whether [`Self::process`] scans every sample for the input peak.
    ///
    /// Off by default. It is a pass over the whole stream — on an RX-888 at
    /// 32.4 Msps, thirty-two million squared magnitudes a second — kept for one
    /// caller: the terminal waterfall prints a dBFS column beside each line.
    /// Nothing in the GUI or the server reads it, and a lane that nobody asks
    /// should not pay for it. See [`Self::take_peak_dbfs`].
    peak_track: bool,
    seq: u32,
    /// Hide the hardware DC/LO-leakage spike in emitted frames.
    dc_suppress: bool,
    /// Transforms folded into the average since this analyser was built.
    ///
    /// The rate this climbs at is the *real* update rate of anything drawn
    /// from it: emitted frames carry a fresh `seq` whether or not a new
    /// transform landed between them, so a lane can publish at 30 fps while
    /// showing the same numbers for a tenth of a second. Diagnostic only.
    transforms: u64,
}

impl SpectrumAnalyzer {
    pub fn new(fft_size: usize, sample_rate: f64, avg_tc_secs: f32) -> Self {
        Self::with_hop_div(fft_size, sample_rate, avg_tc_secs, 2)
    }

    /// [`Self::new`] with the overlap named: the analyser steps
    /// `fft_size / hop_div` samples between transforms, so `2` is `new`'s 50 %.
    ///
    /// Worth raising where one transform takes a long time to fill. Resolution
    /// and update rate are the same trade on any analyser — a transform cannot
    /// resolve finer than the reciprocal of the time it covers — but the rate
    /// rows reach the waterfall need not be the rate whole windows arrive at.
    /// A zoomed-in lane running at a few kilohertz fills a 4096-point window
    /// only a couple of times a second; stepping an eighth of it instead of a
    /// half puts four times as many rows on screen for the same resolution and
    /// a few hundred extra microseconds of FFT a second.
    pub fn with_hop_div(
        fft_size: usize,
        sample_rate: f64,
        avg_tc_secs: f32,
        hop_div: usize,
    ) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(fft_size);
        let window = blackman_harris(fft_size);
        let coherent_gain: f32 = window.iter().sum();
        let hop = (fft_size / hop_div.max(1)).max(1);
        let scratch = vec![Complex32::default(); fft.get_inplace_scratch_len()];

        let mut analyzer = SpectrumAnalyzer {
            fft,
            fft_size,
            hop,
            window,
            coherent_gain,
            pending: Vec::with_capacity(fft_size * 2),
            scratch,
            work: vec![Complex32::default(); fft_size],
            avg_power: vec![0.0; fft_size],
            hold: Vec::new(),
            read_hold: false,
            hold_read: false,
            alpha: 1.0,
            primed: false,
            peak_abs: 0.0,
            peak_track: false,
            seq: 0,
            transforms: 0,
            dc_suppress: true,
        };
        analyzer.set_avg_tc(avg_tc_secs, sample_rate);
        analyzer
    }

    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Whether the bins either side of DC are replaced by their neighbours.
    ///
    /// On by default, because on a front end feeding this directly DC is the
    /// hardware's own LO leakage and a spike there is an artefact. Off for an
    /// analyser looking at a *mixed-down* window, where DC is the middle of
    /// whatever the operator is pointed at: blanking it would punch a hole
    /// through the signal they are looking at.
    pub fn set_dc_suppress(&mut self, on: bool) {
        self.dc_suppress = on;
    }

    /// Clear the overlap/averaging state (e.g. across a TX→RX transition so
    /// transmit samples don't contaminate the first receive frames).
    pub fn reset(&mut self) {
        self.pending.clear();
        self.avg_power.iter_mut().for_each(|p| *p = 0.0);
        self.hold.iter_mut().for_each(|p| *p = 0.0);
        self.primed = false;
        self.peak_abs = 0.0;
    }

    /// Start (or stop) holding the per-bin peak between waterfall rows.
    ///
    /// Costs one `max` per bin per transform, so it is only switched on for a
    /// lane whose rows somebody is actually clocking.
    pub fn set_row_hold(&mut self, on: bool) {
        if on == self.row_hold() {
            return;
        }
        self.hold = if on { self.avg_power.clone() } else { Vec::new() };
    }

    /// Whether the peak hold is running.
    pub fn row_hold(&self) -> bool {
        !self.hold.is_empty()
    }

    /// Read the next [`Self::make_frame`] from the held peaks rather than the
    /// running average.
    ///
    /// A flag rather than a second `make_frame`, so a row and the frame it
    /// rides in are pooled by *exactly* the same code over exactly the same
    /// viewport — two copies of that arithmetic would drift apart on the first
    /// change to either. Set it, build the row, clear it; see
    /// `Engine::make_row`.
    ///
    /// Armed even where no peaks are being held yet: such a frame reads the
    /// running average, exactly as every build before the row clock did, and
    /// [`Self::took_row`] then reports that this lane is the one drawing rows
    /// so its caller can switch the hold on. That is how a lane pays for the
    /// hold only while it is the lane somebody is looking at.
    pub fn set_read_hold(&mut self, on: bool) {
        self.read_hold = on;
    }

    /// Whether the last [`Self::make_frame`] was built as a waterfall row, and
    /// clear the flag.
    ///
    /// The caller sets [`Self::set_read_hold`] on every lane that *could* draw
    /// and then asks each of them this, rather than working out in advance
    /// which one will: the choice of lane is a dozen conditions deep in the
    /// engine's frame builder, and a second copy of it here would go wrong the
    /// first time either changed.
    pub fn took_row(&mut self) -> bool {
        std::mem::take(&mut self.hold_read)
    }

    /// Begin a fresh row: the peak starts again from where the spectrum is now,
    /// so a row whose interval contained no transform at all still reads as the
    /// current spectrum rather than as zero.
    pub fn reset_hold(&mut self) {
        if !self.hold.is_empty() {
            self.hold.copy_from_slice(&self.avg_power);
        }
    }

    pub fn set_avg_tc(&mut self, tc_secs: f32, sample_rate: f64) {
        let hop_time = self.hop as f32 / sample_rate as f32;
        self.alpha = if tc_secs <= 0.0 { 1.0 } else { 1.0 - (-hop_time / tc_secs).exp() };
    }

    /// Start (or stop) tracking the input peak [`Self::take_peak_dbfs`] reads.
    ///
    /// Off by default, because it costs a pass over every sample and only the
    /// terminal waterfall wants it — see [`Self::peak_track`](#structfield.peak_track).
    pub fn set_peak_track(&mut self, on: bool) {
        self.peak_track = on;
        if !on {
            self.peak_abs = 0.0;
        }
    }

    /// Consume IQ samples, running as many overlapped FFTs as fit.
    pub fn process(&mut self, iq: &[Complex32]) {
        if self.peak_track {
            let mut peak = self.peak_abs;
            for s in iq {
                let a = s.norm_sqr();
                if a > peak {
                    peak = a;
                }
            }
            self.peak_abs = peak;
        }
        self.pending.extend_from_slice(iq);

        // Walk the buffer with an offset and compact it once at the end, rather
        // than draining a hop off the front after every transform.
        //
        // `Vec::drain` from the front shifts everything after it down, so the
        // old loop memmoved the whole remaining buffer once per transform: on a
        // block of sixteen thousand samples through a 4096-point window that is
        // eight shifts of some ten thousand complex samples apiece, tens of
        // megabytes a second of pure copying, and it grows with the front end's
        // rate. One compaction per block does the same job.
        let mut at = 0;
        while self.pending.len() - at >= self.fft_size {
            let frame = &self.pending[at..at + self.fft_size];
            for (w, (x, win)) in self.work.iter_mut().zip(frame.iter().zip(&self.window)) {
                *w = x * win;
            }
            self.fft.process_with_scratch(&mut self.work, &mut self.scratch);

            let norm = 1.0 / (self.coherent_gain * self.coherent_gain);
            let alpha = if self.primed { self.alpha } else { 1.0 };
            self.primed = true;
            // Two loops rather than one with a branch inside: this runs
            // `fft_size` times per transform and hundreds of times a second on
            // a wide front end, and the hold is off on most lanes.
            if self.hold.is_empty() {
                for (avg, x) in self.avg_power.iter_mut().zip(&self.work) {
                    let p = x.norm_sqr() * norm;
                    *avg += alpha * (p - *avg);
                }
            } else {
                for ((avg, h), x) in self.avg_power.iter_mut().zip(&mut self.hold).zip(&self.work) {
                    let p = x.norm_sqr() * norm;
                    *avg += alpha * (p - *avg);
                    if *avg > *h {
                        *h = *avg;
                    }
                }
            }
            self.transforms = self.transforms.wrapping_add(1);
            at += self.hop;
        }
        if at > 0 {
            self.pending.drain(..at);
        }
    }

    /// Transforms folded in since this analyser was built — see
    /// [`Self::transforms`](#structfield.transforms).
    pub fn transforms(&self) -> u64 {
        self.transforms
    }

    /// Peak input magnitude (dBFS) since the last call; resets on read.
    ///
    /// Reads zero unless [`Self::set_peak_track`] switched the scan on.
    pub fn take_peak_dbfs(&mut self) -> f32 {
        let p = self.peak_abs;
        self.peak_abs = 0.0;
        10.0 * (p + 1e-20).log10()
    }

    /// Averaged spectrum in dBFS, frequency-ascending (DC centered).
    pub fn spectrum_db(&self, out: &mut Vec<f32>) {
        out.clear();
        out.reserve(self.fft_size);
        let half = self.fft_size / 2;
        for &p in self.avg_power[half..].iter().chain(&self.avg_power[..half]) {
            out.push(10.0 * (p + 1e-20).log10());
        }
    }

    /// Build a display frame with `out_bins` bins (max-pooled), u8-mapped
    /// over `[db_floor, db_ceil]`. With `viewport = Some((lo_hz, hi_hz))`
    /// only that sub-span is extracted (zoomed display); the frame's
    /// center/span then describe the viewport.
    pub fn make_frame(
        &mut self,
        center_hz: f64,
        span_hz: f64,
        db_floor: f32,
        db_ceil: f32,
        out_bins: usize,
        viewport: Option<(f64, f64)>,
    ) -> SpectrumFrame {
        let n = self.fft_size;
        let half = n / 2;
        let out_bins = out_bins.clamp(1, n * 4);
        let scale = 255.0 / (db_ceil - db_floor).max(1e-3);

        let (frac_lo, frac_hi, out_center, out_span) = match viewport {
            Some((lo, hi)) if hi > lo && span_hz > 0.0 => {
                let full_lo = center_hz - span_hz / 2.0;
                let flo = ((lo - full_lo) / span_hz).clamp(0.0, 0.998);
                let fhi = ((hi - full_lo) / span_hz).clamp(flo + 0.002, 1.0);
                (flo, fhi, full_lo + (flo + fhi) / 2.0 * span_hz, (fhi - flo) * span_hz)
            }
            _ => (0.0, 1.0, center_hz, span_hz),
        };

        // Held peaks for a waterfall row, or the running average for the
        // trace — see [`Self::set_read_hold`]. Same pooling either way. A lane
        // asked for a row before its hold was switched on answers from the
        // average, which is the current spectrum and so a correct row; the
        // hold starts on the next one.
        self.hold_read = self.read_hold;
        let power: &[f32] =
            if self.read_hold && !self.hold.is_empty() { &self.hold } else { &self.avg_power };

        // DC spike suppression (hardware LO leakage): read the ±2 bins
        // around DC as the average of their neighbors. Patch at read time so
        // the running average stays uncontaminated.
        let dc_repl = if self.dc_suppress && n > 16 {
            let mut acc = 0.0f32;
            for d in 3..=6 {
                acc += power[d] + power[n - d];
            }
            Some(acc / 8.0)
        } else {
            None
        };
        let shifted = |i: usize| {
            let nat = (i + half) % n;
            if let Some(repl) = dc_repl {
                if nat.min(n - nat) <= 2 {
                    return repl;
                }
            }
            power[nat]
        };
        let lo_bin = frac_lo * n as f64;
        let bin_range = (frac_hi - frac_lo) * n as f64;

        let mut bins = Vec::with_capacity(out_bins);
        if bin_range < out_bins as f64 {
            // Stretching, not pooling: the window holds fewer bins than there
            // are columns to fill, so each bin has to cover several. Reading
            // between them rather than repeating them — a repeated bin is a
            // hard block, and a wall of them is what a zoom past the transform's
            // resolution used to look like. The same rule the finished-sweep
            // pooler uses; see `pool_window_to_frame` in the engine.
            //
            // Interpolated in power and then taken to dB, so the ramp between
            // two bins is the ramp between the two levels rather than between
            // their logarithms.
            for b in 0..out_bins {
                // Centre of this column in bin coordinates, less the half bin
                // that puts bin centres on column centres.
                let at = lo_bin + (b as f64 + 0.5) * bin_range / out_bins as f64 - 0.5;
                let k = at.floor().clamp(0.0, (n - 1) as f64) as usize;
                let t = (at - k as f64).clamp(0.0, 1.0) as f32;
                let (p0, p1) = (shifted(k), shifted((k + 1).min(n - 1)));
                let db = 10.0 * (p0 + (p1 - p0) * t + 1e-20).log10();
                bins.push(((db - db_floor) * scale).clamp(0.0, 255.0) as u8);
            }
        } else {
            for b in 0..out_bins {
                let lo = (lo_bin + b as f64 * bin_range / out_bins as f64) as usize;
                let hi = ((lo_bin + (b + 1) as f64 * bin_range / out_bins as f64) as usize)
                    .clamp(lo + 1, n);
                let mut max_p = 0.0f32;
                for i in lo..hi.max(lo + 1).min(n) {
                    max_p = max_p.max(shifted(i));
                }
                let db = 10.0 * (max_p + 1e-20).log10();
                bins.push(((db - db_floor) * scale).clamp(0.0, 255.0) as u8);
            }
        }

        self.seq = self.seq.wrapping_add(1);
        SpectrumFrame {
            seq: self.seq,
            center_hz: out_center,
            span_hz: out_span,
            db_floor,
            db_ceil,
            bins,
            rows: Vec::new(),
            rows_clocked: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    /// A lane asked for a row before its hold exists answers from the running
    /// average and says it drew, so the caller knows to switch the hold on.
    ///
    /// That is what lets a hold cost anything only on the lane somebody is
    /// actually pooling rows from: a device-wide analyser sitting behind a zoom
    /// lane pays a compare and a store per bin on every transform for a picture
    /// nobody is looking at, which on a 2.4 Msps front end through a
    /// 131072-point window is half a megabyte of write traffic per transform
    /// (issue #216).
    #[test]
    fn a_lane_asked_for_a_row_says_so_before_it_holds_anything() {
        let fs = 1_000_000.0;
        let n = 1024;
        let mut an = SpectrumAnalyzer::new(n, fs, 0.0);
        an.set_dc_suppress(false);
        assert!(!an.row_hold(), "nothing is held until somebody asks");

        let tone: Vec<Complex32> = (0..n * 2)
            .map(|i| {
                let ph = TAU * 250_000.0 * i as f32 / fs as f32;
                Complex32::new(ph.cos(), ph.sin())
            })
            .collect();
        an.process(&tone);

        // A row from a lane with no hold: the current spectrum, which is what
        // every build before the row clock drew, and not a frame of zeros.
        an.set_read_hold(true);
        let row = an.make_frame(0.0, fs, -120.0, 0.0, n, None);
        an.set_read_hold(false);
        assert!(an.took_row(), "the lane that drew the row has to say so");
        assert!(!an.took_row(), "…once");
        assert!(row.bins.iter().any(|&b| b > 40), "the row is empty: {:?}", row.bins.len());

        // A lane that was armed but did not draw says nothing, so its caller
        // can give the hold up.
        let mut idle = SpectrumAnalyzer::new(n, fs, 0.0);
        idle.set_read_hold(true);
        idle.set_read_hold(false);
        assert!(!idle.took_row(), "a lane nobody pooled from must not claim the row");
    }

    /// A tone that came and went between two waterfall rows still shows up in
    /// the row it happened in.
    ///
    /// The running average keeps only the newest transform (`avg_tc` 0, which
    /// is what the panadapter asks for), so by the time a row is taken the
    /// burst is already gone from it. That is how a CW dot shorter than the
    /// gap between two frames used to vanish entirely — the transform that saw
    /// it was computed, folded in, and overwritten before anyone looked. The
    /// peak hold is what makes a row mean "the loudest thing in this slice of
    /// time" instead of "whatever was last".
    #[test]
    fn a_burst_between_rows_survives_into_the_row_it_happened_in() {
        let fs = 1_000_000.0;
        let n = 1024;
        let tone_hz = 250_000.0f32;
        let mut an = SpectrumAnalyzer::new(n, fs, 0.0);
        an.set_dc_suppress(false);
        an.set_row_hold(true);

        let burst: Vec<Complex32> = (0..n * 4)
            .map(|i| {
                let ph = TAU * tone_hz * i as f32 / fs as f32;
                Complex32::new(ph.cos(), ph.sin())
            })
            .collect();
        let quiet = vec![Complex32::new(0.0, 0.0); n * 8];

        // The burst, then silence — all inside one row interval.
        an.process(&burst);
        an.process(&quiet);

        // The strongest column anywhere in the frame — the tone's exact bin is
        // the other test's business.
        let loudest = |f: &SpectrumFrame| f.bins.iter().copied().max().unwrap_or(0);

        // The running average has forgotten it: this is what the trace draws,
        // and it is correct for the trace — the band *is* quiet now.
        let now = an.make_frame(0.0, fs, -120.0, 0.0, n, None);
        assert!(loudest(&now) < 40, "the average should have moved on: {}", loudest(&now));

        // The row has not.
        an.set_read_hold(true);
        let row = an.make_frame(0.0, fs, -120.0, 0.0, n, None);
        an.set_read_hold(false);
        assert!(loudest(&row) > 200, "the row should still hold the burst: {}", loudest(&row));

        // And the next row starts from where the band is now, not from the
        // peak it just reported — otherwise one burst would smear down the
        // whole waterfall.
        an.reset_hold();
        an.set_read_hold(true);
        let next = an.make_frame(0.0, fs, -120.0, 0.0, n, None);
        an.set_read_hold(false);
        assert!(loudest(&next) < 40, "the hold should have been released: {}", loudest(&next));
    }

    /// With the hold off, nothing about the analyser changes — the same
    /// numbers, the same frames, for every lane that does not clock rows.
    #[test]
    fn the_hold_is_inert_until_it_is_switched_on() {
        let fs = 1_000_000.0;
        let n = 256;
        let iq: Vec<Complex32> = (0..n * 4)
            .map(|i| {
                let ph = TAU * 100_000.0 * i as f32 / fs as f32;
                Complex32::new(ph.cos(), ph.sin())
            })
            .collect();

        let mut plain = SpectrumAnalyzer::new(n, fs, 0.0);
        let mut held = SpectrumAnalyzer::new(n, fs, 0.0);
        held.set_row_hold(true);
        plain.process(&iq);
        held.process(&iq);

        assert!(!plain.row_hold());
        assert!(held.row_hold());
        assert_eq!(
            plain.make_frame(0.0, fs, -120.0, 0.0, n, None).bins,
            held.make_frame(0.0, fs, -120.0, 0.0, n, None).bins,
        );
    }

    #[test]
    fn tone_lands_in_the_right_bin_at_the_right_level() {
        let fs = 1_000_000.0;
        let n = 1024;
        let tone_hz = 250_000.0f32; // exactly bin n*0.25 above DC
        let mut an = SpectrumAnalyzer::new(n, fs, 0.0);

        let iq: Vec<Complex32> = (0..n * 4)
            .map(|i| {
                let ph = TAU * tone_hz * i as f32 / fs as f32;
                Complex32::new(ph.cos(), ph.sin())
            })
            .collect();
        an.process(&iq);

        let mut db = Vec::new();
        an.spectrum_db(&mut db);

        // DC-centered ordering: +fs/4 sits at 3/4 of the display.
        let expect = n * 3 / 4;
        let peak_bin = db.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        assert_eq!(peak_bin, expect);
        // Full-scale coherent tone should read close to 0 dBFS.
        assert!(db[peak_bin] > -1.0 && db[peak_bin] < 1.0, "{}", db[peak_bin]);
        // Blackman-Harris sidelobes: far bins well below -80 dB.
        assert!(db[n / 4] < -80.0, "{}", db[n / 4]);
    }
}
