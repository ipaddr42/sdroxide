//! `Pi4Controller` — the PI4 propagation beacon, as the engine sees it.
//!
//! Decode only: unlike [`crate::WsprController`] this has no burst, no duty
//! cycle and no band hopping — it is a receiving amateur's decoder for a
//! beacon network's signal, not a beacon implementation (see
//! `sdroxide_digi::pi4`'s module doc).
//!
//! No slot-boundary buffer reset either, unlike WSPR's. WSPR needs the whole
//! two-minute slot because its signal can start anywhere from a couple of
//! seconds early to the end of the burst, and the only way to know a slot is
//! over is to wait for the next one to start. PI4's message is 24.333 s long
//! inside a 60 s cycle and starts within a couple of seconds of the minute by
//! convention, so a rolling window of the last half-minute-plus, snapshotted
//! once the window has had time to fill past the message's end, is both
//! simpler and catches a beacon whose clock runs early — which a slot reset
//! at the boundary would have already thrown away the samples for.

use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::SystemTime;

use sdroxide_dsp::MonoResampler;
use sdroxide_types::{DigiConfig, DigiStatus, Mode, Pi4Spot, Pi4Status, QsoStep};

use crate::DigiEngine;
use crate::controller::DigiAction;
use crate::params::DECODE_RATE;
use crate::pi4::{self, Pi4Decode};
use crate::scheduler::SlotScheduler;

/// Real audio kept before the nominal minute boundary, so a beacon (or a
/// receiving station's clock) running a little fast is not simply missing
/// from the window — see [`crate::pi4::decode::decode_window`]'s own time
/// search, which this pad has to cover.
const PREROLL_S: f64 = 3.0;

/// Audio kept after the boundary: the 24.333 s message plus slack for
/// [`PREROLL_S`]'s worth of search radius at the far end.
///
/// That runs past the CW identification and unmodulated carrier the beacon
/// sends from 25 s, so both are inside the window the decoder searches —
/// there is no way to hold a whole late-starting message without them. They
/// cost nothing: a keyed 800 Hz tone and a steady one are not four-tone FSK,
/// they score nothing against the sync vector, and a candidate built from
/// them does not survive [`crate::pi4::decode::fit_of`]. What they must not
/// do is push the window past the next minute boundary, which is why this
/// stops at 30 s rather than filling the cycle.
const POSTROLL_S: f64 = 30.0;

/// One window of audio handed to the decode worker.
struct DecodeJob {
    audio: Vec<f32>,
    /// Where the nominal boundary falls inside `audio`.
    boundary_sample: i64,
    slot_utc: i64,
    /// The dial the audio was recorded on — carried through rather than read
    /// back from the engine later, for the reason
    /// [`crate::wspr_controller::DecodeJob`]'s own field is: band hopping (or
    /// here, simply the operator retuning) may have moved the dial by the
    /// time the decode comes back.
    dial_hz: f64,
}

struct DecodeResult {
    slot_utc: i64,
    dial_hz: f64,
    decodes: Vec<Pi4Decode>,
}

pub struct Pi4Controller {
    cfg: DigiConfig,
    scheduler: SlotScheduler,
    resampler: Option<MonoResampler>,
    tap_scratch: Vec<f32>,
    /// The last [`PREROLL_S`] + [`POSTROLL_S`] seconds of 12 kHz audio.
    ring: VecDeque<f32>,
    ring_cap: usize,
    dial_hz: f64,
    audio_hz: f32,

    job_tx: Sender<DecodeJob>,
    res_rx: Receiver<DecodeResult>,
    _worker: std::thread::JoinHandle<()>,
    /// The slot a decode is outstanding for, if one is. Only ever one in
    /// flight — the search is a few hundred milliseconds of work inside a
    /// 60 s cycle, nothing here needs to overlap it.
    pending: Option<i64>,
    last_submitted_idx: i64,

    spots_last_slot: u32,
    status_dirty: bool,
}

impl Pi4Controller {
    pub fn new(cfg: DigiConfig, tap_rate: f64) -> Self {
        let (job_tx, job_rx) = std::sync::mpsc::channel::<DecodeJob>();
        let (res_tx, res_rx) = std::sync::mpsc::channel::<DecodeResult>();
        let worker = std::thread::Builder::new()
            .name("sdroxide-pi4-decode".into())
            .spawn(move || {
                while let Ok(job) = job_rx.recv() {
                    let decodes =
                        pi4::decode_window(&job.audio, DECODE_RATE as u32, job.boundary_sample);
                    let res =
                        DecodeResult { slot_utc: job.slot_utc, dial_hz: job.dial_hz, decodes };
                    if res_tx.send(res).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn pi4 decode worker");

        let ring_cap = ((PREROLL_S + POSTROLL_S) * DECODE_RATE) as usize;
        Pi4Controller {
            cfg,
            scheduler: SlotScheduler::for_mode(Mode::Pi4),
            resampler: MonoResampler::new(tap_rate, DECODE_RATE),
            tap_scratch: Vec::new(),
            ring: VecDeque::with_capacity(ring_cap),
            ring_cap,
            dial_hz: 144_470_000.0,
            audio_hz: 800.0,
            job_tx,
            res_rx,
            _worker: worker,
            pending: None,
            last_submitted_idx: i64::MIN,
            spots_last_slot: 0,
            status_dirty: true,
        }
    }

    fn to_spot(&self, d: &Pi4Decode, slot_utc: i64, dial_hz: f64) -> Pi4Spot {
        Pi4Spot {
            slot_utc,
            text: d.text.clone(),
            variant: d.variant.label().to_string(),
            tone0_hz: dial_hz + d.tone0_hz as f64,
            dt_sec: d.dt_sec,
            snr_db: d.snr_db,
            fit: d.fit,
        }
    }

    fn pi4_status(&self, idx: i64) -> Pi4Status {
        Pi4Status {
            slot_utc: self.scheduler.slot_start_unix(idx) as i64,
            decoding: self.pending.is_some(),
            last_slot_spots: self.spots_last_slot,
        }
    }

    pub fn status(&self) -> DigiStatus {
        let mut s = DigiStatus::idle(self.cfg.clone());
        s.mode = Mode::Pi4;
        s.step = QsoStep::Idle;
        s.audio_hz = self.audio_hz;
        s.pi4 = Some(self.pi4_status(self.scheduler.slot_index(SystemTime::now())));
        s
    }
}

impl DigiEngine for Pi4Controller {
    fn mode(&self) -> Mode {
        Mode::Pi4
    }

    fn on_rx_audio(&mut self, tap: &[f32]) {
        self.tap_scratch.clear();
        match &mut self.resampler {
            Some(r) => r.push(tap, &mut self.tap_scratch),
            None => self.tap_scratch.extend_from_slice(tap),
        }
        for &s in &self.tap_scratch {
            if self.ring.len() >= self.ring_cap {
                self.ring.pop_front();
            }
            self.ring.push_back(s.clamp(-1.0, 1.0));
        }
    }

    fn poll(&mut self, now: SystemTime, dial_hz: f64) -> Vec<DigiAction> {
        self.dial_hz = dial_hz;
        let mut actions = Vec::new();

        loop {
            match self.res_rx.try_recv() {
                Ok(res) => {
                    self.pending = None;
                    let spots: Vec<Pi4Spot> = res
                        .decodes
                        .iter()
                        .map(|d| self.to_spot(d, res.slot_utc, res.dial_hz))
                        .collect();
                    self.spots_last_slot = spots.len() as u32;
                    self.status_dirty = true;
                    if !spots.is_empty() {
                        actions.push(DigiAction::Pi4Spots(spots));
                    }
                }
                Err(TryRecvError::Empty) => break,
                // The worker is gone and no answer is ever coming. Release
                // the slot rather than leaving it outstanding: `pending` is
                // what gates the next submission as well as what the panel
                // shows, so holding it would wedge the controller at
                // "decoding…" for the rest of the session instead of simply
                // decoding nothing.
                Err(TryRecvError::Disconnected) => {
                    if self.pending.take().is_some() {
                        self.status_dirty = true;
                    }
                    break;
                }
            }
        }

        let idx = self.scheduler.slot_index(now);
        let into_slot = self.scheduler.secs_into_slot(now);
        // Fire once per minute, once the rolling window has had time to fill
        // past the message's end — see this module's own doc comment for why
        // this is a snapshot of an always-current buffer rather than a
        // per-slot reset.
        if into_slot >= POSTROLL_S && idx != self.last_submitted_idx && self.pending.is_none() {
            self.last_submitted_idx = idx;
            // Only once the ring actually holds something: at startup it is
            // still filling, and a job over near-silence would only ever come
            // back empty.
            if !self.ring.is_empty() {
                let audio: Vec<f32> = self.ring.iter().copied().collect();
                let boundary_sample = (audio.len() as f64 - POSTROLL_S * DECODE_RATE) as i64;
                self.pending = Some(idx);
                self.status_dirty = true;
                let _ = self.job_tx.send(DecodeJob {
                    audio,
                    boundary_sample,
                    slot_utc: self.scheduler.slot_start_unix(idx) as i64,
                    dial_hz: self.dial_hz,
                });
            }
        }

        if self.status_dirty {
            self.status_dirty = false;
            actions.push(DigiAction::Status(self.status()));
        }
        actions
    }

    fn tx_burst_active(&self) -> bool {
        false
    }

    fn fill_tx_block(&mut self, out: &mut [f32]) -> bool {
        out.fill(0.0);
        true
    }

    fn on_burst_done(&mut self) {}

    fn abort(&mut self) {
        self.ring.clear();
    }

    fn abort_tx(&mut self) {}

    fn set_config(&mut self, cfg: DigiConfig) {
        self.cfg = cfg;
        self.status_dirty = true;
    }

    fn set_audio_hz(&mut self, hz: f32) {
        self.audio_hz = hz;
        self.status_dirty = true;
    }

    fn audio_hz(&self) -> f32 {
        self.audio_hz
    }

    fn status(&self) -> DigiStatus {
        Pi4Controller::status(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctrl() -> Pi4Controller {
        Pi4Controller::new(DigiConfig::default(), 48_000.0)
    }

    #[test]
    fn a_fresh_controller_reports_pi4_mode_and_is_idle() {
        let c = ctrl();
        let s = c.status();
        assert_eq!(s.mode, Mode::Pi4);
        let p4 = s.pi4.expect("pi4 status");
        assert!(!p4.decoding);
        assert_eq!(p4.last_slot_spots, 0);
    }

    /// A slot with real audio must dispatch a decode job once the window has
    /// filled past the message's end, and not before.
    #[test]
    fn a_full_window_of_audio_dispatches_a_decode_once_per_minute() {
        let mut c = ctrl();
        // A real minute boundary, not the epoch.
        let start = 1_785_760_440.0;
        let mut fired = 0;
        for t in (0..90).map(|s| start + s as f64) {
            let now = std::time::UNIX_EPOCH + std::time::Duration::from_secs_f64(t);
            c.on_rx_audio(&vec![0.0f32; (48_000.0 / 1.0) as usize]);
            for a in c.poll(now, 144_470_000.0) {
                if let DigiAction::Status(s) = &a
                    && s.pi4.as_ref().is_some_and(|p| p.decoding)
                {
                    fired += 1;
                }
            }
        }
        // The `decoding` flag only ever flips true once inside this window
        // (POSTROLL_S = 30 s into the 60 s cycle, once, over 90 s of ticks).
        assert!(fired >= 1, "never entered the decoding state over a minute of ticks");
    }
}
