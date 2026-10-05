//! `Q65Controller` — Q65, the modern weak-signal mode, receive only.
//!
//! Slotted like FST4 and shaped the same way, with the same twist: Q65's
//! sub-mode ([`Q65Mode`]) fixes both the T/R period and the tone-spacing
//! letter, so it is an operator setting rather than part of the mode. All ten
//! sub-modes share one waveform shape and one 77-bit message, so the decode is
//! the same call with a different protocol type — and the slot the buffer
//! holds changes length when the operator picks a new sub-mode.
//!
//! # Why a worker thread
//!
//! A Q65-300 scan is tens of seconds of work over a five-minute slot, and even
//! the short sub-modes are far from free. The decode runs on its own thread and
//! the result is drained from [`Self::poll`], as WSPR's and FST4's do — one
//! slot in flight at a time, dropping rather than queueing when the machine
//! cannot keep up.
//!
//! Receive only in this build: transmit needs a sequencer.

use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::SystemTime;

use sdroxide_dsp::MonoResampler;
use sdroxide_types::{Decode, DigiConfig, DigiStatus, Mode, Q65Mode, QsoStep};

use crate::DigiEngine;
use crate::controller::DigiAction;
use crate::modem::decode_q65_slot;
use crate::params::DECODE_RATE;
use crate::scheduler::SlotScheduler;

/// One slot of audio handed to the decode worker.
struct DecodeJob {
    audio: Vec<i16>,
    mode: Q65Mode,
    slot_utc: i64,
}

/// What the worker sends back.
struct DecodeResult {
    decodes: Vec<Decode>,
}

pub struct Q65Controller {
    cfg: DigiConfig,
    /// The sub-mode in force, mirroring `cfg.q65_mode`; re-read whenever the
    /// config changes, because it decides both the slot length and the decode.
    mode: Q65Mode,
    scheduler: SlotScheduler,
    resampler: Option<MonoResampler>,
    /// 12 kHz audio accumulated for the slot in progress.
    slot_buf: Vec<i16>,
    /// Whether `slot_buf` began at a slot boundary. It does not after a start,
    /// a reset or a period change part-way through a slot, and a buffer that
    /// began mid-slot is not the slot the decoder is told it is — so such a
    /// slot is dropped rather than decoded.
    buf_aligned: bool,
    tap_scratch: Vec<f32>,
    last_slot_idx: i64,
    audio_hz: f32,

    job_tx: Sender<DecodeJob>,
    res_rx: Receiver<DecodeResult>,
    _worker: std::thread::JoinHandle<()>,
    pending: bool,
    last_count: u32,
    status_dirty: bool,
}

impl Q65Controller {
    pub fn new(cfg: DigiConfig, tap_rate: f64) -> Self {
        let mode = cfg.q65_mode;
        let (job_tx, job_rx) = std::sync::mpsc::channel::<DecodeJob>();
        let (res_tx, res_rx) = std::sync::mpsc::channel::<DecodeResult>();
        let worker = std::thread::Builder::new()
            .name("sdroxide-q65-decode".into())
            .spawn(move || {
                while let Ok(job) = job_rx.recv() {
                    let decodes = decode_q65_slot(&job.audio, job.mode, job.slot_utc);
                    if res_tx.send(DecodeResult { decodes }).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn q65 decode worker");

        Q65Controller {
            cfg,
            mode,
            scheduler: SlotScheduler::new(mode.slot_s(), mode.start_delay_s()),
            resampler: MonoResampler::new(tap_rate, DECODE_RATE),
            slot_buf: Vec::new(),
            buf_aligned: false,
            tap_scratch: Vec::new(),
            last_slot_idx: i64::MIN,
            audio_hz: 1000.0,
            job_tx,
            res_rx,
            _worker: worker,
            pending: false,
            last_count: 0,
            status_dirty: true,
        }
    }

    fn slot_samples(&self) -> usize {
        (self.mode.slot_s() * DECODE_RATE) as usize
    }

    fn digi_status(&self) -> DigiStatus {
        let mut s = DigiStatus::idle(self.cfg.clone());
        s.mode = Mode::Q65;
        s.step = QsoStep::Idle;
        s.audio_hz = self.audio_hz;
        s
    }
}

impl DigiEngine for Q65Controller {
    fn mode(&self) -> Mode {
        Mode::Q65
    }

    fn on_rx_audio(&mut self, tap: &[f32]) {
        self.tap_scratch.clear();
        match &mut self.resampler {
            Some(r) => r.push(tap, &mut self.tap_scratch),
            None => self.tap_scratch.extend_from_slice(tap),
        }
        let cap = self.slot_samples() + self.slot_samples() / 8;
        for &s in &self.tap_scratch {
            if self.slot_buf.len() < cap {
                self.slot_buf.push((s.clamp(-1.0, 1.0) * 28_000.0) as i16);
            }
        }
    }

    fn poll(&mut self, now: SystemTime, _dial_hz: f64) -> Vec<DigiAction> {
        let mut actions = Vec::new();

        // Drain the worker.
        loop {
            match self.res_rx.try_recv() {
                Ok(res) => {
                    self.pending = false;
                    self.last_count = res.decodes.len() as u32;
                    self.status_dirty = true;
                    if !res.decodes.is_empty() {
                        actions.push(DigiAction::Decodes(res.decodes));
                    }
                }
                Err(TryRecvError::Empty) => break,
                // The worker is gone and no answer is coming. Release the slot
                // rather than holding `pending` for the rest of the session,
                // which would stop every later slot being dispatched.
                Err(TryRecvError::Disconnected) => {
                    self.pending = false;
                    break;
                }
            }
        }

        let idx = self.scheduler.slot_index(now);
        if idx != self.last_slot_idx {
            // Only a slot whose audio began on its own boundary is decoded, and
            // half a slot of it is the floor: less than that is a stream
            // hiccup, not a transmission.
            let min_samples = (self.mode.slot_s() * DECODE_RATE * 0.5) as usize;
            if self.buf_aligned && self.slot_buf.len() >= min_samples && !self.pending {
                let audio = std::mem::take(&mut self.slot_buf);
                // The slot that just ended is the one before this boundary.
                let slot_utc = self.scheduler.slot_start_unix(idx - 1) as i64;
                self.pending =
                    self.job_tx.send(DecodeJob { audio, mode: self.mode, slot_utc }).is_ok();
            }
            self.slot_buf.clear();
            // The very first poll is not a boundary crossing, only the first
            // look at the clock — part-way through a slot.
            self.buf_aligned = self.last_slot_idx != i64::MIN;
            self.last_slot_idx = idx;
        }
        if self.status_dirty {
            self.status_dirty = false;
            actions.push(DigiAction::Status(self.digi_status()));
        }
        actions
    }

    fn tx_burst_active(&self) -> bool {
        false
    }

    fn fill_tx_block(&mut self, _out: &mut [f32]) -> bool {
        false
    }

    fn on_burst_done(&mut self) {}

    fn abort(&mut self) {
        self.slot_buf.clear();
        self.buf_aligned = false;
        self.status_dirty = true;
    }

    fn abort_tx(&mut self) {}

    fn set_config(&mut self, cfg: DigiConfig) {
        // A sub-mode change moves the slot length and the decode, so the
        // scheduler and the buffer bound follow it. Drop the audio in hand: it
        // belongs to the old geometry.
        if cfg.q65_mode != self.mode {
            self.mode = cfg.q65_mode;
            self.scheduler = SlotScheduler::new(self.mode.slot_s(), self.mode.start_delay_s());
            self.slot_buf.clear();
            self.buf_aligned = false;
            self.last_slot_idx = i64::MIN;
        }
        self.cfg = cfg;
        self.status_dirty = true;
    }

    fn clear_rx(&mut self) {
        self.last_count = 0;
        self.status_dirty = true;
    }

    fn set_audio_hz(&mut self, hz: f32) {
        self.audio_hz = hz.clamp(200.0, 3500.0);
    }

    fn audio_hz(&self) -> f32 {
        self.audio_hz
    }

    fn status(&self) -> DigiStatus {
        self.digi_status()
    }
}
