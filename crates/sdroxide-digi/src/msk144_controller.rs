//! `Msk144Controller` — MSK144, the meteor-scatter mode, receive only.
//!
//! Slotted like FT8 and shaped like [`crate::WsprController`], but the slot is
//! hunted rather than read at a fixed offset: an operator transmits through the
//! whole 15-second period, so the decoder slides a window across it looking for
//! the short bursts a meteor trail leaves. What comes back is an ordinary
//! [`Decode`] (`<to> <from> <grid|report>`), so the panel is the decode list
//! the FT8 modes already share. There is no QSO sequencer and, in this build,
//! no transmit — a meteor-scatter contact is a timed handshake and transmit is
//! not wired yet (see [`Mode::is_rx_only`]).
//!
//! # Why a worker thread
//!
//! A full-slot MSK144 scan at the deepest search depth is far from free. The
//! engine polls the controller on the audio thread, so the decode runs on its
//! own thread and the result is drained from [`Self::poll`] — the same shape
//! [`crate::WsprController`] uses, and for the same reason. One slot in flight
//! at a time: a machine that cannot keep up must drop a slot rather than fall a
//! slot further behind every fifteen seconds.

use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::SystemTime;

use sdroxide_dsp::MonoResampler;
use sdroxide_types::{Decode, DigiConfig, DigiStatus, Mode, QsoStep};

use crate::DigiEngine;
use crate::controller::DigiAction;
use crate::modem::decode_msk144_slot;
use crate::params::{DECODE_RATE, DigiParams};
use crate::scheduler::SlotScheduler;

/// One slot of audio handed to the decode worker.
struct DecodeJob {
    audio: Vec<i16>,
    /// The audio cursor when the slot ended: the centre the decoder searches.
    audio_hz: f32,
    slot_utc: i64,
}

/// What the worker sends back.
struct DecodeResult {
    decodes: Vec<Decode>,
}

pub struct Msk144Controller {
    params: DigiParams,
    scheduler: SlotScheduler,
    cfg: DigiConfig,
    resampler: Option<MonoResampler>,
    /// 12 kHz audio accumulated for the slot in progress.
    slot_buf: Vec<i16>,
    /// Whether `slot_buf` began at a slot boundary. It does not after a start,
    /// a reset or a config change part-way through a slot, and a buffer that
    /// began mid-slot puts every burst at the wrong time into it — so such a
    /// slot is dropped rather than decoded.
    buf_aligned: bool,
    tap_scratch: Vec<f32>,
    last_slot_idx: i64,
    /// Where the operator's audio cursor sits: the centre the decoder searches
    /// ([`crate::modem::MSK144_TOLERANCE_HZ`] either side), the readout and the
    /// passband marker.
    audio_hz: f32,

    job_tx: Sender<DecodeJob>,
    res_rx: Receiver<DecodeResult>,
    _worker: std::thread::JoinHandle<()>,
    /// A slot dispatched but not yet answered. One at a time: a machine that
    /// cannot keep up drops a slot rather than queueing them.
    pending: bool,

    /// Decodes from the last completed slot, and how many, for the status.
    last_count: u32,
    status_dirty: bool,
}

impl Msk144Controller {
    pub fn new(cfg: DigiConfig, tap_rate: f64) -> Self {
        let mode = Mode::Msk144;
        let params = DigiParams::for_mode(mode);
        let (job_tx, job_rx) = std::sync::mpsc::channel::<DecodeJob>();
        let (res_tx, res_rx) = std::sync::mpsc::channel::<DecodeResult>();
        let worker = std::thread::Builder::new()
            .name("sdroxide-msk144-decode".into())
            .spawn(move || {
                while let Ok(job) = job_rx.recv() {
                    let decodes = decode_msk144_slot(&job.audio, job.audio_hz, job.slot_utc);
                    if res_tx.send(DecodeResult { decodes }).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn msk144 decode worker");

        Msk144Controller {
            params,
            scheduler: SlotScheduler::for_mode(mode),
            cfg,
            resampler: MonoResampler::new(tap_rate, DECODE_RATE),
            slot_buf: Vec::new(),
            buf_aligned: false,
            tap_scratch: Vec::new(),
            last_slot_idx: i64::MIN,
            // WSJT-X's own MSK144 working frequency: 1500 Hz up from the dial.
            audio_hz: 1500.0,
            job_tx,
            res_rx,
            _worker: worker,
            pending: false,
            last_count: 0,
            status_dirty: true,
        }
    }

    fn slot_samples(&self) -> usize {
        self.params.slot_samples()
    }

    fn digi_status(&self) -> DigiStatus {
        let mut s = DigiStatus::idle(self.cfg.clone());
        s.mode = Mode::Msk144;
        s.step = QsoStep::Idle;
        s.audio_hz = self.audio_hz;
        s
    }
}

impl DigiEngine for Msk144Controller {
    fn mode(&self) -> Mode {
        Mode::Msk144
    }

    fn on_rx_audio(&mut self, tap: &[f32]) {
        self.tap_scratch.clear();
        match &mut self.resampler {
            Some(r) => r.push(tap, &mut self.tap_scratch),
            None => self.tap_scratch.extend_from_slice(tap),
        }
        // The buffer is the same i16 the slotted FT8 path carries, so mfsk-core
        // and the panel see one scale; `decode_msk144_slot` reads it directly.
        let cap = self.slot_samples() + self.slot_samples() / 8;
        for &s in &self.tap_scratch {
            if self.slot_buf.len() < cap {
                self.slot_buf.push((s.clamp(-1.0, 1.0) * 28_000.0) as i16);
            }
        }
    }

    fn poll(&mut self, now: SystemTime, _dial_hz: f64) -> Vec<DigiAction> {
        let mut actions = Vec::new();

        // 1. Drain the worker.
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

        // 2. Slot boundary: dispatch the completed slot to the worker.
        let idx = self.scheduler.slot_index(now);
        if idx != self.last_slot_idx {
            // Only a slot whose audio began on its own boundary is decoded, and
            // half a slot of it is the floor: less than that is a stream
            // hiccup, not a transmission.
            let min_samples = (self.params.slot_s * DECODE_RATE * 0.5) as usize;
            if self.buf_aligned && self.slot_buf.len() >= min_samples && !self.pending {
                let audio = std::mem::take(&mut self.slot_buf);
                // The slot that just ended is the one before this boundary.
                let slot_utc = self.scheduler.slot_start_unix(idx - 1) as i64;
                let job = DecodeJob { audio, audio_hz: self.audio_hz, slot_utc };
                self.pending = self.job_tx.send(job).is_ok();
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
        self.cfg = cfg;
        self.status_dirty = true;
    }

    fn clear_rx(&mut self) {
        self.last_count = 0;
        self.status_dirty = true;
    }

    fn set_audio_hz(&mut self, hz: f32) {
        self.audio_hz = hz.clamp(200.0, 3500.0);
        self.status_dirty = true;
    }

    fn audio_hz(&self) -> f32 {
        self.audio_hz
    }

    fn status(&self) -> DigiStatus {
        self.digi_status()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, UNIX_EPOCH};

    /// A slot is decoded only when its audio began on the slot's own boundary.
    /// Starting part-way through a period must not hand the decoder a buffer
    /// whose start is not the period's start — every burst in it would carry
    /// the wrong time into the slot.
    #[test]
    fn only_a_slot_that_began_on_its_boundary_is_decoded() {
        let mut c = Msk144Controller::new(DigiConfig::default(), DECODE_RATE);
        let at = |s: f64| UNIX_EPOCH + Duration::from_secs_f64(1_800_000_000.0 + s);
        let second = vec![0.0f32; DECODE_RATE as usize];

        // Start 5 s into a period and hear the rest of it.
        c.poll(at(5.0), 0.0);
        for _ in 0..10 {
            c.on_rx_audio(&second);
        }
        c.poll(at(15.0), 0.0);
        assert!(!c.pending, "the partial first period was dispatched");

        // The next period is heard from its boundary, so it is decoded.
        for _ in 0..15 {
            c.on_rx_audio(&second);
        }
        c.poll(at(30.0), 0.0);
        assert!(c.pending, "a whole, aligned period was not dispatched");
    }
}
