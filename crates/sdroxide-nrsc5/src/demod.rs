//! [`HdDemod`] — HD Radio as a demodulator: channel-rate complex baseband in,
//! decoded programme audio out.
//!
//! It sits where `DrmDemod` or `WfmDemod` would, because that is what it is —
//! RF in, sound out, no keyboard, no transmit, no QSO. The differences from an
//! analog demod are the same ones DRM has, and for the same reasons:
//!
//! * the audio for a block is not made from that block's samples. HD Radio's
//!   audio is a block-interleaved HDC stream behind an OFDM frame, so what
//!   comes back now was transmitted a fraction of a second ago;
//! * the decoder produces audio at its own rate (44.1 kHz), not ours, so the
//!   two are rate-matched here rather than assumed equal;
//! * the decoder is a C library, and on a pipe it does all of its work
//!   inside the call that hands it samples. So it runs on a thread of its own
//!   (see `src/worker.rs`): this side queues channel I/Q for it and plays
//!   back what it has decoded, and nothing heavier than a copy happens on the
//!   receive chain's thread.
//!
//! FM only for now. The AM-band variant (HD on AM) needs the decoder opened in
//! its AM mode and fed at 46,511.71875 S/s instead, which the engine would have
//! to choose from the dial frequency; until then this demod serves the FM band,
//! where nearly all HD Radio listening is.

use std::collections::VecDeque;
use std::sync::atomic::Ordering;

use num_complex::Complex32;
use sdroxide_dsp::Demodulator;
use sdroxide_types::HdRadioStatus;
use tracing::{debug, warn};

use crate::Mode;
use crate::worker::HdWorker;

/// FM hybrid HD Radio's native pipe rate, in samples per second.
pub const FM_RATE_HZ: f64 = 744_187.5;

/// HD-on-AM's native pipe rate, in samples per second.
pub const AM_RATE_HZ: f64 = 46_511.71875;

/// Rate of the decoded audio nrsc5 emits, in samples per second.
pub const AUDIO_RATE: f64 = 44_100.0;

/// Decoded audio held back before playback, and the point at which a backlog is
/// dropped rather than allowed to become latency. In steady state the decoder
/// produces almost exactly real time and neither applies; the cap is what keeps
/// a burst after re-acquisition from playing out seconds late.
const MAX_BACKLOG_FRAMES: usize = (0.6 * AUDIO_RATE) as usize;
const TARGET_BACKLOG_FRAMES: usize = (0.3 * AUDIO_RATE) as usize;

/// HD Radio (NRSC-5) as a receive-chain demodulator, FM only.
pub struct HdDemod {
    /// `None` when the decoder could not start — the mode then behaves as a
    /// silent receiver rather than taking the radio down.
    worker: Option<HdWorker>,
    channel_rate: f64,

    /// The side, `(L - R) / 2`, of the block being played, for the stereo
    /// blend. Built by `process`, read once by `take_side`.
    side: Vec<f32>,

    /// Post-filter signal power for the S-meter.
    power: f32,
    /// Fractional part of the input-to-audio sample accounting.
    frame_debt: f64,

    /// Audio frames dropped by the backlog trim, cumulatively. A steady
    /// stream of these is a decoder that is not being drained at the rate it
    /// produces, which is how the old one-value-per-frame pacing bug showed
    /// itself; exposing the count lets a test or the capture harness assert
    /// it stays zero.
    drops: u64,

    /// The status to publish once when there is no decoder to ask.
    idle_status: Option<HdRadioStatus>,
}

impl HdDemod {
    /// Build an HD Radio demodulator for a chain whose channel rate is
    /// `channel_rate`, decoding the analogue source `mode` — the FM hybrid, or
    /// HD on AM (the caller chooses from the dial; see
    /// `sdroxide_dsp::hd_radio_is_am`).
    pub fn new(channel_rate: f64, mode: Mode) -> Self {
        let mut unavailable = None;
        // The machine before the stream: no rate helps without the library.
        let worker = if let Some(why) = crate::unavailable_reason() {
            unavailable = Some(why.to_string());
            None
        } else if crate::worker::library_cannot_play() {
            unavailable = Some(crate::worker::NO_AUDIO_DECODER_WHY.to_string());
            None
        } else if channel_rate < mode.min_channel_rate_hz() {
            let why = format!(
                "HD Radio ({}) needs at least {:.0} kHz of stream, and this one is {:.1} kHz — \
                 raise the device sample rate, or use a receiver that hands over I/Q rather than \
                 demodulated audio",
                match mode {
                    Mode::Fm => "the FM hybrid",
                    Mode::Am => "the AM-band variant",
                },
                mode.min_channel_rate_hz() / 1e3,
                channel_rate / 1e3
            );
            warn!("{why}");
            unavailable = Some(why);
            None
        } else {
            match HdWorker::new(channel_rate, mode) {
                Ok(w) => {
                    debug!(channel_rate, "HD Radio decoder attached to the receive chain");
                    Some(w)
                }
                Err(e) => {
                    warn!(?e, "could not start the HD Radio decoder; the mode will be silent");
                    unavailable = Some("the HD Radio decoder could not be started".to_string());
                    None
                }
            }
        };
        let idle_status =
            worker.is_none().then(|| HdRadioStatus { unavailable, ..Default::default() });
        HdDemod {
            worker,
            channel_rate,
            side: Vec::new(),
            power: 0.0,
            frame_debt: 0.0,
            drops: 0,
            idle_status,
        }
    }

    /// Re-acquire from scratch. The demod cannot see a retune — the DDC ahead of
    /// it absorbs that — so the engine says.
    ///
    /// The programme goes back to HD-1. The one selected belonged to the station
    /// tuned away from: kept, it would leave a single-programme station silent
    /// behind lights that stay off, and a click on HD-2 would then do nothing
    /// because HD-2 was already "selected". Reset here rather than on the
    /// decoder thread, so a programme picked straight after the retune is not
    /// undone when the thread gets round to restarting.
    pub fn restart(&mut self) {
        self.frame_debt = 0.0;
        let Some(w) = self.worker.as_ref() else { return };
        {
            let mut audio = w.shared.audio();
            audio.selected = 0;
            audio.frames.clear();
        }
        w.shared.status().program = 0;
        w.shared.status_dirty.store(true, Ordering::Relaxed);
        w.restart();
    }

    /// Listen to a different programme of the multiplex, 0-based. Audio for the
    /// previous one is dropped rather than mixed in.
    ///
    /// A programme the multiplex has not announced is ignored rather than
    /// clamped (see `Command::SetHdProgram`): a stale click from a client that
    /// has not seen the multiplex change should do nothing rather than land
    /// somewhere else. HD-1 is always there.
    pub fn select_program(&mut self, program: u8) {
        let Some(w) = self.worker.as_ref() else { return };
        if !program_announced(&w.shared.status(), program) {
            debug!(program, "ignored a programme the HD Radio multiplex does not carry");
            return;
        }
        {
            let mut audio = w.shared.audio();
            if program == audio.selected {
                return;
            }
            audio.selected = program;
            audio.frames.clear();
        }
        w.shared.status().program = program;
        w.shared.status_dirty.store(true, Ordering::Relaxed);
    }

    /// Channel samples handed to [`Demodulator::process`] that the decoder
    /// thread has not taken yet — zero when no decoder is running.
    ///
    /// A receiver never needs this: it hands over I/Q in real time, and a
    /// decoder that cannot keep up loses the oldest rather than stalling it.
    /// Something feeding a recording can go far faster than real time, and
    /// waiting on this between blocks is how it keeps the queue from
    /// overflowing — see `examples/hd_capture.rs`.
    pub fn queued_input(&self) -> usize {
        self.worker.as_ref().map_or(0, HdWorker::queued)
    }

    /// Audio frames dropped by the backlog trim since construction.
    ///
    /// Zero on a healthy decode at the right rate. A climbing count means the
    /// queue is not being drained as fast as the decoder fills it — the shape
    /// the old one-value-per-frame pacing bug took — so a capture test or the
    /// harness can assert this stays at zero.
    pub fn backlog_drops(&self) -> u64 {
        self.drops
    }
}

/// Whether `program` is one the multiplex carries: HD-1, or a programme its
/// station information has announced.
fn program_announced(status: &HdRadioStatus, program: u8) -> bool {
    program == 0 || status.audio_services.iter().any(|s| s.program == program)
}

/// Move `want` audio frames from the interleaved stereo queue into `out` (as
/// mono, `(L + R) / 2`) and `side` (`(L - R) / 2`), padding with silence for
/// frames the queue cannot supply.
///
/// Free rather than a method so the pairing can be tested without a decoder.
fn emit_frames(audio: &mut VecDeque<i16>, side: &mut Vec<f32>, want: usize, out: &mut Vec<f32>) {
    let available = audio.len() / 2;
    let take = want.min(available);
    out.reserve(want);
    side.reserve(want);
    for _ in 0..take {
        let l = audio.pop_front().unwrap_or(0) as f32 / 32_768.0;
        let r = audio.pop_front().unwrap_or(0) as f32 / 32_768.0;
        out.push((l + r) * 0.5);
        side.push((l - r) * 0.5);
    }
    // Whatever the queue could not supply is silence, not a gap: the block
    // still has to be as long as real time says.
    for _ in take..want {
        out.push(0.0);
        side.push(0.0);
    }
}

impl Demodulator for HdDemod {
    fn process(&mut self, iq: &[Complex32], out: &mut Vec<f32>) {
        self.side.clear();
        if iq.is_empty() {
            return;
        }

        let mut p = 0.0f32;
        for z in iq {
            p += z.re * z.re + z.im * z.im;
        }
        self.power = p / iq.len() as f32;

        let Some(worker) = self.worker.as_mut() else {
            return;
        };
        worker.push(iq);

        // How much audio this block is worth in real time. The decoder's output
        // is paced by the broadcast, so this keeps the two clocks together
        // instead of playing back whatever happens to have arrived.
        self.frame_debt += iq.len() as f64 * AUDIO_RATE / self.channel_rate;
        let want = self.frame_debt as usize;
        self.frame_debt -= want as f64;
        if want == 0 {
            return;
        }

        let mut audio = worker.shared.audio();
        // A backlog is latency; drop the oldest audio rather than play it out
        // late. It only builds if the decoder catches up in a burst after
        // acquiring. The queue holds interleaved stereo pairs, so it is two
        // values per frame and the caps are in frames.
        if audio.frames.len() > MAX_BACKLOG_FRAMES * 2 {
            let drop = (audio.frames.len() / 2 - TARGET_BACKLOG_FRAMES) * 2;
            audio.frames.drain(..drop);
            self.drops += (drop / 2) as u64;
            debug!(frames = drop / 2, "dropped an HD Radio audio backlog");
        }

        // nrsc5 hands over interleaved stereo pairs, two `i16` per frame, so a
        // frame is taken as a pair: the sum for the mono output the receive
        // chain plays, the difference for the side `take_side` offers the
        // stereo blend. Popping one value per frame (the old behaviour) halved
        // the rate, played L and R alternately, and grew the queue until the
        // backlog trim fired several times a second.
        emit_frames(&mut audio.frames, &mut self.side, want, out);
    }

    /// Nothing to do: the HD Radio channel's width is fixed by the FM hybrid,
    /// which the decoder reads for itself. The operator's filter edges still set
    /// what the panadapter draws and what the S-meter measures.
    fn set_filter(&mut self, _lo_hz: f32, _hi_hz: f32) {}

    fn audio_rate(&self) -> f64 {
        AUDIO_RATE
    }

    fn power_dbfs(&self) -> f32 {
        if self.power <= 1e-20 { -200.0 } else { 10.0 * self.power.log10() }
    }

    /// The side channel of the block just decoded, for the stereo blend. HDC
    /// audio is stereo, so this is offered whenever there is anything to offer;
    /// a mono transmission has `L == R` and a zero side, which sums back to the
    /// same mono signal.
    fn take_side(&mut self, out: &mut Vec<f32>) -> bool {
        if self.side.is_empty() {
            return false;
        }
        out.extend_from_slice(&self.side);
        true
    }

    fn stereo_locked(&self) -> bool {
        self.worker.as_ref().is_some_and(|w| w.shared.status().audio)
    }

    fn reset_hd_radio(&mut self) {
        self.restart();
    }

    fn set_hd_program(&mut self, program: u8) {
        self.select_program(program);
    }

    fn take_hd_radio(&mut self) -> Option<HdRadioStatus> {
        let Some(w) = self.worker.as_ref() else {
            return self.idle_status.take();
        };
        if !w.shared.status_dirty.swap(false, Ordering::Relaxed) {
            return None;
        }
        Some(w.shared.status().clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One output sample per stereo *frame*, not per queued value: nrsc5 hands
    /// over two `i16` per frame, and consuming one per frame halved the rate
    /// and played L and R alternately (the on-air bug).
    #[test]
    fn a_frame_is_a_stereo_pair() {
        let mut audio: VecDeque<i16> = VecDeque::new();
        audio.extend([-32768i16, -32768, 16384, -16384]);
        let mut side = Vec::new();
        let mut out = Vec::new();
        emit_frames(&mut audio, &mut side, 2, &mut out);
        assert_eq!(out.len(), 2, "one sample per frame, not per value");
        assert_eq!(side.len(), 2);
        assert!((out[0] + 1.0).abs() < 1e-6, "mono is (L+R)/2: {}", out[0]);
        assert!(side[0].abs() < 1e-6, "equal channels carry no side: {}", side[0]);
        assert!(out[1].abs() < 1e-6);
        assert!((side[1] - 0.5).abs() < 1e-6, "side is (L-R)/2: {}", side[1]);
        assert!(audio.is_empty(), "both frames consumed, four values");
    }

    /// The two variants carry their own native pipe rate and their own minimum
    /// channel width — the FM hybrid's ±198 kHz against the AM variant's
    /// ±15 kHz (issue #489).
    #[test]
    fn each_variant_sets_its_own_rate_and_minimum() {
        assert_eq!(Mode::Fm.native_rate_hz(), FM_RATE_HZ);
        assert_eq!(Mode::Am.native_rate_hz(), AM_RATE_HZ);
        assert!(Mode::Am.min_channel_rate_hz() < Mode::Fm.min_channel_rate_hz());
        // The AM floor clears the ~±15 kHz the variant occupies; the FM floor
        // is the hybrid's ±198 kHz sidebands.
        assert!(Mode::Am.min_channel_rate_hz() >= 30_000.0);
        assert!(Mode::Fm.min_channel_rate_hz() >= 396_800.0);
    }

    /// A stream too narrow for the digital sidebands starts no decoder, and the
    /// status says why, naming the rate — unless there is no library at all,
    /// which no rate would fix and so is said first.
    #[test]
    fn a_channel_too_narrow_for_the_sidebands_says_so() {
        let mut demod = HdDemod::new(48_000.0, Mode::Fm);
        let status = demod.take_hd_radio().expect("the reason is published");
        let why = status.unavailable.expect("unavailable");
        match crate::unavailable_reason() {
            Some(missing) => assert_eq!(why, missing),
            None => assert!(why.contains("48.0 kHz"), "{why}"),
        }
        assert!(demod.take_hd_radio().is_none(), "once, not every poll");
        let mut out = Vec::new();
        demod.process(&[Complex32::new(0.1, 0.0); 480], &mut out);
        assert!(out.is_empty());
    }

    /// HD-1 is always selectable; anything else only once the station has
    /// announced it.
    #[test]
    fn only_an_announced_programme_is_selectable() {
        let mut status = HdRadioStatus::default();
        assert!(program_announced(&status, 0));
        assert!(!program_announced(&status, 1), "nothing announced yet");
        status.audio_services.push(sdroxide_types::HdAudioService {
            program: 1,
            access: 0,
            codec_mode: 0,
        });
        assert!(program_announced(&status, 1));
        assert!(!program_announced(&status, 2), "HD-3 was never announced");
    }

    /// A block is always as long as real time says, whether or not the decoder
    /// has caught up.
    #[test]
    fn a_short_queue_is_padded_with_silence() {
        let mut audio: VecDeque<i16> = VecDeque::new();
        audio.extend([16384i16, 16384]);
        let mut side = Vec::new();
        let mut out = Vec::new();
        emit_frames(&mut audio, &mut side, 3, &mut out);
        assert_eq!(out.len(), 3);
        assert_eq!(side.len(), 3);
        assert_eq!(out[1], 0.0);
        assert_eq!(out[2], 0.0);
        assert!(audio.is_empty());
    }
}
