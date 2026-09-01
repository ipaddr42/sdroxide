//! Binaural CW through the whole engine (issue #263): I/Q in at one end, a
//! stereo pair out of the mixer at the other.
//!
//! The widener itself is held to its own arithmetic in `sdroxide-dsp`, and the
//! rules about when it runs in the engine's unit tests. What neither of those
//! can see is the wiring between them — that the mode gate is asked about the
//! right receiver, that the passband and the speaker's rate reach the filter,
//! and above all that the ears come out in the order the operator's head
//! expects. So this drives the real engine with a real note and reads the two
//! channels out of the audio ring.

use std::time::Duration;

use sdroxide_radio::{AudioParams, Complex32, EngineConfig, IqSource, Result, rtrb, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RxId};

const RATE: f64 = 48_000.0;
const CENTER: f64 = 14_050_000.0;
/// Where the note sits: 900 Hz above the dial, which in CW is 200 Hz above the
/// 700 Hz sidetone pitch the passband is centred on — well inside the filter
/// and well off centre, so it has a side of the head to be on.
const TONE_HZ: f64 = 900.0;
/// A whole number of cycles of the note at the speaker's rate (900 Hz is 3
/// cycles per 160 samples), so the bin the ears are read out of is exact.
const WINDOW: usize = 160 * 30;

/// A steady carrier `TONE_HZ` above the dial: a key held down, which is the
/// signal a CW operator is placing.
struct Note {
    center: f64,
    phase: f64,
}

impl IqSource for Note {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        // Paced like a front end so the engine loop is not a spin.
        std::thread::sleep(Duration::from_millis(10));
        let n = buf.len().min(480);
        let step = std::f64::consts::TAU * TONE_HZ / RATE;
        for s in buf[..n].iter_mut() {
            self.phase += step;
            *s = Complex32::new(0.05 * self.phase.cos() as f32, 0.05 * self.phase.sin() as f32);
        }
        self.phase = self.phase.rem_euclid(std::f64::consts::TAU);
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock CW note".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(10_000.0, 30_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run the engine in CW for a couple of seconds and hand back the last
/// `WINDOW` stereo frames the mixer produced.
fn run(cmds: &[Command]) -> Vec<(f32, f32)> {
    let (producer, mut consumer) = rtrb::RingBuffer::<f32>::new(48_000 * 4);
    let mut h = start_engine(
        Box::new(Note { center: CENTER, phase: 0.0 }),
        caps(),
        EngineConfig {
            audio: Some(AudioParams { producer, out_rate: RATE }),
            initial_mode: Some(Mode::Cw),
            ..Default::default()
        },
    );
    let thread = h.thread.take();
    for c in cmds {
        h.cmd_tx.send(c.clone()).unwrap();
    }

    // Long enough for the AGC to settle, so what is read out is a steady note
    // rather than the front of a ramp. Whole frames only: the mixer never
    // pushes half of one, so left and right cannot slip past each other.
    let mut frames: Vec<(f32, f32)> = Vec::new();
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    while std::time::Instant::now() < deadline {
        while let Ok(l) = consumer.pop() {
            match consumer.pop() {
                Ok(r) => frames.push((l, r)),
                Err(_) => break,
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    drop(h);
    if let Some(t) = thread {
        let _ = t.join();
    }
    assert!(frames.len() > WINDOW * 2, "only {} frames reached the speaker", frames.len());
    frames.split_off(frames.len() - WINDOW)
}

/// Amplitude and phase of one channel at the note's frequency.
fn bin(frames: &[(f32, f32)], right: bool) -> num_complex::Complex<f64> {
    let w = std::f64::consts::TAU * TONE_HZ / RATE;
    let s: num_complex::Complex<f64> = frames
        .iter()
        .enumerate()
        .map(|(n, f)| {
            let v = if right { f.1 } else { f.0 };
            f64::from(v) * num_complex::Complex::from_polar(1.0, -w * n as f64)
        })
        .sum();
    s * 2.0 / frames.len() as f64
}

fn rms(x: impl Iterator<Item = f32>) -> f32 {
    let (sum, n) = x.fold((0.0f64, 0usize), |(s, n), v| (s + f64::from(v) * f64::from(v), n + 1));
    (sum / n.max(1) as f64).sqrt() as f32
}

/// With it switched off, the two ears are the same samples — which is what
/// every other mode and every previous version does, and the thing the rest of
/// this file is measured against.
#[test]
fn without_it_the_two_ears_are_identical() {
    let frames = run(&[]);
    assert!(rms(frames.iter().map(|f| f.0)) > 1e-3, "the note never reached the speaker");
    assert!(
        frames.iter().all(|(l, r)| l == r),
        "the ears differ with binaural off — something else is filling the right one"
    );
}

/// With it on, a note 200 Hz above the sidetone pitch is placed to the *right*:
/// the ear it reaches first, and the louder of the two. Both cues, because
/// getting either one backwards would put the image on the wrong side of the
/// head for half the operators and be inaudible to the rest.
#[test]
fn a_note_above_the_pitch_is_placed_to_the_right() {
    let frames = run(&[Command::SetBinaural { rx: RxId::Main, on: true }]);
    let (l, r) = (bin(&frames, false), bin(&frames, true));
    assert!(l.norm() > 1e-3 && r.norm() > 1e-3, "the note never reached the speaker");

    let ild = 20.0 * (r.norm() / l.norm()).log10();
    assert!(ild > 3.0, "the right ear should be the louder, got {ild} dB");

    // Positive when the right ear leads, which is what places the image there.
    let mut ipd = (r.arg() - l.arg()).to_degrees();
    if ipd > 180.0 {
        ipd -= 360.0;
    } else if ipd < -180.0 {
        ipd += 360.0;
    }
    assert!((45.0..135.0).contains(&ipd), "the right ear should lead by ~84°, got {ipd}°");
}

/// …and the mono downmix is untouched: the same note at the same level as with
/// the widener off. This is what a remote client is sent and what the recorder
/// writes, so it has to survive the effect exactly.
#[test]
fn the_downmix_is_what_it_always_was() {
    let plain = run(&[]);
    let wide = run(&[Command::SetBinaural { rx: RxId::Main, on: true }]);
    let mono = rms(plain.iter().map(|f| f.0));
    let sum = rms(wide.iter().map(|f| 0.5 * (f.0 + f.1)));
    let db = 20.0 * f64::from(sum / mono).log10();
    assert!(db.abs() < 1.0, "the downmix moved by {db} dB");
}

/// SSB is spread too, and the image follows the *filter* rather than the mode:
/// the same 900 Hz note that sits above CW's 700 Hz sidetone pitch sits 600 Hz
/// *below* the centre of a 150–2850 Hz voice passband, so it comes out of the
/// other ear. Nothing in the widener knows which mode it is in — this is what
/// that means from the outside.
#[test]
fn the_same_note_swaps_ears_in_ssb() {
    let frames = run(&[
        Command::SetMode { rx: RxId::Main, mode: Mode::Usb },
        Command::SetBinaural { rx: RxId::Main, on: true },
    ]);
    let (l, r) = (bin(&frames, false), bin(&frames, true));
    assert!(l.norm() > 1e-3 && r.norm() > 1e-3, "the note never reached the speaker");

    let ild = 20.0 * (l.norm() / r.norm()).log10();
    assert!(ild > 2.0, "the left ear should be the louder in SSB, got {ild} dB");

    let mut ipd = (l.arg() - r.arg()).to_degrees();
    if ipd > 180.0 {
        ipd -= 360.0;
    } else if ipd < -180.0 {
        ipd += 360.0;
    }
    assert!((20.0..120.0).contains(&ipd), "the left ear should lead in SSB, got {ipd}°");
}

/// The mode gate, through the engine: a mode the chip is not drawn in ignores
/// the request even though a stale setting or a remote client can still make
/// it. DIGU is the honest case to check — same demodulator as SSB, same audio
/// in the passband, and no reason to spread a decoder's input across a head.
#[test]
fn a_mode_without_it_ignores_the_request() {
    let frames = run(&[
        Command::SetMode { rx: RxId::Main, mode: Mode::Digu },
        Command::SetBinaural { rx: RxId::Main, on: true },
    ]);
    assert!(rms(frames.iter().map(|f| f.0)) > 1e-3, "the note never reached the speaker");
    assert!(frames.iter().all(|(l, r)| l == r), "DIGU was widened");
}
