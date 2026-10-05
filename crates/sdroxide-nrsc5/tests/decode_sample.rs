//! End-to-end decode check against nrsc5's own sample capture.
//!
//! `support/sample.xz` in the nrsc5 repository is upstream's CI fixture: a few
//! seconds of FM HD Radio (KUT), decoded by upstream's CLI to prove a build
//! works (their workflow watches the log for the station text). Piping it
//! through this crate instead proves the whole chain here against the
//! `libnrsc5` the machine has: the library found and loaded, the event layout
//! read correctly out of it, and its HDC audio coming back.
//!
//! Defaults to ignored: it decompresses ~48 MB and decodes in real-ish time
//! per sample, so it is not part of the ordinary `cargo test` run. It needs the
//! capture named by `SDROXIDE_NRSC5_SAMPLE`, the `xz` binary on PATH, and a
//! `libnrsc5` (`SDROXIDE_NRSC5_LIB` names one outside the usual places):
//!
//! ```text
//! SDROXIDE_NRSC5_SAMPLE=~/src/nrsc5/support/sample.xz \
//!     cargo test -p sdroxide-nrsc5 --release -- --ignored --nocapture
//! ```
//!
//! Each skips, saying why, when any of the three is missing.

use std::io::Read;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

use num_complex::Complex32;

use sdroxide_nrsc5::{Event, HdReceiver, Mode};

/// The capture, where there is one and a library to decode it with.
fn sample() -> Option<PathBuf> {
    let Some(sample) = std::env::var_os("SDROXIDE_NRSC5_SAMPLE").map(PathBuf::from) else {
        eprintln!("skipping: set SDROXIDE_NRSC5_SAMPLE to nrsc5's support/sample.xz");
        return None;
    };
    if !sample.exists() {
        eprintln!("skipping: {} does not exist", sample.display());
        return None;
    }
    if let Some(why) = sdroxide_nrsc5::unavailable_reason() {
        eprintln!("skipping: {why}");
        return None;
    }
    Some(sample)
}

#[test]
#[ignore = "decompresses 48 MB and decodes for several seconds"]
fn decode_sample_capture() {
    let Some(sample) = sample() else { return };

    // xz -dc streams the CU8 I/Q directly into the pipe.
    let mut child = match Command::new("xz")
        .args(["-dc", sample.to_str().unwrap()])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
    {
        Ok(c) => c,
        Err(e) => {
            eprintln!("skipping: cannot run xz ({e}); needs the binary on PATH");
            return;
        }
    };
    let mut stdin_off = child.stdout.take().expect("xz stdout");
    let rx = HdReceiver::open(Mode::Fm).expect("open HD Radio receiver");

    let mut buf = vec![0u8; 1 << 16];
    let mut fed = 0u64;
    let mut got = Summary::default();
    loop {
        match stdin_off.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                rx.pipe_cu8(&buf[..n]).expect("pipe samples");
                fed += n as u64;
                // Drain whatever decoded during this chunk so the channel
                // never balloons.
                for ev in rx.drain() {
                    got.absorb(ev);
                }
            }
            Err(e) => panic!("failed reading xz stream: {e}"),
        }
    }
    let status = child.wait().expect("xz exit");
    assert!(status.success(), "xz failed: {status:?}");

    // Let the tail of the capture drain, then tear the receiver down.
    std::thread::sleep(std::time::Duration::from_millis(500));
    for ev in rx.drain() {
        got.absorb(ev);
    }
    drop(rx);

    eprintln!("fed {fed} bytes of CU8 I/Q");
    eprintln!("{got:#?}");

    assert!(got.syncs > 0, "no sync event — the receive path never locked onto the capture");
    assert!(!got.text.is_empty(), "no station name/slogan/message was decoded from the capture");
    assert!(
        got.sounding_frames > 0,
        "no decoded audio — only filled-in silence, so the HDC decoder never produced sound"
    );
    eprintln!("station text: {}", got.text.join(" | "));
}

/// The same capture through [`HdDemod`], the way the receive chain drives it:
/// channel I/Q in blocks, audio out paced to real time, status taken on a
/// timer. That puts the decoder thread, the queue into it and the audio queue
/// out of it under test, which the raw receiver above does not touch.
///
/// Fed as fast as the decoder thread takes it, waiting on `queued_input`
/// between blocks — faster, the demod would drop what its queue cannot hold,
/// as it does on a machine that cannot keep up, and that is a different test.
#[test]
#[ignore = "decompresses 48 MB and decodes for several seconds"]
fn decode_sample_capture_through_the_demod() {
    use sdroxide_dsp::Demodulator;
    use sdroxide_nrsc5::HdDemod;

    /// `NRSC5_SAMPLE_RATE_CU8`: the capture is at twice the decoder's own rate.
    const CU8_RATE: f64 = 1_488_375.0;

    let Some(sample) = sample() else { return };
    let Ok(mut child) = Command::new("xz")
        .args(["-dc", sample.to_str().unwrap()])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
    else {
        eprintln!("skipping: xz is not on PATH");
        return;
    };
    let mut stream = child.stdout.take().expect("xz stdout");

    let mut demod = HdDemod::new(CU8_RATE, Mode::Fm);
    let mut bytes = vec![0u8; 1 << 16];
    let mut iq = Vec::new();
    let mut audio = Vec::new();
    let (mut locked, mut station, mut loud) = (false, String::new(), 0usize);
    let mut absorb = |demod: &mut HdDemod, audio: &[f32]| {
        loud += audio.iter().filter(|a| a.abs() > 1e-3).count();
        if let Some(st) = demod.take_hd_radio() {
            locked |= st.locked;
            if !st.station_name.is_empty() {
                station = st.station_name;
            }
        }
    };
    loop {
        let n = stream.read(&mut bytes).expect("read the xz stream");
        if n == 0 {
            break;
        }
        iq.clear();
        iq.extend(
            bytes[..n & !1].chunks_exact(2).map(|p| {
                Complex32::new((p[0] as f32 - 127.5) / 127.5, (p[1] as f32 - 127.5) / 127.5)
            }),
        );
        audio.clear();
        demod.process(&iq, &mut audio);
        absorb(&mut demod, &audio);
        while demod.queued_input() > iq.len() {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    assert!(child.wait().expect("xz exit").success());
    // The decoder is behind the feed by its own latency; keep the chain
    // running on silence while it finishes, as a receiver would.
    let silence = vec![Complex32::new(0.0, 0.0); 32_768];
    for _ in 0..60 {
        audio.clear();
        demod.process(&silence, &mut audio);
        absorb(&mut demod, &audio);
        std::thread::sleep(Duration::from_millis(20));
    }
    drop(absorb);

    eprintln!("locked {locked}, station {station:?}, {loud} audible samples");
    assert!(locked, "the decoder thread never locked onto the capture");
    assert_eq!(station.trim(), "KUT", "the station name came back through the status");
    assert!(loud > 44_100, "at least a second of decoded audio reached the chain, got {loud}");
    assert_eq!(demod.backlog_drops(), 0, "the audio queue was drained at the rate it filled");
}

/// What the assertions look at, without holding on to the PCM.
#[derive(Debug, Default)]
struct Summary {
    syncs: usize,
    lost_syncs: usize,
    audio_frames: usize,
    /// Frames that were sound rather than filled-in silence.
    sounding_frames: usize,
    audio_samples: usize,
    text: Vec<String>,
}

impl Summary {
    fn absorb(&mut self, ev: Event) {
        match ev {
            Event::Sync { .. } => self.syncs += 1,
            Event::LostSync => self.lost_syncs += 1,
            Event::Audio { data, unavailable, .. } => {
                self.audio_frames += 1;
                self.sounding_frames += usize::from(!unavailable);
                self.audio_samples += data.len();
            }
            Event::StationName(s) | Event::StationSlogan(s) | Event::StationMessage(s) => {
                if self.text.last() != Some(&s) {
                    self.text.push(s);
                }
            }
            _ => {}
        }
    }
}
