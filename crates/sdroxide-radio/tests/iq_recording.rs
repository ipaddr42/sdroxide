//! Recording the raw spectrum, and reading it back (issue #217).
//!
//! The whole of what the feature promises in one test: the operator presses
//! REC, the engine writes a file, and that file plays back — at the rate and on
//! the frequency it was made, with nothing typed at it. A capture that cannot
//! be re-opened is not a capture, and the failure mode is silent (a header
//! whose sizes were never patched reads as an empty recording), so it is worth
//! checking on the file rather than on the counters.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, RadioEvent};

const RATE: f64 = 240_000.0;
const CENTER: f64 = 145_500_000.0;

/// A ramp, so a sample can be recognised in the file it lands in.
struct Ramp {
    n: Arc<AtomicU64>,
}

impl IqSource for Ramp {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        CENTER
    }
    fn set_center_hz(&mut self, _hz: f64) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let take = buf.len().min(4096);
        let base = self.n.fetch_add(take as u64, Ordering::Relaxed);
        for (i, s) in buf[..take].iter_mut().enumerate() {
            let k = (base + i as u64) % 1000;
            *s = Complex32::new(k as f32 / 1000.0, -(k as f32) / 2000.0);
        }
        // Paced, or the engine consumes as fast as it can and a two-second
        // recording is a gigabyte.
        std::thread::sleep(Duration::from_millis(4));
        Ok(take)
    }
    fn describe(&self) -> String {
        "ramp".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "test".into(),
        label: "test".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 2_000_000_000.0)],
        ..DeviceCaps::default()
    }
}

#[test]
fn a_capture_is_written_and_plays_back_at_the_rate_it_was_made() {
    // Its own config dir: the engine puts captures in the operator's audio
    // folder when there is one, and a test must not write there.
    let dir = std::env::temp_dir().join(format!("sdroxide-iqrec-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &dir) };
    unsafe { std::env::set_var("XDG_MUSIC_DIR", dir.join("music")) };

    let counted = Arc::new(AtomicU64::new(0));
    let src = Ramp { n: Arc::clone(&counted) };
    let mut h = start_engine(Box::new(src), caps(), EngineConfig::default());
    let thread = h.thread.take();

    h.cmd_tx.send(Command::SetIqRecording(true)).unwrap();
    // The engine answers with a state carrying the file name.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut name = None;
    while Instant::now() < deadline && name.is_none() {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::State(s) = ev
                && s.iq_recording
            {
                name = s.iq_recording_file.clone();
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let name = name.expect("the engine never reported a capture starting");
    assert!(name.ends_with("_IQ.wav"), "{name} is not named like a capture");
    assert!(name.contains("145500000Hz"), "{name} does not carry the frequency");

    std::thread::sleep(Duration::from_millis(600));
    h.cmd_tx.send(Command::SetIqRecording(false)).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    drop(h);
    let _ = thread.map(|t| t.join());

    // Find it wherever `recordings_dir` put it.
    let path = find(&dir, &name).unwrap_or_else(|| panic!("{name} was not written under {dir:?}"));
    let len = std::fs::metadata(&path).unwrap().len();
    assert!(len > 1000, "the capture is {len} bytes — the header and nothing else");

    // …and it reads back as what it is, with no help.
    let info = sdroxide_radio::iq_wav::probe(&path).expect("a capture we just wrote");
    assert_eq!(info.rate_hz, RATE, "the rate has to survive the round trip");
    assert_eq!(info.center_hz, Some(CENTER));
    assert_eq!(
        info.data_start + info.data_len,
        len,
        "the data chunk's size was never patched in, so a player sees an empty file"
    );

    // Every sample is one of ours: the ramp, in order, I and Q as they were.
    let raw = std::fs::read(&path).unwrap();
    let body = &raw[info.data_start as usize..];
    for (n, chunk) in body.chunks_exact(8).enumerate().take(2000) {
        let i = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let q = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        assert!((q + i / 2.0).abs() < 1e-6, "frame {n} is not one of the source's: {i}, {q}");
    }

    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
    let _ = std::fs::remove_dir_all(&dir);
}

fn find(dir: &std::path::Path, name: &str) -> Option<std::path::PathBuf> {
    for e in std::fs::read_dir(dir).ok()? {
        let p = e.ok()?.path();
        if p.is_dir() {
            if let Some(f) = find(&p, name) {
                return Some(f);
            }
        } else if p.file_name().and_then(|s| s.to_str()) == Some(name) {
            return Some(p);
        }
    }
    None
}
