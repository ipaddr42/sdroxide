//! An over that went out with nothing on the microphone says so (issue #215).
//!
//! The report it answers: TUNE makes full power, FT8 makes full power, SSB
//! makes milliwatts, and the drive slider is at 100. Every control reads
//! correctly because every control *is* correct — the modulator was handed
//! silence, which is what a microphone that opened on the wrong sound card
//! looks like from the operator's side, and nothing anywhere said so.

use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RxId};

const RATE: f64 = 384_000.0;
const CENTER: f64 = 14_200_000.0;

struct Quiet;

impl IqSource for Quiet {
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
        let n = buf.len().min(4096);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        std::thread::sleep(Duration::from_millis(8));
        Ok(n)
    }
    fn tx_begin(&mut self, _center_hz: f64, rate: f64) -> Result<f64> {
        Ok(rate)
    }
    fn tx_write(&mut self, _iq: &[Complex32]) -> Result<()> {
        std::thread::sleep(Duration::from_millis(8));
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
    fn describe(&self) -> String {
        "quiet".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "test".into(),
        label: "test".into(),
        rx_channels: 1,
        tx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 60_000_000.0)],
        freq_ranges_tx: vec![(1_000_000.0, 60_000_000.0)],
        ..DeviceCaps::default()
    }
}

#[test]
fn a_voice_over_with_no_microphone_is_reported() {
    let dir = std::env::temp_dir().join(format!("sdroxide-silentmic-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &dir) };

    // No microphone at all — `EngineConfig::default()` opens none, which is
    // the same silence a microphone on the wrong card delivers.
    let mut h = start_engine(Box::new(Quiet), caps(), EngineConfig::default());
    let thread = h.thread.take();
    h.cmd_tx.send(Command::SetMode { rx: RxId::Main, mode: Mode::Usb }).unwrap();
    h.cmd_tx.send(Command::SetPtt(true)).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    h.cmd_tx.send(Command::SetPtt(false)).unwrap();

    let deadline = Instant::now() + Duration::from_secs(3);
    let mut said = None;
    while Instant::now() < deadline && said.is_none() {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::Notice(Some(text)) = ev
                && text.contains("no audio")
            {
                said = Some(text);
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    drop(h);
    let _ = thread.map(|t| t.join());
    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
    let _ = std::fs::remove_dir_all(&dir);

    let said = said.expect(
        "an over went out with a silent modulator and nothing was said about it — which is \
         exactly the report",
    );
    assert!(
        said.contains("Settings"),
        "the notice has to say where to go and not only that something is wrong: {said}"
    );
}

/// …and a tune does not get the notice. It is a carrier by definition and has
/// no microphone behind it, so complaining would be noise on every tune-up.
#[test]
fn a_tune_is_not_a_silent_over() {
    let dir = std::env::temp_dir().join(format!("sdroxide-silentmic-t-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &dir) };

    let mut h = start_engine(Box::new(Quiet), caps(), EngineConfig::default());
    let thread = h.thread.take();
    h.cmd_tx.send(Command::SetMode { rx: RxId::Main, mode: Mode::Usb }).unwrap();
    h.cmd_tx.send(Command::SetTune(true)).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    h.cmd_tx.send(Command::SetTune(false)).unwrap();
    std::thread::sleep(Duration::from_millis(400));

    let mut said = None;
    while let Ok(ev) = h.event_rx.try_recv() {
        if let RadioEvent::Notice(Some(text)) = ev
            && text.contains("no audio")
        {
            said = Some(text);
        }
    }
    drop(h);
    let _ = thread.map(|t| t.join());
    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
    let _ = std::fs::remove_dir_all(&dir);

    assert!(said.is_none(), "a tune was reported as a silent over: {said:?}");
}
