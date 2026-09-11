//! One Drive setting, one output power, on every band (issue #295).
//!
//! Real amplifiers have noticeably different gain per band — 10 m typically
//! wants several decibels more drive than 40 m for the same watts out — so a
//! constant Drive number makes a different power on each. Without a calibration
//! table an operator holding a constant output has to remember a different
//! Drive for every band and set it by hand on each band change, which is
//! exactly what Thetis's per-band drive table automates.
//!
//! The trim is measured in decibels of *output power*, so what it means in
//! "drive" depends on the radio underneath: on a rig that commands its own
//! power the fraction is already a power, and on an IQ transmitter drive scales
//! an amplitude. This exercises the first, because that is the one whose
//! commanded value can be watched from outside.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{
    Band, BandDriveTrim, Command, DeviceCaps, RadioConfig, RadioEvent, RadioState, Vfo,
};

const RATE: f64 = 2_400_000.0;

/// A transceiver with its own power control: `set_tx_drive` is a fraction of
/// its rated watts, the way a CAT, LAN or TCI rig takes it, and every value it
/// is handed is recorded.
struct MockRig {
    center: f64,
    drive: Arc<Mutex<Vec<f64>>>,
}

impl IqSource for MockRig {
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
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(2048);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock rig with its own power control".into()
    }
    fn set_tx_drive(&mut self, frac: f64) {
        self.drive.lock().unwrap().push(frac);
    }
    fn commands_tx_power(&self) -> bool {
        true
    }
}

/// Point the config at a scratch directory, once for the whole binary: this
/// test writes a `radio.json` and none of the operator's own may be touched.
fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root = std::env::temp_dir().join(format!("sdroxide-drive-trim-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };
    });
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock".into(),
        rx_channels: 1,
        tx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 60_000_000.0)],
        freq_ranges_tx: vec![(1_000_000.0, 60_000_000.0)],
        antennas_tx: vec!["TX/RX".into()],
        ..DeviceCaps::default()
    }
}

/// The whole feature: 40 m is calibrated six decibels down, 20 m is not, and
/// moving the dial between them moves the power the rig is commanded — with no
/// hand on the Drive slider.
#[test]
fn the_drive_that_reaches_the_rig_follows_the_bands_calibration() {
    isolate_config();
    let drive = Arc::new(Mutex::new(Vec::new()));
    let mut h = start_engine(
        Box::new(MockRig { center: 7_074_000.0, drive: Arc::clone(&drive) }),
        caps(),
        EngineConfig::default(),
    );
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));

    // 40 m makes twice the power this station wants for a given Drive; 20 m is
    // where the amplifier was calibrated and needs no correction. A row set to
    // zero is carried deliberately: it must behave exactly like no row at all.
    let cfg = RadioConfig {
        tx_drive_trim: vec![
            BandDriveTrim { band: Band::M40, db: -6.0 },
            BandDriveTrim { band: Band::M20, db: 0.0 },
        ],
        ..RadioConfig::default()
    };
    h.cmd_tx.send(Command::SetRadioConfig { cfg: Box::new(cfg), reopen: false }).unwrap();
    std::thread::sleep(Duration::from_millis(150));

    // Full drive, on 40 m.
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: 7_074_000.0 }).unwrap();
    std::thread::sleep(Duration::from_millis(150));
    drive.lock().unwrap().clear();
    h.cmd_tx.send(Command::SetTxDrive(1.0)).unwrap();
    std::thread::sleep(Duration::from_millis(150));
    let on_40 = *drive.lock().unwrap().last().expect("the rig should have been given a power");
    assert!(
        (on_40 - 0.2511886).abs() < 1e-3,
        "6 dB off a power setting is a quarter of it, got {on_40}"
    );

    // …and 20 m, which the operator never touched the slider for.
    drive.lock().unwrap().clear();
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: 14_074_000.0 }).unwrap();
    std::thread::sleep(Duration::from_millis(250));
    let on_20 = *drive
        .lock()
        .unwrap()
        .last()
        .expect("moving to an uncalibrated band should hand the rig its power back");
    assert!((on_20 - 1.0).abs() < 1e-3, "20 m has no trim, so full drive is full, got {on_20}");

    // The slider itself never moved: the calibration is a property of the
    // station, not of the setting, and an operator who set 100% still reads
    // 100% on the band that is transmitting at a quarter of it.
    let mut last: Option<RadioState> = None;
    while let Ok(ev) = h.event_rx.try_recv() {
        if let RadioEvent::State(s) = ev {
            last = Some(s);
        }
    }
    if let Some(s) = last {
        assert!(
            (s.tx.drive - 1.0).abs() < 1e-6,
            "the Drive setting is untouched, got {}",
            s.tx.drive
        );
    }

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
}

/// A radio where drive scales the modulated I/Q instead of commanding a rig's
/// power — a LimeSDR, a HackRF, a Pluto, an HPSDR board. Records the peak
/// magnitude of everything written to the transmit path.
struct MockIqTx {
    center: f64,
    peak: Arc<Mutex<f32>>,
}

impl IqSource for MockIqTx {
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
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(2048);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock IQ transmitter".into()
    }
    fn tx_begin(&mut self, _center_hz: f64, rate: f64) -> Result<f64> {
        Ok(rate)
    }
    fn tx_write(&mut self, samples: &[Complex32]) -> Result<()> {
        let mut p = self.peak.lock().unwrap();
        for z in samples {
            *p = p.max(z.norm());
        }
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
}

/// The same calibration on the radios that have no power control of their own,
/// where it lands on the samples rather than on a commanded fraction — which is
/// every direct-sampling and zero-IF transmitter sdroxide modulates itself, and
/// the half a LimeSDR operator reported as doing nothing (issue #376).
///
/// Measured as a *ratio* between two bands rather than against an absolute
/// figure: the upconverter's own passband gain is not the subject, the trim is.
#[test]
fn the_samples_a_lime_transmits_follow_the_bands_calibration() {
    isolate_config();
    let peak = Arc::new(Mutex::new(0.0f32));
    let caps = DeviceCaps {
        freq_ranges_rx: vec![(1_000_000.0, 3_800_000_000.0)],
        freq_ranges_tx: vec![(1_000_000.0, 3_800_000_000.0)],
        ..caps()
    };
    let mut h = start_engine(
        Box::new(MockIqTx { center: 145_500_000.0, peak: Arc::clone(&peak) }),
        caps,
        EngineConfig { tx_ham_only: false, ..EngineConfig::default() },
    );
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));

    // 2 m comes out 20 dB hot on this station's amplifier; 23 cm is where it
    // was calibrated.
    let cfg = RadioConfig {
        tx_drive_trim: vec![BandDriveTrim { band: Band::M2, db: -20.0 }],
        ..RadioConfig::default()
    };
    h.cmd_tx.send(Command::SetRadioConfig { cfg: Box::new(cfg), reopen: false }).unwrap();
    h.cmd_tx.send(Command::SetTuneDrive(1.0)).unwrap();
    std::thread::sleep(Duration::from_millis(150));

    // A held carrier is the cleanest thing to measure: no microphone, no
    // modulation, one amplitude.
    let tune_peak = |hz: f64| -> f32 {
        h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz }).unwrap();
        std::thread::sleep(Duration::from_millis(250));
        *peak.lock().unwrap() = 0.0;
        h.cmd_tx.send(Command::SetTune(true)).unwrap();
        std::thread::sleep(Duration::from_millis(400));
        h.cmd_tx.send(Command::SetTune(false)).unwrap();
        std::thread::sleep(Duration::from_millis(200));
        *peak.lock().unwrap()
    };

    let on_23cm = tune_peak(1_296_100_000.0);
    let on_2m = tune_peak(145_500_000.0);
    assert!(on_23cm > 0.01, "the uncalibrated band should have transmitted, got {on_23cm}");
    assert!(on_2m > 0.0, "2 m should have transmitted at all, got {on_2m}");

    // 20 dB of output power is a tenth of the amplitude.
    let ratio = on_2m / on_23cm;
    assert!(
        (ratio - 0.1).abs() < 0.02,
        "2 m is trimmed 20 dB down, so it should transmit at a tenth of the amplitude \
         23 cm does: got {on_2m} against {on_23cm} (ratio {ratio})"
    );

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
}
