//! Each VFO remembers the mode it was left in.
//!
//! A VFO is a whole listening position, not just a number: CW on A while B sits
//! on an SSB net is what the pair is for, and every transceiver with an A/B
//! button remembers the mode alongside the frequency. Before issue #286 the two
//! shared one mode, so switching VFO meant reaching for the mode buttons as
//! well — twice per look at the other frequency.
//!
//! The filter travels with the mode, because selecting a mode installs that
//! mode's default width; a VFO restored on its mode alone would come back with
//! a passband the operator had already narrowed and lost.

use std::time::Duration;

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RadioState, RxId, Vfo};

const RATE: f64 = 2_400_000.0;

struct MockSource {
    center: f64,
}

impl IqSource for MockSource {
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
        "mock rx source".into()
    }
}

/// Point the config at a scratch directory, once for the whole binary — the
/// engine reads and seeds files as it starts, and no test may write to the
/// operator's own.
fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root = std::env::temp_dir().join(format!("sdroxide-vfo-mode-{}", std::process::id()));
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
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(0.0, 1_000_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run `cmds` against a fresh engine and report the last state it published.
fn after(cmds: &[Command]) -> RadioState {
    isolate_config();
    let mut h = start_engine(
        Box::new(MockSource { center: 14_200_000.0 }),
        caps(),
        EngineConfig { tx_ham_only: false, ..Default::default() },
    );
    let thread = h.thread.take();

    std::thread::sleep(Duration::from_millis(150));
    for c in cmds {
        h.cmd_tx.send(c.clone()).unwrap();
        std::thread::sleep(Duration::from_millis(60));
    }
    std::thread::sleep(Duration::from_millis(200));

    let mut last = None;
    while let Ok(ev) = h.event_rx.try_recv() {
        if let RadioEvent::State(s) = ev {
            last = Some(s);
        }
    }

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    last.expect("the engine should publish state")
}

fn mode(rx: Vfo, hz: f64, m: Mode) -> Vec<Command> {
    vec![
        Command::SelectVfo(rx),
        Command::SetVfo { vfo: rx, hz },
        Command::SetMode { rx: RxId::Main, mode: m },
    ]
}

/// The whole of issue #286: watch CW on A and an SSB net on B, and switching
/// between them puts the receiver into the right mode by itself.
#[test]
fn each_vfo_comes_back_in_the_mode_it_was_left_in() {
    let mut cmds = mode(Vfo::A, 14_030_000.0, Mode::Cw);
    cmds.extend(mode(Vfo::B, 14_250_000.0, Mode::Usb));
    // Back to A, which was left in CW.
    cmds.push(Command::SelectVfo(Vfo::A));
    let s = after(&cmds);
    assert_eq!(s.rx[0].mode, Mode::Cw, "A was left in CW");
    assert!((s.active_freq_hz() - 14_030_000.0).abs() < 1.0);

    // And over to B again, which was left in USB.
    let mut cmds = mode(Vfo::A, 14_030_000.0, Mode::Cw);
    cmds.extend(mode(Vfo::B, 14_250_000.0, Mode::Usb));
    cmds.push(Command::SelectVfo(Vfo::A));
    cmds.push(Command::SelectVfo(Vfo::B));
    let s = after(&cmds);
    assert_eq!(s.rx[0].mode, Mode::Usb, "B was left in USB");
    assert!((s.active_freq_hz() - 14_250_000.0).abs() < 1.0);
}

/// A→B is a copy of the whole position. The frequency alone would leave the
/// duplicated VFO listening to A's frequency in something else.
#[test]
fn copying_a_to_b_copies_the_mode_with_it() {
    let mut cmds = mode(Vfo::B, 14_250_000.0, Mode::Usb);
    cmds.extend(mode(Vfo::A, 14_030_000.0, Mode::Cw));
    cmds.push(Command::CopyAtoB);
    cmds.push(Command::SelectVfo(Vfo::B));
    let s = after(&cmds);
    assert_eq!(s.rx[0].mode, Mode::Cw, "B is a copy of A now, mode included");
    assert!((s.active_freq_hz() - 14_030_000.0).abs() < 1.0);
}

/// A swap exchanges the listening positions, so the active VFO — which does not
/// change — is holding the other one's setup afterwards and the receiver has to
/// follow it there.
#[test]
fn swapping_exchanges_the_modes_as_well_as_the_dials() {
    let mut cmds = mode(Vfo::B, 14_250_000.0, Mode::Usb);
    cmds.extend(mode(Vfo::A, 14_030_000.0, Mode::Cw));
    cmds.push(Command::SwapVfos);
    let s = after(&cmds);
    assert_eq!(s.active_vfo, Vfo::A, "a swap does not change which VFO is in use");
    assert_eq!(s.rx[0].mode, Mode::Usb, "A is holding what B had");
    assert!((s.active_freq_hz() - 14_250_000.0).abs() < 1.0);
}

/// Re-selecting the VFO already in use must not undo a mode the operator has
/// just chosen: the shelf is written on the way out, so a redundant select
/// shelves what is in force and puts it straight back.
#[test]
fn selecting_the_vfo_already_in_use_changes_nothing() {
    let mut cmds = mode(Vfo::A, 14_030_000.0, Mode::Cw);
    cmds.push(Command::SetMode { rx: RxId::Main, mode: Mode::Lsb });
    cmds.push(Command::SelectVfo(Vfo::A));
    let s = after(&cmds);
    assert_eq!(s.rx[0].mode, Mode::Lsb);
}

/// The width goes with the mode. Narrowing CW down on A and coming back to it
/// has to give back the filter that was set, not the mode's default.
#[test]
fn a_narrowed_passband_survives_a_trip_to_the_other_vfo() {
    let mut cmds = mode(Vfo::A, 14_030_000.0, Mode::Cw);
    cmds.push(Command::SetFilter { rx: RxId::Main, lo: -125.0, hi: 125.0 });
    cmds.extend(mode(Vfo::B, 14_250_000.0, Mode::Usb));
    cmds.push(Command::SelectVfo(Vfo::A));
    let s = after(&cmds);
    assert_eq!(s.rx[0].mode, Mode::Cw);
    assert!(
        (s.rx[0].filter_hi - s.rx[0].filter_lo - 250.0).abs() < 1.0,
        "the 250 Hz passband came back as {} .. {}",
        s.rx[0].filter_lo,
        s.rx[0].filter_hi
    );
}
