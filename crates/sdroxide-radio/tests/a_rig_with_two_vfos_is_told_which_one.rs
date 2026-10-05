//! A front end that has its own pair of VFOs is told which one is in use.
//!
//! Only a handful of radios have a second VFO to select — an ELAD FDM-DUO is
//! the first — but the two things this pins are not ELAD's:
//!
//! * the source hears about the switch at all, with the frequency beside it,
//!   so selecting a VFO holding a stale number is one command wide rather than
//!   lasting until the next retune;
//! * the number is the **rig's**, not the dial. In CW a radio that keys its own
//!   transmitter sits a sidetone above the dial, which is how `follow_dial`
//!   commands it. Handing over the bare dial puts such a rig one pitch low, and
//!   on a rig whose dial is read back and believed the error compounds: the
//!   frequency walks down by one pitch on every switch. That was measured on an
//!   FDM-DUO before this test existed — 1200.0, then 1199.3, then 1198.6 kHz;
//! * the rig hears which VFO *before* anything else about the switch. Taking
//!   up a VFO left in another mode retunes for that mode, and a retune sent
//!   ahead of the selection lands on the VFO being left — overwriting the
//!   radio's other dial with this one's.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RxId, Vfo};

const RATE: f64 = 192_000.0;
const PITCH: f64 = 700.0;

/// A rig whose window is its VFO and which keys its own transmitter — the
/// shape that makes the CW offset apply, as on the FDM-DUO.
#[derive(Clone)]
struct RigWithTwoVfos {
    center: f64,
    told: Arc<Mutex<Vec<(Vfo, f64)>>>,
    /// Every selection and every retune, in the order the rig was given them.
    log: Arc<Mutex<Vec<String>>>,
}

impl IqSource for RigWithTwoVfos {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.log.lock().unwrap().push(format!("tune {hz:.0}"));
        Ok(())
    }
    fn center_is_dial(&self) -> bool {
        true
    }
    fn cw_iq_on_vfo(&self) -> bool {
        true
    }
    fn select_vfo(&mut self, vfo: Vfo, hz: f64) {
        self.told.lock().unwrap().push((vfo, hz));
        self.log.lock().unwrap().push(format!("select {vfo:?}"));
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(2048);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock rig with two VFOs".into()
    }
}

fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root = std::env::temp_dir().join(format!("sdroxide-rig-vfo-{}", std::process::id()));
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

/// Run `cmds` and report every `select_vfo` the source was given.
fn told_after(cmds: &[Command]) -> Vec<(Vfo, f64)> {
    run(cmds).0
}

/// Run `cmds` and report every `select_vfo` the source was given, and the log
/// of selections and retunes together.
fn run(cmds: &[Command]) -> (Vec<(Vfo, f64)>, Vec<String>) {
    isolate_config();
    let told = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut h = start_engine(
        Box::new(RigWithTwoVfos {
            center: 940_000.0,
            told: Arc::clone(&told),
            log: Arc::clone(&log),
        }),
        caps(),
        EngineConfig { tx_ham_only: false, ..Default::default() },
    );
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(150));
    for c in cmds {
        h.cmd_tx.send(c.clone()).unwrap();
        std::thread::sleep(Duration::from_millis(80));
    }
    std::thread::sleep(Duration::from_millis(150));
    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    let out = (told.lock().unwrap().clone(), log.lock().unwrap().clone());
    out
}

#[test]
fn taking_up_the_other_vfo_tells_the_rig_which_one_and_where() {
    let told = told_after(&[
        Command::SetVfo { vfo: Vfo::A, hz: 940_000.0 },
        Command::SetMode { rx: RxId::Main, mode: Mode::Am },
        Command::SelectVfo(Vfo::B),
        Command::SetVfo { vfo: Vfo::B, hz: 1_200_000.0 },
        Command::SelectVfo(Vfo::A),
    ]);
    let vfos: Vec<Vfo> = told.iter().map(|(v, _)| *v).collect();
    assert_eq!(vfos, vec![Vfo::B, Vfo::A], "each switch is announced once, in order");
    let (_, back_on_a) = told.last().expect("a switch back to A");
    assert!((back_on_a - 940_000.0).abs() < 1.0, "A is on 940 kHz, got {back_on_a}");
}

/// The regression: in CW the rig is told the frequency it must sit on, which is
/// one sidetone above the dial, so a dial read back from the rig means the same
/// thing on the way in as on the way out.
#[test]
fn a_cw_vfo_is_handed_the_frequency_the_rig_must_sit_on() {
    let told = told_after(&[
        Command::SetVfo { vfo: Vfo::A, hz: 940_000.0 },
        Command::SetMode { rx: RxId::Main, mode: Mode::Am },
        Command::SelectVfo(Vfo::B),
        Command::SetVfo { vfo: Vfo::B, hz: 1_200_000.0 },
        Command::SetMode { rx: RxId::Main, mode: Mode::Cw },
        Command::SelectVfo(Vfo::A),
        Command::SelectVfo(Vfo::B),
    ]);
    let (vfo, hz) = *told.last().expect("a switch onto the CW VFO");
    assert_eq!(vfo, Vfo::B);
    assert!(
        (hz - (1_200_000.0 + PITCH)).abs() < 1.0,
        "a CW VFO on 1200 kHz puts the rig on 1200 kHz + one pitch, got {hz}"
    );

    // And the AM VFO is still handed its bare dial: the offset belongs to CW,
    // not to switching.
    let (vfo, hz) = told[told.len() - 2];
    assert_eq!(vfo, Vfo::A);
    assert!((hz - 940_000.0).abs() < 1.0, "an AM VFO takes no sidetone step, got {hz}");
}

/// Switching between VFOs in different modes retunes for the mode being taken
/// up — here CW, a sidetone above the dial — and that retune has to reach the
/// rig *after* the selection. Sent before it, it goes to the VFO being left and
/// overwrites the radio's other dial with this one's number.
#[test]
fn the_rig_is_told_which_vfo_before_it_is_retuned_for_it() {
    let (told, log) = run(&[
        Command::SetVfo { vfo: Vfo::A, hz: 940_000.0 },
        Command::SetMode { rx: RxId::Main, mode: Mode::Am },
        Command::SelectVfo(Vfo::B),
        Command::SetVfo { vfo: Vfo::B, hz: 1_200_000.0 },
        Command::SetMode { rx: RxId::Main, mode: Mode::Cw },
        Command::SelectVfo(Vfo::A),
        Command::SelectVfo(Vfo::B),
    ]);
    let (vfo, hz) = *told.last().expect("a switch onto the CW VFO");
    assert_eq!(vfo, Vfo::B);
    assert!((hz - (1_200_000.0 + PITCH)).abs() < 1.0, "the CW VFO's rig frequency, got {hz}");
    // Between the last switch onto A and the switch back to B, nothing may put
    // the rig on B's number — that retune belongs after B is selected. And once
    // B is selected, nothing puts it back on A's.
    let a = log.iter().rposition(|e| e == "select A").expect("a switch to A");
    let b = log.iter().rposition(|e| e == "select B").expect("a switch back to B");
    assert!(b > a, "{log:?}");
    assert!(
        !log[a + 1..b].iter().any(|e| e == "tune 1200700"),
        "B's retune reached the rig before B was selected, so it landed on A: {log:?}"
    );
    assert!(
        !log[b + 1..].iter().any(|e| e == "tune 940000"),
        "A's dial reached the rig after B was selected: {log:?}"
    );
}

/// A profile saved on the other VFO puts the station back on that VFO — and the
/// rig is told so before the dial is sent, exactly as an A/B press tells it.
/// Setting the active VFO behind the rig's back left its own A/B on the old one
/// and put the profile's frequency into the dial it was not meant for (#197).
#[test]
fn a_profile_saved_on_the_other_vfo_tells_the_rig_before_retuning() {
    let (told, log) = run(&[
        Command::SetVfo { vfo: Vfo::B, hz: 1_200_000.0 },
        Command::SelectVfo(Vfo::B),
        Command::ProfileSave("on B".into()),
        Command::SelectVfo(Vfo::A),
        Command::ProfileApply("on B".into()),
    ]);
    let vfos: Vec<Vfo> = told.iter().map(|(v, _)| *v).collect();
    assert_eq!(vfos, vec![Vfo::B, Vfo::A, Vfo::B], "the apply never told the rig");
    let (_, hz) = told.last().unwrap();
    assert!((hz - 1_200_000.0).abs() < 1.0, "B is on 1.2 MHz, the rig was told {hz}");

    let selected = log.iter().rposition(|l| l == "select B").unwrap();
    let retuned = log.iter().rposition(|l| l == "tune 1200000").expect("the apply retuned");
    assert!(selected < retuned, "the dial went ahead of the selection: {log:?}");
}
