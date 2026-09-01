//! Editing a stored memory in place (issue #138).
//!
//! Correcting a typo, or a frequency that moved, used to mean deleting the
//! channel and storing it again — which takes the operator to that frequency
//! first and loses the channel's place in the list. So what is checked here is
//! that an edit reaches the store as an edit: the same channel, same id, new
//! contents, and the passband still the one the mode needs.
//!
//! One test function on purpose: `SDROXIDE_CONFIG_DIR` is process-global, and
//! this one writes a real `memories.json` under it.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, MemoryChannel, Mode, RadioEvent, RxId, Vfo};

const RATE: f64 = 192_000.0;
const STORED_HZ: f64 = 14_070_000.0;
const EDITED_HZ: f64 = 10_100_800.0;

/// A front end that tunes and delivers silence: nothing here listens to the
/// audio, and a memory is a dial and a mode.
struct Rig {
    center: Arc<Mutex<f64>>,
}

impl IqSource for Rig {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        *self.center.lock().unwrap()
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        *self.center.lock().unwrap() = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(256);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "memory bench".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "bench".into(),
        label: "bench rig".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 60_000_000.0)],
        // Three sockets, like an RSPdx: enough for the editor's antenna field
        // to be a choice rather than a formality (issue #246).
        antennas_rx: vec!["Antenna A".into(), "Antenna B".into(), "Antenna C".into()],
        ..DeviceCaps::default()
    }
}

/// The memory list as the engine last announced it, once it holds `want`
/// channels and the first of them satisfies `ready`.
///
/// A command is acted on when the engine gets round to it, so every assertion
/// has to wait for the announcement that says it has been — and the list is
/// republished on every change, so waiting for the *content* rather than for
/// any announcement at all is what keeps this from reading the state before
/// the edit.
fn memories(
    rx: &crossbeam_channel::Receiver<RadioEvent>,
    want: usize,
    ready: impl Fn(&MemoryChannel) -> bool,
) -> Vec<MemoryChannel> {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last = Vec::new();
    while Instant::now() < deadline {
        if let Ok(RadioEvent::Memories(m)) = rx.recv_timeout(Duration::from_millis(100)) {
            last = m;
            if last.len() == want && last.first().is_some_and(&ready) {
                return last;
            }
        }
    }
    panic!("the engine never announced {want} memories in the expected state; last: {last:?}");
}

#[test]
fn an_edit_rewrites_the_channel_it_names() {
    let root = std::env::temp_dir().join(format!("sdroxide-memory-edit-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };

    let center = Arc::new(Mutex::new(STORED_HZ));
    let mut h = start_engine(
        Box::new(Rig { center }),
        caps(),
        EngineConfig { remember_session: false, ..Default::default() },
    );
    let thread = h.thread.take();
    let send = |c: Command| h.cmd_tx.send(c).unwrap();

    // ---- Stored with a passband of the operator's own ----
    send(Command::SetVfo { vfo: Vfo::A, hz: STORED_HZ });
    send(Command::SetMode { rx: RxId::Main, mode: Mode::Usb });
    send(Command::SetFilter { rx: RxId::Main, lo: 200.0, hi: 1800.0 });
    send(Command::StoreMemory { name: "tpyo".into() });
    let stored = memories(&h.event_rx, 1, |m| m.name == "tpyo");
    let id = stored[0].id;
    assert_eq!(stored[0].freq_hz, STORED_HZ);
    assert_eq!((stored[0].filter_lo, stored[0].filter_hi), (200.0, 1800.0));

    // ---- A dial that is not a frequency is refused ----
    // Both would take the channel with them if they landed: each also switches
    // the mode, which is what re-defaults the passband.
    send(Command::EditMemory {
        id,
        name: "zero".into(),
        freq_hz: 0.0,
        mode: Mode::Cw,
        repeater: None,
        antenna: None,
    });
    send(Command::EditMemory {
        id,
        name: "nan".into(),
        freq_hz: f64::NAN,
        mode: Mode::Cw,
        repeater: None,
        antenna: None,
    });

    // ---- The name alone: everything else, the filter included, survives ----
    send(Command::EditMemory {
        id,
        name: "DWD".into(),
        freq_hz: STORED_HZ,
        mode: Mode::Usb,
        repeater: None,
        antenna: None,
    });
    let renamed = memories(&h.event_rx, 1, |m| m.name == "DWD");
    assert_eq!(renamed[0].id, id, "an edit is the same channel, not a new one");
    assert_eq!(renamed[0].freq_hz, STORED_HZ);
    // Which is also the assertion that the two refused edits were refused: had
    // either landed, this channel would be in CW by now, and the edit back to
    // USB would have handed it USB's default passband instead of the one the
    // operator stored.
    assert_eq!(
        (renamed[0].filter_lo, renamed[0].filter_hi),
        (200.0, 1800.0),
        "a filter the operator chose is not a default to be re-applied"
    );

    // ---- Frequency and mode: the passband follows the new mode ----
    send(Command::EditMemory {
        id,
        name: "DDK2".into(),
        freq_hz: EDITED_HZ,
        mode: Mode::Cw,
        repeater: None,
        antenna: None,
    });
    let edited = memories(&h.event_rx, 1, |m| m.name == "DDK2");
    assert_eq!(edited[0].id, id);
    assert_eq!(edited[0].freq_hz, EDITED_HZ);
    assert_eq!(edited[0].mode, Mode::Cw);
    assert_eq!(
        (edited[0].filter_lo, edited[0].filter_hi),
        Mode::Cw.default_filter_at(EDITED_HZ),
        "the stored SSB passband would be a CW channel nobody could hear"
    );

    // ---- The antenna is the operator's to set, not only the radio's ----
    // Storing a channel captures whichever socket the radio was on; the editor
    // is where an operator says which one it should have been.
    send(Command::EditMemory {
        id,
        name: "DDK2".into(),
        freq_hz: EDITED_HZ,
        mode: Mode::Cw,
        repeater: None,
        antenna: Some("Antenna B".into()),
    });
    let on_b = memories(&h.event_rx, 1, |m| m.antenna.as_deref() == Some("Antenna B"));
    assert_eq!(on_b[0].id, id, "still the same channel");

    // A socket this front end does not have would be a channel that recalls
    // onto nothing, so it reads as "leave the antenna alone" instead.
    send(Command::EditMemory {
        id,
        name: "DDK2".into(),
        freq_hz: EDITED_HZ,
        mode: Mode::Cw,
        repeater: None,
        antenna: Some("Beverage".into()),
    });
    let cleared = memories(&h.event_rx, 1, |m| m.antenna.is_none());
    assert_eq!(cleared[0].id, id);

    // ---- And it is on disk, not merely announced ----
    // The operator who corrected a typo did it once.
    drop(h);
    let _ = thread.map(|t| t.join());
    let saved = sdroxide_config::load_memories();
    assert_eq!(saved.len(), 1);
    assert_eq!(saved[0].id, id);
    assert_eq!(saved[0].name, "DDK2");
    assert_eq!(saved[0].freq_hz, EDITED_HZ);
    assert_eq!(saved[0].mode, Mode::Cw);

    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
    let _ = std::fs::remove_dir_all(&root);
}
