//! A memory carries its repeater setup with it (issue #137).
//!
//! A repeater channel is a frequency, a shift and a tone, and a memory that
//! kept only the first of the three would leave the operator setting the other
//! two from notes every time. So what is checked here is the round trip: stored
//! from the live controls, recalled onto them, and — the part that is easy to
//! get wrong — a plainly simplex memory taking the shift back *off* rather than
//! leaving the last repeater's shift standing on a simplex channel.
//!
//! One test function on purpose: `SDROXIDE_CONFIG_DIR` is process-global, and
//! this one writes a real `memories.json` under it.

use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{
    Command, DeviceCaps, MemoryChannel, Mode, RadioEvent, RadioState, RepeaterState, RxId, Shift,
    ToneMode, Vfo,
};

const RATE: f64 = 192_000.0;
const RPT_HZ: f64 = 145_712_500.0;
const SIMPLEX_HZ: f64 = 145_500_000.0;

/// A front end that tunes and delivers silence: a memory is a dial, a mode and
/// — now — a repeater setup, and nothing here listens to the audio.
struct Rig {
    center: f64,
}

impl IqSource for Rig {
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
        let n = buf.len().min(256);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "repeater memory bench".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "bench".into(),
        label: "bench rig".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 1_000_000_000.0)],
        ..DeviceCaps::default()
    }
}

fn memories(
    rx: &crossbeam_channel::Receiver<RadioEvent>,
    ready: impl Fn(&[MemoryChannel]) -> bool,
) -> Vec<MemoryChannel> {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last = Vec::new();
    while Instant::now() < deadline {
        if let Ok(RadioEvent::Memories(m)) = rx.recv_timeout(Duration::from_millis(100)) {
            last = m;
            if ready(&last) {
                return last;
            }
        }
    }
    panic!("the engine never announced the expected memories; last: {last:?}");
}

fn state_where(
    rx: &crossbeam_channel::Receiver<RadioEvent>,
    ready: impl Fn(&RadioState) -> bool,
) -> RadioState {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last = None;
    while Instant::now() < deadline {
        if let Ok(RadioEvent::State(s)) = rx.recv_timeout(Duration::from_millis(100)) {
            if ready(&s) {
                return s;
            }
            last = Some(s);
        }
    }
    panic!("the engine never reached the expected state; last: {last:#?}");
}

#[test]
fn a_memory_carries_its_repeater_setup_both_ways() {
    let root = std::env::temp_dir().join(format!("sdroxide-repeater-mem-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };

    let mut h = start_engine(
        Box::new(Rig { center: RPT_HZ }),
        caps(),
        EngineConfig { remember_session: false, ..Default::default() },
    );
    let thread = h.thread.take();
    let send = |c: Command| h.cmd_tx.send(c).unwrap();

    let rpt = RepeaterState {
        shift: Shift::Minus,
        offset_hz: 600_000,
        tone: ToneMode::Ctcss,
        ctcss_tenths: 1230,
        burst_auto: true,
        burst_ms: 500,
        ..RepeaterState::default()
    };

    // ---- Stored from the live controls ----
    send(Command::SetVfo { vfo: Vfo::A, hz: RPT_HZ });
    send(Command::SetMode { rx: RxId::Main, mode: Mode::Nfm });
    send(Command::SetRepeater(rpt));
    send(Command::StoreMemory { name: "GB3xx".into() });
    let stored = memories(&h.event_rx, |m| m.len() == 1);
    assert_eq!(stored[0].repeater, Some(rpt), "the channel forgot how to work its repeater");

    // ---- A simplex channel, stored while the shift is still on ----
    // What it must store is "simplex", not "no opinion": the recall below is
    // what has to take the shift back off.
    send(Command::SetVfo { vfo: Vfo::A, hz: SIMPLEX_HZ });
    send(Command::SetRepeater(RepeaterState::default()));
    send(Command::StoreMemory { name: "S20".into() });
    let stored = memories(&h.event_rx, |m| m.len() == 2);
    let (rpt_id, simplex_id) = (stored[0].id, stored[1].id);
    assert_eq!(stored[1].repeater, Some(RepeaterState::default()));

    // ---- Recalling the repeater channel sets the radio up for it ----
    send(Command::RecallMemory(rpt_id));
    let s = state_where(&h.event_rx, |s| {
        s.repeater.shift == Shift::Minus && s.active_freq_hz() == RPT_HZ
    });
    assert_eq!(s.repeater, rpt);
    assert_eq!(s.tx_freq_hz(), RPT_HZ - 600_000.0);

    // ---- …and recalling the simplex one takes it back off ----
    send(Command::RecallMemory(simplex_id));
    let s = state_where(&h.event_rx, |s| {
        s.active_freq_hz() == SIMPLEX_HZ && s.repeater.shift == Shift::Simplex
    });
    assert_eq!(s.repeater, RepeaterState::default(), "the shift outlived the channel it was for");
    assert_eq!(s.tx_freq_hz(), SIMPLEX_HZ);

    // ---- A channel from before the field existed is simplex too ----
    // `memories.json` files written before repeater support carry no setup at
    // all, and the list draws them exactly like a simplex channel. Recalling
    // one has to leave the radio simplex with no tone rather than on whatever
    // the last repeater recall set (issue #204) — the operator is reading
    // "145.500 NFM" off the list, not "and keep the −600 kHz shift".
    send(Command::RecallMemory(rpt_id));
    state_where(&h.event_rx, |s| s.repeater.shift == Shift::Minus);
    send(Command::EditMemory {
        id: simplex_id,
        name: "S20".into(),
        freq_hz: SIMPLEX_HZ,
        mode: Mode::Nfm,
        repeater: None,
        antenna: None,
    });
    memories(&h.event_rx, |m| m[1].repeater.is_none());
    send(Command::RecallMemory(simplex_id));
    let s = state_where(&h.event_rx, |s| {
        s.active_freq_hz() == SIMPLEX_HZ && s.repeater.shift == Shift::Simplex
    });
    assert_eq!(s.repeater.tx_tone(), None, "an unset channel must not keep the last tone");
    assert_eq!(s.tx_freq_hz(), SIMPLEX_HZ);

    // ---- An edit rewrites the setup in place ----
    let edited = RepeaterState { tone: ToneMode::Dcs, dcs_code: 754, ..rpt };
    send(Command::EditMemory {
        id: rpt_id,
        name: "GB3xx".into(),
        freq_hz: RPT_HZ,
        mode: Mode::Nfm,
        repeater: Some(edited),
        antenna: None,
    });
    let after = memories(&h.event_rx, |m| m[0].repeater == Some(edited));
    assert_eq!(after[0].id, rpt_id, "an edit is the same channel, not a new one");

    // ---- And it is on disk, not merely announced ----
    drop(h);
    let _ = thread.map(|t| t.join());
    let saved = sdroxide_config::load_memories();
    assert_eq!(saved.len(), 2);
    assert_eq!(saved[0].repeater, Some(edited));
    // Left as the edit above put it: an unset channel stays unset on disk, and
    // it is the *recall* that reads it as simplex.
    assert_eq!(saved[1].repeater, None);

    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
    let _ = std::fs::remove_dir_all(&root);
}
