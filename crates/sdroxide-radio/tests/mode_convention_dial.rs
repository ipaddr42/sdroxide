//! Selecting a slot-based mode puts the dial on that band's agreed frequency.
//!
//! FT8, FT4, FT2, JS8 and WSPR are worked on one dial per band and decoded in
//! lockstep with everyone else on it, and each keeps its own: 20 m is 14.074,
//! 14.080, 14.084, 14.078 and 14.095600 respectively. Picking the mode and then
//! having to look the number up — or to arrive on the neighbouring mode's
//! frequency and hear nothing — is the trip this removes.
//!
//! What the rule must *not* do is as much of the point as what it does, so the
//! guards get as many tests as the move: a dial already on one of the mode's
//! own frequencies stays, a band the mode has no convention in is left alone,
//! and every mode outside the slotted set keeps the frequency the operator
//! tuned.

use std::time::Duration;

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RxId, Vfo};

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

/// Point the config at a scratch directory, once for the whole binary.
///
/// Two reasons. An engine reads (and seeds) `bandplan.json` and the region as
/// it starts, so without this the tests run against whatever the operator has
/// installed — and the frequencies asserted below are the built-in plan's. And
/// `SDROXIDE_CONFIG_DIR` unset means the *live* directory, which no test may
/// write to.
fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root =
            std::env::temp_dir().join(format!("sdroxide-mode-convention-{}", std::process::id()));
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

/// Tune to `from`, select `mode`, and report the dial the engine ended on.
fn dial_after(from: f64, mode: Mode) -> f64 {
    isolate_config();
    let mut h = start_engine(
        Box::new(MockSource { center: from }),
        caps(),
        EngineConfig { tx_ham_only: false, ..Default::default() },
    );
    let thread = h.thread.take();

    std::thread::sleep(Duration::from_millis(150));
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: from }).unwrap();
    h.cmd_tx.send(Command::SetMode { rx: RxId::Main, mode }).unwrap();
    std::thread::sleep(Duration::from_millis(300));

    let mut last = None;
    while let Ok(ev) = h.event_rx.try_recv() {
        if let RadioEvent::State(s) = ev {
            last = Some(s.active_freq_hz());
        }
    }

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    last.expect("the engine should publish state")
}

fn assert_dial(from: f64, mode: Mode, want: f64, why: &str) {
    let got = dial_after(from, mode);
    assert!(
        (got - want).abs() < 1.0,
        "{} in {:?} from {:.6} MHz: expected {:.6} MHz, got {:.6} MHz",
        why,
        mode,
        from / 1e6,
        want / 1e6,
        got / 1e6
    );
}

#[test]
fn each_slotted_mode_lands_on_its_own_frequency() {
    // From the middle of the 20 m phone band, which is nowhere near any of
    // them: every one of these is a move, and they are all different moves.
    for (mode, want) in [
        (Mode::Ft8, 14_074_000.0),
        (Mode::Ft4, 14_080_000.0),
        (Mode::Ft2, 14_084_000.0),
        (Mode::Js8, 14_078_000.0),
        (Mode::Wspr, 14_095_600.0),
    ] {
        assert_dial(14_200_000.0, mode, want, "selecting the mode should tune to 20 m's dial");
    }
}

#[test]
fn the_band_the_operator_is_on_is_the_band_they_stay_on() {
    // The rule moves a receiver to the right spot in the band it is already in.
    // It does not decide which band the operator wanted.
    assert_dial(7_150_000.0, Mode::Ft8, 7_074_000.0, "40 m");
    assert_dial(28_400_000.0, Mode::Ft8, 28_074_000.0, "10 m");
    assert_dial(144_300_000.0, Mode::Ft8, 144_174_000.0, "2 m");
}

#[test]
fn a_dial_already_on_one_of_the_modes_frequencies_is_left_alone() {
    // Arriving on the calling frequency is not a reason to command a tune...
    assert_dial(14_074_000.0, Mode::Ft8, 14_074_000.0, "already on the calling frequency");
    // ...and neither is arriving on one of the mode's *other* agreed dials.
    // 14.090 is FT8's DXpedition (Fox/Hound) window: an operator who put the
    // radio there did it on purpose, and dropping them onto the calling
    // frequency would take the DX away.
    assert_dial(14_090_000.0, Mode::Ft8, 14_090_000.0, "the DXpedition window");
    // The same argument one band over, where the mode's own frequency is not
    // the one the current band's convention would have chosen.
    assert_dial(7_056_000.0, Mode::Ft8, 7_056_000.0, "40 m's DXpedition window");
}

#[test]
fn a_band_with_no_convention_keeps_its_dial() {
    // 15 MHz is WWV, in no band at all: a frequency the operator chose, and no
    // reason to throw them into the nearest ham band.
    assert_dial(15_000_000.0, Mode::Ft8, 15_000_000.0, "WWV");
    // 26.5 MHz is inside no band either — 11 m starts at 26.965 (issue #396).
    assert_dial(26_500_000.0, Mode::Ft8, 26_500_000.0, "below the citizens' band");
    // 60 m has no FT8 convention in this table even though it is a band.
    assert_dial(5_357_000.0, Mode::Ft8, 5_357_000.0, "60 m");
}

/// Issue #396: 11 m is not an amateur band, but it *does* have digimode
/// channels its operators agreed on, and selecting a mode there lands on the
/// mode's channel exactly as it does on 20 m. Channel 19 is where a CB radio
/// idles; channel 26 is where the FT8 is.
#[test]
fn the_citizens_band_has_conventions_of_its_own() {
    assert_dial(27_185_000.0, Mode::Ft8, 27_265_000.0, "11 m FT8 (ch 26)");
    assert_dial(27_185_000.0, Mode::Js8, 27_245_000.0, "11 m JS8 (ch 25)");
    // ...and a mode with no 11 m channel is left where the operator put it.
    assert_dial(27_185_000.0, Mode::Ft4, 27_185_000.0, "11 m has no FT4 channel");
}

#[test]
fn the_modes_outside_the_rule_keep_the_frequency_they_were_given() {
    // The keyboard and image modes are worked across a segment rather than on
    // one spot: an operator calling CQ RTTY on 14.085 is where they meant to
    // be, and so is one running PSK31 at 14.072 or SSTV up from 14.230. The
    // ⇵ FREQ picker is how those get to a convention, by asking.
    assert_dial(14_085_000.0, Mode::Rtty, 14_085_000.0, "RTTY inside its sub-band");
    assert_dial(14_072_000.0, Mode::Psk, 14_072_000.0, "PSK31 inside its sub-band");
    assert_dial(14_233_000.0, Mode::Sstv, 14_233_000.0, "SSTV up from the calling frequency");
    // ...and so are the modes with no convention at all.
    assert_dial(14_200_000.0, Mode::Usb, 14_200_000.0, "SSB");
    assert_dial(14_030_000.0, Mode::Cw, 14_030_000.0, "CW");
}

#[test]
fn the_sub_receiver_does_not_move_the_dial() {
    // The sub receiver has its own frequency and no claim on the main dial:
    // putting it into FT8 must not retune the radio underneath it.
    isolate_config();
    let mut h = start_engine(
        Box::new(MockSource { center: 14_200_000.0 }),
        caps(),
        EngineConfig { tx_ham_only: false, ..Default::default() },
    );
    let thread = h.thread.take();

    std::thread::sleep(Duration::from_millis(150));
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: 14_200_000.0 }).unwrap();
    h.cmd_tx.send(Command::SetMode { rx: RxId::Sub, mode: Mode::Ft8 }).unwrap();
    std::thread::sleep(Duration::from_millis(300));

    let mut last = None;
    while let Ok(ev) = h.event_rx.try_recv() {
        if let RadioEvent::State(s) = ev {
            last = Some(s.active_freq_hz());
        }
    }

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    let got = last.expect("the engine should publish state");
    assert!((got - 14_200_000.0).abs() < 1.0, "the main dial moved to {:.6} MHz", got / 1e6);
}
