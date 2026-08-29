//! A dial the *rig* moved has to move the receiver, not just the readout.
//!
//! The arrangement is the panadapter pairing (§2.19): a transceiver on CAT
//! beside an SDR that supplies the spectrum. The SDR's window does not follow
//! the transceiver's knob — that separation is the point of the pairing — so
//! when flrig retunes the rig from WSJT-X or fldigi, the engine's own DDC is
//! the only thing that can put the receiver on the new frequency.
//!
//! It used to be the one thing that did not happen. `adopt_source_center`
//! changes nothing here (the receiver's centre has not moved) and
//! `keep_vfo_in_span` changes nothing either while the new dial is still inside
//! the span — so the readout and the passband marker followed the rig and the
//! audio stayed on the frequency before the move (issue #206).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, ControlUpdate, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{DeviceCaps, RadioEvent};

const RATE: f64 = 192_000.0;
const CENTER: f64 = 14_100_000.0;

#[derive(Default)]
struct Rig {
    /// Set by the test to stand in for flrig moving the transceiver; reported
    /// on the next poll exactly as the CAT thread would.
    knob: Option<f64>,
    /// Every DDC offset the engine pushed at the source, in order. This is what
    /// `update_tuning` sends and the only thing on this side that decides which
    /// frequency is demodulated — the pairing's own `set_if_offset` is where
    /// the transceiver's dial is reconstructed from it.
    offsets: Vec<f64>,
}

/// An SDR whose span is fixed while the transceiver beside it tunes: exactly
/// what `PanadapterSource` presents to the engine.
struct PairedRig {
    rig: Arc<Mutex<Rig>>,
}

impl IqSource for PairedRig {
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
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(1024);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "transceiver + panadapter receiver".into()
    }
    /// The window belongs to the receiver, not to the transceiver's knob.
    fn center_is_dial(&self) -> bool {
        false
    }
    fn poll_control(&mut self) -> Vec<ControlUpdate> {
        let mut r = self.rig.lock().unwrap();
        r.knob.take().map(ControlUpdate::Freq).into_iter().collect()
    }
    fn set_if_offset(&mut self, hz: f64) {
        self.rig.lock().unwrap().offsets.push(hz);
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "bench".into(),
        label: "paired rig".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 60_000_000.0)],
        ..DeviceCaps::default()
    }
}

fn wait_for_state(
    rx: &crossbeam_channel::Receiver<RadioEvent>,
    want: impl Fn(&sdroxide_types::RadioState) -> bool,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        if let Ok(RadioEvent::State(s)) = rx.recv_timeout(Duration::from_millis(100))
            && want(&s)
        {
            return true;
        }
    }
    false
}

#[test]
fn a_dial_the_rig_moved_reseats_the_receiver() {
    let rig = Arc::new(Mutex::new(Rig::default()));
    let mut h = start_engine(
        Box::new(PairedRig { rig: Arc::clone(&rig) }),
        caps(),
        EngineConfig::default(),
    );
    let thread = h.thread.take();

    // The dial the engine opened on: the receiver sits on it, so no offset.
    assert!(wait_for_state(&h.event_rx, |s| s.active_freq_hz() == CENTER));

    // flrig moves the rig 1 kHz up — well inside the SDR's 192 kHz span, so
    // nothing about the window needs to change and nothing about it does.
    let moved = CENTER + 1_000.0;
    rig.lock().unwrap().knob = Some(moved);
    assert!(
        wait_for_state(&h.event_rx, |s| s.active_freq_hz() == moved),
        "the engine has to adopt the dial the rig reported"
    );

    // …and the receiver has to be there too. Anything else is the bug: a
    // readout and a passband marker on the new frequency with the audio still
    // coming off the old one.
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut last = Vec::new();
    while Instant::now() < deadline {
        last = rig.lock().unwrap().offsets.clone();
        if last.last().is_some_and(|o| (o - 1_000.0).abs() < 0.5) {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(
        last.last().map(|o| o.round()),
        Some(1_000.0),
        "the DDC never followed the rig's dial; offsets pushed: {last:?}"
    );
    assert_eq!(
        rig.lock().unwrap().offsets.iter().filter(|o| **o == 0.0).count(),
        1,
        "the centre never moved, so the engine must not keep re-seating the DDC on it"
    );

    drop(h);
    let _ = thread.map(|t| t.join());
}
