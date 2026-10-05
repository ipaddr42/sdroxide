//! The panadapter's CTR must never park the hardware LO on the VFO.
//!
//! The report this comes from: an AM broadcast distorted "as if tuned 5 kHz
//! off" the moment CTR was clicked, cleared by any mode change, and clean
//! again once CTR was off. CTR keeps the window centred on the dial and asks
//! for that centre with `SetCenter`; taken literally it puts the LO exactly on
//! the VFO. A zero-IF front end has a DC spike there, and the carrier-centred
//! modes have passbands that contain DC — AM's is ±5 kHz — so the spike landed
//! in the demodulated channel and beat against the carrier. SSB and CW never
//! showed it because their passbands start a few hundred hertz up.
//!
//! `lo_guard_hz` already existed to keep the VFO away from the LO; the
//! `SetCenter` path was the one that went around it.
//!
//! FM HD Radio is the exception, and the last tests here hold it to that: its
//! digital sidebands sit well clear of the carrier, a DC spike on the carrier
//! was measured to cost it nothing, and a guard sized from its 744 kHz channel
//! left CTR 400 kHz short of the dial.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;
use sdroxide_radio::{AudioParams, Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RxId, Vfo};

const RATE: f64 = 2_000_000.0;
const VFO: f64 = 13_720_000.0;
/// What a zero-IF front end asks for: its LO parked this far off the VFO.
const LO_OFFSET: f64 = 500_000.0;
/// What the guard is actually for: DC has to land outside the demodulated
/// channel, and AM's passband is ±5 kHz. Asserted as a range rather than the
/// exact figure because `lo_guard_hz` is 0.6 of the *channel rate*, which the
/// mode and the front end's rate between them decide — the property is what
/// matters, not the arithmetic.
const MIN_CLEARANCE: f64 = 10_000.0;
/// ...and CTR is still centring the window, so the LO must not be thrown back
/// out to the full `lo_offset_hz` either.
const MAX_CLEARANCE: f64 = 60_000.0;

/// A zero-IF front end that takes every tune and remembers where it was sent.
struct ZeroIf {
    center_hz: f64,
    landed: Arc<Mutex<f64>>,
}

impl IqSource for ZeroIf {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center_hz
    }
    fn lo_offset_hz(&self) -> f64 {
        LO_OFFSET
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center_hz = hz;
        *self.landed.lock().unwrap() = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(256);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "zero-IF test front end".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "test".into(),
        label: "test".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 200_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// The centre the engine settled on, after letting the commands run.
fn settled_center(rx: &Receiver<RadioEvent>, secs: f64) -> f64 {
    let mut center = f64::NAN;
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    while Instant::now() < deadline {
        while let Ok(ev) = rx.try_recv() {
            if let RadioEvent::State(s) = ev {
                center = s.center_hz;
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    center
}

/// Drive an AM receiver on `VFO`, ask for `want` as the hardware centre, and
/// report where the engine and the front end each ended up.
fn ask_for_center(want: f64) -> (f64, f64) {
    ask_for_center_in(Mode::Am, VFO, want)
}

/// The same for any mode and dial. The front end starts `LO_OFFSET` above the
/// dial, where a zero-IF receiver parks it on an ordinary retune.
fn ask_for_center_in(mode: Mode, dial: f64, want: f64) -> (f64, f64) {
    let start = dial + LO_OFFSET;
    let landed = Arc::new(Mutex::new(start));
    let source = ZeroIf { center_hz: start, landed: Arc::clone(&landed) };
    // A ring nothing reads, because without somewhere to play audio the engine
    // never builds the main receive chain — and `lo_guard_hz` then sizes the
    // guard from a 48 kHz fallback instead of the mode's real channel. That
    // happens to be AM's, which is why the tests above passed without one, but
    // it hides FM HD's 744 kHz channel and the 400 kHz guard that came of it.
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    let cfg = EngineConfig {
        remember_session: false,
        audio: Some(AudioParams { producer, out_rate: 48_000.0 }),
        ..Default::default()
    };
    let mut h = start_engine(Box::new(source), caps(), cfg);
    let thread = h.thread.take();

    h.cmd_tx.send(Command::SetMode { rx: RxId::Main, mode }).unwrap();
    h.cmd_tx.send(Command::SetVfo { vfo: Vfo::A, hz: dial }).unwrap();
    let _ = settled_center(&h.event_rx, 0.4);
    h.cmd_tx.send(Command::SetCenter(want)).unwrap();
    let center = settled_center(&h.event_rx, 0.6);
    let lo = *landed.lock().unwrap();

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    (center, lo)
}

/// CTR asking for the dial itself is honoured only as far as the guard: the
/// window still moves, the LO stays out of the passband.
#[test]
fn centring_the_window_on_the_dial_keeps_the_lo_off_it() {
    let (center, lo) = ask_for_center(VFO);
    assert!(
        (center - VFO).abs() >= MIN_CLEARANCE,
        "the LO was parked {:.0} Hz from the dial, inside AM's passband",
        (center - VFO).abs()
    );
    assert!(
        (lo - VFO).abs() >= MIN_CLEARANCE,
        "the front end was sent to {lo}, {:.0} Hz from the dial",
        (lo - VFO).abs()
    );
}

/// ...and no further than the guard: CTR is still centring the window, so a
/// request inside it must not be thrown back out to the full LO offset.
#[test]
fn the_window_is_moved_as_near_the_dial_as_the_guard_allows() {
    let (center, _) = ask_for_center(VFO);
    assert!(
        (center - VFO).abs() <= MAX_CLEARANCE,
        "the window was pushed {:.0} Hz from the dial, further than the guard needs",
        (center - VFO).abs()
    );
}

/// A centre outside the guard is nobody's business but the caller's: an
/// ordinary pan of the panadapter goes through untouched.
#[test]
fn a_centre_outside_the_guard_is_honoured_exactly() {
    let want = VFO + 250_000.0;
    let (center, _) = ask_for_center(want);
    assert!((center - want).abs() < 1.0, "asked for {want}, engine settled on {center}");
}

/// FM HD Radio's dial, one of the stations the exemption was measured on.
const FM_HD_DIAL: f64 = 107_300_000.0;
/// HD on AM, where the guard stays.
const AM_HD_DIAL: f64 = 1_650_000.0;

/// In FM HD, CTR puts the LO on the dial — exactly, and it stays there.
///
/// The guard is sized from the DDC channel, and FM HD's is nrsc5's 744 kHz
/// sample rate, so on this front end it came to 400 kHz and CTR left the dial a
/// fifth of the window below centre. Measured on air it protects nothing: the
/// digital carriers sit at +/-129 to +/-198 kHz, the middle is analog FM that
/// nrsc5 does not decode, and MER and CBER were unchanged with the LO on the
/// carrier.
///
/// Waiting for the centre to settle is part of the test, not incidental:
/// `keep_vfo_in_span` reads the same guard on every pass, and if it still
/// applied it would retune the LO straight back off the carrier.
#[test]
fn in_fm_hd_centring_the_window_puts_the_lo_on_the_dial() {
    let (center, lo) = ask_for_center_in(Mode::HdRadio, FM_HD_DIAL, FM_HD_DIAL);
    assert!(
        (center - FM_HD_DIAL).abs() < 1.0,
        "CTR asked for the dial and the window settled {:.0} Hz off it",
        center - FM_HD_DIAL
    );
    assert!(
        (lo - FM_HD_DIAL).abs() < 1.0,
        "the front end was left {:.0} Hz off the dial",
        lo - FM_HD_DIAL
    );
}

/// HD on AM keeps the guard. Its innermost digital carriers sit within a few
/// kHz of the carrier, under the analog audio, so there is somewhere for the
/// spike to land — and nobody has measured it the way FM HD was.
#[test]
fn in_am_hd_centring_the_window_still_keeps_the_lo_off_the_dial() {
    let (center, _) = ask_for_center_in(Mode::HdRadio, AM_HD_DIAL, AM_HD_DIAL);
    let clear = (center - AM_HD_DIAL).abs();
    assert!(clear >= MIN_CLEARANCE, "the LO was parked {clear:.0} Hz from an AM HD carrier");
    assert!(clear <= MAX_CLEARANCE, "the window was pushed {clear:.0} Hz from the dial");
}
