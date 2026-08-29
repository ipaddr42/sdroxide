//! CLEAR RX empties the receive window and nothing else.
//!
//! The button sits next to controls that key the transmitter, so the property
//! worth pinning is not that the text goes — it is that everything else stays:
//! a page cleared in the middle of an over must not cut the over short, and the
//! decoder must carry on copying afterwards rather than needing to be restarted.

use std::time::SystemTime;

use sdroxide_digi::DigiEngine;
use sdroxide_digi::text_modem::TextModemController;
use sdroxide_dsp::RttyTx;
use sdroxide_types::{DigiConfig, Mode};

const RATE: f64 = 8000.0;

/// Each message carries a digit group early on, on purpose: RTTY's letters /
/// figures shift is stateful and nothing resets it between transmissions, so a
/// spliced pair that never shifts is read in whichever case the decoder started
/// in — and the clear would get the blame for the mojibake.
///
/// `center_hz` comes from the controller under test rather than a literal: RTTY
/// sits on a *standard* tone pair, and when that centre moved to 2125/2295 Hz
/// this generator was left transmitting 1.2 kHz below where the receiver was
/// listening, which reads as "the clear broke the decoder" and is not.
fn rtty_audio(center_hz: f64, msg: &str) -> Vec<f32> {
    let mut tx = RttyTx::new(RATE, center_hz, 50.0, 450.0);
    tx.push_text(msg);
    let mut audio = Vec::new();
    let mut guard = 0;
    while tx.sent_chars() < tx.total_chars() && guard < 20_000 {
        let mut b = [0.0f32; 2000];
        tx.next_block(&mut b);
        audio.extend_from_slice(&b);
        guard += 1;
    }
    audio
}

fn rtty_controller() -> TextModemController {
    let cfg = DigiConfig { rtty_baud: 50.0, rtty_shift_hz: 450.0, ..Default::default() };
    TextModemController::new(Mode::Rtty, cfg, RATE)
}

#[test]
fn clearing_empties_the_window_and_copying_carries_on() {
    let mut ctl = rtty_controller();
    let center = ctl.audio_hz() as f64;
    for chunk in rtty_audio(center, "CQ 599 DE DELTA DELTA ").chunks(960) {
        ctl.on_rx_audio(chunk);
    }
    let copied = ctl.status().text_rx;
    assert!(copied.contains("DELTA"), "nothing was copied to clear: {copied:?}");

    ctl.clear_rx();
    assert!(ctl.status().text_rx.is_empty(), "the window still holds {:?}", ctl.status().text_rx);

    // The receiver was not stood down, only the page torn off.
    for chunk in rtty_audio(center, "TEST 599 DE PAPA PAPA ").chunks(960) {
        ctl.on_rx_audio(chunk);
    }
    let after = ctl.status().text_rx;
    assert!(after.contains("PAPA"), "copying stopped after a clear: {after:?}");
    assert!(!after.contains("DELTA"), "the cleared text came back: {after:?}");
}

/// Pressing it mid-over must not disturb what is going out. `tx_sent` is what
/// the panel greens the already-sent prefix with, so a reset there would make
/// the transmit box appear to re-send from the top.
#[test]
fn clearing_leaves_the_transmit_side_alone() {
    let mut ctl = rtty_controller();
    ctl.set_tx_text("CQ CQ DE W1AW".into());
    ctl.set_tx_active(true);
    ctl.poll(SystemTime::now(), 14_080_000.0);
    let mut block = [0.0f32; 2048];
    ctl.fill_tx_block(&mut block);
    let before = ctl.status();
    assert!(ctl.tx_burst_active(), "the test never got an over going");

    ctl.clear_rx();
    let after = ctl.status();
    assert_eq!(after.tx_sent, before.tx_sent, "the sent-character count moved");
    assert!(after.tx_next, "the over was stood down");
    assert!(ctl.tx_burst_active(), "the burst was aborted");
}
