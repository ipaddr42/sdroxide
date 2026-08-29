//! Tuning to a station that is *already* transmitting must decode.
//!
//! The keyboard-mode magnitude squelch seeds its noise floor from the first
//! block it sees. Switch to RTTY while a strong signal is on the air and the
//! "floor" is seeded at the signal's own level, so `mag > floor * ratio` is
//! never true and the gate stays shut for as long as the signal stays steady —
//! the decoder works perfectly and not a character reaches the screen. Tuning
//! to a continuous broadcast is exactly how this happens (issue #23 follow-up).

use sdroxide_digi::DigiEngine;
use sdroxide_digi::text_modem::TextModemController;
use sdroxide_dsp::RttyTx;
use sdroxide_types::{DigiConfig, Mode};

const RATE: f64 = 8000.0;

/// The seeding trap itself, driven directly with a steady magnitude.
#[test]
fn squelch_seeded_on_signal_never_opens() {
    let mut sq = sdroxide_digi::squelch::Squelch::new();
    let mut opened = false;
    for _ in 0..500 {
        // A perfectly steady signal, present from the very first call.
        opened |= sq.open(0.25, 0.35);
    }
    assert!(
        !opened,
        "squelch now opens on a steady already-present signal — if that is \
         deliberate, the self-gating exemption in TextModemController can go"
    );
}

/// The behaviour that actually matters: the text reaches the screen.
#[test]
fn rtty_decodes_signal_present_from_first_block() {
    let msg = "CQ CQ CQ DE DDK9 DDK9 FREQUENCIES 4583 KHZ 7646 KHZ 10100.8 KHZ ";
    let cfg = DigiConfig {
        rtty_baud: 50.0,
        rtty_shift_hz: 450.0,
        // The default, and the value that used to latch the gate shut.
        digi_squelch: 0.35,
        ..Default::default()
    };

    // tap_rate == the modem's internal rate, so no resampling is involved.
    let mut ctl = TextModemController::new(Mode::Rtty, cfg, RATE);

    // Transmit on the pair the receiver is actually listening to, read off the
    // controller rather than written out here: RTTY's centre is a standard, and
    // when it moved to 2125/2295 Hz a literal left this generator a kilohertz
    // low — which decodes as nothing at all and reads as the squelch latching
    // shut again, the very bug this test guards.
    let mut tx = RttyTx::new(RATE, ctl.audio_hz() as f64, 50.0, 450.0);
    tx.push_text(msg);
    let mut audio = Vec::new();
    let mut guard = 0;
    while tx.sent_chars() < tx.total_chars() && guard < 20_000 {
        let mut b = [0.0f32; 2000];
        tx.next_block(&mut b);
        audio.extend_from_slice(&b);
        guard += 1;
    }

    for chunk in audio.chunks(960) {
        ctl.on_rx_audio(chunk);
    }
    let text = ctl.status().text_rx;
    assert!(
        text.contains("DDK9") && text.contains("10100.8"),
        "signal present from the first block did not reach the screen: {text:?}"
    );
}
