//! What a chrono-driven transmit client actually gets on the air.
//!
//! WSJT-X on TCI does not stream transmit audio on its own clock: it answers
//! `TxChrono` packets, one audio packet per chrono, of exactly the size the
//! chrono asked for. That is what a real ExpertSDR3 rig asks for and what this
//! crate's own client does when a rig chronos (`NetThread::answer_chrono`).
//!
//! So the server's chronos *are* the client's transmit clock, and the engine
//! must not ask for more audio than it can take. It issues one chrono per
//! 10 ms transmit block; a client whose audio callback runs on a coarser timer
//! answers a whole run of them at once, and every chrono the engine sent while
//! the first was still in flight is audio it has nowhere to put. Dropping it is
//! not a lost buffer of microphone — it is the client's *waveform* jumping
//! forward, which is why a 15 s FT8 slot went out as a few seconds of signal
//! (issue #202).
//!
//! The client here sends a plain counter as its audio, so the engine can check
//! that what it transmits is the client's stream entire, with nothing skipped.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use sdroxide_tci::server::{TciServerController, TciStateSnapshot};
use sdroxide_types::{DeviceCaps, Mode, TciServerConfig};

/// One engine transmit block: 10 ms at 48 kHz.
const BLOCK: usize = 480;
const RATE: usize = 48_000;
/// How far ahead of the transmitter the engine asks a client to run
/// (`TCI_TX_LEAD`).
const LEAD: usize = BLOCK * 24;
/// Blocks of silence before an unanswered chrono is written off
/// (`TCI_TX_ASK_TIMEOUT_BLOCKS`).
const ASK_TIMEOUT_BLOCKS: u32 = 50;
/// The engine's queue bound (`TCI_TX_FIFO_CAP`), 0.5 s.
const FIFO_CAP: usize = 24_000;
/// How often the client's audio callback runs — a GUI application's timer, not
/// the engine's block clock.
const CLIENT_TICK: Duration = Duration::from_millis(250);

fn caps() -> DeviceCaps {
    DeviceCaps {
        tx_channels: 1,
        freq_ranges_rx: vec![(100_000.0, 60_000_000.0)],
        freq_ranges_tx: vec![(100_000.0, 60_000_000.0)],
        ..DeviceCaps::default()
    }
}

fn snapshot() -> TciStateSnapshot {
    TciStateSnapshot {
        vfo_a_hz: 14_074_000.0,
        vfo_b_hz: 14_074_000.0,
        center_hz: 14_074_000.0,
        if_span_hz: 48_000.0,
        mode: Mode::Digu,
        drive_pct: 40,
        iq_rate: 96_000,
        vfo_lo_hz: 100_000.0,
        vfo_hi_hz: 60_000_000.0,
        can_tx: true,
        ..TciStateSnapshot::default()
    }
}

/// The client: keys up, then answers chronos — but only when its own timer
/// fires, so several chronos are outstanding at once, exactly as they are
/// against a real application. Its audio is a running sample counter.
fn chrono_client(port: u16, sent: Arc<AtomicUsize>, stop: Arc<AtomicBool>) {
    let addr = format!("127.0.0.1:{port}");
    let stream = std::net::TcpStream::connect(&addr).expect("tcp connect");
    stream.set_nodelay(true).unwrap();
    let url = format!("ws://{addr}/");
    // Blocking for the handshake; the poll timeout goes on afterwards, so a
    // slow accept cannot come back as `HandshakeError::Interrupted`.
    let (mut ws, _) = tungstenite::client(url.as_str(), stream).expect("ws handshake");
    ws.get_ref().set_read_timeout(Some(Duration::from_millis(5))).unwrap();
    ws.send(tungstenite::Message::Text("trx:0,true,tci;".into())).expect("key up");

    let mut owed = 0usize; // frames asked for and not yet answered
    let mut next = 0.0f32; // the client's own waveform position
    let mut due = Instant::now() + CLIENT_TICK;
    while !stop.load(Ordering::Relaxed) {
        match ws.read() {
            Ok(tungstenite::Message::Binary(b)) => {
                if let Some(h) = sdroxide_tci::protocol::parse_header(&b)
                    && h.dtype == sdroxide_tci::protocol::DataType::TxChrono
                {
                    // `length` counts floats across both channels.
                    owed += h.length as usize / 2;
                }
            }
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => return,
        }
        if Instant::now() < due || owed == 0 {
            continue;
        }
        due += CLIENT_TICK;
        let mono: Vec<f32> = (0..owed)
            .map(|i| {
                let v = next + i as f32;
                v
            })
            .collect();
        next += owed as f32;
        sent.fetch_add(owed, Ordering::Relaxed);
        owed = 0;
        let pkt = sdroxide_tci::protocol::build_tx_audio(48_000, 0, &mono);
        if ws.send(tungstenite::Message::Binary(pkt.into())).is_err() {
            return;
        }
    }
}

#[test]
fn a_chrono_driven_client_is_never_asked_for_more_than_fits() {
    let port = 50061;
    let cfg = TciServerConfig { port, ..TciServerConfig::default() };
    let Ok(mut srv) = TciServerController::start(&cfg, &caps(), snapshot()) else {
        eprintln!("skip: cannot bind 127.0.0.1:{port}");
        return;
    };

    let sent = Arc::new(AtomicUsize::new(0));
    let stop = Arc::new(AtomicBool::new(false));
    let (c_sent, c_stop) = (Arc::clone(&sent), Arc::clone(&stop));
    let client = std::thread::spawn(move || chrono_client(port, c_sent, c_stop));

    let deadline = Instant::now() + Duration::from_secs(3);
    while !srv.tx_keyed() && Instant::now() < deadline {
        let _ = srv.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(srv.tx_keyed(), "the client never got the key");

    // The engine: `Engine::fill_tx_audio_fifo` in miniature — one 10 ms block
    // at a time, paced to real time, asking for what would restore the standing
    // depth.
    let mut fifo: Vec<f32> = Vec::new();
    let mut block = [0.0f32; BLOCK];
    let mut asked_total = 0usize;
    let mut supplied = 0usize;
    let mut consumed = 0usize;
    let mut quiet = 0u32;
    let mut played: Vec<f32> = Vec::new();
    let mut padded = 0usize;
    let start = Instant::now();
    let blocks = 300; // 3 s
    for i in 0..blocks {
        let mut got = 0usize;
        loop {
            let n = srv.read_tx_audio(&mut block);
            fifo.extend_from_slice(&block[..n]);
            got += n;
            if n < BLOCK {
                break;
            }
        }
        supplied += got;
        consumed += BLOCK;
        quiet = if got > 0 { 0 } else { quiet + 1 };
        if quiet >= ASK_TIMEOUT_BLOCKS {
            quiet = 0;
            asked_total = supplied;
        }
        if fifo.len() < LEAD * 2 {
            let deficit = (consumed + LEAD).saturating_sub(asked_total);
            if deficit >= BLOCK {
                srv.request_chrono(deficit as u32);
                asked_total += deficit;
            }
        }
        if fifo.len() > FIFO_CAP {
            let cut = fifo.len() - FIFO_CAP;
            fifo.drain(..cut);
        }
        let take = fifo.len().min(BLOCK);
        played.extend_from_slice(&fifo[..take]);
        fifo.drain(..take);
        if take < BLOCK {
            padded += BLOCK - take;
        }
        let due = Duration::from_secs_f64((i + 1) as f64 * BLOCK as f64 / RATE as f64);
        if let Some(d) = due.checked_sub(start.elapsed()) {
            std::thread::sleep(d);
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = client.join();

    let asked = sent.load(Ordering::Relaxed);
    eprintln!(
        "over {:.2}s: client sent {asked} frames, {} reached the air, {padded} silent; \
         asked for {asked_total}, supplied {supplied}",
        start.elapsed().as_secs_f64(),
        played.len(),
    );

    // Nothing of the client's stream may be skipped: its samples are a counter,
    // so a jump is audio thrown away — the client's waveform running ahead of
    // real time, which on a timed digital mode is the whole over lost.
    for w in played.windows(2) {
        assert_eq!(
            w[1],
            w[0] + 1.0,
            "the client's stream jumped from {} to {} — {} frames of its waveform were \
             thrown away",
            w[0],
            w[1],
            w[1] - w[0] - 1.0,
        );
    }
    // …and once the queue has found this client's cadence, the over is fed.
    // The first fifth of the run is the learning, so judge the rest of it.
    let wanted = blocks * BLOCK;
    assert!(
        padded * 5 < wanted,
        "{padded} of {wanted} frames went out as silence — the queue never caught up \
         with the client"
    );
}
