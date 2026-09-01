//! Opening a session on a receiver that is slow to answer the WebSocket
//! upgrade — which, over the internet, is every receiver.
//!
//! The socket carries a short read timeout so the streaming loop can serve its
//! control channel between reads, and for a long time it was set before the
//! upgrade rather than after. tungstenite resumes an interrupted handshake for
//! `WouldBlock` alone, and only a Unix reports a receive timeout that way:
//! Windows raises `WSAETIMEDOUT`, which it cannot tell from a real I/O failure
//! and abandons the connection over. Every KiwiSDR more than 20 ms away was
//! therefore unreachable from Windows and only from Windows — issue #266, where
//! it surfaced as `WebSocket handshake failed: IO error: … (os error 10060)`.
//!
//! This test cannot raise that errno on a Unix. What it pins down is the
//! invariant that makes the platforms agree: the socket holds the whole upgrade
//! budget until there *is* a WebSocket, so the ordinary case never depends on a
//! resumed handshake at all.
//!
//! Ports are in the 8173.. range rather than the Kiwi's own 8073, which a local
//! receiver would hold; a test whose port is unavailable skips rather than
//! fails.

use std::io::ErrorKind;
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use sdroxide_kiwisdr::{KiwiHandle, proto};
use sdroxide_types::KiwiConfig;
use tungstenite::{Message, WebSocket};

/// A receiver taking its time over the upgrade. Well inside the client's
/// budget, and far past the 20 ms poll it switches to once the socket is a
/// WebSocket — so a budget narrowed back to that poll fails this on Windows.
const STALL: Duration = Duration::from_millis(700);

fn bind(port: u16) -> Option<TcpListener> {
    match TcpListener::bind(("127.0.0.1", port)) {
        Ok(l) => Some(l),
        Err(e) => {
            eprintln!("skip: cannot bind 127.0.0.1:{port} ({e})");
            None
        }
    }
}

/// The opening burst: what the receiver says about itself, and the `audio_init`
/// that tells the client the stream may be configured.
fn hello() -> Message {
    let mut v = b"MSG".to_vec();
    v.extend_from_slice(
        b"badp=0 version_maj=1 version_min=902 rx_chans=8 wf_cal=0 \
sample_rate=12000.000 center_freq=15000000 bandwidth=30000000 audio_init=0",
    );
    Message::Binary(v.into())
}

/// One `SND` frame of I/Q: stereo and uncompressed, as `mod=iq` always is.
fn snd() -> Message {
    let mut v = b"SND".to_vec();
    v.push(proto::SND_FLAG_STEREO);
    v.extend_from_slice(&0u32.to_le_bytes());
    v.extend_from_slice(&350u16.to_be_bytes());
    v.extend_from_slice(&[0u8; 10]);
    for n in 0..512i16 {
        v.extend_from_slice(&n.to_be_bytes());
        v.extend_from_slice(&(-n).to_be_bytes());
    }
    Message::Binary(v.into())
}

/// Stand in for a receiver: accept, stall, then talk until the client leaves.
///
/// The `SND` frames keep coming because the client waits for the first one as
/// its proof the session is up, and it discards any that arrive before it has
/// sent its own configuration.
fn serve(listener: &TcpListener, stall: Duration) {
    let (stream, _) = listener.accept().expect("accept");
    std::thread::sleep(stall);
    let mut ws: WebSocket<TcpStream> = tungstenite::accept(stream).expect("server handshake");
    ws.get_ref().set_read_timeout(Some(Duration::from_millis(5))).ok();
    if ws.send(hello()).and_then(|()| ws.flush()).is_err() {
        return;
    }

    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if ws.send(snd()).and_then(|()| ws.flush()).is_err() {
            return;
        }
        match ws.read() {
            Ok(Message::Close(_)) => return,
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(_) => return,
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn config(port: u16) -> KiwiConfig {
    KiwiConfig {
        address: format!("127.0.0.1:{port}"),
        // One socket is enough to prove the point, and the waterfall's failure
        // is deliberately not the session's.
        wide_lane: false,
        ..KiwiConfig::default()
    }
}

/// A receiver that takes its time over the upgrade still opens.
#[test]
fn slow_upgrade_still_connects() {
    let Some(listener) = bind(8173) else { return };
    let server = std::thread::spawn(move || serve(&listener, STALL));

    let handle =
        KiwiHandle::connect(&config(8173), "sdroxide", 14_074_000.0).expect("the session opens");
    assert_eq!(handle.info.sample_rate_hz, 12_000.0);
    assert_eq!(handle.info.rx_chans, 8);
    drop(handle);
    server.join().expect("server thread");
}

/// And so does one that answers at once — the ordinary case, kept beside the
/// slow one so a fix for either cannot break the other.
#[test]
fn prompt_upgrade_still_connects() {
    let Some(listener) = bind(8174) else { return };
    let server = std::thread::spawn(move || serve(&listener, Duration::ZERO));

    let handle =
        KiwiHandle::connect(&config(8174), "sdroxide", 14_074_000.0).expect("the session opens");
    assert!(handle.is_alive());
    drop(handle);
    server.join().expect("server thread");
}
