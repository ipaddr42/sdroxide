//! Connecting to a rig that is slow to answer the WebSocket upgrade.
//!
//! Treating a slow answer as a failure is what made a freshly started sdroxide
//! fail to attach to an ExpertSDR3 that was running all along, leaving the
//! operator to open Settings → Radio and press "Apply / reconnect" by hand. Two
//! things keep that from happening: tungstenite reports a read that timed out
//! during the upgrade as `Interrupted` — the server hasn't answered *yet* — and
//! the client resumes rather than giving up; and the socket carries the whole
//! upgrade budget as its read timeout until there is a WebSocket to poll. The
//! second is what makes it work on Windows, where the expiry arrives as
//! `WSAETIMEDOUT` and tungstenite cannot tell it from a real I/O failure.
//!
//! Ports are in the 50081.. range (not the default 50001, which a local
//! ExpertSDR3 or another sdroxide would hold); a test whose port is unavailable
//! skips rather than fails.

use std::io::ErrorKind;
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use sdroxide_tci::TciHandle;
use tungstenite::{Message, WebSocket};

const IQ_RATE: f64 = 96_000.0;

/// A rig taking its time. Well inside the client's upgrade budget, and far past
/// the 20 ms poll it switches to once the socket is a WebSocket — so a budget
/// narrowed back to that poll fails this test on Windows.
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

/// The status burst a rig sends before `ready;`.
fn burst() -> Message {
    Message::Text("device:SunSDR2DX;protocol:ExpertSDR3,1.9;drive:40;tune_drive:10;ready;".into())
}

/// Accept one connection, stall `delay` before answering the upgrade, then send
/// the status burst. Returns the open socket.
fn accept_after(listener: &TcpListener, delay: Duration) -> WebSocket<TcpStream> {
    let (stream, _) = listener.accept().expect("accept");
    std::thread::sleep(delay);
    let mut ws = tungstenite::accept(stream).expect("server handshake");
    ws.send(burst()).expect("send burst");
    ws
}

/// Read and discard whatever the client sends, until it goes away.
fn drain(ws: &mut WebSocket<TcpStream>) {
    loop {
        match ws.read() {
            Ok(Message::Close(_)) | Err(_) => return,
            Ok(_) => {}
        }
    }
}

/// A rig that takes its time over the upgrade still connects.
#[test]
fn slow_upgrade_still_connects() {
    let Some(listener) = bind(50081) else { return };
    let server = std::thread::spawn(move || {
        let mut ws = accept_after(&listener, STALL);
        drain(&mut ws);
    });

    let handle = TciHandle::connect("127.0.0.1:50081", IQ_RATE).expect("connect");
    assert_eq!(handle.device, "SunSDR2DX");
    assert!(handle.is_alive());
    drop(handle);
    server.join().expect("server thread");
}

/// A rig that hangs up — ExpertSDR3 quitting — marks the connection dead, which
/// is the signal the engine reconnects on.
#[test]
fn closed_connection_is_reported_dead() {
    let Some(listener) = bind(50082) else { return };
    let server = std::thread::spawn(move || {
        let mut ws = accept_after(&listener, Duration::ZERO);
        // Let the client finish its start-up commands, then hang up.
        ws.get_ref().set_read_timeout(Some(Duration::from_millis(50))).ok();
        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            match ws.read() {
                Ok(_) => {}
                Err(tungstenite::Error::Io(e))
                    if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
                Err(_) => break,
            }
        }
        let _ = ws.close(None);
        let _ = ws.flush();
    });

    let handle = TciHandle::connect("127.0.0.1:50082", IQ_RATE).expect("connect");
    assert!(handle.is_alive());

    let deadline = Instant::now() + Duration::from_secs(5);
    while handle.is_alive() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(!handle.is_alive(), "the client should notice the rig hung up");
    server.join().expect("server thread");
}
