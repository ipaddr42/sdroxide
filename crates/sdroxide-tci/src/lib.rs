//! Native TCI (Transceiver Control Interface) over WebSocket — both halves.
//!
//! NATIVE ONLY. Pure-Rust WebSocket (tungstenite); this crate must never be a
//! dependency of any wasm-targeted crate.
//!
//! [`TciDevice`] / [`TciRx`] are the **client**: sdroxide as the operator of
//! somebody else's rig (ExpertSDR3, Thetis). One connection per rig, one
//! stream per receiver — a SunSDR2DX serves two radios from one WebSocket.
//! Receive is wideband IQ (sdroxide demodulates); transmit is audio (the rig
//! modulates the audio we send) and belongs to receiver 0's stream. Reached
//! only from the root binary; the settings UI talks to it exclusively through
//! the `RadioController` trait. [`TciHandle`] is the single-receiver
//! convenience the tests and the live example drive.
//!
//! [`server`] is the **server**: sdroxide as the rig, driven by third-party TCI
//! clients (WSJT-X, JTDX, MSHV, skimmers). It is owned by the DSP engine, which
//! feeds it IQ and RX audio and drains its client requests.
//!
//! [`protocol`] holds the wire format both halves share.

mod net;
pub mod protocol;
pub mod server;

use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

use tungstenite::{HandshakeError, Message, WebSocket};

pub use net::{TX_RATE_HZ, TciDevice, TciError, TciHandle, TciRx, TciUpdate};

/// Default TCI port (ExpertSDR3).
pub const DEFAULT_PORT: u16 = 50001;

/// How long the WebSocket upgrade itself may take, before any status is read.
pub(crate) const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);

/// Perform the WebSocket upgrade on an already-connected socket, retrying until
/// `deadline`.
///
/// The socket's read timeout must cover the whole upgrade, which is why every
/// caller widens it to the budget here and only tightens it to its polling
/// interval once the WebSocket exists. A read that expires mid-upgrade is
/// `Interrupted` — the server simply hasn't answered yet — on Unix alone;
/// Windows reports the same expiry as `WSAETIMEDOUT`, which tungstenite cannot
/// tell from a real I/O failure and will not resume from. Resuming an
/// `Interrupted` handshake is still what makes connecting to a busy ExpertSDR3
/// reliable, and taking it as fatal is why a freshly started sdroxide so often
/// needed a manual "Apply / reconnect" to attach to a rig that was running all
/// along — but on Windows only the wide timeout keeps that from happening
/// (see the KiwiSDR client, issue #266, where the same shape broke outright).
pub(crate) fn ws_handshake(
    stream: TcpStream,
    url: &str,
    deadline: Instant,
) -> Result<WebSocket<TcpStream>, String> {
    let mut attempt = tungstenite::client(url, stream);
    loop {
        match attempt {
            Ok((ws, _response)) => return Ok(ws),
            Err(HandshakeError::Interrupted(mid)) => {
                if Instant::now() > deadline {
                    return Err("timed out during the WebSocket handshake".into());
                }
                attempt = mid.handshake();
            }
            Err(e) => return Err(format!("WebSocket handshake failed: {e}")),
        }
    }
}

/// Split a `host[:port]` (or `ws://host:port`) address into `(host, port)`,
/// defaulting the port to [`DEFAULT_PORT`].
pub(crate) fn split_addr(address: &str) -> Result<(String, u16), String> {
    let a = address.trim();
    let a = a.strip_prefix("ws://").or_else(|| a.strip_prefix("wss://")).unwrap_or(a);
    let a = a.trim_end_matches('/');
    match a.rsplit_once(':') {
        Some((host, port)) if !host.is_empty() => {
            let port: u16 = port.parse().map_err(|_| format!("invalid port in {address:?}"))?;
            Ok((host.to_string(), port))
        }
        _ => Ok((a.to_string(), DEFAULT_PORT)),
    }
}

/// Test a TCI server: connect, read the status burst until `ready;` (or
/// `timeout`), and return a one-line summary (device / protocol / IQ rates) or
/// an error message.
pub fn test_connection(address: &str, timeout: Duration) -> Result<String, String> {
    let (host, port) = split_addr(address)?;
    let sockaddr = (host.as_str(), port)
        .to_socket_addrs()
        .map_err(|e| format!("resolve {host}:{port}: {e}"))?
        .next()
        .ok_or_else(|| format!("no address for {host}:{port}"))?;
    let stream = TcpStream::connect_timeout(&sockaddr, timeout.min(Duration::from_secs(3)))
        .map_err(|e| format!("connect {host}:{port}: {e}"))?;

    let url = format!("ws://{host}:{port}/");
    let deadline = Instant::now() + timeout;
    let upgrade_by = deadline.min(Instant::now() + HANDSHAKE_TIMEOUT);
    // The upgrade's whole budget, in one read: see `ws_handshake`.
    stream.set_read_timeout(Some(timeout.min(HANDSHAKE_TIMEOUT))).ok();
    let mut ws = ws_handshake(stream, url.as_str(), upgrade_by)?;
    // Status polling from here on: don't sit in a blocking read.
    ws.get_ref().set_read_timeout(Some(Duration::from_millis(250))).ok();

    let mut device = String::new();
    let mut protocol = String::new();
    let mut iq_rate = String::new();
    loop {
        if Instant::now() > deadline {
            let _ = ws.close(None);
            return Err("timed out waiting for 'ready;'".into());
        }
        match ws.read() {
            Ok(Message::Text(t)) => {
                for (cmd, args) in protocol::parse_status(t.as_str()) {
                    match cmd.as_str() {
                        "device" => device = args,
                        "protocol" => protocol = args,
                        "iq_samplerate" if iq_rate.is_empty() => iq_rate = args,
                        "ready" => {
                            let _ = ws.close(None);
                            let mut s =
                                if device.is_empty() { "connected".to_string() } else { device };
                            if !protocol.is_empty() {
                                s = format!("{s}  [{protocol}]");
                            }
                            if !iq_rate.is_empty() {
                                s = format!("{s}, IQ {iq_rate} Hz");
                            }
                            return Ok(s);
                        }
                        _ => {}
                    }
                }
            }
            Ok(Message::Close(_)) => return Err("server closed the connection".into()),
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(format!("read error: {e}")),
        }
    }
}
