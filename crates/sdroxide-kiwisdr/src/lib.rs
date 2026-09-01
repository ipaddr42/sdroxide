//! A KiwiSDR client: the protocol the KiwiSDR and the Web-888 speak, and with
//! it the ~900 receivers published on `rx.kiwisdr.com`.
//!
//! # One shape, and it is not the usual one
//!
//! A KiwiSDR does not offer wideband I/Q and never has. What it offers is up to
//! eight *user channels*, each a ~12 kHz slice the receiver demodulates for
//! you — and one of its demodulator modes is `iq`, which hands the slice over
//! as complex baseband instead. That is the receiver this crate opens: a narrow
//! I/Q window that follows the dial.
//!
//! Alongside it, on a second socket, the receiver will send the waterfall its
//! own web client draws: 1024 finished dBm bins across the whole 0–30 MHz, a
//! couple of dozen times a second. That is the band view, and it reaches the
//! engine through `IqSource::wide_spectrum_db` — the same seam the RX-888's
//! full band and an Icom's own scope already use.
//!
//! So the panadapter is 12 kHz wide and honest about it, the strip above it is
//! 30 MHz, and tuning across the strip retunes the far end. The interface this
//! is closest to is [`sdroxide_spyserver`]'s VFO+FFT one, and that is not a
//! coincidence: it is what a receiver on the far side of a network link can
//! actually deliver.
//!
//! # What this crate is not
//!
//! It does not implement the receiver's own demodulators, its IMA-ADPCM audio
//! compression, its compressed waterfall, or its extension channels. sdroxide
//! demodulates its own signal from the I/Q, and each of the compressed forms is
//! refused by name rather than guessed at — a half-rate ADPCM stream decoded as
//! linear bytes would draw a convincing band that was not there.
//!
//! It also does not transmit, and that is not a missing feature. These are
//! other people's antennas.
//!
//! # Being a guest
//!
//! Every public receiver here has a hard channel limit, most have an inactivity
//! timeout, and some have a per-address daily limit. When one of those ends a
//! session it is not a fault — it is the receiver saying no — and the session
//! comes back as [`Error::Refused`], which reports
//! [`Error::is_retryable`] false so nothing reconnects into it. See the note on
//! [`error`].

mod error;
mod handle;
pub mod net;
pub mod proto;

use std::time::Duration;

use sdroxide_types::KiwiConfig;

pub use error::{Error, Result};
pub use handle::{KiwiHandle, KiwiInfo, WaterfallFrame};

impl KiwiHandle {
    /// Connect to a KiwiSDR and start an I/Q stream at `center_hz`.
    ///
    /// `ident` is what this end announces itself as — see
    /// [`KiwiConfig::ident_or`], which resolves the station callsign for it.
    ///
    /// Blocks until the first frame of I/Q has arrived, so a wrong address, a
    /// full receiver or a wrong password comes back as an ordinary error rather
    /// than as a stream that never starts.
    pub fn connect(cfg: &KiwiConfig, ident: &str, center_hz: f64) -> Result<KiwiHandle> {
        net::spawn(cfg, ident, center_hz)
    }
}

/// What a receiver says about itself on its `/status` page.
///
/// The same key=value fields the public listing is built from, read live. Worth
/// having beside the directory rather than instead of it: the listing is a few
/// minutes old and does not cover a private receiver at all.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct KiwiStatus {
    pub name: String,
    pub location: String,
    pub sdr_hw: String,
    pub users: u32,
    pub users_max: u32,
    /// Channels the operator has left open to non-browser clients. sdroxide is
    /// one, so zero here means it will be refused however many are free.
    pub ext_api: u32,
    pub min_hz: f64,
    pub max_hz: f64,
    pub antenna: String,
    pub offline: bool,
}

impl KiwiStatus {
    fn parse(text: &str) -> KiwiStatus {
        let mut s = KiwiStatus::default();
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else { continue };
            let v = v.trim();
            match k.trim() {
                "name" => s.name = v.to_string(),
                "loc" => s.location = v.to_string(),
                "sdr_hw" => s.sdr_hw = v.to_string(),
                "users" => s.users = v.parse().unwrap_or(0),
                "users_max" => s.users_max = v.parse().unwrap_or(0),
                "ext_api" => s.ext_api = v.parse().unwrap_or(0),
                "antenna" => s.antenna = v.to_string(),
                "offline" => s.offline = v == "yes",
                "bands" => {
                    let mut it = v.split('-').map(|n| n.trim().parse::<f64>().unwrap_or(0.0));
                    s.min_hz = it.next().unwrap_or(0.0);
                    s.max_hz = it.next().unwrap_or(0.0);
                }
                _ => {}
            }
        }
        s
    }
}

/// Ask a receiver what it is, over HTTP, without taking one of its channels.
///
/// Deliberately *not* a WebSocket session: opening one to find out whether a
/// receiver is worth opening would take a channel from somebody for the length
/// of the question, and on a busy receiver would be the reason it was full.
/// Its `/status` page answers the same thing for the cost of a GET.
pub fn probe(address: &str, timeout: Duration) -> Result<KiwiStatus> {
    let (host, port) = net::split_addr(address)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_connect(Some(timeout))
        .timeout_global(Some(timeout))
        .user_agent(concat!("sdroxide/", env!("CARGO_PKG_VERSION")))
        .build()
        .into();
    let url = format!("http://{host}:{port}/status");
    let mut resp = agent.get(&url).call().map_err(|e| Error::Net(e.to_string()))?;
    if !resp.status().is_success() {
        return Err(Error::Net(format!("HTTP {} from {url}", resp.status())));
    }
    // Bytes, then UTF-8 by assertion, for the same reason the directory fetch
    // does it: this page declares no charset either, and the name and hardware
    // fields on it are full of emoji.
    let bytes = resp
        .body_mut()
        .with_config()
        .limit(64 * 1024)
        .read_to_vec()
        .map_err(|e| Error::Net(e.to_string()))?;
    let body = String::from_utf8_lossy(&bytes);
    let status = KiwiStatus::parse(&body);
    if status.name.is_empty() && status.users_max == 0 {
        return Err(Error::Proto(format!("{url} did not answer like a KiwiSDR")));
    }
    Ok(status)
}

/// [`probe`] as one line for the settings tab, or the reason it failed.
///
/// Says the two things an operator needs before pressing Apply: whether there
/// is a channel free, and whether this receiver's owner allows non-browser
/// clients at all — which is a refusal sdroxide would otherwise only discover
/// by being turned away.
pub fn test_connection(address: &str, timeout: Duration) -> std::result::Result<String, String> {
    match probe(address, timeout) {
        Ok(s) => {
            let hw = if s.sdr_hw.is_empty() {
                "KiwiSDR".to_string()
            } else {
                s.sdr_hw.chars().take_while(|c| c.is_ascii()).collect::<String>().trim().to_string()
            };
            let access = if s.offline {
                "the operator has taken it offline".to_string()
            } else if s.ext_api == 0 {
                "but its operator has not enabled connections from non-browser apps, \
                 so sdroxide will be refused"
                    .to_string()
            } else if s.users >= s.users_max && s.users_max > 0 {
                "and every channel is in use right now".to_string()
            } else {
                format!("and {} of its {} channels are free", s.users_max - s.users, s.users_max)
            };
            Ok(format!(
                "{hw} — {} ({}), {:.0}–{:.0} kHz; {access}",
                s.name,
                s.location,
                s.min_hz / 1e3,
                s.max_hz / 1e3,
            ))
        }
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The real `/status` body, trimmed — the same fields the public listing is
    /// generated from.
    const STATUS: &str = "status=active\noffline=no\nname=Wessex KiWi V1 - G8JNJ\n\
        sdr_hw=KiwiSDR 1 v1.902 \u{2063} GPS\nbands=50000-30000000\nusers=3\nusers_max=8\n\
        ext_api=1\nloc=South West England, UK\nantenna=Switched\n";

    #[test]
    fn a_status_page_parses() {
        let s = KiwiStatus::parse(STATUS);
        assert_eq!(s.name, "Wessex KiWi V1 - G8JNJ");
        assert_eq!(s.location, "South West England, UK");
        assert_eq!(s.users, 3);
        assert_eq!(s.users_max, 8);
        assert_eq!(s.ext_api, 1);
        assert_eq!(s.min_hz, 50_000.0);
        assert_eq!(s.max_hz, 30_000_000.0);
        assert!(!s.offline);
    }

    #[test]
    fn the_test_line_names_the_refusal_before_it_happens() {
        let closed = STATUS.replace("ext_api=1", "ext_api=0");
        let s = KiwiStatus::parse(&closed);
        assert_eq!(s.ext_api, 0);

        let full = STATUS.replace("users=3", "users=8");
        assert_eq!(KiwiStatus::parse(&full).users, 8);
    }

    #[test]
    fn something_that_is_not_a_kiwi_is_not_taken_for_one() {
        let s = KiwiStatus::parse("<html><body>hello</body></html>");
        assert!(s.name.is_empty() && s.users_max == 0);
    }
}
