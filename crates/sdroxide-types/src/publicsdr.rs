//! Receivers published on the public internet, and the directories that list
//! them.
//!
//! Two networks, chosen because between them they cover the band and because
//! sdroxide can drive both honestly:
//!
//! * **SpyServer** — Airspy's own server and the re-implementations that speak
//!   its protocol, listed at `airspy.com/directory/status.json`. Full I/Q, and
//!   [`crate::Backend::SpyServer`] already drives it, so the directory is the
//!   only thing that was missing. Mostly VHF/UHF dongles.
//! * **KiwiSDR** — the 0–30 MHz receivers (and the KiwiSDR 2 / Web-888 boards,
//!   which run the same firmware and speak the same protocol), listed by the
//!   `rx.kiwisdr.com` mirror at `rx.linkfanel.net`. A narrow I/Q window that
//!   follows the dial plus a full-band waterfall — the shape
//!   [`crate::Backend::SpyServerVfo`] and the Icom LAN scope already have.
//!
//! Deliberately *not* here: OpenWebRX, which delivers demodulated audio only
//! over a protocol that has diverged between its forks and has no
//! machine-readable directory; and PA3FWM's WebSDR, whose codec is proprietary
//! and whose author asks that third-party clients stay away.
//!
//! Parsing lives here rather than in `sdroxide-config` for the same reason
//! [`crate::broadcast::parse_schedule`] does: it is pure, it is wasm-safe, and
//! it can be tested against a checked-in sample with no network. The fetching
//! and caching are `sdroxide-config`'s.

use serde::{Deserialize, Serialize};

use crate::radio::{Backend, RadioConfig, SpyServerConfig};

/// Which directory an entry came from, which is also which protocol it speaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PublicSdrNetwork {
    SpyServer,
    KiwiSdr,
}

impl PublicSdrNetwork {
    pub const ALL: [PublicSdrNetwork; 2] = [PublicSdrNetwork::SpyServer, PublicSdrNetwork::KiwiSdr];

    pub fn label(self) -> &'static str {
        match self {
            PublicSdrNetwork::SpyServer => "SpyServer",
            PublicSdrNetwork::KiwiSdr => "KiwiSDR",
        }
    }
}

/// One receiver somebody has published, flattened to the fields that decide
/// whether it is worth connecting to.
///
/// The two directories describe their receivers very differently — one is
/// machine-generated JSON from a server that registers itself, the other is a
/// scrape of an operator-filled listing page — so everything here is the
/// intersection, normalised. Anything a network does not report is `0`, empty
/// or `None` rather than invented.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicSdrEntry {
    pub network: PublicSdrNetwork,
    /// What the operator called it.
    pub name: String,
    /// Where it is, as free text. Both directories carry a human sentence and
    /// neither carries a structured place, so this is for reading and for the
    /// search box, never for arithmetic — use `lat`/`lon` for that.
    pub location: String,
    pub antenna: String,
    /// The receiver model, as the far end describes itself.
    pub device: String,
    /// `host:port`, ready to drop into a backend's address field. Always
    /// carries an explicit port: a Kiwi behind `*.proxy.kiwisdr.com` answers on
    /// 80, not on the 8073 a default would supply.
    pub address: String,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
    /// Maidenhead locator where the directory gives one. KiwiSDR only.
    pub grid: String,
    pub min_hz: f64,
    pub max_hz: f64,
    pub users: u32,
    pub max_users: u32,
    /// Channels the operator has left open to non-browser clients — KiwiSDR's
    /// `ext_api`. sdroxide *is* a non-browser client, so a receiver reporting
    /// zero here will refuse it however many channels are otherwise free.
    /// `None` where the network has no such notion.
    pub api_channels: Option<u32>,
    /// Widest I/Q the far end will stream, in Hz. A KiwiSDR's is its fixed
    /// ~12 kHz window; a SpyServer's is whatever its ADC and its ladder allow.
    pub max_iq_rate: f64,
    /// Whether this client would be allowed to tune the receiver rather than
    /// ride whatever slice its owner has it parked on.
    pub full_control: bool,
    /// Session limit in **minutes**, `0` for none. From SpyServer's
    /// `maxSessionDuration`; the unit is Airspy's `spyserver.config` wording
    /// rather than anything the directory states, and has not been checked
    /// against a server that enforces one.
    pub session_limit_min: u32,
    /// The receiver's own noise-floor score, KiwiSDR only. Its scale is the
    /// Kiwi network's, not dB above anything in particular — useful for
    /// ranking receivers against each other and for nothing else.
    pub snr_db: Option<u8>,
}

impl PublicSdrEntry {
    /// Which sdroxide interface drives this receiver.
    pub fn backend(&self, low_bandwidth: bool) -> Backend {
        match self.network {
            // A public SpyServer can be asked for either shape. Wideband with
            // automatic decimation is the default because it gives the
            // panadapter an operator expects; the narrow one is there for a
            // link that cannot carry it.
            PublicSdrNetwork::SpyServer if low_bandwidth => Backend::SpyServerVfo,
            PublicSdrNetwork::SpyServer => Backend::SpyServer,
            // A Kiwi has only the one shape.
            PublicSdrNetwork::KiwiSdr => Backend::KiwiSdr,
        }
    }

    /// This entry folded into a radio configuration, leaving every other block
    /// of `base` alone.
    ///
    /// One function because there are two callers that must not drift: "use
    /// this for the current radio" and "open this as a new radio" have to
    /// produce byte-identical configurations, or the same receiver behaves
    /// differently depending on how it was opened.
    pub fn radio_config(&self, base: &RadioConfig, low_bandwidth: bool) -> RadioConfig {
        let mut cfg = base.clone();
        // Through `set_backend`, not by assigning the field: whatever ranges
        // the operator had stated for the interface this radio is leaving are
        // its own radio's, and have to be parked with it rather than
        // overwritten by the two lines below. That was issue #254 — a public
        // SpyServer's 200-350 MHz left clamped over an IC-9700.
        cfg.set_backend(self.backend(low_bandwidth));
        match self.network {
            PublicSdrNetwork::SpyServer => {
                let block = if low_bandwidth { &mut cfg.spyserver_vfo } else { &mut cfg.spyserver };
                block.address = self.address.clone();
                // Let the program pick the stage nearest the interface's target
                // rate once the server has said what it offers: the directory
                // reports a ceiling, not a ladder, so there is no stage to name
                // from here.
                block.iq_decimation = SpyServerConfig::AUTO_DECIMATION;
            }
            PublicSdrNetwork::KiwiSdr => {
                cfg.kiwi.address = self.address.clone();
                cfg.kiwi.password.clear();
            }
        }
        // The receiver's published range, so the dial and the transmit gate are
        // held to what the far end actually covers. A Kiwi that starts at
        // 10 kHz and one that starts at 1.8 MHz are different radios. Written
        // over whatever this interface last carried rather than merged: it
        // describes the receiver being taken now, not the one taken last week.
        cfg.freq_ranges_rx.clear();
        if self.min_hz < self.max_hz {
            cfg.freq_ranges_rx = vec![(self.min_hz, self.max_hz)];
        }
        // Nothing here transmits: these are other people's antennas.
        cfg.freq_ranges_tx.clear();
        cfg
    }

    /// Why this receiver cannot be used right now, or `None` when it can.
    ///
    /// Shown beside the row rather than used to hide it. An operator looking
    /// for a receiver in a particular part of the world is better served by
    /// "there is one there, and it is full" than by an empty list.
    pub fn blocked_reason(&self) -> Option<String> {
        if self.api_channels == Some(0) {
            return Some("operator has not enabled connections from non-browser apps".into());
        }
        if self.max_users > 0 && self.users >= self.max_users {
            return Some(format!("full — {} of {} channels in use", self.users, self.max_users));
        }
        None
    }

    /// Everything the search box should match on, in one string.
    ///
    /// The range goes in twice, in MHz and in kHz, so both "7" and "7000" find
    /// a 40 m receiver — the same trick [`crate::Spot`]'s haystack plays.
    pub fn haystack(&self) -> String {
        format!(
            "{} {} {} {} {} {} {} {:.3}-{:.3} MHz {:.0}-{:.0} kHz",
            self.network.label(),
            self.name,
            self.location,
            self.antenna,
            self.device,
            self.grid,
            self.address,
            self.min_hz / 1e6,
            self.max_hz / 1e6,
            self.min_hz / 1e3,
            self.max_hz / 1e3,
        )
    }

    /// Whether the receiver covers `hz`, for the "in band" filter.
    pub fn covers(&self, hz: f64) -> bool {
        hz >= self.min_hz && hz <= self.max_hz
    }

    /// The tuning range as one short string for a table cell.
    pub fn range_label(&self) -> String {
        if self.min_hz >= self.max_hz {
            return "—".into();
        }
        let mhz = |hz: f64| {
            if hz < 1e6 { format!("{:.0}k", hz / 1e3) } else { format!("{:.1}M", hz / 1e6) }
        };
        format!("{}–{}", mhz(self.min_hz), mhz(self.max_hz))
    }
}

/// Everything both directories said, and when.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PublicSdrDirectory {
    pub entries: Vec<PublicSdrEntry>,
    /// When this was fetched, unix seconds. `0` when nothing has ever been.
    pub fetched_unix: i64,
    /// Served from cache without going to the network.
    pub stale: bool,
    /// One line per source that failed, so a dead directory is visible instead
    /// of just being an unexplained gap in the list.
    pub notes: Vec<String>,
}

impl PublicSdrDirectory {
    pub fn count(&self, network: PublicSdrNetwork) -> usize {
        self.entries.iter().filter(|e| e.network == network).count()
    }
}

// -------------------------------------------------------------------------
// SpyServer
// -------------------------------------------------------------------------

/// `airspy.com/directory/status.json`, the only field of which we need is the
/// array.
#[derive(Deserialize)]
struct SpyServerFile {
    #[serde(default)]
    servers: Vec<SpyServerRow>,
}

/// One row of Airspy's directory. Every field optional: the directory is
/// written by whatever version of `spyserver` registered itself, and older ones
/// send less.
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct SpyServerRow {
    #[serde(default)]
    streaming_host: String,
    #[serde(default)]
    streaming_port: u16,
    #[serde(default)]
    owner_name: String,
    #[serde(default)]
    general_description: String,
    #[serde(default)]
    device_type: String,
    #[serde(default)]
    antenna_type: String,
    #[serde(default)]
    antenna_location: Option<SpyServerLoc>,
    #[serde(default)]
    minimum_frequency: f64,
    #[serde(default)]
    maximum_frequency: f64,
    /// Spelled with the initialism in caps, which is not what
    /// `rename_all = "camelCase"` would produce.
    #[serde(default, rename = "maximumIQSampleRate")]
    maximum_iq_sample_rate: f64,
    #[serde(default)]
    max_clients: u32,
    #[serde(default)]
    current_client_count: u32,
    #[serde(default)]
    max_session_duration: u32,
    #[serde(default)]
    full_control_allowed: bool,
    #[serde(default)]
    online: bool,
}

#[derive(Deserialize)]
struct SpyServerLoc {
    #[serde(default)]
    lat: f64,
    #[serde(default)]
    long: f64,
}

/// Parse Airspy's directory.
///
/// Offline servers are dropped: the directory keeps them listed with
/// `online: false` and a `lastSeen` age, and a row that cannot be connected to
/// is not a receiver as far as this feature is concerned.
pub fn parse_spyserver_directory(json: &str) -> Result<Vec<PublicSdrEntry>, String> {
    let file: SpyServerFile = serde_json::from_str(json).map_err(|e| e.to_string())?;
    Ok(file
        .servers
        .into_iter()
        .filter(|s| s.online && !s.streaming_host.trim().is_empty() && s.streaming_port != 0)
        .map(|s| {
            // 0,0 in the Gulf of Guinea is the sentinel every registry with an
            // optional coordinate ends up with, and it is never a receiver.
            let (lat, lon) = match &s.antenna_location {
                Some(l) if l.lat != 0.0 || l.long != 0.0 => {
                    (Some(l.lat as f32), Some(l.long as f32))
                }
                _ => (None, None),
            };
            let name = if s.owner_name.trim().is_empty() {
                s.general_description.trim().to_string()
            } else {
                s.owner_name.trim().to_string()
            };
            PublicSdrEntry {
                network: PublicSdrNetwork::SpyServer,
                name,
                location: describe_place(&s.general_description),
                antenna: s.antenna_type.trim().to_string(),
                device: s.device_type.trim().to_string(),
                address: join_host_port(s.streaming_host.trim(), s.streaming_port),
                lat,
                lon,
                grid: String::new(),
                min_hz: s.minimum_frequency,
                max_hz: s.maximum_frequency,
                users: s.current_client_count,
                max_users: s.max_clients,
                api_channels: None,
                max_iq_rate: s.maximum_iq_sample_rate,
                full_control: s.full_control_allowed,
                session_limit_min: s.max_session_duration,
                snr_db: None,
            }
        })
        .collect())
}

// -------------------------------------------------------------------------
// KiwiSDR
// -------------------------------------------------------------------------

/// One row of the KiwiSDR listing. Every value arrives as a string — the file
/// is generated for a JavaScript map viewer, which never needed numbers — and
/// a handful of rows carry `null` where an older receiver reported nothing.
#[derive(Deserialize)]
struct KiwiRow {
    #[serde(default)]
    url: Option<String>,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    loc: Option<String>,
    #[serde(default)]
    antenna: Option<String>,
    #[serde(default)]
    sdr_hw: Option<String>,
    #[serde(default)]
    gps: Option<String>,
    #[serde(default)]
    grid: Option<String>,
    #[serde(default)]
    bands: Option<String>,
    #[serde(default)]
    users: Option<String>,
    #[serde(default)]
    users_max: Option<String>,
    #[serde(default)]
    ext_api: Option<String>,
    #[serde(default)]
    snr: Option<String>,
    #[serde(default)]
    offline: Option<String>,
    #[serde(default)]
    status: Option<String>,
}

/// Parse the KiwiSDR listing.
///
/// The file is not JSON. It is a JavaScript source file for the map viewer it
/// was written for — `var kiwisdr_com =` in front, a `;` behind, and a trailing
/// comma before the closing bracket that `serde_json` rejects outright. So the
/// array is cut out by hand and the trailing commas removed before parsing.
/// That is fragile in exactly one way, and the test below is what holds it: if
/// the wrapper ever changes, the parse fails loudly with the byte offset rather
/// than silently returning nothing.
pub fn parse_kiwisdr_directory(js: &str) -> Result<Vec<PublicSdrEntry>, String> {
    let open = js.find('[').ok_or("no array in the KiwiSDR listing")?;
    let close = js.rfind(']').ok_or("the KiwiSDR listing's array is unterminated")?;
    if close <= open {
        return Err("the KiwiSDR listing's array is empty or malformed".into());
    }
    let array = strip_trailing_commas(&js[open..=close]);
    let rows: Vec<KiwiRow> = serde_json::from_str(&array).map_err(|e| e.to_string())?;

    Ok(rows
        .into_iter()
        .filter(|r| {
            r.offline.as_deref() != Some("yes") && r.status.as_deref().unwrap_or("active") != "down"
        })
        .filter_map(|r| {
            let address = kiwi_address(r.url.as_deref()?)?;
            let (min_hz, max_hz) = kiwi_bands(r.bands.as_deref().unwrap_or_default());
            let (lat, lon) = kiwi_gps(r.gps.as_deref().unwrap_or_default());
            let num = |s: Option<&str>| s.unwrap_or("0").trim().parse::<u32>().unwrap_or(0);
            Some(PublicSdrEntry {
                network: PublicSdrNetwork::KiwiSdr,
                name: r.name.unwrap_or_default().trim().to_string(),
                location: r.loc.unwrap_or_default().trim().to_string(),
                antenna: r.antenna.unwrap_or_default().trim().to_string(),
                device: kiwi_hardware(r.sdr_hw.as_deref().unwrap_or_default()),
                address,
                lat,
                lon,
                grid: r.grid.unwrap_or_default().trim().to_string(),
                min_hz,
                max_hz,
                users: num(r.users.as_deref()),
                max_users: num(r.users_max.as_deref()),
                api_channels: Some(num(r.ext_api.as_deref())),
                // Fixed by the protocol: `SET mod=iq` delivers the receiver's
                // audio channel as I/Q, and that channel is ~12 kHz whatever
                // the board.
                max_iq_rate: KIWI_IQ_RATE_HZ,
                // Every Kiwi channel is the client's own to tune.
                full_control: true,
                session_limit_min: 0,
                // "36,36" — the two figures are the whole band and the ham
                // bands; the first is the one the listing page ranks on.
                snr_db: r
                    .snr
                    .as_deref()
                    .and_then(|s| s.split(',').next())
                    .and_then(|s| s.trim().parse::<u8>().ok()),
            })
        })
        .collect())
}

/// Nominal I/Q rate of a KiwiSDR channel in `iq` mode.
///
/// The receiver reports its own, slightly off, figure at connect time
/// (11998.874997 Hz on the one this was measured against) and that reported
/// value is what the source must resample from. This is only the round number
/// for a directory listing.
pub const KIWI_IQ_RATE_HZ: f64 = 12_000.0;

/// `http://host:8073` → `host:8073`, `http://x.proxy.kiwisdr.com` → `…:80`.
///
/// The port is always made explicit. Nearly half the public Kiwis are reached
/// through the project's own reverse proxy, which answers on 80 — leaving the
/// port off and letting a default supply 8073 would send every one of them to a
/// closed port.
fn kiwi_address(url: &str) -> Option<String> {
    let url = url.trim();
    let (scheme, rest) = match url.split_once("://") {
        Some((s, r)) => (s, r),
        None => ("http", url),
    };
    let host = rest.split(['/', '?', '#']).next()?.trim();
    if host.is_empty() {
        return None;
    }
    let has_port = match host.rfind(']') {
        Some(close) => host[close + 1..].starts_with(':'),
        None => host.contains(':'),
    };
    if has_port {
        Some(host.to_string())
    } else {
        Some(format!("{host}:{}", if scheme.eq_ignore_ascii_case("https") { 443 } else { 80 }))
    }
}

/// `"0-30000000"` → `(0.0, 30_000_000.0)`.
fn kiwi_bands(bands: &str) -> (f64, f64) {
    let mut it = bands.split('-').map(|s| s.trim().parse::<f64>().unwrap_or(0.0));
    let lo = it.next().unwrap_or(0.0);
    let hi = it.next().unwrap_or(0.0);
    if hi > lo { (lo, hi) } else { (0.0, 0.0) }
}

/// `"(50.08, -113.68)"` → `(50.08, -113.68)`.
fn kiwi_gps(gps: &str) -> (Option<f32>, Option<f32>) {
    let inner = gps.trim().trim_start_matches('(').trim_end_matches(')');
    let mut it = inner.split(',').map(|s| s.trim().parse::<f32>().ok());
    let lat = it.next().flatten();
    let lon = it.next().flatten();
    match (lat, lon) {
        (Some(a), Some(b)) if a != 0.0 || b != 0.0 => (Some(a), Some(b)),
        _ => (None, None),
    }
}

/// `"KiwiSDR 2 v1.902 ⁣ 📡 GPS ⁣ ⏳🚫 Limits …"` → `"KiwiSDR 2 v1.902"`.
///
/// The listing packs the receiver's whole feature set into this field as emoji,
/// separated by invisible word-joiners. Only the model and firmware ahead of
/// them are wanted, and they are plain ASCII — which is also what tells the
/// KiwiSDR 1 from the KiwiSDR 2 (the Web-888 board).
fn kiwi_hardware(sdr_hw: &str) -> String {
    sdr_hw
        .chars()
        .take_while(|c| c.is_ascii_alphanumeric() || matches!(c, ' ' | '.' | '-' | '+' | '_'))
        .collect::<String>()
        .trim()
        .to_string()
}

// -------------------------------------------------------------------------
// Shared helpers
// -------------------------------------------------------------------------

/// A free-text place, with the placeholders the field is usually left at
/// removed.
///
/// `spyserver`'s own default configuration ships `generalDescription` as "no
/// description", and a good few operators never change it. Carrying that
/// through means a column of rows all saying nothing where the space could
/// have gone to the distance instead — so an uninformative value becomes an
/// absent one, which the row can then fill with something better.
fn describe_place(raw: &str) -> String {
    let t = raw.trim();
    let folded = t.to_ascii_lowercase();
    let placeholder = t.is_empty()
        || matches!(folded.as_str(), "none" | "n/a" | "na" | "-" | "unknown")
        || folded.starts_with("no description");
    if placeholder { String::new() } else { t.to_string() }
}

/// `host` + port → `host:port`, bracketing a bare IPv6 literal.
fn join_host_port(host: &str, port: u16) -> String {
    if host.contains(':') && !host.starts_with('[') {
        format!("[{host}]:{port}")
    } else {
        format!("{host}:{port}")
    }
}

/// Remove commas that sit directly before a `]` or `}`.
///
/// String literals are skipped over, escapes included, so a comma inside a
/// receiver's name survives. Nothing else about the text is touched: this is
/// the smallest change that makes the KiwiSDR listing parse, not a lenient
/// JSON reader.
///
/// Iterates over *characters*, not bytes. Plenty of operators put emoji and
/// non-Latin scripts in their receiver's name — a fair number of the listings
/// are a row of flag emoji — and walking bytes would rebuild each of those as
/// one Latin-1 character per byte, which is mojibake by the time it reaches a
/// label.
fn strip_trailing_commas(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut in_string = false;
    let mut escaped = false;
    let mut it = s.char_indices().peekable();
    while let Some((i, c)) = it.next() {
        if in_string {
            out.push(c);
            if escaped {
                escaped = false;
            } else if c == '\\' {
                escaped = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        if c == '"' {
            in_string = true;
            out.push('"');
            continue;
        }
        if c == ',' {
            let next = s[i + 1..].chars().find(|c| !c.is_whitespace());
            if matches!(next, Some(']') | Some('}')) {
                continue;
            }
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two rows lifted verbatim from `airspy.com/directory/status.json`, plus
    /// an offline one that must be dropped.
    const SPYSERVER_SAMPLE: &str = r#"{"servers":[
      {"serverVersion":"2.0.1922","operatingSystem":"Microsoft Windows 11 64-bit",
       "maxClients":1,"currentClientCount":0,"currentClients":[],"maxSessionDuration":0,
       "streamingPort":5555,"ownerName":"RTL-SDR V4 Server","ownerEmail":"none",
       "antennaType":"dipole","antennaLocation":{"lat":0,"long":0},
       "generalDescription":"RTL-SDR V4 SpyServer","deviceType":"RTL-SDR","deviceSerial":"0",
       "deviceResolution":8,"fullControlAllowed":true,"currentCenterFrequency":476900000,
       "minimumFrequency":24000000,"maximumFrequency":1800000000,
       "maximumDisplayedBandwidth":1700000,"displayFPS":15,"maximumStreamedBandwidth":1700000,
       "maximumIQSampleRate":2048000,"streamingHost":"185.208.144.72","online":true,
       "registered":false,"lastSeen":8},
      {"serverVersion":"2.0.1922","maxClients":5,"currentClientCount":2,"maxSessionDuration":30,
       "streamingPort":5556,"ownerName":"Russ Gladden, SWL - Ottawa, Canada",
       "antennaType":"discone","antennaLocation":{"lat":45.42,"long":-75.7},
       "generalDescription":"Ottawa wideband","deviceType":"AirspyOne",
       "fullControlAllowed":false,"minimumFrequency":24000000,"maximumFrequency":1700000000,
       "maximumIQSampleRate":10000000,"streamingHost":"135.23.92.149","online":true},
      {"streamingPort":5555,"ownerName":"gone fishing","deviceType":"AirspyHF+",
       "streamingHost":"10.0.0.1","online":false}
    ]}"#;

    #[test]
    fn spyserver_rows_become_entries() {
        let e = parse_spyserver_directory(SPYSERVER_SAMPLE).expect("parses");
        assert_eq!(e.len(), 2, "the offline server must be dropped");

        assert_eq!(e[0].address, "185.208.144.72:5555");
        assert_eq!(e[0].name, "RTL-SDR V4 Server");
        assert_eq!(e[0].location, "RTL-SDR V4 SpyServer");
        assert_eq!(e[0].device, "RTL-SDR");
        assert_eq!(e[0].min_hz, 24e6);
        assert_eq!(e[0].max_hz, 1.8e9);
        assert_eq!(e[0].max_iq_rate, 2_048_000.0);
        assert!(e[0].full_control);
        // 0,0 is the "no location given" sentinel, not the Gulf of Guinea.
        assert_eq!(e[0].lat, None);
        assert_eq!(e[0].lon, None);
        assert_eq!(e[0].api_channels, None, "SpyServer has no per-app channel limit");

        assert_eq!(e[1].address, "135.23.92.149:5556");
        assert_eq!(e[1].lat, Some(45.42));
        assert_eq!(e[1].session_limit_min, 30);
        assert!(!e[1].full_control);
        assert_eq!(e[1].users, 2);
        assert_eq!(e[1].max_users, 5);
    }

    /// Four rows in the shape `rx.linkfanel.net/kiwisdr_com.js` really serves
    /// them: the `var … =` wrapper, the trailing `;`, a trailing comma before
    /// the closing bracket, a proxied receiver with no port, a receiver with
    /// `ext_api` closed, and one that is offline.
    const KIWI_SAMPLE: &str = r#"// KiwiSDR.com receiver list for dyatlov map maker
var kiwisdr_com =
[
	{
		"id":"38d2697cae44","status":"active","offline":"no","name":"ve6ars kiwi #1",
		"sdr_hw":"KiwiSDR 1 v1.902 ⁣ 📡 GPS ⁣ Limits",
		"bands":"0-30000000","users":"0","users_max":"8","ext_api":"4",
		"gps":"(50.08, -113.68)","grid":"DO30db","loc":"Southern Alberta 1",
		"antenna":"160 inv ell BCB reject","snr":"36,34","mode":null,
		"url":"http://porky.dahlbros.net:8073"
	},
	{
		"status":"active","offline":"no","name":"proxied one","sdr_hw":"KiwiSDR 2 v1.902 ⁣",
		"bands":"10000-30000000","users":"3","users_max":"4","ext_api":"0",
		"gps":"(0.00, 0.00)","grid":"","loc":"Somewhere","antenna":"Mini-Whip","snr":"1,1",
		"url":"http://22033.proxy.kiwisdr.com"
	},
	{
		"status":"active","offline":"no","name":"secure one","sdr_hw":"KiwiSDR 2 v1.902",
		"bands":"1800000-30000000","users":"8","users_max":"8","ext_api":"2",
		"gps":"(0.74, -2.63)","loc":"UK","antenna":"Switched","snr":"20,21",
		"url":"https://kiwi.example.org"
	},
	{
		"status":"active","offline":"yes","name":"dark","url":"http://dead.example:8073",
		"bands":"0-30000000","users":"0","users_max":"8","ext_api":"1"
	},
]
;"#;

    #[test]
    fn the_javascript_wrapper_and_trailing_comma_are_handled() {
        let e = parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses");
        assert_eq!(e.len(), 3, "the offline receiver must be dropped");
    }

    #[test]
    fn kiwi_rows_become_entries() {
        let e = parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses");

        assert_eq!(e[0].address, "porky.dahlbros.net:8073");
        assert_eq!(e[0].device, "KiwiSDR 1 v1.902", "the emoji feature list is trimmed off");
        assert_eq!(e[0].min_hz, 0.0);
        assert_eq!(e[0].max_hz, 30e6);
        assert_eq!(e[0].lat, Some(50.08));
        assert_eq!(e[0].grid, "DO30db");
        assert_eq!(e[0].snr_db, Some(36), "the first of the pair is the whole-band figure");
        assert_eq!(e[0].api_channels, Some(4));
        assert_eq!(e[0].max_iq_rate, KIWI_IQ_RATE_HZ);
        assert!(e[0].full_control);

        // A proxied Kiwi answers on 80. Leaving the port off and letting a
        // default supply 8073 sends the connection to a closed port.
        assert_eq!(e[1].address, "22033.proxy.kiwisdr.com:80");
        assert_eq!(e[1].device, "KiwiSDR 2 v1.902");
        assert_eq!(e[1].lat, None, "0,0 is 'not stated'");

        assert_eq!(e[2].address, "kiwi.example.org:443");
    }

    #[test]
    fn a_receiver_says_why_it_cannot_be_used() {
        let e = parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses");
        assert_eq!(e[0].blocked_reason(), None);
        // ext_api=0 is a refusal even though 1 of its 4 channels is free.
        assert!(e[1].blocked_reason().expect("blocked").contains("non-browser"));
        assert!(e[2].blocked_reason().expect("blocked").contains("full"));
    }

    #[test]
    fn a_comma_inside_a_name_survives_the_trailing_comma_strip() {
        let js = r#"var x = [{"name":"Ottawa, Canada,","url":"http://a.example:8073",
                    "offline":"no","bands":"0-30000000","users":"0","users_max":"8",
                    "ext_api":"1"},];"#;
        let e = parse_kiwisdr_directory(js).expect("parses");
        assert_eq!(e[0].name, "Ottawa, Canada,");
    }

    #[test]
    fn a_broken_listing_fails_loudly() {
        assert!(parse_kiwisdr_directory("var kiwisdr_com = ;").is_err());
        assert!(parse_kiwisdr_directory("[{\"url\": }]").is_err());
        assert!(parse_spyserver_directory("<html>captive portal</html>").is_err());
    }

    #[test]
    fn an_entry_configures_the_backend_it_belongs_to() {
        let base = RadioConfig::default();
        let kiwi = &parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses")[0];
        let cfg = kiwi.radio_config(&base, false);
        assert_eq!(cfg.backend, Backend::KiwiSdr);
        assert_eq!(cfg.kiwi.address, "porky.dahlbros.net:8073");
        assert_eq!(cfg.freq_ranges_rx, vec![(0.0, 30e6)]);
        assert!(cfg.freq_ranges_tx.is_empty(), "these are other people's antennas");

        let spy = &parse_spyserver_directory(SPYSERVER_SAMPLE).expect("parses")[0];
        let wide = spy.radio_config(&base, false);
        assert_eq!(wide.backend, Backend::SpyServer);
        assert_eq!(wide.spyserver.address, "185.208.144.72:5555");
        assert_eq!(wide.spyserver.iq_decimation, SpyServerConfig::AUTO_DECIMATION);

        let narrow = spy.radio_config(&base, true);
        assert_eq!(narrow.backend, Backend::SpyServerVfo);
        assert_eq!(narrow.spyserver_vfo.address, "185.208.144.72:5555");
        assert_eq!(
            narrow.spyserver, base.spyserver,
            "the low-bandwidth pick must not also write the wideband block"
        );
    }

    /// Issue #254. Taking a public receiver in the tab a transceiver was in
    /// must not leave the receiver's coverage behind on the transceiver: the
    /// operator switched an IC-9700 tab to a SpyServer, switched it straight
    /// back, and found the dial still held to the SpyServer's 200-350 MHz with
    /// nothing on the page to say why.
    #[test]
    fn taking_a_receiver_leaves_the_transceivers_own_ranges_where_they_were() {
        // A rig on the LAN with the operator's own word for what it covers.
        let mut base = RadioConfig { backend: Backend::IcomNet, ..RadioConfig::default() };
        base.freq_ranges_rx = vec![(144e6, 148e6), (430e6, 450e6)];
        base.freq_ranges_tx = vec![(144e6, 146e6)];

        let kiwi = &parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses")[0];
        let taken = kiwi.radio_config(&base, false);
        assert_eq!(taken.freq_ranges_rx, vec![(0.0, 30e6)], "the Kiwi's own coverage");
        assert!(taken.freq_ranges_tx.is_empty(), "these are other people's antennas");

        // And back again, the way the reporter did it: the interface picker.
        let mut back = taken.clone();
        back.set_backend(Backend::IcomNet);
        assert_eq!(back.freq_ranges_rx, base.freq_ranges_rx, "the Icom's own ranges, restored");
        assert_eq!(back.freq_ranges_tx, base.freq_ranges_tx);
        assert_eq!(back.kiwi.address, taken.kiwi.address, "and the Kiwi is still configured");
    }

    /// Two receivers on the same interface, one after the other: the second
    /// one's coverage replaces the first's rather than being added to it.
    #[test]
    fn a_second_receiver_replaces_the_first_ones_coverage() {
        let entries = parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses");
        let first = entries[0].radio_config(&RadioConfig::default(), false);
        let second = entries[2].radio_config(&first, false);
        assert_eq!(second.freq_ranges_rx.len(), 1, "{:?}", second.freq_ranges_rx);
    }

    #[test]
    fn the_search_haystack_finds_a_band_by_either_unit() {
        let e = &parse_kiwisdr_directory(KIWI_SAMPLE).expect("parses")[0];
        let hay = e.haystack();
        assert!(hay.contains("Southern Alberta"));
        assert!(hay.contains("MHz") && hay.contains("kHz"));
        assert!(hay.contains("KiwiSDR"));
    }

    /// `spyserver` ships "no description" as its default, and plenty of
    /// operators leave it — a whole column of rows saying nothing.
    /// The listing is full of emoji and non-Latin names — a good few
    /// receivers are named with a row of flag emoji — and the array has to be
    /// walked to strip its trailing commas before it will parse. Doing that a
    /// byte at a time rebuilds every one of those as a run of Latin-1
    /// characters, which is mojibake by the time it reaches a label.
    #[test]
    fn a_name_with_emoji_and_non_latin_text_survives_the_strip() {
        let js = "var x = [{\"name\":\"0 - 30 MHz SDR | \u{1f1e9}\u{1f1ea} \u{041c}\u{043e}\u{0441}\u{043a}\u{0432}\u{0430}\",\
                   \"url\":\"http://a.example:8073\",\"offline\":\"no\",\"bands\":\"0-30000000\",\
                   \"users\":\"0\",\"users_max\":\"8\",\"ext_api\":\"1\"},];";
        let e = parse_kiwisdr_directory(js).expect("parses");
        assert_eq!(
            e[0].name,
            "0 - 30 MHz SDR | \u{1f1e9}\u{1f1ea} \u{041c}\u{043e}\u{0441}\u{043a}\u{0432}\u{0430}"
        );
    }

    #[test]
    fn a_placeholder_description_is_treated_as_no_place_at_all() {
        assert_eq!(describe_place("  no description  "), "");
        assert_eq!(describe_place("No Description Set"), "");
        assert_eq!(describe_place("none"), "");
        assert_eq!(describe_place(""), "");
        assert_eq!(describe_place("Ottawa wideband"), "Ottawa wideband");
    }

    #[test]
    fn an_ipv6_spyserver_host_is_bracketed() {
        assert_eq!(join_host_port("2001:db8::1", 5555), "[2001:db8::1]:5555");
        assert_eq!(join_host_port("[2001:db8::1]", 5555), "[2001:db8::1]:5555");
        assert_eq!(join_host_port("example.org", 5555), "example.org:5555");
    }
}
