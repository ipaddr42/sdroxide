//! Fetching and caching the public-SDR directories.
//!
//! The parsing is [`sdroxide_types::publicsdr`]'s — pure, wasm-safe and tested
//! against checked-in samples. This half is the part that touches the network
//! and the disk, and it is modelled on [`crate::fetch_broadcast_schedule`],
//! including its central rule: a download replaces the cache only after it
//! parses into something plausible, so a captive portal's login page never
//! overwrites a good list.
//!
//! Both directories are fetched at once, on two threads, because they are
//! independent and because this is called from the probe worker — whose caller
//! gives up after twenty seconds. One slow source must not cost the other its
//! answer.

use std::path::PathBuf;
use std::time::Duration;

use serde::{Deserialize, Serialize};

use sdroxide_types::publicsdr::{self, PublicSdrDirectory, PublicSdrEntry, PublicSdrNetwork};

use crate::{ConfigError, config_dir, now_unix};

/// Airspy's own registry. Servers register themselves with it, so it is
/// machine-written and stable.
const SPYSERVER_URL: &str = "https://airspy.com/directory/status.json";

/// The `rx.kiwisdr.com` listing, through the mirror that publishes it as a
/// file rather than behind a bot captcha. Generated for a map viewer, which is
/// why it arrives as JavaScript rather than as JSON.
const KIWISDR_URL: &str = "http://rx.linkfanel.net/kiwisdr_com.js";

/// How long a fetched list is served without going back to the network.
///
/// The only field that moves faster than this is the user count, and a count
/// that is a few minutes old is still the difference between "worth trying" and
/// "full". A refresh is always one button away.
const TTL_S: i64 = 15 * 60;

/// Fewest entries a download must yield to be believed.
///
/// Both directories carry hundreds. Anything in single figures is a truncated
/// transfer or an error page that happened to parse, and the cached copy is
/// better than that. Same reasoning as `MIN_SCHEDULE_ROWS`.
const MIN_ENTRIES: usize = 20;

/// The KiwiSDR listing was 907 KB when this was written and grows with the
/// network; Airspy's was 173 KB.
const MAX_BODY: u64 = 8 * 1024 * 1024;

/// Long enough for a large file over a slow link, short enough that both
/// sources plus the caller's own overhead fit inside the twenty seconds the
/// settings UI waits for a probe answer.
const TIMEOUT: Duration = Duration::from_secs(12);

/// Where the parsed lists are kept between runs.
fn cache_dir() -> Result<PathBuf, ConfigError> {
    let dir = config_dir()?.join("publicsdr");
    std::fs::create_dir_all(&dir)?;
    Ok(dir)
}

/// One source's cached list. The *parsed* entries rather than the payload:
/// a tenth the size, and it means a start-up costs no parsing at all.
#[derive(Serialize, Deserialize)]
struct Cached {
    fetched_unix: i64,
    entries: Vec<PublicSdrEntry>,
}

/// A source, in the one shape the code below needs.
struct Source {
    network: PublicSdrNetwork,
    url: &'static str,
    file: &'static str,
    parse: fn(&str) -> Result<Vec<PublicSdrEntry>, String>,
}

const SOURCES: [Source; 2] = [
    Source {
        network: PublicSdrNetwork::SpyServer,
        url: SPYSERVER_URL,
        file: "spyserver.json",
        parse: publicsdr::parse_spyserver_directory,
    },
    Source {
        network: PublicSdrNetwork::KiwiSdr,
        url: KIWISDR_URL,
        file: "kiwisdr.json",
        parse: publicsdr::parse_kiwisdr_directory,
    },
];

/// Both directories, from cache when it is fresh enough and from the network
/// otherwise.
///
/// Blocking — the caller is the probe worker, which exists for exactly this.
/// Never fails as a whole: a source that cannot be reached contributes its last
/// known list and a line in [`PublicSdrDirectory::notes`], because a receiver
/// list that is an hour old is worth far more than an error message.
pub fn public_sdr_directory(refresh: bool) -> PublicSdrDirectory {
    let results: Vec<(Vec<PublicSdrEntry>, i64, bool, Option<String>)> =
        std::thread::scope(|scope| {
            let handles: Vec<_> =
                SOURCES.iter().map(|s| scope.spawn(move || one_source(s, refresh))).collect();
            handles
                .into_iter()
                .map(|h| {
                    h.join().unwrap_or_else(|_| {
                        (Vec::new(), 0, false, Some("directory worker panicked".into()))
                    })
                })
                .collect()
        });

    let mut out = PublicSdrDirectory { stale: true, ..Default::default() };
    for (entries, fetched, from_network, note) in results {
        out.entries.extend(entries);
        out.fetched_unix = out.fetched_unix.max(fetched);
        if from_network {
            out.stale = false;
        }
        if let Some(n) = note {
            out.notes.push(n);
        }
    }
    // Steadiest order there is: the receiver an operator saw yesterday is in
    // the same place today, whatever the user counts have done.
    out.entries.sort_by(|a, b| {
        (a.network as u8, a.name.to_lowercase(), a.address.clone()).cmp(&(
            b.network as u8,
            b.name.to_lowercase(),
            b.address.clone(),
        ))
    });
    out
}

/// One source: its entries, when they were fetched, whether that was just now,
/// and anything the operator should be told.
fn one_source(src: &Source, refresh: bool) -> (Vec<PublicSdrEntry>, i64, bool, Option<String>) {
    let cached = load_cache(src);
    if !refresh
        && let Some(c) = &cached
        && now_unix().saturating_sub(c.fetched_unix) < TTL_S
    {
        return (c.entries.clone(), c.fetched_unix, false, None);
    }

    match download(src) {
        Ok(entries) => {
            let now = now_unix();
            store_cache(src, &Cached { fetched_unix: now, entries: entries.clone() });
            (entries, now, true, None)
        }
        Err(e) => {
            let note = format!("{} directory unavailable: {e}", src.network.label());
            match cached {
                Some(c) => (
                    c.entries,
                    c.fetched_unix,
                    false,
                    Some(format!("{note} — showing the last list")),
                ),
                None => (Vec::new(), 0, false, Some(note)),
            }
        }
    }
}

/// Fetch and parse, refusing anything too small to be a directory.
fn download(src: &Source) -> Result<Vec<PublicSdrEntry>, String> {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_connect(Some(Duration::from_secs(6)))
        .timeout_global(Some(TIMEOUT))
        .user_agent(concat!("sdroxide/", env!("CARGO_PKG_VERSION")))
        .build()
        .into();

    let mut resp = agent.get(src.url).call().map_err(|e| e.to_string())?;
    if !resp.status().is_success() {
        return Err(format!("HTTP {}", resp.status()));
    }
    // Bytes, then UTF-8 by assertion, rather than `read_to_string`. Both files
    // are UTF-8 and neither says so: the KiwiSDR listing is served as
    // `application/javascript` with no charset at all, and the rule for a
    // missing one is ISO-8859-1. Nothing goes wrong today — ureq's `charset`
    // feature is off, so it does no conversion either way — but a good few
    // receivers are named in emoji or in a non-Latin script, and this way that
    // stays true whoever turns a feature on.
    let bytes =
        resp.body_mut().with_config().limit(MAX_BODY).read_to_vec().map_err(|e| e.to_string())?;
    let body = String::from_utf8_lossy(&bytes);

    let entries = (src.parse)(&body)?;
    if entries.len() < MIN_ENTRIES {
        return Err(format!(
            "{} yielded {} receivers, which is not a directory",
            src.url,
            entries.len()
        ));
    }
    Ok(entries)
}

fn load_cache(src: &Source) -> Option<Cached> {
    let path = cache_dir().ok()?.join(src.file);
    let text = std::fs::read_to_string(&path).ok()?;
    match serde_json::from_str::<Cached>(&text) {
        Ok(c) => Some(c),
        Err(e) => {
            // A cache written by an older build whose entry layout has since
            // moved. Not worth reporting to the operator: the next line
            // re-fetches it.
            tracing::debug!("discarding {} cache: {e}", src.network.label());
            None
        }
    }
}

fn store_cache(src: &Source, c: &Cached) {
    let Ok(dir) = cache_dir() else { return };
    let path = dir.join(src.file);
    match serde_json::to_string(c)
        .map_err(|e| e.to_string())
        .and_then(|t| std::fs::write(&path, t).map_err(|e| e.to_string()))
    {
        Ok(()) => {}
        // Freshness, never functionality: an unwritable cache costs a fetch
        // next time and nothing else.
        Err(e) => tracing::warn!("could not cache the {} directory: {e}", src.network.label()),
    }
}
