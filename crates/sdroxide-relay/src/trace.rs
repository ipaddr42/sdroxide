//! A bounded record of what the T/R switch was told, and what it said back.
//!
//! Here for the reason the LimeRFE's trace next door is: on these boards there
//! is no other record of anything, anywhere. A relay board answers nothing, a
//! handshake line answers nothing, and a GPIO pin answers nothing — so a
//! station reporting "the SDR still gets blasted on transmit" has, without
//! this, produced no evidence at all about the one piece of hardware in
//! question.
//!
//! What it records is the *decision and its outcome* rather than the bytes.
//! This driver deduplicates on the resolved contact state, so "nothing was
//! sent" is frequently the correct behaviour and always the confusing one. A
//! report showing the sequence at startup, one key-down per over and the
//! contacts each one set is a report that answers "why did it not switch".

use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

/// How many entries to keep. Enough for the opening configuration, a run of
/// overs and whatever went wrong at the end of them; a long healthy session is
/// not what anybody is reporting.
const CAP: usize = 128;

#[derive(Clone)]
struct Entry {
    at_ms: u128,
    what: String,
    outcome: String,
}

#[derive(Clone)]
pub struct Trace {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    started: Instant,
    link: String,
    entries: std::collections::VecDeque<Entry>,
    dropped: u64,
}

impl Default for Trace {
    fn default() -> Self {
        Self::new()
    }
}

impl Trace {
    pub fn new() -> Trace {
        Trace {
            inner: Arc::new(Mutex::new(Inner {
                started: Instant::now(),
                link: String::new(),
                entries: std::collections::VecDeque::with_capacity(CAP),
                dropped: 0,
            })),
        }
    }

    /// Which cable this switch is on, and what kind it is.
    pub fn set_link(&self, link: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        i.link = link.as_ref().to_string();
    }

    /// Record one decision, or one thing that happened to the link.
    pub fn note(&self, what: impl AsRef<str>, outcome: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let at_ms = i.started.elapsed().as_millis();
        if i.entries.len() == CAP {
            i.entries.pop_front();
            i.dropped += 1;
        }
        i.entries.push_back(Entry {
            at_ms,
            what: what.as_ref().to_string(),
            outcome: outcome.as_ref().to_string(),
        });
    }

    pub fn dump(&self) -> String {
        let i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = String::new();
        if !i.link.is_empty() {
            out.push_str(&format!("{}\n", i.link));
        }
        if i.dropped > 0 {
            out.push_str(&format!("({} earlier entries dropped)\n", i.dropped));
        }
        for e in &i.entries {
            out.push_str(&format!("{:>7} ms  {:<64} {}\n", e.at_ms, e.what, e.outcome));
        }
        out
    }
}

fn last() -> &'static Mutex<Option<Trace>> {
    static T: OnceLock<Mutex<Option<Trace>>> = OnceLock::new();
    T.get_or_init(|| Mutex::new(None))
}

/// Remember this switch's trace as the one a report should carry.
pub fn remember(trace: &Trace) {
    *last().lock().unwrap_or_else(|e| e.into_inner()) = Some(trace.clone());
}

/// What to put in a bug report about the T/R switch. `None` before any has been
/// opened, which is the ordinary case for the great majority of operators and
/// not worth a heading of its own.
pub fn diagnostics() -> Option<String> {
    let t = last().lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let dump = t.dump();
    if dump.is_empty() { None } else { Some(dump) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dump_carries_the_link_and_the_decisions() {
        let t = Trace::new();
        t.set_link("LCUS / CH340 relay board on /dev/ttyUSB0");
        t.note("configuration applied", "Key-down: SDR at \u{2212}25 ms, then PA at \u{2212}5 ms.");
        t.note("key-down", "");
        t.note("contacts 0b00000011", "ok");
        let d = t.dump();
        assert!(d.contains("ttyUSB0"), "{d}");
        assert!(d.contains("key-down"), "{d}");
        assert!(d.contains("0b00000011"), "{d}");
    }

    #[test]
    fn an_overlong_session_reports_what_it_dropped() {
        let t = Trace::new();
        for i in 0..(CAP + 5) {
            t.note(format!("over {i}"), "ok");
        }
        let d = t.dump();
        assert!(d.contains("5 earlier entries dropped"), "{d}");
        assert!(!d.contains("over 0 "), "the oldest are gone: {d}");
    }
}
