//! Who may drive this station from somewhere else.
//!
//! The server hands out one radio, and a radio is a transmitter. Left open, a
//! port forwarded so the operator can listen from work is also a port anybody
//! else can key, on the operator's licence and callsign — so the server asks
//! for a username and a password before it hands anything over.
//!
//! This is the *configuration* half, kept here rather than in `sdroxide-config`
//! because three crates need to name it: the config file that stores it, the
//! server that enforces it, and the settings dialog that edits it.

use serde::{Deserialize, Serialize};

/// The credentials a remote client has to present before the server will let it
/// near the radio.
///
/// Both fields empty means the server is open to anyone who can reach the port,
/// which is what every version before this one was — so an existing
/// `config.toml` upgrades into exactly the behaviour it already had, and the
/// operator turns this on when they mean to.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct RemoteAccess {
    pub username: String,
    pub password: String,
}

impl RemoteAccess {
    /// Whether connecting clients are challenged at all.
    ///
    /// A password on its own is enough. A single-operator station has no use
    /// for a username, and insisting on one would only produce a shack full of
    /// accounts called `admin` — the password is the secret either way.
    pub fn is_enforced(&self) -> bool {
        !self.username.is_empty() || !self.password.is_empty()
    }

    /// Whether these are the configured credentials.
    ///
    /// Both halves are always compared and neither comparison stops early, so
    /// the time this takes says nothing about how much of a guess was right.
    /// The server's turnstile already makes a timing attack impractical; this
    /// costs a few hundred nanoseconds and means it never has to be the only
    /// thing standing in the way.
    pub fn accepts(&self, username: &str, password: &str) -> bool {
        let user_ok = ct_eq(self.username.as_bytes(), username.as_bytes());
        let pass_ok = ct_eq(self.password.as_bytes(), password.as_bytes());
        // Non-short-circuiting `&`, deliberately: `&&` would skip the second
        // test whenever the first failed, which is a timing signal telling an
        // attacker they have the username right.
        user_ok & pass_ok
    }
}

/// Byte comparison that takes as long when the first byte differs as when the
/// last one does.
///
/// The lengths are folded in rather than checked first — a length check that
/// returned early would leak the length of the secret, which for a password is
/// most of the search space. The loop runs over the longer of the two, so its
/// duration depends only on what the caller supplied.
fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    let mut diff = (a.len() ^ b.len()) as u64;
    for i in 0..a.len().max(b.len()) {
        let x = a.get(i).copied().unwrap_or(0);
        let y = b.get(i).copied().unwrap_or(0);
        diff |= u64::from(x ^ y);
    }
    diff == 0
}

/// The other end of the same wire: which sdroxide server *this* screen dials.
///
/// [`RemoteAccess`] above is what a station demands of its visitors;
/// this is where a visitor goes. It belongs to the machine the operator is
/// sitting at rather than to the station — two laptops pointed at the same
/// shack each remember their own — which is why it is persisted next to the
/// sound-device selection in `config.toml` and never travels in the
/// [`StationConfig`](crate::StationConfig) bundle.
///
/// No credentials here on purpose. The server challenges the socket once it is
/// open and the sign-in dialog answers it, so a password would be a second copy
/// of a secret this file has no need to hold.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct RemoteServer {
    /// Host name or address of the machine running `sdroxide --server`. Empty
    /// until the operator has entered one.
    pub host: String,
    /// The port that server listens on — `server_port` in *its* `config.toml`.
    pub port: u16,
    /// Dial `wss://` rather than `ws://`: the server is behind something that
    /// terminates TLS for it.
    ///
    /// sdroxide's own server speaks plain WebSocket and nothing else, so this
    /// is never about the server itself — it is about the reverse proxy in
    /// front of it, which is how a station ends up answering on port 443. A
    /// browser client never needs the setting because the page it was served
    /// from already says which scheme to use; the native client has no page to
    /// follow and would otherwise offer the proxy a `ws://` handshake it can
    /// only refuse (issue #360).
    ///
    /// Off by default, so a `config.toml` written before this reads back as the
    /// plain link it was.
    pub tls: bool,
}

impl Default for RemoteServer {
    fn default() -> Self {
        // The port every sdroxide server binds unless it was told otherwise, so
        // the operator only has to type the half that is actually theirs.
        RemoteServer { host: String::new(), port: 4950, tls: false }
    }
}

impl RemoteServer {
    /// The WebSocket URL to dial, matching what `--connect` builds.
    ///
    /// A host that already carries a scheme is taken as a complete URL and used
    /// as typed: pasting a `ws://…/ws` (or a `wss://` one from a reverse proxy)
    /// into the address box is a reasonable thing to do, and rebuilding it
    /// around the port box would only break it — so [`Self::tls`] does not
    /// touch it either. An IPv6 literal is bracketed if the operator did not
    /// bracket it themselves, because `::1:4950` is not an address.
    pub fn url(&self) -> String {
        let host = self.host.trim();
        if host.contains("://") {
            return host.to_string();
        }
        let scheme = self.scheme();
        if host.contains(':') && !host.starts_with('[') {
            return format!("{scheme}://[{host}]:{}/ws", self.port);
        }
        format!("{scheme}://{host}:{}/ws", self.port)
    }

    /// Which WebSocket scheme [`Self::url`] will build, for a caller that wants
    /// to say so rather than dial it.
    pub fn scheme(&self) -> &'static str {
        if self.tls { "wss" } else { "ws" }
    }

    /// What to call the tab this connection opens: the address as typed, minus
    /// the scheme and the endpoint path that are the same for every server.
    pub fn label(&self) -> String {
        let host = self.host.trim();
        if host.contains("://") {
            return host
                .split_once("://")
                .map_or(host, |(_, rest)| rest)
                .trim_end_matches("/ws")
                .to_string();
        }
        format!("{host}:{}", self.port)
    }
}

/// What a station says when it turned an answer down: those credentials are
/// not the ones it wants.
///
/// Deliberately the same sentence whether the username or the password was
/// wrong — which one it was is exactly what an attacker with a list of
/// callsigns wants told.
pub const AUTH_REFUSED: &str = "username or password not accepted";

/// What a station says when it did not judge an answer at all, because another
/// one was already being judged.
///
/// A station allows one attempt at a time, so this is the ordinary answer to
/// several radios of one station signing in together — and it is *not* a
/// refusal: nothing was compared, and the same answer offered again a moment
/// later is as good as it ever was. Named here rather than written out at each
/// end so the client can tell the two apart by identity instead of by guessing
/// at a sentence; a client that cannot (an older one, or a station that
/// phrases it differently) falls back to asking the operator, which is what it
/// did before.
pub const AUTH_BUSY: &str = "another sign-in is being checked — try again";

/// Whether a rejection reason is the "come back" above rather than a verdict.
pub fn is_auth_busy(why: &str) -> bool {
    why == AUTH_BUSY
}

/// Where a connection stands with the server's sign-in challenge.
///
/// Reported by [`RadioController::auth_phase`](crate::RadioController::auth_phase)
/// so the UI can put a dialog up. An in-process engine is always [`Open`]:
/// there is no network and nothing to prove.
///
/// [`Open`]: AuthPhase::Open
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum AuthPhase {
    /// Nothing to answer — a local engine, or a server with no credentials set.
    #[default]
    Open,
    /// The server is waiting for a username and password. `Some` carries why
    /// the previous attempt was turned down, for the dialog to show.
    Prompt(Option<String>),
    /// Credentials are with the server; waiting for its verdict.
    Checking,
}

impl AuthPhase {
    /// Whether the operator is being asked to sign in, either way.
    pub fn is_pending(&self) -> bool {
        !matches!(self, AuthPhase::Open)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn creds(user: &str, pass: &str) -> RemoteAccess {
        RemoteAccess { username: user.into(), password: pass.into() }
    }

    /// The upgrade path: a config that has never heard of this leaves the
    /// server exactly as open as it was.
    #[test]
    fn empty_credentials_do_not_challenge_anyone() {
        assert!(!RemoteAccess::default().is_enforced());
        assert!(creds("", "").accepts("", ""), "an open server accepts anything");
    }

    /// A password with no username is a complete configuration.
    #[test]
    fn a_password_alone_is_enough_to_close_the_door() {
        let want = creds("", "hunter2");
        assert!(want.is_enforced());
        assert!(want.accepts("", "hunter2"));
        assert!(!want.accepts("", "hunter3"));
        // ...and the empty username still has to match, so a client that sends
        // one cannot be treated as having sent none.
        assert!(!want.accepts("op", "hunter2"));
    }

    /// The two answers a station gives are not the same answer, and the client
    /// acts on the difference: a refusal throws a stored password away, a
    /// "come back" must not.
    #[test]
    fn a_busy_station_is_not_a_refusal() {
        assert!(is_auth_busy(AUTH_BUSY));
        assert!(!is_auth_busy(AUTH_REFUSED));
        assert!(!is_auth_busy(""));
    }

    #[test]
    fn both_halves_have_to_match() {
        let want = creds("oe1test", "hunter2");
        assert!(want.accepts("oe1test", "hunter2"));
        assert!(!want.accepts("oe1test", "hunter"), "wrong password");
        assert!(!want.accepts("someone", "hunter2"), "wrong username");
        assert!(!want.accepts("", ""), "no credentials at all");
        // A prefix is not a match, in either direction.
        assert!(!want.accepts("oe1test", "hunter22"));
        assert!(!want.accepts("oe1tes", "hunter2"));
    }

    /// The comparison must not be fooled by anything the length arithmetic
    /// could paper over — an empty secret matching a long guess, or a guess
    /// whose length happens to XOR to zero against it.
    #[test]
    fn lengths_are_part_of_the_comparison() {
        assert!(ct_eq(b"", b""));
        assert!(!ct_eq(b"", b"x"));
        assert!(!ct_eq(b"x", b""));
        // "abc" vs "abc\0": equal over the overlap, and the trailing NUL is
        // exactly what `unwrap_or(0)` pads the shorter side with — only the
        // length term catches this one.
        assert!(!ct_eq(b"abc", b"abc\0"));
        assert!(ct_eq(b"abc", b"abc"));
    }

    /// What the address box produces for the shapes an operator actually
    /// types: a bare host, a host with the port in it, and a pasted URL.
    #[test]
    fn the_dialled_url_follows_what_was_typed() {
        let plain = RemoteServer { host: "shack".into(), port: 4950, tls: false };
        assert_eq!(plain.url(), "ws://shack:4950/ws");
        assert_eq!(plain.label(), "shack:4950");
        // A whole URL wins over the port box — including a `wss://` one from a
        // reverse proxy, which this could not have built.
        let url = RemoteServer { host: "wss://shack.example/ws".into(), port: 4950, tls: false };
        assert_eq!(url.url(), "wss://shack.example/ws");
        assert_eq!(url.label(), "shack.example");
        // An IPv6 literal needs brackets before a port can be appended to it.
        let v6 = RemoteServer { host: "fe80::1".into(), port: 4950, tls: false };
        assert_eq!(v6.url(), "ws://[fe80::1]:4950/ws");
        let bracketed = RemoteServer { host: "[fe80::1]".into(), port: 4950, tls: false };
        assert_eq!(bracketed.url(), "ws://[fe80::1]:4950/ws");
    }

    /// The secure switch, which is the whole of what a station behind a reverse
    /// proxy needs from this end: the scheme changes and nothing else does, so
    /// the port the operator typed is still the port dialled (issue #360).
    #[test]
    fn the_secure_switch_changes_the_scheme_and_nothing_else() {
        let tls = RemoteServer { host: "shack.example".into(), port: 443, tls: true };
        assert_eq!(tls.url(), "wss://shack.example:443/ws");
        assert_eq!(tls.label(), "shack.example:443");
        // Brackets are still the IPv6 answer, secure or not.
        let v6 = RemoteServer { host: "fe80::1".into(), port: 443, tls: true };
        assert_eq!(v6.url(), "wss://[fe80::1]:443/ws");
        // A pasted URL is used as typed, so the switch has nothing to say about
        // it — turning it on must not rewrite somebody's plain-text tunnel.
        let typed = RemoteServer { host: "ws://shack.example/ws".into(), port: 443, tls: true };
        assert_eq!(typed.url(), "ws://shack.example/ws");
    }

    #[test]
    fn a_local_engine_never_prompts() {
        assert!(!AuthPhase::default().is_pending());
        assert!(AuthPhase::Prompt(None).is_pending());
        assert!(AuthPhase::Checking.is_pending());
    }
}
