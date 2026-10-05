//! The version string the program gives when an operator asks what it is.
//!
//! [`VERSION`] is the crate version plus, on a build CI did not cut from a tag,
//! what kind of build it is and the commit it came from —
//! `1.6.8-nightly.20260922.abc1234`. A release and a local `cargo build` both
//! give the bare `1.6.8`, unchanged from what `CARGO_PKG_VERSION` gave before.
//!
//! It is deliberately **not** a drop-in replacement for `CARGO_PKG_VERSION`.
//! Use it for what a person reads — `--version`, the About box, a crash report
//! — and leave the plain crate version everywhere something other than a
//! person parses it:
//!
//! - `sdroxide_config::version_is_newer` splits on `.` and compares the parts
//!   as numbers, so a stamped version reads as `1.6.0` and the update banner
//!   would never switch off again;
//! - rigctld's `Info` reply, the WSJT-X and Winlink identifiers, and the
//!   WSPRnet / PSK Reporter / FreeDV Reporter software fields ride wire
//!   protocols other people's software parses;
//! - the SSTV ID goes out over the air, where the extra characters do not fit.

/// What this build calls itself.
pub const VERSION: &str = env!("SDROXIDE_VERSION");

#[cfg(test)]
mod tests {
    /// An unstamped build must stay byte-identical to what the call sites
    /// printed before the stamp existed, because a release is built that way.
    #[test]
    fn unstamped_build_is_the_plain_crate_version() {
        if option_env!("SDROXIDE_BUILD").unwrap_or("").is_empty() {
            assert_eq!(super::VERSION, env!("CARGO_PKG_VERSION"));
        }
    }

    /// A stamped one keeps the crate version as its prefix, so anything that
    /// eyeballs the leading digits still reads the release it came from.
    #[test]
    fn stamped_build_keeps_the_crate_version_as_its_prefix() {
        assert!(super::VERSION.starts_with(env!("CARGO_PKG_VERSION")));
    }
}
