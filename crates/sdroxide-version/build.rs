//! Stamps the identity of a build that was not cut from a tag into the binary.
//!
//! `CARGO_PKG_VERSION` cannot tell a release apart from a nightly built from a
//! later commit: both say `1.6.8`, so a bug report from someone running a
//! nightly reads exactly like one against the release, and the first question
//! back is always "which build is that". CI sets `SDROXIDE_BUILD` on every
//! build it does not cut from a tag, and this appends it.
//!
//! Nothing here shells out to git. The -compat builds compile inside a
//! container that has no repository, and a source tarball has no `.git` at
//! all — so the one input is the environment variable, and a plain
//! `cargo build` anywhere produces the release string byte for byte.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // Without this cargo has no reason to believe anything changed between two
    // builds of the same commit, and the second would keep the first's stamp.
    println!("cargo:rerun-if-env-changed=SDROXIDE_BUILD");

    let base = std::env::var("CARGO_PKG_VERSION").unwrap();
    let stamp = std::env::var("SDROXIDE_BUILD").unwrap_or_default();
    // The value is interpolated into a `cargo:` directive, which is parsed a
    // line at a time, and from there into a window title and a crash report.
    // Keep it to what semver allows after the hyphen and drop the rest, rather
    // than let a stray newline rewrite this script's output.
    let stamp: String = stamp
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | '_'))
        .collect();

    let version = if stamp.is_empty() { base } else { format!("{base}-{stamp}") };
    println!("cargo:rustc-env=SDROXIDE_VERSION={version}");
}
