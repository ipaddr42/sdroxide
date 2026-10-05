//! Builds faad2 for the DRM receiver: knik0's 2.11.2 from `vendor/faad2`, as it
//! is, with `DRM_SUPPORT`.
//!
//! Stock sources and nothing patched. The HD Radio decoder needs faad2's HDC
//! variant, which upstream does not carry, and it gets it from the `libnrsc5`
//! it loads at run time, which keeps its own patched copy private (issue #488).
//! So the one faad2 compiled into sdroxide is the plain one DRM needs.
//!
//! Dependents find the headers through `DEP_FAAD2_INCLUDE`.

use std::path::{Path, PathBuf};

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let faad2 = manifest.join("../../vendor/faad2");

    if !faad2.join("include/neaacdec.h").exists() {
        panic!(
            "vendored faad2 is missing at {}\nrun: git submodule update --init --recursive",
            faad2.display()
        );
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", faad2.join("include").display());
    println!("cargo:rerun-if-changed={}", faad2.join("libfaad").display());

    build(&faad2);
    println!("cargo:include={}", faad2.join("include").display());
}

/// The stock sources with `DRM_SUPPORT`, which brings in `NeAACDecInitDRM`.
/// Upstream ships that as a second library, `libfaad_drm`, because the plain
/// one cannot decode DRM at all.
fn build(faad2: &Path) {
    let mut build = cc::Build::new();
    build
        .include(faad2.join("libfaad"))
        .include(faad2.join("include"))
        .define("HAVE_INTTYPES_H", "1")
        .define("HAVE_MEMCPY", "1")
        .define("HAVE_STRING_H", "1")
        .define("HAVE_STRINGS_H", "1")
        .define("HAVE_SYS_STAT_H", "1")
        .define("HAVE_SYS_TYPES_H", "1")
        .define("PACKAGE_VERSION", "\"2.11.2\"")
        .define("APPLY_DRC", None)
        .define("DRM_SUPPORT", None)
        .opt_level(2)
        .warnings(false);
    // The target's C library, not the host's: a build script runs on the host.
    if std::env::var("CARGO_CFG_TARGET_ENV").as_deref() != Ok("msvc") {
        build.define("HAVE_LRINTF", "1");
    }
    let mut sources: Vec<PathBuf> = std::fs::read_dir(faad2.join("libfaad"))
        .expect("read faad2/libfaad")
        .map(|entry| entry.expect("faad2 dir entry").path())
        .filter(|path| path.extension().is_some_and(|e| e == "c"))
        .collect();
    sources.sort();
    build.files(sources);
    build.compile("sdroxide_faad2");
}
