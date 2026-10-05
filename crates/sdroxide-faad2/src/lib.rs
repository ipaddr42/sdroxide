//! faad2 for the DRM receiver: knik0's 2.11.2 as it is, compiled with
//! `DRM_SUPPORT`.
//!
//! There is no Rust API. `sdroxide-drm` calls faad2 from Dream's C++, compiles
//! against the headers this crate's build script exports as
//! `DEP_FAAD2_INCLUDE`, and names this crate in its `lib.rs` so the archive is
//! linked. `links = "faad2"` keeps it to one: two faad2 archives would collide
//! on every `NeAACDec*` symbol.
//!
//! The HD Radio decoder's faad2 is not this one. HDC needs a patched faad2,
//! which the `libnrsc5` that `sdroxide-nrsc5` loads at run time carries
//! privately (issue #488).
