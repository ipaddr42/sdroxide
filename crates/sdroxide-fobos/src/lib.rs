//! RigExpert Fobos SDR support through `libfobos`.
//!
//! # Why a vendor library, and why dlopen
//!
//! `libfobos` (LGPL-2.1, github.com/rigexpert/libfobos) is the one interface
//! this radio speaks — there is no separately documented raw USB protocol to
//! reimplement the way `sdroxide-airspy`/`sdroxide-hydrasdr` do for their own
//! open-protocol radios. So this crate finds it with **dlopen at runtime**,
//! the same arrangement `sdroxide-sdrplay` uses for the closed `sdrplay_api`
//! and `sdroxide-lime` uses for the open-source `libLimeSuite`: nothing is
//! linked at build time regardless of which of those two libraries happens
//! to be open, because the reason is build portability, not licensing — this
//! crate compiles and ships in every build variant without every contributor
//! needing `libfobos` installed, and on a machine without it the device list
//! is simply empty with an explanation.
//!
//! # Layout
//!
//! [`ffi`] holds the bindings, [`device`] owns the process-global library
//! handle plus enumerate/open, [`handle`] (built on [`stream`], its own
//! thread) streams receive into an `rtrb` ring — RF port, single-channel HF
//! (direct sampling), or both HF channels at once for diversity combining
//! (done in `src/fobos_source.rs` at the workspace root). Written around a
//! few real-hardware quirks: this vendor library's async streaming mode
//! never delivered a callback on the units this was verified against, so
//! [`handle`] drives synchronous reads from its own thread instead; a
//! reopen needs a short settling delay after close or the next open fails;
//! and gain ranges come from the vendor's own source rather than a
//! datasheet.
//!
//! Must never be a dependency of any wasm-targeted crate — dlopen is a
//! native-only concept, same reasoning as every sibling backend here.

pub mod device;
pub mod error;
pub mod ffi;
pub mod handle;
pub(crate) mod stream;

pub use device::{BoardInfo, DevInfo, board_info, close, list, open, samplerates, try_list};
pub use error::{Error, Result};
pub use handle::{FobosHandle, OpenParams, Port};
