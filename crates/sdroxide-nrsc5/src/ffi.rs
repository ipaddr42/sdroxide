//! `libnrsc5`, found with dlopen at run time.
//!
//! Nothing of nrsc5 is compiled into sdroxide (issue #488, following the
//! decision in #437). HD Radio's audio is HDC, iBiquity's proprietary codec,
//! and the only faad2 that decodes it is nrsc5's patched copy — which upstream's
//! shared library carries inside itself, exporting nothing but the `nrsc5_*`
//! calls (`src/libnrsc5.map`, `libnrsc5.sym`, `--exclude-all-symbols` on
//! Windows). So that faad2 never meets the stock one the DRM receiver links,
//! and nothing patched is built here at all. The decoder comes from the
//! machine — a distribution's nrsc5 package, or upstream's `make install` — and
//! a machine without it keeps the mode, greyed out, with
//! [`unavailable_reason`]'s sentence saying what to install.
//!
//! # Which nrsc5 this reads
//!
//! The calls are nrsc5's own `nrsc5.h`, and so are the event numbers and
//! structs in `lib.rs`. Upstream has no version number to check at run time —
//! `nrsc5_get_version` is the git hash the library was built from, `unknown`
//! when it was built from a tarball — so rather than refuse versions, this
//! reads only what has not moved since v3.0:
//!
//! * the event numbers and the members read out of the event union are where
//!   they are in v3.1.0, v3.2.0 and master (0225922), measured against each
//!   header. v3.0's sync event carried no payload, which is why `lib.rs` checks
//!   the offset and the service mode it reads before believing them;
//! * `nrsc5_pipe_samples_cf32` arrived after v3.2.0, so it is optional and
//!   [`crate::HdReceiver::pipe_cf32`] falls back to `cs16` without it;
//! * the audio event's `flags` member arrived later still, and a library older
//!   than it leaves the same bytes uninitialised, so it is never read — see
//!   [`crate::Event::Audio`].

use std::ffi::{CStr, OsString, c_char, c_float, c_int, c_uint, c_void};
use std::sync::OnceLock;

use tracing::{info, warn};

/// The environment variable naming a `libnrsc5` to try before the usual
/// places — one built somewhere of its own, or a second one to compare.
pub const LIB_ENV: &str = "SDROXIDE_NRSC5_LIB";

/// Opaque receiver handle, from the C library's point of view.
#[repr(C)]
pub(crate) struct NrsCtx {
    _private: [u8; 0],
}

/// `nrsc5_callback_t`.
pub(crate) type NrsCallback =
    unsafe extern "C" fn(evt: *const crate::NrsEvent, opaque: *mut c_void);

/// The calls `sdroxide-nrsc5` makes, resolved out of the loaded library.
pub(crate) struct Api {
    /// Kept for the life of the process: every pointer below is into it.
    _lib: libloading::Library,
    pub open_pipe: unsafe extern "C" fn(st: *mut *mut NrsCtx) -> c_int,
    pub close: unsafe extern "C" fn(st: *mut NrsCtx),
    pub start: unsafe extern "C" fn(st: *mut NrsCtx),
    pub stop: unsafe extern "C" fn(st: *mut NrsCtx),
    pub set_mode: unsafe extern "C" fn(st: *mut NrsCtx, mode: c_int) -> c_int,
    pub set_callback:
        unsafe extern "C" fn(st: *mut NrsCtx, callback: Option<NrsCallback>, opaque: *mut c_void),
    pub pipe_cu8:
        unsafe extern "C" fn(st: *mut NrsCtx, samples: *const u8, length: c_uint) -> c_int,
    pub pipe_cs16:
        unsafe extern "C" fn(st: *mut NrsCtx, samples: *const i16, length: c_uint) -> c_int,
    /// Newer than v3.2.0, the last release; `None` on a library without it.
    pub pipe_cf32: Option<
        unsafe extern "C" fn(st: *mut NrsCtx, samples: *const c_float, length: c_uint) -> c_int,
    >,
}

/// The loaded library, or the sentence saying why there is none.
///
/// Loaded once per process, on first use. A library installed while sdroxide
/// runs is picked up at the next start, which the sentence says.
pub(crate) fn api() -> Result<&'static Api, &'static str> {
    static API: OnceLock<Result<Api, String>> = OnceLock::new();
    API.get_or_init(Api::load).as_ref().map_err(String::as_str)
}

/// Why HD Radio cannot be decoded on this machine, or `None` once `libnrsc5`
/// has been found and loaded.
///
/// The sentence is written to be shown as it is: on the greyed-out mode, and in
/// the HD Radio window.
pub fn unavailable_reason() -> Option<&'static str> {
    api().err()
}

/// The file name upstream's CMake gives the shared library on this platform.
#[cfg(target_os = "windows")]
const LIB_FILE: &str = "libnrsc5.dll";
#[cfg(target_os = "macos")]
const LIB_FILE: &str = "libnrsc5.dylib";
#[cfg(not(any(target_os = "windows", target_os = "macos")))]
const LIB_FILE: &str = "libnrsc5.so";

/// Library names to try, in order.
///
/// Upstream sets no `SOVERSION`, so what `make install` leaves is the bare name
/// — `libnrsc5.so` is the soname too — and that is what a system search finds.
/// Its default prefix is `/usr/local`, named outright as well because a fresh
/// install there is not in the loader's cache until `ldconfig` has run. And
/// beside the executable last, for a portable download: Windows searches there
/// by itself, the others do not.
fn lib_candidates() -> Vec<OsString> {
    let mut out: Vec<OsString> = Vec::new();
    if let Some(path) = std::env::var_os(LIB_ENV).filter(|p| !p.is_empty()) {
        out.push(path);
    }
    out.push(LIB_FILE.into());
    if cfg!(target_os = "windows") {
        // Named without the `lib` prefix by a build that is not MinGW's.
        out.push("nrsc5.dll".into());
    } else {
        out.push(format!("/usr/local/lib/{LIB_FILE}").into());
        if cfg!(target_os = "macos") {
            out.push(format!("/opt/homebrew/lib/{LIB_FILE}").into());
        }
        if let Some(dir) = std::env::current_exe().ok().as_deref().and_then(|e| e.parent()) {
            out.push(dir.join(LIB_FILE).into_os_string());
        }
    }
    out
}

impl Api {
    fn load() -> Result<Api, String> {
        let mut last = String::new();
        for name in lib_candidates() {
            match unsafe { libloading::Library::new(&name) } {
                Ok(lib) => {
                    let api = unsafe { Api::from_lib(lib) }.map_err(|missing| {
                        let why = format!(
                            "The nrsc5 library sdroxide found ({}) has no {missing}, so it is \
                             not a build of nrsc5 that HD Radio can use. Install nrsc5 3.1 or \
                             newer, then restart sdroxide.",
                            name.to_string_lossy()
                        );
                        warn!("{why}");
                        why
                    })?;
                    info!(
                        library = %name.to_string_lossy(),
                        build = api.1.as_deref().unwrap_or("unknown"),
                        cf32 = api.0.pipe_cf32.is_some(),
                        "loaded libnrsc5 for HD Radio"
                    );
                    return Ok(api.0);
                }
                Err(e) => last = e.to_string(),
            }
        }
        info!(last = %last, "no libnrsc5; HD Radio is unavailable");
        Err(format!(
            "HD Radio needs nrsc5's decoder library ({LIB_FILE}), which is not installed on this \
             machine. sdroxide loads it at run time rather than building it in: install nrsc5 \
             from your distribution or from github.com/theori-io/nrsc5, then restart sdroxide."
        ))
    }

    /// Resolve every call, or name the first one missing. Also returns what
    /// `nrsc5_get_version` says the library was built from, for the log.
    unsafe fn from_lib(lib: libloading::Library) -> Result<(Api, Option<String>), &'static str> {
        macro_rules! sym {
            ($name:literal) => {
                *unsafe { lib.get(concat!($name, "\0").as_bytes()) }.map_err(|_| $name)?
            };
        }
        macro_rules! opt {
            ($name:literal) => {
                unsafe { lib.get(concat!($name, "\0").as_bytes()) }.ok().map(|s| *s)
            };
        }
        let get_version: Option<unsafe extern "C" fn(version: *mut *const c_char)> =
            opt!("nrsc5_get_version");
        let build = get_version.and_then(|f| {
            let mut p: *const c_char = std::ptr::null();
            unsafe { f(&mut p) };
            (!p.is_null()).then(|| unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned())
        });
        let api = Api {
            open_pipe: sym!("nrsc5_open_pipe"),
            close: sym!("nrsc5_close"),
            start: sym!("nrsc5_start"),
            stop: sym!("nrsc5_stop"),
            set_mode: sym!("nrsc5_set_mode"),
            set_callback: sym!("nrsc5_set_callback"),
            pipe_cu8: sym!("nrsc5_pipe_samples_cu8"),
            pipe_cs16: sym!("nrsc5_pipe_samples_cs16"),
            pipe_cf32: opt!("nrsc5_pipe_samples_cf32"),
            _lib: lib,
        };
        Ok((api, build))
    }
}
