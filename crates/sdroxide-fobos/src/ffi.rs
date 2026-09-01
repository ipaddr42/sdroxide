//! Hand-written bindings for `libfobos`, current upstream (v2.4.1,
//! github.com/rigexpert/libfobos).
//!
//! The library is loaded with dlopen at runtime — nothing is linked at build
//! time, so this crate builds and ships everywhere and merely finds
//! `libfobos` missing where it is not installed. See the crate's own
//! `Cargo.toml` comment for why that holds regardless of the library being
//! open source.
//!
//! # No structs to lay out
//!
//! Unlike `sdrplay_api` or LimeSuite, nothing here crosses the FFI boundary
//! by value: every call takes an opaque `struct fobos_dev_t *` (never
//! dereferenced on this side — the vendor library owns its layout entirely)
//! plus primitive types. There is nothing to `offset_of!`-pin and nothing an
//! ABI drift could silently misalign.
//!
//! # A missing symbol is not always a missing library
//!
//! Three of the bindings below — `fobos_rx_reset`, `fobos_rx_read_firmware`
//! and `fobos_rx_write_firmware` — are newer than the header this crate first
//! targeted, and nothing here calls any of them. They are bound as `Option`
//! so an older `libfobos` still loads: a library missing a call this crate
//! never makes can stream just as well as one that has it, and refusing to
//! load it would report "no Fobos SDR found" for a receiver that is plugged
//! in and working perfectly. Everything this crate actually calls stays
//! required, so a genuinely wrong library still fails loudly at load rather
//! than at the first read.
//!
//! # What's bound, and what's deliberately not
//!
//! Every function in the current header except the two low-level chip-tuning
//! calls (`fobos_max2830_set_frequency`, `fobos_rffc507x_set_lo_frequency_hz`)
//! — `fobos_rx_set_frequency` covers normal tuning, and binding the two
//! chip-level calls now would be dead code until something concrete needs
//! them. `fobos_rx_read_async`/`fobos_rx_cancel_async` *are* bound, for API
//! completeness, but `device.rs`/`handle.rs` must never call them: async
//! streaming did not deliver a callback on the units this was verified
//! against, so sync (`fobos_rx_start_sync`/`read_sync`/`stop_sync`) is the
//! one this crate actually uses.

#![allow(dead_code)]

use std::ffi::{CStr, c_char, c_int, c_void};

/// `struct fobos_dev_t *` — an opaque device handle owned by the library.
/// Never dereferenced here.
pub type Dev = *mut c_void;

/// `fobos_rx_cb_t`. Bound for completeness; not used (see module doc).
pub type RxCb = unsafe extern "C" fn(buf: *mut f32, buf_length: u32, ctx: *mut c_void);

/// `FOBOS_INFO_LEN` — the buffer size every `fobos_rx_get_board_info` output
/// wants, and (per this crate's own `libfobos` update) an answer pinned
/// against the current header rather than guessed.
pub const INFO_LEN: usize = 64;

/// Error codes, `FOBOS_ERR_*`. Plain constants rather than a Rust enum for
/// the same reason `sdroxide-sdrplay`/`sdroxide-lime` use plain constants for
/// their own vendor error codes: a value the foreign library hands back is
/// whatever it is, and transmuting an unrecognised one into a Rust enum
/// would be undefined behaviour.
pub type ErrT = c_int;
pub const ERR_OK: ErrT = 0;
pub const ERR_NO_DEV: ErrT = -1;
pub const ERR_NOT_OPEN: ErrT = -2;
pub const ERR_NO_MEM: ErrT = -3;
pub const ERR_CONTROL: ErrT = -4;
pub const ERR_ASYNC_IN_SYNC: ErrT = -5;
pub const ERR_SYNC_IN_ASYNC: ErrT = -6;
pub const ERR_SYNC_NOT_STARTED: ErrT = -7;
pub const ERR_UNSUPPORTED: ErrT = -8;
pub const ERR_LIBUSB: ErrT = -9;

pub struct Api {
    /// The loaded library, kept for the life of the process — nothing here
    /// hands the library a callback pointer that would need to outlive it
    /// (unlike LimeSuite's log handler), but there is no reason to unload it
    /// either.
    _lib: libloading::Library,

    pub get_api_info:
        unsafe extern "C" fn(lib_version: *mut c_char, drv_version: *mut c_char) -> c_int,
    pub get_device_count: unsafe extern "C" fn() -> c_int,
    pub list_devices: unsafe extern "C" fn(serials: *mut c_char) -> c_int,
    pub open: unsafe extern "C" fn(out_dev: *mut Dev, index: u32) -> c_int,
    pub close: unsafe extern "C" fn(dev: Dev) -> c_int,
    /// `fobos_rx_reset` — close and reset the device. Added to the vendor
    /// library after the header this crate first targeted; not called
    /// anywhere yet, kept in mind for `device.rs`'s error-recovery path as a
    /// stronger option than plain close-then-reopen.
    ///
    /// `Option`, not a plain pointer, precisely *because* it is newer than
    /// the rest and nothing here calls it: an older `libfobos` that streams
    /// perfectly well must not be turned into "no Fobos backend at all" over
    /// a symbol this crate never invokes. Same arrangement `sdroxide-lime`
    /// uses for the LimeRFE half of LimeSuite.
    pub reset: Option<unsafe extern "C" fn(dev: Dev) -> c_int>,
    pub get_board_info: unsafe extern "C" fn(
        dev: Dev,
        hw_revision: *mut c_char,
        fw_version: *mut c_char,
        manufacturer: *mut c_char,
        product: *mut c_char,
        serial: *mut c_char,
    ) -> c_int,
    pub set_frequency: unsafe extern "C" fn(dev: Dev, value: f64, actual: *mut f64) -> c_int,
    pub set_direct_sampling: unsafe extern "C" fn(dev: Dev, enabled: c_int) -> c_int,
    /// 0..3, confirmed against `fobos.c`'s own register-write masking
    /// (`value & 0x0003`) rather than trusted from a header comment alone —
    /// the header's own text and the register mask itself disagreed, and
    /// the register mask is what this settled on.
    pub set_lna_gain: unsafe extern "C" fn(dev: Dev, value: c_int) -> c_int,
    /// 0..31, likewise confirmed (`value & 0x001F`).
    pub set_vga_gain: unsafe extern "C" fn(dev: Dev, value: c_int) -> c_int,
    pub get_samplerates: unsafe extern "C" fn(dev: Dev, values: *mut f64, count: *mut u32) -> c_int,
    pub set_samplerate: unsafe extern "C" fn(dev: Dev, value: f64, actual: *mut f64) -> c_int,
    /// Bound but never called — see module doc.
    pub read_async: unsafe extern "C" fn(
        dev: Dev,
        cb: Option<RxCb>,
        ctx: *mut c_void,
        buf_count: u32,
        buf_length: u32,
    ) -> c_int,
    /// Bound but never called — see module doc.
    pub cancel_async: unsafe extern "C" fn(dev: Dev) -> c_int,
    pub set_user_gpo: unsafe extern "C" fn(dev: Dev, value: u8) -> c_int,
    /// 0 internal (default), 1 external.
    pub set_clk_source: unsafe extern "C" fn(dev: Dev, value: c_int) -> c_int,
    pub start_sync: unsafe extern "C" fn(dev: Dev, buf_length: u32) -> c_int,
    pub read_sync:
        unsafe extern "C" fn(dev: Dev, buf: *mut f32, actual_buf_length: *mut u32) -> c_int,
    pub stop_sync: unsafe extern "C" fn(dev: Dev) -> c_int,
    /// Added to the vendor library after the header this crate first
    /// targeted; not called anywhere yet, and `Option` for the same reason
    /// as [`Api::reset`].
    pub read_firmware:
        Option<unsafe extern "C" fn(dev: Dev, file_name: *const c_char, verbose: c_int) -> c_int>,
    /// Added to the vendor library after the header this crate first
    /// targeted; not called anywhere yet, and `Option` for the same reason
    /// as [`Api::reset`].
    pub write_firmware:
        Option<unsafe extern "C" fn(dev: Dev, file_name: *const c_char, verbose: c_int) -> c_int>,
    pub error_name: unsafe extern "C" fn(error: c_int) -> *const c_char,
}

/// Library names to try, most specific first.
///
/// The build this crate was verified against sets `OUTPUT_NAME fobos` with
/// `VERSION 2.4.1`/`SOVERSION 0` (`libfobos`'s own `CMakeLists.txt`), so the
/// unversioned name resolves everywhere the *development* package (or its
/// unversioned symlink) is installed, and the SOVERSION entry covers a
/// runtime-only install where only that symlink exists.
#[cfg(target_os = "linux")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    ["libfobos.so", "libfobos.so.0"].iter().map(Into::into).collect()
}

#[cfg(target_os = "macos")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    ["libfobos.dylib", "/usr/local/lib/libfobos.dylib", "/opt/homebrew/lib/libfobos.dylib"]
        .iter()
        .map(Into::into)
        .collect()
}

/// `fobos.dll` has no `lib` prefix on Windows (the vendor's own installer
/// places it at `C:/Program Files/RigExpert/Fobos/lib/fobos.dll`) — the same
/// Program-Files guess `sdroxide-lime` uses for PothosSDR's bundled
/// LimeSuite.dll, since a vendor installer's own directory is rarely on
/// anyone's DLL search path.
#[cfg(target_os = "windows")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    use std::path::PathBuf;
    let mut out: Vec<std::ffi::OsString> = vec!["fobos.dll".into()];
    for var in ["ProgramFiles", "ProgramFiles(x86)"] {
        if let Some(pf) = std::env::var_os(var) {
            out.push(
                PathBuf::from(pf)
                    .join("RigExpert")
                    .join("Fobos")
                    .join("lib")
                    .join("fobos.dll")
                    .into_os_string(),
            );
        }
    }
    out
}

impl Api {
    pub fn load() -> Result<Api, String> {
        let mut last = String::new();
        for name in lib_candidates() {
            match unsafe { libloading::Library::new(&name) } {
                Ok(lib) => return unsafe { Api::from_lib(lib) },
                Err(e) => last = e.to_string(),
            }
        }
        Err(format!(
            "libfobos was not found ({last}) — build/install it from \
             github.com/rigexpert/libfobos (LGPL-2.1), then rescan"
        ))
    }

    unsafe fn from_lib(lib: libloading::Library) -> Result<Api, String> {
        macro_rules! sym {
            ($name:literal) => {
                *unsafe { lib.get(concat!($name, "\0").as_bytes()) }
                    .map_err(|e| format!("{} missing from libfobos: {e}", $name))?
            };
        }
        // Symbols newer than the header this crate first targeted, none of
        // which anything here calls. Absent on an older `libfobos` that is
        // otherwise perfectly capable of streaming, so their absence must not
        // fail the load — see [`Api::reset`]'s own doc comment.
        macro_rules! opt {
            ($name:literal) => {
                unsafe { lib.get(concat!($name, "\0").as_bytes()) }.ok().map(|s| *s)
            };
        }
        Ok(Api {
            get_api_info: sym!("fobos_rx_get_api_info"),
            get_device_count: sym!("fobos_rx_get_device_count"),
            list_devices: sym!("fobos_rx_list_devices"),
            open: sym!("fobos_rx_open"),
            close: sym!("fobos_rx_close"),
            reset: opt!("fobos_rx_reset"),
            get_board_info: sym!("fobos_rx_get_board_info"),
            set_frequency: sym!("fobos_rx_set_frequency"),
            set_direct_sampling: sym!("fobos_rx_set_direct_sampling"),
            set_lna_gain: sym!("fobos_rx_set_lna_gain"),
            set_vga_gain: sym!("fobos_rx_set_vga_gain"),
            get_samplerates: sym!("fobos_rx_get_samplerates"),
            set_samplerate: sym!("fobos_rx_set_samplerate"),
            read_async: sym!("fobos_rx_read_async"),
            cancel_async: sym!("fobos_rx_cancel_async"),
            set_user_gpo: sym!("fobos_rx_set_user_gpo"),
            set_clk_source: sym!("fobos_rx_set_clk_source"),
            start_sync: sym!("fobos_rx_start_sync"),
            read_sync: sym!("fobos_rx_read_sync"),
            stop_sync: sym!("fobos_rx_stop_sync"),
            read_firmware: opt!("fobos_rx_read_firmware"),
            write_firmware: opt!("fobos_rx_write_firmware"),
            error_name: sym!("fobos_rx_error_name"),

            _lib: lib,
        })
    }

    /// `libfobos`'s own text for an error code, via `fobos_rx_error_name` —
    /// unlike LimeSuite/`sdrplay_api` this takes the code directly rather
    /// than reading process-global state, so it's safe to call any time.
    pub fn err_text(&self, code: c_int) -> String {
        let p = unsafe { (self.error_name)(code) };
        if p.is_null() {
            return format!("error {code}");
        }
        unsafe { CStr::from_ptr(p) }.to_string_lossy().trim().to_string()
    }
}
