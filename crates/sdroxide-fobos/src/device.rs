//! The process-global `libfobos` handle, device enumeration, and open — with
//! a settling delay on reopen that a real unit needed before it would answer
//! again. Streaming itself lives in `handle.rs`.

use std::ffi::{c_char, c_int};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use crate::error::{Error, Result};
use crate::ffi;

/// `fobos_rx_list_devices`'s own example (`eval/fobos_devinfo_main.c`) uses
/// this exact size for its `serials` buffer — the header gives the call no
/// length parameter to bound a write with, so the vendor's own choice is
/// followed here rather than guessed at independently.
const SERIALS_BUF_LEN: usize = 256;

/// How many times [`open`] retries `fobos_rx_open` before giving up, and how
/// long it waits between attempts. Real hardware quirk, not a made-up
/// number: the device will not reopen immediately after a previous session
/// closed it — needing roughly half a second before it answers again, and
/// landing on the second attempt every time at this 5×500ms retry shape.
const REOPEN_ATTEMPTS: u32 = 5;
const REOPEN_DELAY: Duration = Duration::from_millis(500);

/// Rates [`samplerates`] adds to whatever the receiver itself reports — see
/// that function's own doc comment for what they are for and why they are
/// offered on every port. Narrowest last, matching the order they are
/// appended in; `samplerates` sorts the whole list afterwards anyway.
const EXTRA_HF_RATES_HZ: [f64; 4] = [5_000_000.0, 2_500_000.0, 1_250_000.0, 625_000.0];

struct ApiState {
    api: Option<Arc<ffi::Api>>,
    /// One log line per absence, not one per rescan tick.
    complained: bool,
}

fn state() -> &'static Mutex<ApiState> {
    static STATE: OnceLock<Mutex<ApiState>> = OnceLock::new();
    STATE.get_or_init(|| Mutex::new(ApiState { api: None, complained: false }))
}

/// Load the library, idempotent.
fn ensure_loaded(s: &mut ApiState) -> Result<Arc<ffi::Api>> {
    if let Some(a) = &s.api {
        return Ok(Arc::clone(a));
    }
    match ffi::Api::load() {
        Ok(a) => {
            let a = Arc::new(a);
            let mut lib_version = [0 as c_char; ffi::INFO_LEN];
            let mut drv_version = [0 as c_char; ffi::INFO_LEN];
            unsafe { (a.get_api_info)(lib_version.as_mut_ptr(), drv_version.as_mut_ptr()) };
            tracing::info!(
                "libfobos loaded: lib {}, driver {}",
                cstr_buf_to_string(&lib_version),
                cstr_buf_to_string(&drv_version),
            );
            s.api = Some(Arc::clone(&a));
            s.complained = false;
            Ok(a)
        }
        Err(e) => {
            if !s.complained {
                s.complained = true;
                tracing::debug!("Fobos backend unavailable: {e}");
            }
            Err(Error::LibMissing(e))
        }
    }
}

/// The loaded library, for callers that need to reach it directly (e.g. a
/// later `handle.rs`'s streaming path).
pub(crate) fn api() -> Result<Arc<ffi::Api>> {
    let mut s = state().lock().expect("fobos api state poisoned");
    ensure_loaded(&mut s)
}

/// One entry from `fobos_rx_list_devices` — the index `fobos_rx_open` wants,
/// and the serial it belongs to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DevInfo {
    pub index: u32,
    pub serial: String,
}

/// Board identity, from `fobos_rx_get_board_info` on an already-open device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardInfo {
    pub hw_revision: String,
    pub fw_version: String,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
}

/// Ask `libfobos` what is attached, or say why that is impossible — the
/// distinction a rescan/probe needs between "no library" and "no devices",
/// same as every other dlopen'd backend here.
pub fn try_list() -> Result<Vec<DevInfo>> {
    let api = api()?;
    let mut buf = [0 as c_char; SERIALS_BUF_LEN];
    let n = unsafe { (api.list_devices)(buf.as_mut_ptr()) };
    if n < 0 {
        return Err(Error::api("fobos_rx_list_devices", api.err_text(n)));
    }
    if n == 0 {
        return Ok(Vec::new());
    }
    // Space-delimited, per the header's own comment on `fobos_rx_list_devices`.
    let text = cstr_buf_to_string(&buf);
    let devices: Vec<DevInfo> = text
        .split_whitespace()
        .enumerate()
        .map(|(i, serial)| DevInfo { index: i as u32, serial: serial.to_string() })
        .collect();
    if devices.len() as c_int != n {
        // Not fatal — the count is still informational on this API, and a
        // truncated parse is still a usable (if short) device list — but
        // worth knowing about if the serials buffer above ever turns out to
        // be too small for someone's setup.
        tracing::debug!(
            "fobos_rx_list_devices reported {n} device(s) but the serials buffer parsed \
             {}",
            devices.len()
        );
    }
    Ok(devices)
}

/// The boards `libfobos` reports. Best-effort: no library and no devices are
/// the same answer to a rescan button.
pub fn list() -> Vec<DevInfo> {
    match try_list() {
        Ok(d) => d,
        Err(e) => {
            tracing::debug!("Fobos enumeration failed: {e}");
            Vec::new()
        }
    }
}

/// Open the board `want` names — an exact serial match, or the first one
/// found when `want` is empty. Retries the underlying `fobos_rx_open` per
/// [`REOPEN_ATTEMPTS`]/[`REOPEN_DELAY`], since a device that was just closed
/// (this process's own previous session, or another program's) genuinely
/// needs that long before it will answer again — see the module-level
/// constant doc.
pub fn open(want: &str) -> Result<(Arc<ffi::Api>, ffi::Dev, DevInfo)> {
    let found = try_list()?;
    let chosen = if want.trim().is_empty() {
        found.first().cloned()
    } else {
        found.iter().find(|d| d.serial == want.trim()).cloned()
    }
    .ok_or_else(|| {
        if found.is_empty() {
            Error::NotFound("no Fobos SDR found — is one plugged in?".to_string())
        } else {
            Error::NotFound(format!(
                "no Fobos SDR matching {want:?} — found: {}",
                found.iter().map(|d| d.serial.as_str()).collect::<Vec<_>>().join(", ")
            ))
        }
    })?;

    let api = api()?;
    let mut dev: ffi::Dev = std::ptr::null_mut();
    let mut rc = unsafe { (api.open)(&mut dev, chosen.index) };
    let mut attempt = 0;
    while rc != ffi::ERR_OK && attempt < REOPEN_ATTEMPTS {
        std::thread::sleep(REOPEN_DELAY);
        rc = unsafe { (api.open)(&mut dev, chosen.index) };
        attempt += 1;
    }
    if rc != ffi::ERR_OK || dev.is_null() {
        let text = api.err_text(rc);
        // The API gives no code of its own for "another program has it" —
        // same situation LimeSuite leaves sdroxide-lime in — so this is a
        // guess from the text, not a distinct return value.
        let lowered = text.to_ascii_lowercase();
        if lowered.contains("busy") || lowered.contains("access") || lowered.contains("in use") {
            return Err(Error::InUse(chosen.serial.clone()));
        }
        return Err(Error::api("fobos_rx_open", format!("{}: {text}", chosen.serial)));
    }
    Ok((api, dev, chosen))
}

/// `fobos_rx_close`. Logs rather than returns an error — by the time a
/// caller is closing a device there is nothing useful left to do with a
/// failure here beyond knowing about it.
///
/// # Safety
/// `dev` must be a still-open handle from [`open`] belonging to `api`, and
/// must not be used again (by this call or any other) once this returns.
pub unsafe fn close(api: &ffi::Api, dev: ffi::Dev) {
    let rc = unsafe { (api.close)(dev) };
    if rc != ffi::ERR_OK {
        tracing::warn!("fobos_rx_close: {}", api.err_text(rc));
    }
}

/// The sample rates `fobos_rx_get_samplerates` reports, plus four lower ones
/// this crate adds itself.
///
/// The added four are for the **HF ports**, where the requested rate is a
/// target for this crate's own `WbDdc` (see `stream.rs`) rather than anything
/// the hardware is asked for: they are the narrow views that make the low end
/// of HF reachable at all, since the downconverter cannot centre closer to DC
/// than half its own output rate, and the hardware's own list bottoms out
/// well above them. 625 kHz is both the narrowest the DDC's smallest bin
/// count can produce and the one that keeps the whole AM/mediumwave
/// broadcast band tunable — see `sdroxide_types::FobosConfig::default`'s own
/// comment, and `stream.rs`'s
/// `the_default_hf_rate_keeps_the_whole_am_broadcast_band_reachable`. Leaving
/// it out of this list while shipping it as the default is what made the
/// default unreachable from the settings dropdown: it rendered as the
/// selected text with no entry behind it, so touching that control once lost
/// it for good.
///
/// On `Rf` there is no software DDC — `stream::open_rf` asks the hardware for
/// the rate and reports back whatever it landed on — and the receiver snaps
/// any request below 8 Msps up to 8 Msps. So all four are offered there too
/// and none of them is honoured; the achieved rate is reported rather than
/// hidden, which is what keeps the engine's own view of the span correct.
/// They are not filtered per port because this call has no port to filter by:
/// the operator picks the rate in the same settings tab that picks the port,
/// before either reaches the hardware.
///
/// # Safety
/// `dev` must be a still-open handle from [`open`] belonging to `api`.
pub unsafe fn samplerates(api: &ffi::Api, dev: ffi::Dev) -> Result<Vec<f64>> {
    // 128-entry buffer — the header gives the call no length parameter to
    // bound a write with, so this is sized generously rather than guessed
    // tightly.
    let mut values = [0.0f64; 128];
    let mut count: u32 = 0;
    let rc = unsafe { (api.get_samplerates)(dev, values.as_mut_ptr(), &mut count) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_get_samplerates", api.err_text(rc)));
    }
    let mut values: Vec<f64> = values[..(count as usize).min(values.len())].to_vec();
    for extra in EXTRA_HF_RATES_HZ {
        if !values.iter().any(|&v| (v - extra).abs() < 1.0) {
            values.push(extra);
        }
    }
    values.sort_by(|a, b| a.partial_cmp(b).expect("sample rates are never NaN"));
    Ok(values)
}

/// `fobos_rx_get_board_info` on an already-open device.
///
/// # Safety
/// `dev` must be a still-open handle from [`open`] belonging to `api`.
pub unsafe fn board_info(api: &ffi::Api, dev: ffi::Dev) -> Result<BoardInfo> {
    let mut hw = [0 as c_char; ffi::INFO_LEN];
    let mut fw = [0 as c_char; ffi::INFO_LEN];
    let mut mfr = [0 as c_char; ffi::INFO_LEN];
    let mut product = [0 as c_char; ffi::INFO_LEN];
    let mut serial = [0 as c_char; ffi::INFO_LEN];
    let rc = unsafe {
        (api.get_board_info)(
            dev,
            hw.as_mut_ptr(),
            fw.as_mut_ptr(),
            mfr.as_mut_ptr(),
            product.as_mut_ptr(),
            serial.as_mut_ptr(),
        )
    };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_get_board_info", api.err_text(rc)));
    }
    Ok(BoardInfo {
        hw_revision: cstr_buf_to_string(&hw),
        fw_version: cstr_buf_to_string(&fw),
        manufacturer: cstr_buf_to_string(&mfr),
        product: cstr_buf_to_string(&product),
        serial: cstr_buf_to_string(&serial),
    })
}

/// Read a `char[N]` the library filled as a C string, tolerating one that
/// isn't NUL-terminated within bounds (a buffer exactly filled to its own
/// length) rather than assuming the terminator is always there.
fn cstr_buf_to_string(buf: &[c_char]) -> String {
    let bytes: &[u8] = unsafe { std::slice::from_raw_parts(buf.as_ptr().cast(), buf.len()) };
    match bytes.iter().position(|&b| b == 0) {
        Some(end) => String::from_utf8_lossy(&bytes[..end]).trim().to_string(),
        None => {
            // No NUL within bounds at all — treat the whole buffer as text
            // rather than reaching past it the way `CStr::from_ptr` would.
            String::from_utf8_lossy(bytes).trim().to_string()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `sdroxide_types::FobosConfig`'s default `sample_rate_hz` is 625 kHz,
    /// and the settings dropdown builds its entries from whatever this
    /// module hands back once a receiver has answered. A default with no
    /// entry behind it renders as selected text and then cannot be chosen
    /// again — so the narrow rates have to survive here as well as in
    /// `FobosConfig::SAMPLE_RATES`, which is the *other* list, shown only
    /// until a device answers. The figure is duplicated rather than imported
    /// because this crate deliberately has no dependency on
    /// `sdroxide-types` — see `Cargo.toml`'s own comment on why.
    #[test]
    fn the_added_rates_include_the_backends_own_default() {
        assert!(EXTRA_HF_RATES_HZ.iter().any(|r| (r - 625_000.0).abs() < 1.0));
        // Every one of them is below the 8 Msps floor the hardware's own list
        // bottoms out at, or there would be no reason to add it.
        assert!(EXTRA_HF_RATES_HZ.iter().all(|&r| r < 8_000_000.0));
    }
}
