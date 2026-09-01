//! The process-global connection to the SDRplay API service.
//!
//! The vendor API is process-global state — one `sdrplay_api_Open` serves
//! every device — so this module owns it behind one mutex. Everything that
//! touches the service's device table (enumerate, select, release) runs under
//! that mutex, which is what makes the settings dialog's Rescan safe while a
//! device is streaming: the API's own `LockDeviceApi`/`UnlockDeviceApi` pair
//! is designed for exactly that, and the mutex keeps this process from
//! interleaving its halves.

use std::sync::{Arc, Mutex, OnceLock};

use sdroxide_types::{SdrPlayDevice, SdrPlayDuoTuner, SdrPlayModel};

use crate::error::{Error, Result};
use crate::ffi;

struct ApiState {
    /// The loaded library, kept for the life of the process. Never unloaded:
    /// callback pointers handed to the service must stay valid.
    api: Option<Arc<ffi::Api>>,
    /// Whether `sdrplay_api_Open` has succeeded and not been reset.
    opened: bool,
    /// One log line per absence, not one per rescan tick.
    complained: bool,
    /// The receivers this process has selected and not yet released, as
    /// (serial, `hwVer`) — one entry per live selection.
    ///
    /// The service's device table only lists receivers that are *free*: the
    /// moment `SelectDevice` succeeds the device disappears from
    /// `GetDevices`, for us as much as for anybody else. So on a station with
    /// two RSPs, opening one left the picker listing the other and nothing
    /// else — the operator's own receiver had vanished from the list of
    /// receivers, and every row the dialog draws from the listed model (ports,
    /// tuners, LNA range) then described the wrong box (issue #259). This is
    /// what puts it back.
    held: Vec<(String, u8)>,
}

fn state() -> &'static Mutex<ApiState> {
    static STATE: OnceLock<Mutex<ApiState>> = OnceLock::new();
    STATE.get_or_init(|| {
        Mutex::new(ApiState { api: None, opened: false, complained: false, held: Vec::new() })
    })
}

/// Load the library and connect to the service, both idempotent.
fn ensure_open(s: &mut ApiState) -> Result<Arc<ffi::Api>> {
    let api = match &s.api {
        Some(a) => Arc::clone(a),
        None => match ffi::Api::load() {
            Ok(a) => {
                let a = Arc::new(a);
                s.api = Some(Arc::clone(&a));
                a
            }
            Err(e) => {
                if !s.complained {
                    s.complained = true;
                    tracing::debug!("SDRplay backend unavailable: {e}");
                }
                return Err(Error::LibMissing(e));
            }
        },
    };
    if !s.opened {
        let err = unsafe { (api.open)() };
        if err != ffi::ERR_SUCCESS {
            // A present library whose Open fails means the background service
            // is not there to answer, whatever the exact code says.
            if !s.complained {
                s.complained = true;
                tracing::debug!("SDRplay service not reachable: {}", api.err_text(err));
            }
            return Err(Error::ServiceDown(api.err_text(err)));
        }
        s.opened = true;
        s.complained = false;
        let mut ver = 0.0f32;
        if unsafe { (api.api_version)(&mut ver) } == ffi::ERR_SUCCESS {
            tracing::info!("SDRplay API connected, version {ver:.2}");
            if !(3.0..4.0).contains(&ver) {
                tracing::warn!(
                    "SDRplay API version {ver:.2} is not the 3.x these bindings were written \
                     against — proceeding, but expect the service to refuse"
                );
            }
        }
    }
    Ok(api)
}

/// Drop the service connection so the next call reconnects from scratch.
///
/// Called when the service stops answering mid-session: the engine's
/// background retry keeps calling [`crate::spawn`], and without this it would
/// keep talking down a dead socket instead of dialling again.
pub(crate) fn reset() {
    let mut s = state().lock().expect("sdrplay api state poisoned");
    if s.opened
        && let Some(api) = &s.api
    {
        unsafe { (api.close)() };
    }
    s.opened = false;
}

/// Fetch the service's device table. Must be called with the state lock held;
/// takes and always releases the API's own device lock.
fn devices_locked(api: &ffi::Api) -> Result<Vec<ffi::DeviceT>> {
    let err = unsafe { (api.lock_device_api)() };
    if err != ffi::ERR_SUCCESS {
        return Err(Error::from_status(api, "LockDeviceApi", err));
    }
    let mut devs = [ffi::DeviceT::zeroed(); ffi::MAX_DEVICES];
    let mut n: u32 = 0;
    let err = unsafe { (api.get_devices)(devs.as_mut_ptr(), &mut n, ffi::MAX_DEVICES as u32) };
    unsafe { (api.unlock_device_api)() };
    if err != ffi::ERR_SUCCESS {
        return Err(Error::from_status(api, "GetDevices", err));
    }
    Ok(devs[..(n as usize).min(ffi::MAX_DEVICES)]
        .iter()
        .filter(|d| d.valid != 0)
        .copied()
        .collect())
}

/// List the RSPs this machine has, or say why that is impossible — the
/// distinction `--probe` needs between "no library", "no service" and simply
/// "no devices".
///
/// The receivers this process already holds lead the list, ahead of the free
/// ones the service reports: they are missing from the service's table
/// precisely *because* they are ours, and a picker that leaves them out is a
/// picker that cannot describe the receiver it is open on. Leading rather
/// than trailing because an empty serial means "the first one found", and the
/// one already found is the one already open.
pub fn try_list() -> Result<Vec<SdrPlayDevice>> {
    let mut s = state().lock().expect("sdrplay api state poisoned");
    let api = ensure_open(&mut s)?;
    let mut out: Vec<SdrPlayDevice> = s
        .held
        .iter()
        .map(|(serial, hw_ver)| SdrPlayDevice { serial: serial.clone(), hw_ver: *hw_ver })
        .collect();
    for d in devices_locked(&api)? {
        let serial = d.serial();
        // A device cannot be both free and ours, but the service has been
        // known to list a stale entry across a re-enumeration; one row per
        // receiver either way.
        if !out.iter().any(|h| h.serial == serial) {
            out.push(SdrPlayDevice { serial, hw_ver: d.hw_ver });
        }
    }
    Ok(out)
}

/// List the RSPs the service reports. Best-effort: no library, no service or
/// no devices all come back as an empty list, because for a Rescan button
/// those are the same answer.
pub fn list() -> Vec<SdrPlayDevice> {
    match try_list() {
        Ok(devs) => devs,
        Err(e) => {
            tracing::debug!("SDRplay enumeration failed: {e}");
            Vec::new()
        }
    }
}

/// Select the device `serial` names (empty = the first one found) and hand
/// its API handle over.
///
/// An RSPduo is put in single-tuner mode on the requested tuner, or — with
/// `dual` — in dual-tuner mode on both. Either way the choice is fixed here,
/// at selection time, which is why changing it costs a reopen. Dual-tuner
/// mode also fixes the ADC clock, and that number goes in the device record
/// rather than in the parameter block: `SelectDevice` is what programs it.
pub(crate) fn select(
    serial: &str,
    duo_tuner: SdrPlayDuoTuner,
    dual: bool,
) -> Result<(Arc<ffi::Api>, ffi::DeviceT)> {
    let mut s = state().lock().expect("sdrplay api state poisoned");
    let api = ensure_open(&mut s)?;

    let err = unsafe { (api.lock_device_api)() };
    if err != ffi::ERR_SUCCESS {
        return Err(Error::from_status(&api, "LockDeviceApi", err));
    }
    // From here every path must unlock.
    let result = (|| {
        let mut devs = [ffi::DeviceT::zeroed(); ffi::MAX_DEVICES];
        let mut n: u32 = 0;
        let err = unsafe { (api.get_devices)(devs.as_mut_ptr(), &mut n, ffi::MAX_DEVICES as u32) };
        if err != ffi::ERR_SUCCESS {
            return Err(Error::from_status(&api, "GetDevices", err));
        }
        let want = serial.trim();
        let mut dev = *devs[..(n as usize).min(ffi::MAX_DEVICES)]
            .iter()
            .filter(|d| d.valid != 0)
            .find(|d| want.is_empty() || d.serial() == want)
            .ok_or_else(|| {
                Error::NotFound(if want.is_empty() {
                    "no SDRplay RSP found — is one plugged in, and is the SDRplay API service \
                     running?"
                        .into()
                } else {
                    format!(
                        "no SDRplay RSP with serial {want} found — replug it, or pick another \
                         receiver in Settings → Radio"
                    )
                })
            })?;

        let model = SdrPlayModel::from_hw_ver(dev.hw_ver);
        if model == SdrPlayModel::RspDuo {
            if dual {
                dev.tuner = ffi::TUNER_BOTH;
                dev.rsp_duo_mode = ffi::RSPDUO_MODE_DUAL_TUNER;
                dev.rsp_duo_sample_freq = crate::device::DUAL_FS_HZ;
            } else {
                dev.tuner = match duo_tuner {
                    SdrPlayDuoTuner::Tuner1 => ffi::TUNER_A,
                    SdrPlayDuoTuner::Tuner2 => ffi::TUNER_B,
                };
                dev.rsp_duo_mode = ffi::RSPDUO_MODE_SINGLE_TUNER;
                // Single-tuner mode lets the ADC clock follow the sample rate.
                dev.rsp_duo_sample_freq = 0.0;
            }
        }

        let err = unsafe { (api.select_device)(&mut dev) };
        if err != ffi::ERR_SUCCESS {
            // The service arbitrates ownership: a select that fails on a
            // device the table just listed is nearly always "someone else has
            // it" — but keep the API's own words in the message too.
            return Err(Error::InUse(format!(
                "{} (serial {}) — {}",
                model.label(),
                dev.serial(),
                api.err_text(err)
            )));
        }
        Ok(dev)
    })();
    unsafe { (api.unlock_device_api)() };
    if let Ok(dev) = &result {
        s.held.push((dev.serial(), dev.hw_ver));
    }
    result.map(|dev| (api, dev))
}

/// Hand a selected device back to the service. Best-effort: on an unplugged
/// device the service has already reclaimed it and the error means nothing.
pub(crate) fn release(dev: &mut ffi::DeviceT) {
    let mut s = state().lock().expect("sdrplay api state poisoned");
    let serial = dev.serial();
    if let Some(i) = s.held.iter().position(|(h, _)| *h == serial) {
        s.held.remove(i);
    }
    if let Some(api) = &s.api {
        let err = unsafe { (api.release_device)(dev) };
        if err != ffi::ERR_SUCCESS {
            tracing::debug!("ReleaseDevice: {}", api.err_text(err));
        }
    }
}
