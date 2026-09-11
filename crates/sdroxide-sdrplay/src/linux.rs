//! What the machine looks like from *outside* the vendor's library.
//!
//! The SDRplay API is a closed library talking to a background service, and
//! when it cannot find a receiver all it says is `sdrplay_api_Fail`. That is
//! the same answer for "nothing is plugged in" as for the one cause that
//! accounts for most of them on Linux: the kernel got there first.
//!
//! An RSP1 is a Mirics MSi2500 + MSi001, and it enumerates as `1df7:2500` —
//! which is exactly what the in-tree `msi2500` DVB driver binds to. Every
//! mainstream distribution ships that driver and loads it on hotplug, so on a
//! stock Ubuntu the kernel claims the receiver before `sdrplay_apiService` ever
//! sees it. The RSPs after it (`1df7:3000` and up) are not in the driver's
//! table, which is why this is an RSP1 story specifically, and why the same
//! receiver on the same version of sdroxide works on a Mac and not here
//! (issue #277).
//!
//! None of this is something sdroxide can fix — unbinding another driver's
//! device is the operator's call and needs root — but it is something it can
//! *say*, which is the whole of this module.

/// SDRplay's USB vendor id, as `idVendor` spells it.
const VENDOR: &str = "1df7";

/// The in-tree drivers that will claim a Mirics front end out from under the
/// vendor service. `msi2500` is the USB bridge and `msi001` the tuner;
/// `sdr_msi3101` is the older out-of-tree name for the same thing.
const CLAIMANTS: [&str; 3] = ["msi2500", "msi001", "sdr_msi3101"];

/// A sentence to add to an SDRplay error, or `None` when this machine has
/// nothing to add — including on every OS but Linux, where none of it applies.
pub(crate) fn hint() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        diagnose(std::path::Path::new("/sys/bus/usb/devices"))
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// [`hint`], against a sysfs tree named explicitly so it can be tested.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn diagnose(root: &std::path::Path) -> Option<String> {
    let entries = std::fs::read_dir(root).ok()?;
    let mut seen_rsp = false;
    let mut claimed: Vec<String> = Vec::new();
    for entry in entries.flatten() {
        let dev = entry.path();
        // A USB *device* directory has idVendor; the interface directories
        // under it (`1-2:1.0`) do not, which is what tells the two apart.
        let Ok(vendor) = std::fs::read_to_string(dev.join("idVendor")) else { continue };
        if !vendor.trim().eq_ignore_ascii_case(VENDOR) {
            continue;
        }
        seen_rsp = true;
        // The driver binds to an *interface*, so the symlink to look for is
        // one level down. Interfaces of this device are the subdirectories
        // whose names start with the device's own.
        let Some(prefix) = dev.file_name().and_then(|n| n.to_str()) else { continue };
        let Ok(kids) = std::fs::read_dir(&dev) else { continue };
        for kid in kids.flatten() {
            let name = kid.file_name();
            let Some(name) = name.to_str() else { continue };
            if !name.starts_with(prefix) || !name.contains(':') {
                continue;
            }
            let Ok(target) = std::fs::read_link(kid.path().join("driver")) else { continue };
            let Some(driver) = target.file_name().and_then(|n| n.to_str()) else { continue };
            if CLAIMANTS.contains(&driver) && !claimed.iter().any(|d| d == driver) {
                claimed.push(driver.to_string());
            }
        }
    }
    if !claimed.is_empty() {
        claimed.sort();
        return Some(format!(
            "the kernel's own {} driver has claimed the receiver, so the SDRplay service cannot \
             open it — this is why an RSP1 works on macOS and Windows but not on a stock Linux. \
             Put \"blacklist sdr_msi3101\", \"blacklist msi001\" and \"blacklist msi2500\" in \
             /etc/modprobe.d/blacklist.conf, run \"sudo rmmod msi001 msi2500\", then unplug the \
             receiver and plug it back in.",
            claimed.join(" and "),
        ));
    }
    if !seen_rsp {
        return Some(
            "no SDRplay receiver is on this machine's USB bus at all — nothing with vendor id \
             1df7 is enumerated, so the service has nothing to find."
                .to_string(),
        );
    }
    None
}

// Unix only: the fixture builds symlinks, and the fault this module is about
// only exists on Linux anyway.
#[cfg(all(test, unix))]
mod tests {
    use super::*;

    /// A directory that removes itself, so a panicking test does not leave a
    /// fake sysfs behind. No `tempfile` in this workspace, and one dependency
    /// for three tests would be a poor trade.
    struct Scratch(std::path::PathBuf);

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    impl Scratch {
        fn path(&self) -> &std::path::Path {
            &self.0
        }
    }

    /// Build a fake sysfs USB tree: one device directory per `(name, vendor)`,
    /// with an interface bound to `driver` where one is given.
    fn tree(devs: &[(&str, &str, Option<&str>)]) -> Scratch {
        use std::sync::atomic::{AtomicU32, Ordering};
        static SEQ: AtomicU32 = AtomicU32::new(0);
        let dir = Scratch(std::env::temp_dir().join(format!(
            "sdroxide-sdrplay-usb-{}-{}",
            std::process::id(),
            SEQ.fetch_add(1, Ordering::Relaxed)
        )));
        let _ = std::fs::remove_dir_all(dir.path());
        // Somewhere for the driver symlinks to point at, so `read_link` has a
        // real target — sysfs links are relative and dangling here otherwise.
        let drivers = dir.path().join("drivers");
        std::fs::create_dir_all(&drivers).expect("drivers");
        for (name, vendor, driver) in devs {
            let dev = dir.path().join(name);
            std::fs::create_dir_all(&dev).expect("dev");
            std::fs::write(dev.join("idVendor"), format!("{vendor}\n")).expect("idVendor");
            if let Some(driver) = driver {
                let iface = dev.join(format!("{name}:1.0"));
                std::fs::create_dir_all(&iface).expect("iface");
                let target = drivers.join(driver);
                std::fs::create_dir_all(&target).expect("driver dir");
                std::os::unix::fs::symlink(&target, iface.join("driver")).expect("symlink");
            }
        }
        dir
    }

    /// The fault as reported: an RSP1 on the bus with `msi2500` holding it.
    #[test]
    fn a_kernel_driver_holding_the_receiver_is_named() {
        let dir = tree(&[("1-2", "1df7", Some("msi2500"))]);
        let said = diagnose(dir.path()).expect("something to say");
        assert!(said.contains("msi2500"), "{said}");
        assert!(said.contains("blacklist"), "the remedy has to be in it: {said}");
    }

    /// A receiver the kernel has left alone is not the problem, so this says
    /// nothing rather than inventing a cause.
    #[test]
    fn an_unclaimed_receiver_draws_no_conclusion() {
        let dir = tree(&[("1-2", "1df7", None)]);
        assert_eq!(diagnose(dir.path()), None);
        // Bound to something that is not one of the Mirics drivers — a hub, a
        // vendor tool — is equally not a conclusion.
        let dir = tree(&[("1-2", "1df7", Some("usbfs"))]);
        assert_eq!(diagnose(dir.path()), None);
    }

    /// Nothing of SDRplay's on the bus is worth saying too: it is the
    /// difference between "unplugged" and "the service is broken".
    #[test]
    fn an_empty_bus_says_so() {
        let dir = tree(&[("1-1", "1d6b", None), ("1-3", "0bda", Some("dvb_usb_rtl28xxu"))]);
        let said = diagnose(dir.path()).expect("something to say");
        assert!(said.contains("1df7"), "{said}");
    }

    /// A sysfs that is not there (or not readable) is not a diagnosis.
    #[test]
    fn a_missing_sysfs_is_silent() {
        assert_eq!(diagnose(std::path::Path::new("/nonexistent/usb/devices")), None);
    }
}
