//! Errors, and the translation of USB failures into sentences an operator can
//! act on.
//!
//! Everything here ends up in front of a user: [`crate::Error`] is what
//! `HackRfSource::open` returns and what `IqSource::open_status` puts on
//! screen. "permission denied (os error 13)" tells nobody what to do; "install
//! the udev rule and re-plug the radio" does.

use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// No radio matched — either none is plugged in, or none whose serial ends
    /// with the configured one.
    #[error("{0}")]
    NotFound(String),

    /// The device is there but we cannot have it. Carries the actionable
    /// sentence, not the errno.
    #[error("{0}")]
    Access(String),

    /// A USB transfer failed.
    ///
    /// `op` names the request as well as the direction — "control write
    /// SET_FREQ", not "control write". A stall is the radio refusing *that*
    /// request, and a message that does not say which one is a report nobody
    /// can act on without a second round trip to the operator (issue #220).
    #[error("USB {op} failed: {source}")]
    Transfer { op: String, source: nusb::transfer::TransferError },

    /// A control transfer returned fewer bytes than the caller needed.
    #[error("short control read on request {request}: wanted {want} bytes, got {got}")]
    ShortRead { request: u8, want: usize, got: usize },

    /// A gain request the firmware rejected.
    ///
    /// Its own kind because the gain setters are the one group that answers
    /// rather than just acting: they are IN transfers returning a single byte,
    /// and a zero there is the firmware saying it would not take the value.
    /// Reporting that as a generic transfer failure would send an operator
    /// looking at their cable.
    #[error("the radio refused {element} = {value} dB")]
    GainRefused { element: &'static str, value: u32 },

    /// The radio's descriptors are not the shape this driver expects — a bulk
    /// endpoint missing, most likely. Carries what was found, so a bug report
    /// names the real layout.
    #[error("{0}")]
    Descriptor(String),

    /// A setting the hardware cannot produce.
    #[error("{0}")]
    Unsupported(String),

    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),
}

impl Error {
    /// Translate a device-open failure into an instruction.
    ///
    /// These cases are the entire support burden of this backend, so they are
    /// worth naming precisely. `EBUSY` is nearly always another SDR program
    /// still holding the radio rather than a broken install — and as with the
    /// Airspy HF+ there is no kernel driver to blame, because nothing in-tree
    /// claims `1d50:6089`.
    pub fn from_open(e: nusb::Error, what: &dyn fmt::Display) -> Error {
        use nusb::ErrorKind;
        match e.kind() {
            ErrorKind::PermissionDenied => Error::Access(format!(
                "permission denied opening {what} — install the udev rule \
                 (see the README) and re-plug the radio"
            )),
            ErrorKind::Busy => Error::Access(format!(
                "{what} is held by another program (SDR++, SDRangel, gqrx, \
                 hackrf_transfer, hackrf_sweep, a SoapySDR client)"
            )),
            // On Windows the radio must be bound to WinUSB before anything can
            // claim it; unbound, the open fails as unsupported or not-found
            // rather than as a permission problem. A HackRF ships with the
            // Microsoft OS descriptors that ask Windows to do this by itself,
            // so this is rarer here than on an RTL-SDR — but a stick that has
            // been through Zadig for something else can still land here.
            ErrorKind::Unsupported | ErrorKind::NotFound if cfg!(windows) => {
                Error::Access(format!(
                    "{what} is not bound to WinUSB — run Zadig and select the \
                 WinUSB driver for this device"
                ))
            }
            ErrorKind::Disconnected => {
                Error::NotFound(format!("{what} was unplugged while opening it"))
            }
            // macOS passes unrecognised IOKit failures through as a bare hex
            // `IOReturn`, and the one that shows up here — kIOReturnNoResources,
            // 0xe00002be, from claiming an interface on a device that is
            // mid-hotplug — tells an operator nothing. Both remedies are
            // physical.
            ErrorKind::Other if cfg!(target_os = "macos") => Error::Access(format!(
                "cannot open {what}: {e} — quit any other SDR software holding \
                 the radio, then unplug it and plug it back in"
            )),
            _ => Error::Access(format!("cannot open {what}: {e}")),
        }
    }
}
