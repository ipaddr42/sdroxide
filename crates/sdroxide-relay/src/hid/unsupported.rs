//! Platforms with no HID backend here.
//!
//! An answer rather than a silence: the settings panel lists no devices and the
//! open says why, instead of the operator wondering whether their board is
//! broken.

use crate::error::{Error, Result};

use super::{HidDev, HidEntry};

pub fn enumerate(_ids: &[(u16, u16)]) -> Vec<HidEntry> {
    Vec::new()
}

pub fn open(_key: &str) -> Result<Box<dyn HidDev>> {
    Err(Error::Unsupported(
        "USB HID relays and sound-card GPIO are not supported on this platform — use a serial \
         relay board, an RTS/DTR line, or the external command hook"
            .into(),
    ))
}
