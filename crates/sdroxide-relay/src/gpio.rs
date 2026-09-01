//! A Linux GPIO line — the Raspberry Pi header, and any other board with a
//! `/dev/gpiochip*`.
//!
//! The character device (v2), not the deprecated sysfs interface: sysfs was
//! removed from mainline in 2020 and its per-line files cost a path lookup and
//! two syscalls apiece. One `GPIO_V2_GET_LINE_IOCTL` at open claims every line
//! at once and hands back a single file descriptor; each change after that is
//! one ioctl on it.
//!
//! Written against the kernel's `linux/gpio.h` rather than through a crate,
//! for the reason `sdroxide-hpsdr` reaches the HL2's I²C tunnel itself: it is
//! two structures and two ioctls, and adding a dependency for that would cost
//! more to keep up to date than the code does.
//!
//! Linux only. Everywhere else this module does not exist and the settings
//! panel's GPIO option refuses with a sentence.

#![cfg(target_os = "linux")]

use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

use crate::error::{Error, Result};
use crate::frame::ChannelMask;
use crate::transport::RelayTransport;

/// `linux/gpio.h`: the maximum lines one request may carry.
const GPIO_V2_LINES_MAX: usize = 64;
const GPIO_MAX_NAME_SIZE: usize = 32;

/// `GPIO_V2_LINE_FLAG_OUTPUT`.
const FLAG_OUTPUT: u64 = 1 << 1;

#[repr(C)]
#[derive(Clone, Copy)]
struct LineAttribute {
    id: u32,
    padding: u32,
    value: u64,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct LineConfigAttribute {
    attr: LineAttribute,
    mask: u64,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct LineConfig {
    flags: u64,
    num_attrs: u32,
    padding: [u32; 5],
    attrs: [LineConfigAttribute; 10],
}

#[repr(C)]
struct LineRequest {
    offsets: [u32; GPIO_V2_LINES_MAX],
    consumer: [u8; GPIO_MAX_NAME_SIZE],
    config: LineConfig,
    num_lines: u32,
    event_buffer_size: u32,
    padding: [u32; 5],
    fd: i32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct LineValues {
    bits: u64,
    mask: u64,
}

/// `_IOWR(0xB4, nr, size)` — `GPIO_V2_GET_LINE_IOCTL` is `nr` 0x07 and
/// `GPIO_V2_LINE_SET_VALUES_IOCTL` is 0x0F.
///
/// The `asm-generic` encoding: direction in the top two bits, size, type,
/// number. Correct on x86_64, aarch64, arm and riscv — every target this
/// program is built for. Alpha, MIPS, PA-RISC, PowerPC and SPARC number the
/// direction bits differently and would need their own arm here.
const fn iowr(nr: u64, size: usize) -> u64 {
    const READ: u64 = 2;
    const WRITE: u64 = 1;
    ((READ | WRITE) << 30) | ((size as u64) << 16) | (0xB4 << 8) | nr
}

pub struct GpioTransport {
    /// The line request's fd. The lines are held for as long as it is open and
    /// handed back to the kernel when it is dropped — which is why the chip's
    /// own descriptor is not kept: it has done its job once the request exists.
    lines: OwnedFd,
    path: String,
    /// Channel `n` drives `offsets[n - 1]`; this is the request-local index,
    /// which is what the values ioctl wants.
    managed: ChannelMask,
    last: Option<ChannelMask>,
}

impl GpioTransport {
    /// `offsets` is one line per channel *number*, so channel 1 drives
    /// `offsets[0]` and channel 5 drives `offsets[4]`.
    ///
    /// Indexed by number rather than by position in the channel table because
    /// the kernel's values ioctl addresses lines by their position in this
    /// request, and everything above here addresses contacts by their number.
    /// Making the two the same is what keeps a channel table with a gap in it
    /// from operating the wrong line.
    pub fn open(path: &str, offsets: &[u32], managed: ChannelMask) -> Result<GpioTransport> {
        if offsets.is_empty() {
            return Err(Error::Config("no GPIO lines listed for the T/R switch".into()));
        }
        let highest = ChannelMask::BITS - managed.leading_zeros();
        if managed != 0 && offsets.len() < highest as usize {
            return Err(Error::Config(format!(
                "the T/R switch drives contact {highest} but only {} GPIO line(s) are listed",
                offsets.len()
            )));
        }
        if offsets.len() > GPIO_V2_LINES_MAX {
            return Err(Error::Config(format!(
                "a GPIO request carries at most {GPIO_V2_LINES_MAX} lines"
            )));
        }
        let chip = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| Error::opening(path, e))?;

        let mut req = LineRequest {
            offsets: [0; GPIO_V2_LINES_MAX],
            consumer: [0; GPIO_MAX_NAME_SIZE],
            config: LineConfig {
                flags: FLAG_OUTPUT,
                num_attrs: 0,
                padding: [0; 5],
                attrs: [LineConfigAttribute {
                    attr: LineAttribute { id: 0, padding: 0, value: 0 },
                    mask: 0,
                }; 10],
            },
            num_lines: offsets.len() as u32,
            event_buffer_size: 0,
            padding: [0; 5],
            fd: -1,
        };
        req.offsets[..offsets.len()].copy_from_slice(offsets);
        // Whose it is, for `gpioinfo` and for the next person wondering what
        // has claimed the line.
        let name = b"sdroxide-tr-switch";
        req.consumer[..name.len()].copy_from_slice(name);

        let rc = unsafe {
            libc::ioctl(
                chip.as_raw_fd(),
                iowr(0x07, std::mem::size_of::<LineRequest>()) as _,
                &mut req as *mut LineRequest,
            )
        };
        if rc < 0 {
            return Err(Error::opening(path, std::io::Error::last_os_error()));
        }
        let lines = unsafe { OwnedFd::from_raw_fd(req.fd) };
        drop(chip);
        Ok(GpioTransport { lines, path: path.to_string(), managed, last: None })
    }
}

impl RelayTransport for GpioTransport {
    fn apply(&mut self, want: ChannelMask) -> Result<()> {
        let want = want & self.managed;
        if self.last == Some(want) {
            return Ok(());
        }
        // Every managed line in one ioctl — which is also what makes the lines
        // change together rather than one syscall apart.
        let mut vals = LineValues { bits: u64::from(want), mask: u64::from(self.managed) };
        let rc = unsafe {
            libc::ioctl(
                self.lines.as_raw_fd(),
                iowr(0x0F, std::mem::size_of::<LineValues>()) as _,
                &mut vals as *mut LineValues,
            )
        };
        if rc < 0 {
            self.last = None;
            return Err(std::io::Error::last_os_error().into());
        }
        self.last = Some(want);
        Ok(())
    }

    fn round_trip(&self) -> Duration {
        // One ioctl on a held fd: microseconds. Rounded up to a millisecond
        // because nothing above this benefits from pretending otherwise.
        Duration::from_millis(1)
    }

    fn describe(&self) -> String {
        format!("GPIO lines on {}", self.path)
    }
}
