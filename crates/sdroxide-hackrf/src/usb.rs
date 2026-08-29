//! Enumeration, opening, and the vendor requests as control transfers.
//!
//! # The one invariant in this crate
//!
//! [`UsbDev`] is deliberately **not `Clone`**, and every control transfer runs
//! on the stream thread and nowhere else. `nusb`'s `Interface` is `Send + Sync`,
//! so a second thread poking the radio would compile and would be wrong — and
//! wrong here is worse than on a receiver. A direction change is a sequence of
//! six control transfers that must not be interleaved with anything: a gain
//! write landing between the mode change and the amplifier write is exactly the
//! SoapyHackRF defect this driver exists to avoid, and a tuning write landing
//! mid-transition puts the transmitter on the wrong frequency. Everything from
//! outside arrives as a [`crate::handle::Ctrl`] message.
//!
//! # The `Transport` seam
//!
//! Control transfers go through [`Transport`] rather than straight to `nusb`.
//! None of the other native USB backends here does that, and the reason is
//! transmit: a receive driver that mis-sequences its control transfers produces
//! bad samples, which a test can inspect after the fact; a transmit driver that
//! mis-sequences them produces RF, which it cannot. The seam lets
//! [`crate::device`]'s orderings be asserted against a recording fake with
//! nothing plugged in.
//!
//! The seam stops at control transfers on purpose. Bulk streaming has no
//! in-process stand-in worth the name — a real one would mean `usbip` or
//! `dummy_hcd`, a kernel module in CI — and none of the orderings live there.
//!
//! # Firmware differences
//!
//! Some requests were added over the product's life and older firmware stalls
//! them. libhackrf gates them on `bcdDevice`, which the radio reports as its own
//! API level; [`Request::since_api`] carries the same table, and
//! [`UsbDev::api_version`] is what it is compared against.

use nusb::MaybeFuture;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use sdroxide_types::HackRfDevice;

use crate::error::{Error, Result};
use crate::protocol::{
    BULK_IN, BULK_OUT, BoardKind, CONFIGURATION, CTRL_TIMEOUT, GainSetter, INTERFACE, PID_DFU,
    PID_HACKRF_ONE, PID_JAWBREAKER, PID_RAD1O, Request, VID, VID_DFU, serial_matches,
};
use crate::trace::Trace;

/// Whether a vendor/product pair is a HackRF this driver drives.
fn is_hackrf(vid: u16, pid: u16) -> bool {
    vid == VID && matches!(pid, PID_HACKRF_ONE | PID_JAWBREAKER | PID_RAD1O)
}

/// Enumerate the HackRFs on the USB bus.
///
/// Non-invasive: no device is opened. The strings come from sysfs on Linux, the
/// registry on Windows and IOKit on macOS, so this is safe to call at any time
/// — including while one is streaming or transmitting, which is what makes the
/// settings dialog's Rescan button harmless.
pub fn list() -> Vec<HackRfDevice> {
    let devices = match nusb::list_devices().wait() {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!("USB enumeration failed: {e}");
            return Vec::new();
        }
    };
    devices
        .filter(|d| {
            is_hackrf(d.vendor_id(), d.product_id())
                || (d.vendor_id() == VID_DFU && d.product_id() == PID_DFU)
        })
        .map(|d| {
            let dfu = d.vendor_id() == VID_DFU;
            // The board id would be more precise, but reading it needs a
            // control transfer and enumeration must not open anything. The
            // product id and the product string are what can be known for free
            // — and the string is not a nicety here: a HackRF Pro carries the
            // same product id as a HackRF One, and the settings UI decides
            // which rates to offer from this list.
            let kind = BoardKind::from_product(d.product_id(), d.product_string());
            let name = match d.product_string() {
                Some(p) if !p.is_empty() => p.to_string(),
                _ => kind.name().to_string(),
            };
            HackRfDevice {
                serial: d.serial_number().map(|s| s.trim().to_string()).filter(|s| !s.is_empty()),
                name: if dfu {
                    // Worth naming rather than hiding: a radio left in DFU by an
                    // interrupted `hackrf_spiflash` is otherwise just "gone",
                    // and this driver cannot flash it back.
                    format!("{name} [in DFU — reflash it with hackrf_spiflash, then re-plug]")
                } else {
                    name
                },
                pid: d.product_id(),
            }
        })
        .collect()
}

/// Control transfers, behind a trait so [`crate::device`] can be tested without
/// a radio. See the module header for why this seam exists here and nowhere
/// else in the workspace.
pub trait Transport: Send {
    fn control_in(&self, req: Request, value: u16, index: u16, len: u16) -> Result<Vec<u8>>;
    fn control_out(&self, req: Request, value: u16, index: u16, data: &[u8]) -> Result<()>;
    /// The radio's API level, from `bcdDevice`. Compared against
    /// [`Request::since_api`] to decide whether an absent request is a fault.
    fn api_version(&self) -> u16;
    fn label(&self) -> &str;
    fn trace(&self) -> &Trace;
}

/// Helpers every `Transport` gets for free. Separate from the trait so the
/// trait stays object-safe and a fake only has to implement the four primitives.
pub trait TransportExt: Transport {
    /// A control-OUT with no payload — the shape of most of this protocol.
    fn out(&self, req: Request, value: u16, index: u16) -> Result<()> {
        self.control_out(req, value, index, &[])
    }

    /// A request this firmware may simply not have.
    ///
    /// Any error means "absent", never a fault: an older radio stalls the later
    /// requests, a stalled control transfer self-clears on the next one, and
    /// which `TransferError` a given OS backend reports a stall as is not
    /// something a driver should depend on. The exact error still goes in the
    /// trace, so the mapping becomes visible from a field report even though
    /// nothing branches on it.
    fn optional_out(&self, req: Request, value: u16, index: u16) -> bool {
        debug_assert!(req.is_optional(), "{req:?} is not optional; a failure there is real");
        if !self.has(req) {
            return false;
        }
        self.control_out(req, value, index, &[]).is_ok()
    }

    fn optional_in(&self, req: Request, value: u16, index: u16, len: u16) -> Option<Vec<u8>> {
        debug_assert!(req.is_optional(), "{req:?} is not optional; a failure there is real");
        if !self.has(req) {
            return None;
        }
        self.control_in(req, value, index, len).ok()
    }

    /// Whether this radio's firmware is new enough for `req`.
    fn has(&self, req: Request) -> bool {
        match req.since_api() {
            Some(v) => self.api_version() >= v,
            None => true,
        }
    }

    /// A control-IN whose reply must be at least `len` bytes.
    fn control_in_exact(&self, req: Request, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
        let data = self.control_in(req, value, index, len)?;
        if data.len() < len as usize {
            return Err(Error::ShortRead {
                request: req.code(),
                want: len as usize,
                got: data.len(),
            });
        }
        Ok(data)
    }

    /// Set one gain stage.
    ///
    /// The odd one out in this protocol, and the reason [`GainSetter`] exists:
    /// these are **IN** transfers with the dB value in `wIndex` and `wValue`
    /// zero, and the single byte that comes back is the firmware's verdict — a
    /// zero means it declined. Everything else that sets something is an OUT
    /// with the value in `wValue`. Returns the dB actually applied, which is
    /// the truncated one: the hardware masks the low bits off rather than
    /// rounding, so 15 dB of LNA is 8 dB and the UI has to be told so.
    fn set_gain(&self, stage: GainSetter, db: f64) -> Result<u32> {
        let value = stage.snap(db);
        let reply = self.control_in(stage.request(), 0, value as u16, 1)?;
        match reply.first() {
            Some(0) | None => Err(Error::GainRefused { element: stage.name(), value }),
            Some(_) => Ok(value),
        }
    }
}

impl<T: Transport + ?Sized> TransportExt for T {}

/// An opened radio: the claimed interface plus the identity we opened it by.
///
/// Not `Clone` on purpose — see the module invariant.
pub struct UsbDev {
    iface: nusb::Interface,
    label: String,
    serial: Option<String>,
    speed: Option<nusb::Speed>,
    api: u16,
    kind: BoardKind,
    trace: Trace,
}

impl UsbDev {
    /// Open the radio whose serial ends with `serial`, or the first one found
    /// when it is empty.
    pub fn open(serial: &str, trace: &Trace) -> Result<UsbDev> {
        let want = serial.trim();
        let devices = nusb::list_devices().wait()?;
        let mut candidates = devices.filter(|d| is_hackrf(d.vendor_id(), d.product_id()));

        let info = if want.is_empty() {
            candidates.next().ok_or_else(|| {
                // Name the DFU case explicitly: it is a real state to end up in
                // after an interrupted firmware write, and "no HackRF found"
                // sends people looking at cables.
                let dfu = list().iter().any(|d| d.name.contains("DFU"));
                Error::NotFound(if dfu {
                    "a HackRF is present but sitting in DFU — reflash it with \
                     hackrf_spiflash, then re-plug it"
                        .to_string()
                } else {
                    "no HackRF found on USB".to_string()
                })
            })?
        } else {
            candidates.find(|d| serial_matches(want, d.serial_number())).ok_or_else(|| {
                Error::NotFound(format!(
                    "no HackRF whose serial ends with {want:?} is plugged in \
                         — pick another radio in Settings → Radio, or clear the \
                         serial to use the first one found"
                ))
            })?
        };

        let serial = info.serial_number().map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
        let kind = BoardKind::from_product(info.product_id(), info.product_string());
        let label = match info.product_string() {
            Some(p) if !p.is_empty() => p.to_string(),
            _ => kind.name().to_string(),
        };
        let speed = info.speed();
        // libhackrf's `usb_api_version` is the device descriptor's `bcdDevice`:
        // the radio states its own API level rather than making the host parse
        // a version string.
        let api = info.device_version();

        let device = info.open().wait().map_err(|e| Error::from_open(e, &label))?;

        // Only set the configuration if it is not already the one we want. On
        // Linux `set_configuration` can re-enumerate the device — which would
        // invalidate the handle we are holding — and on Windows it is not
        // supported at all. Every HackRF enumerates configured, so this is
        // normally a no-op that costs one cached read.
        match device.active_configuration() {
            Ok(c) if c.configuration_value() == CONFIGURATION => {}
            Ok(c) => {
                trace.note(format!(
                    "device is on configuration {}, selecting {CONFIGURATION}",
                    c.configuration_value()
                ));
                if let Err(e) = device.set_configuration(CONFIGURATION).wait() {
                    trace.note(format!("set_configuration failed ({e}); continuing"));
                }
            }
            Err(e) => trace.note(format!("no active configuration reported ({e}); continuing")),
        }

        // No alternate setting to select, unlike the Airspy HF+: both bulk
        // endpoints live in the default one.
        let iface = device
            .detach_and_claim_interface(INTERFACE)
            .wait()
            .map_err(|e| Error::from_open(e, &label))?;

        let dev = UsbDev { iface, label, serial, speed, api, kind, trace: trace.clone() };
        dev.check_bulk_endpoints()?;

        tracing::info!(
            "opened {} (usb {VID:04x}:{:04x}, serial {}, {}, API {:04x})",
            dev.label,
            info_pid(&dev),
            dev.serial.as_deref().unwrap_or("none"),
            dev.speed_name(),
            dev.api,
        );
        trace.note(format!(
            "claimed interface {INTERFACE} on {} (serial {}, {}, bcdDevice {:04x})",
            dev.label,
            dev.serial.as_deref().unwrap_or("none"),
            dev.speed_name(),
            dev.api,
        ));
        Ok(dev)
    }

    /// Fail early, and name the real layout when we do.
    ///
    /// Both endpoints are checked at open even though only one is used at a
    /// time: finding out at the first key-down that the transmit endpoint is
    /// not where this driver expects would mean discovering it with the
    /// receive stream already torn down.
    fn check_bulk_endpoints(&self) -> Result<()> {
        let found: Vec<String> = self
            .iface
            .descriptors()
            .flat_map(|d| d.endpoints().collect::<Vec<_>>())
            .map(|e| {
                format!("0x{:02x} {:?} max {}", e.address(), e.transfer_type(), e.max_packet_size())
            })
            .collect();
        self.trace.note(format!("endpoints: [{}]", found.join(", ")));
        let missing: Vec<String> = [BULK_IN, BULK_OUT]
            .iter()
            .filter(|ep| !found.iter().any(|e| e.starts_with(&format!("0x{ep:02x} "))))
            .map(|ep| format!("0x{ep:02x}"))
            .collect();
        if missing.is_empty() {
            return Ok(());
        }
        Err(Error::Descriptor(format!(
            "{} is missing bulk endpoint(s) {}; it offers [{}]. Please report this \
             with the board revision and firmware version.",
            self.label,
            missing.join(" and "),
            found.join(", ")
        )))
    }

    pub fn serial(&self) -> Option<&str> {
        self.serial.as_deref()
    }

    /// What the product id says this board is. Refined by
    /// [`crate::device::Device`] once `BoardIdRead` has answered.
    pub fn kind(&self) -> BoardKind {
        self.kind
    }

    pub fn speed_name(&self) -> &'static str {
        match self.speed {
            Some(nusb::Speed::Low) => "low speed",
            Some(nusb::Speed::Full) => "full speed",
            Some(nusb::Speed::High) => "high speed",
            Some(nusb::Speed::Super) => "SuperSpeed",
            Some(nusb::Speed::SuperPlus) => "SuperSpeed+",
            _ => "unknown link speed",
        }
    }

    /// Whether the link can actually carry the top sample rates.
    ///
    /// 20 Msps is 40 MB/s, which a high-speed link cannot sustain — its
    /// theoretical ceiling is 60 MB/s and real throughput is well under that.
    /// Worth saying at open rather than leaving somebody to diagnose dropped
    /// samples.
    pub fn is_superspeed(&self) -> bool {
        matches!(self.speed, Some(nusb::Speed::Super) | Some(nusb::Speed::SuperPlus))
    }

    /// Borrow the interface so the streaming code can open the bulk endpoints.
    pub fn interface(&self) -> &nusb::Interface {
        &self.iface
    }
}

/// The product id back out of the board kind, for the log line only. A HackRF
/// Pro falls in with the One here and that is not a lossy default: they really
/// do share `0x6089`.
fn info_pid(dev: &UsbDev) -> u16 {
    match dev.kind {
        BoardKind::Jawbreaker => PID_JAWBREAKER,
        BoardKind::Rad1o => PID_RAD1O,
        _ => PID_HACKRF_ONE,
    }
}

impl Transport for UsbDev {
    fn control_in(&self, req: Request, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
        let r = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: req.code(),
                    value,
                    index,
                    length: len,
                },
                CTRL_TIMEOUT,
            )
            .wait();
        match r {
            Ok(data) => {
                self.trace.ctrl(
                    req.code(),
                    &format!("{req:?}"),
                    value,
                    index,
                    len as usize,
                    Some(data.len()),
                    "ok",
                );
                Ok(data)
            }
            Err(source) => {
                self.trace.ctrl(
                    req.code(),
                    &format!("{req:?}"),
                    value,
                    index,
                    len as usize,
                    None,
                    &format!("FAILED: {source}"),
                );
                Err(Error::Transfer { op: format!("control read {req:?}"), source })
            }
        }
    }

    fn control_out(&self, req: Request, value: u16, index: u16, data: &[u8]) -> Result<()> {
        let r = self
            .iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: req.code(),
                    value,
                    index,
                    data,
                },
                CTRL_TIMEOUT,
            )
            .wait();
        match r {
            Ok(()) => {
                self.trace.ctrl(
                    req.code(),
                    &format!("{req:?}"),
                    value,
                    index,
                    data.len(),
                    None,
                    "ok",
                );
                Ok(())
            }
            Err(source) => {
                self.trace.ctrl(
                    req.code(),
                    &format!("{req:?}"),
                    value,
                    index,
                    data.len(),
                    None,
                    &format!("FAILED: {source}"),
                );
                Err(Error::Transfer {
                    op: format!("control write {req:?} (value {value:#06x}, index {index:#06x})"),
                    source,
                })
            }
        }
    }

    fn api_version(&self) -> u16 {
        self.api
    }

    fn label(&self) -> &str {
        &self.label
    }

    fn trace(&self) -> &Trace {
        &self.trace
    }
}

#[cfg(test)]
pub(crate) mod fake {
    //! A recording stand-in for [`Transport`], so [`crate::device`]'s orderings
    //! can be asserted with nothing plugged in.

    use std::sync::Mutex;

    use super::*;

    /// One control transfer as it went out.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct Call {
        pub req: Request,
        pub value: u16,
        pub index: u16,
        pub data: Vec<u8>,
    }

    impl Call {
        /// A short form for assertions: `"SetTransceiverMode(1)"`.
        pub fn brief(&self) -> String {
            format!("{:?}({})", self.req, self.value)
        }
    }

    pub struct FakeTransport {
        calls: Mutex<Vec<Call>>,
        /// Canned replies per request code; anything absent answers with a
        /// single `1`, which is what the gain setters read as "accepted".
        replies: Mutex<Vec<(u8, Vec<u8>)>>,
        /// Request codes this radio refuses, as a real one stalls a request its
        /// board does not have.
        stalls: Mutex<Vec<u8>>,
        api: u16,
        trace: Trace,
    }

    impl Default for FakeTransport {
        fn default() -> Self {
            FakeTransport::new()
        }
    }

    impl FakeTransport {
        pub fn new() -> FakeTransport {
            FakeTransport {
                calls: Mutex::new(Vec::new()),
                replies: Mutex::new(Vec::new()),
                stalls: Mutex::new(Vec::new()),
                // Newest API, so nothing is skipped as too old unless a test
                // asks for that explicitly.
                api: 0x0109,
                trace: Trace::new(),
            }
        }

        pub fn with_api(mut self, api: u16) -> FakeTransport {
            self.api = api;
            self
        }

        pub fn reply(self, req: Request, bytes: &[u8]) -> FakeTransport {
            self.replies.lock().unwrap().push((req.code(), bytes.to_vec()));
            self
        }

        /// Refuse `req` the way a radio whose board does not have it does: a
        /// stalled control transfer.
        pub fn stalling(self, req: Request) -> FakeTransport {
            self.stalls.lock().unwrap().push(req.code());
            self
        }

        fn stalls(&self, req: Request) -> bool {
            self.stalls.lock().unwrap().contains(&req.code())
        }

        pub fn calls(&self) -> Vec<Call> {
            self.calls.lock().unwrap().clone()
        }

        /// Every call as `"Request(value)"`, in order — the form the ordering
        /// assertions read against.
        pub fn brief(&self) -> Vec<String> {
            self.calls().iter().map(Call::brief).collect()
        }

        /// The index of the first call to `req`, if any.
        pub fn first(&self, req: Request) -> Option<usize> {
            self.calls().iter().position(|c| c.req == req)
        }

        /// The indices of every call to `req`.
        pub fn all(&self, req: Request) -> Vec<usize> {
            self.calls().iter().enumerate().filter(|(_, c)| c.req == req).map(|(i, _)| i).collect()
        }

        pub fn clear(&self) {
            self.calls.lock().unwrap().clear();
        }

        fn record(&self, req: Request, value: u16, index: u16, data: &[u8]) {
            self.calls.lock().unwrap().push(Call { req, value, index, data: data.to_vec() });
        }
    }

    impl Transport for FakeTransport {
        fn control_in(&self, req: Request, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
            self.record(req, value, index, &[]);
            let canned = self
                .replies
                .lock()
                .unwrap()
                .iter()
                .find(|(c, _)| *c == req.code())
                .map(|(_, b)| b.clone());
            Ok(canned.unwrap_or_else(|| vec![1u8; len.max(1) as usize]))
        }

        fn control_out(&self, req: Request, value: u16, index: u16, data: &[u8]) -> Result<()> {
            self.record(req, value, index, data);
            if self.stalls(req) {
                return Err(Error::Transfer {
                    op: format!("control write {req:?}"),
                    source: nusb::transfer::TransferError::Stall,
                });
            }
            Ok(())
        }

        fn api_version(&self) -> u16 {
            self.api
        }

        fn label(&self) -> &str {
            "fake HackRF"
        }

        fn trace(&self) -> &Trace {
            &self.trace
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::FakeTransport;
    use super::*;

    /// The gain setters are IN transfers with the value in `wIndex`, and the
    /// byte that comes back is a verdict. Getting either wrong is a gain that
    /// silently does nothing.
    #[test]
    fn a_gain_goes_out_in_windex_and_the_reply_is_a_verdict() {
        let t = FakeTransport::new();
        let applied = t.set_gain(GainSetter::Lna, 16.0).expect("accepted");
        assert_eq!(applied, 16);
        let c = &t.calls()[0];
        assert_eq!(c.req, Request::SetLnaGain);
        assert_eq!(c.value, 0, "wValue is unused for gains");
        assert_eq!(c.index, 16, "the dB figure travels in wIndex");

        // The truncation is reported back, because the UI shows this value.
        let t = FakeTransport::new();
        assert_eq!(t.set_gain(GainSetter::Lna, 15.0).unwrap(), 8);
        assert_eq!(t.calls()[0].index, 8);

        // A zero reply is the firmware declining, not a transfer failure.
        let t = FakeTransport::new().reply(Request::SetVgaGain, &[0]);
        let e = t.set_gain(GainSetter::Vga, 20.0).expect_err("refused");
        assert!(matches!(e, Error::GainRefused { element: "VGA", value: 20 }), "{e}");
    }

    /// An old firmware must not be sent a request it does not have, and the
    /// absence must not read as a fault.
    #[test]
    fn firmware_gated_requests_are_skipped_on_an_old_radio() {
        let new = FakeTransport::new();
        assert!(new.has(Request::SetLeds));
        assert!(new.optional_out(Request::SetLeds, 1, 0));
        assert_eq!(new.calls().len(), 1);

        // 0x0104 predates every gated request above 0x0103.
        let old = FakeTransport::new().with_api(0x0104);
        assert!(old.has(Request::ClkoutEnable), "0x0103 <= 0x0104");
        assert!(!old.has(Request::SetLeds), "0x0107 > 0x0104");
        assert!(!old.optional_out(Request::SetLeds, 1, 0));
        assert!(old.calls().is_empty(), "a request the radio lacks must not go out");

        // Ungated requests are never gated by this.
        assert!(old.has(Request::SetFreq));
        assert!(old.has(Request::SetTransceiverMode));
    }

    #[test]
    fn a_short_reply_on_a_required_read_is_an_error() {
        let t = FakeTransport::new().reply(Request::BoardPartidSerialnoRead, &[0u8; 8]);
        let e = t
            .control_in_exact(Request::BoardPartidSerialnoRead, 0, 0, 24)
            .expect_err("8 bytes is not 24");
        assert!(matches!(e, Error::ShortRead { want: 24, got: 8, .. }), "{e}");
    }
}
