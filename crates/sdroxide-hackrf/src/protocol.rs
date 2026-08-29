//! The HackRF wire protocol, with no `nusb` in it.
//!
//! Every constant and every encode/decode lives here as a pure function, so
//! that the half of this driver most likely to be transcribed wrong is also the
//! half that is fully testable without a radio. [`crate::usb`] turns these into
//! control transfers; nothing here does I/O.
//!
//! Transcribed from libhackrf's `hackrf.c` and `hackrf.h`. Three things in this
//! file are not what they look like and each has a test asserting against
//! literals:
//!
//! * The **gain** requests are `IN` transfers with the value in `wIndex` and a
//!   one-byte reply that must be non-zero — see [`GainSetter`]. Everything else
//!   that sets something is an `OUT` transfer with the value in `wValue`.
//! * [`Request::SpiFlashErase`] and its neighbours are **10, 11, 12**. The
//!   numbering has a gap at 13 and it is easy to close it by accident.
//! * [`compute_filter_bw`] rounds *down* to the table, and libhackrf's version
//!   returns zero for anything above the top entry. This one clamps instead;
//!   see the note there.

/// USB vendor id, shared by every board.
pub const VID: u16 = 0x1d50;

/// Product ids. All of them are driven identically, and all of them tune the
/// same [`TUNING_RANGE_HZ`]; they differ only in the sample rates they accept
/// ([`BoardKind::rate_range`]), whether a bias tee exists, and whether the
/// baseband filter can be set from the host ([`BoardKind::sets_own_filter`]).
///
/// **`0x6089` is not one board.** The HackRF Pro ships the same id as the
/// HackRF One — GSG's firmware shares one device descriptor between them
/// (`firmware/hackrf_usb/usb_descriptor.c`, `#if defined(IS_HACKRF_ONE) ||
/// defined(IS_PRALINE)`) and libhackrf's own device list has no entry for the
/// Pro at all. The two are told apart by the USB **product string**, which is
/// free at enumeration ([`BoardKind::from_product`]), and definitively by
/// [`Request::BoardIdRead`] once the radio is open.
pub const PID_HACKRF_ONE: u16 = 0x6089;
pub const PID_JAWBREAKER: u16 = 0x604b;
pub const PID_RAD1O: u16 = 0xcc15;

/// The product string a HackRF Pro reports, verbatim from the firmware's
/// `usb_descriptor_string_product_praline`.
pub const PRODUCT_HACKRF_PRO: &str = "HackRF Pro";

/// The DFU bootloader's id. This driver never enters it and cannot flash
/// firmware; it is here only so a radio left in DFU by an interrupted
/// `hackrf_spiflash` can be recognised and explained rather than reported as
/// absent.
pub const VID_DFU: u16 = 0x1fc9;
pub const PID_DFU: u16 = 0x000c;

/// The configuration and interface the bulk endpoints live in. There is no
/// alternate setting to select, unlike the Airspy HF+.
pub const CONFIGURATION: u8 = 1;
pub const INTERFACE: u8 = 0;

/// The bulk endpoints. Receive is IN|1, transmit is OUT|2, and only one of them
/// is ever open: the radio is half duplex.
pub const BULK_IN: u8 = 0x81;
pub const BULK_OUT: u8 = 0x02;

/// High-speed bulk packet size. Transfer lengths must be a multiple of this.
pub const BULK_PACKET: usize = 512;

/// Control-transfer timeout, matching libhackrf's `DEFAULT_REQUEST_TIMEOUT`.
pub const CTRL_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(1000);

/// Bytes per complex sample on the wire: one signed byte each for I and Q.
pub const BYTES_PER_SAMPLE: usize = 2;

/// What a HackRF will tune, in Hz. **The same for every board.**
///
/// This is the firmware's limit rather than a board's specification.
/// `firmware/common/tuning.c` clamps a tuning request into 0 Hz – 7250 MHz
/// (`restrict_rf`, against `MAX_HP_FREQ`) with no per-board table anywhere in
/// it; libhackrf documents the same span in as many words — "should be in range
/// 1-6000MHz, but 0-7250MHz is possible" — and SoapyHackRF publishes a flat
/// `Range(0, 7250000000)`, which is why the same radio has always tuned wider
/// through SoapySDR than it did through this driver.
///
/// This driver used to publish a per-board table instead — 1 MHz – 6 GHz for a
/// HackRF One, 100 kHz – 6 GHz for a Pro, 50–4000 MHz for a rad1o — taken from
/// each board's *specified* coverage. That was the wrong figure to publish.
/// Specified coverage is where the radio performs; outside it a HackRF is
/// heavily attenuated on receive and makes little or no transmit power, but it
/// still tunes, and both shortwave under 1 MHz and 5 GHz Wi-Fi are usable in
/// practice (issue #226; Great Scott Gadgets say the same in
/// greatscottgadgets/hackrf#1435). Publishing the narrow figure turned "poor
/// reception" into a dial that would not go there at all.
///
/// A board that genuinely stops short of the edges is still worse there than a
/// HackRF One — that is a matter of sensitivity, which the operator can hear
/// and judge, not of a limit this driver gets to impose on their behalf.
pub const TUNING_RANGE_HZ: (f64, f64) = (0.0, 7.25e9);

/// What the hardware will accept. Not the same thing as what the settings combo
/// offers, which is `HackRfConfig::SAMPLE_RATES` over in `sdroxide-types` —
/// that list has to be wasm-safe because the settings UI is shared, and having
/// it in one place is what stops the two drifting.
///
/// The floor is per-board; see [`BoardKind::rate_range`]. This one is the
/// MAX5864 boards': below 2 Msps the converter is out of spec and the narrowest
/// analog filter is still 1.75 MHz wide, so what comes back is aliased.
pub const MIN_SAMPLE_RATE: f64 = 2.0e6;
pub const MAX_SAMPLE_RATE: f64 = 20.0e6;

/// The HackRF Pro's floor.
///
/// Lower than every other board because the Pro decimates in its FPGA rather
/// than running the converter slowly: the firmware picks a resampling ratio
/// that keeps the analog front end between 200 kHz and 40 MHz whatever the host
/// asks for (`firmware/common/radio.c`, `compute_resample_log`), and clamps the
/// host-side rate to `MIN_MCU_RATE` = 200 kHz. So a narrow rate here is a
/// genuinely narrow, genuinely filtered stream rather than an aliased one.
pub const MIN_SAMPLE_RATE_PRO: f64 = 200.0e3;

/// Vendor requests, as `bRequest`. Values from libhackrf's
/// `hackrf_vendor_request`.
///
/// Only the requests this driver sends are named, plus the few it deliberately
/// does not: the gaps are real. Note there is **no request 13** — the numbering
/// jumps from `SpiFlashRead` to `BoardIdRead` — and that the SPI-flash trio is
/// 10/11/12, which is a different numbering from every other radio in this
/// workspace and the likeliest place to paste in the wrong table.
///
/// # Two of the gaps are load-bearing
///
/// **49–55 are the HackRF Pro's own** — FPGA register access, the P1/P2 SMA
/// signal selectors, the clock-input selector, the narrowband filter and the
/// FPGA bitstream loader. None is needed to receive or transmit, all of them
/// stall on every other board, and one of them is dangerous: request 54 swaps
/// the gateware, and the alternate bitstreams are what put the radio into its
/// 4-bit and 16-bit sample formats. This driver decodes 8-bit I/Q and nothing
/// else, so it never touches 54. (The converse is a real failure mode worth
/// recognising in a bug report: a Pro left in half-precision by
/// `hackrf_debug -P` stays there until it is unplugged, and everything after
/// that is noise.)
///
/// **61 is `GET_BUFFER_SIZE`, and reading it changes behaviour.** The firmware
/// treats the read as the host announcing that it will flush the transmit
/// buffer itself, and switches its own automatic flush off
/// (`auto_tx_flush = false` in `usb_api_transceiver.c`). Leaving it unsent is
/// what makes the firmware hold the mode-off request until the tail of an over
/// has actually gone out — which is the behaviour this driver's transmit path
/// is built on. Do not add it without also adding the flush.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    SetTransceiverMode = 1,
    Max283xWrite = 2,
    Max283xRead = 3,
    Si5351cWrite = 4,
    Si5351cRead = 5,
    SampleRateSet = 6,
    BasebandFilterBandwidthSet = 7,
    Rffc5071Write = 8,
    Rffc5071Read = 9,
    SpiFlashErase = 10,
    SpiFlashWrite = 11,
    SpiFlashRead = 12,
    BoardIdRead = 14,
    VersionStringRead = 15,
    SetFreq = 16,
    AmpEnable = 17,
    BoardPartidSerialnoRead = 18,
    SetLnaGain = 19,
    SetVgaGain = 20,
    SetTxvgaGain = 21,
    AntennaEnable = 23,
    SetFreqExplicit = 24,
    InitSweep = 26,
    SetHwSyncMode = 29,
    Reset = 30,
    ClkoutEnable = 32,
    GetM0State = 41,
    SetTxUnderrunLimit = 42,
    SetRxOverrunLimit = 43,
    BoardRevRead = 45,
    SupportedPlatformRead = 46,
    SetLeds = 47,
}

impl Request {
    pub fn code(self) -> u8 {
        self as u8
    }

    /// The `bcdDevice` this request first appeared in, or `None` if it has
    /// always been there.
    ///
    /// libhackrf gates these on `usb_api_version`, which *is* `bcdDevice` — the
    /// radio states its own API level in its device descriptor rather than
    /// making the host parse a version string. Anything gated may legitimately
    /// stall on an older firmware, and a stalled control transfer self-clears,
    /// so the only correct response is to carry on without whatever it would
    /// have told us.
    ///
    /// The levels are `hackrf.h`'s own "Requires USB API version" list, not a
    /// reconstruction: getting one too *high* is not a safe error in the way it
    /// looks, because it silently withholds a request from a radio that has it.
    ///
    /// Note what this table does **not** contain: the transmit path. Neither
    /// `hackrf_start_tx` nor `hackrf_stop_tx` carries an API gate, and the
    /// requests that key a radio — [`Request::SetTransceiverMode`],
    /// [`Request::SetFreq`], [`Request::AmpEnable`] — have never been gated at
    /// all. A radio that will not transmit is therefore never a radio whose
    /// firmware is too new: everything added since 0x0102 is additive and
    /// optional, and a host that ignores all of it still transmits.
    pub fn since_api(self) -> Option<u16> {
        match self {
            Request::SetHwSyncMode | Request::InitSweep => Some(0x0102),
            Request::ClkoutEnable => Some(0x0103),
            Request::GetM0State
            | Request::SetTxUnderrunLimit
            | Request::SetRxOverrunLimit
            | Request::BoardRevRead
            | Request::SupportedPlatformRead => Some(0x0106),
            Request::SetLeds => Some(0x0107),
            _ => None,
        }
    }

    /// Whether a firmware is allowed not to have this request.
    pub fn is_optional(self) -> bool {
        self.since_api().is_some()
    }
}

/// The radio's operating mode. Everything the driver does is bracketed by a
/// change of this, and every teardown path ends at [`TransceiverMode::Off`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum TransceiverMode {
    Off = 0,
    Receive = 1,
    Transmit = 2,
    /// Sweep-and-sample, used by `hackrf_sweep`. Not driven here.
    Ss = 3,
    /// Puts the bulk OUT endpoint in CPLD-programming mode. Never sent — this
    /// driver cannot brick a radio it does not flash — but named so that a
    /// stray 4 in a trace is recognisable.
    CpldUpdate = 4,
    RxSweep = 5,
}

/// Which board answered [`Request::BoardIdRead`].
///
/// The USB product id says which family; this says which revision, and the two
/// can disagree — a rev-9 HackRF One and an older one share `0x6089`, and so
/// does a HackRF Pro.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardKind {
    Jellybean,
    Jawbreaker,
    HackRfOne,
    Rad1o,
    /// The HackRF Pro. GSG's firmware and libhackrf call this board "Praline"
    /// internally — the codename is what `BOARD_ID_PRALINE` and
    /// `HACKRF_PLATFORM_PRALINE` are named after — but every surface an
    /// operator sees says "HackRF Pro", including `hackrf_board_id_name`, so
    /// that is what this is called here.
    HackRfPro,
    Unknown(u8),
}

impl BoardKind {
    /// From the single byte [`Request::BoardIdRead`] returns.
    pub fn from_code(code: u8) -> BoardKind {
        match code {
            0 => BoardKind::Jellybean,
            1 => BoardKind::Jawbreaker,
            // 2 is a HackRF One before rev 9, 4 is rev 9 and later. The tuning
            // limits and the bias tee are the same, so the driver does not care
            // which — only the name does, and both are "HackRF One".
            2 | 4 => BoardKind::HackRfOne,
            3 => BoardKind::Rad1o,
            5 => BoardKind::HackRfPro,
            other => BoardKind::Unknown(other),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            BoardKind::Jellybean => "Jellybean",
            BoardKind::Jawbreaker => "Jawbreaker",
            BoardKind::HackRfOne => "HackRF One",
            BoardKind::Rad1o => "rad1o",
            BoardKind::HackRfPro => "HackRF Pro",
            BoardKind::Unknown(_) => "HackRF",
        }
    }

    /// The sample rates this board will accept, in Hz.
    ///
    /// Only the floor differs, and it differs by a factor of ten: see
    /// [`MIN_SAMPLE_RATE_PRO`] for why the Pro can go narrow and the others
    /// cannot.
    pub fn rate_range(self) -> (f64, f64) {
        match self {
            BoardKind::HackRfPro => (MIN_SAMPLE_RATE_PRO, MAX_SAMPLE_RATE),
            _ => (MIN_SAMPLE_RATE, MAX_SAMPLE_RATE),
        }
    }

    /// Whether this board has a bias tee to switch at all. The Jawbreaker and
    /// the rad1o do not, and offering the control anyway would be a switch that
    /// silently does nothing.
    pub fn has_bias_tee(self) -> bool {
        matches!(self, BoardKind::HackRfOne | BoardKind::HackRfPro)
    }

    /// Whether the radio picks its own baseband filter and ignores what the
    /// host asks for.
    ///
    /// True on the Pro, and it is not a firmware oversight to be worked around:
    /// `radio_update_bandwidth` in `firmware/common/radio.c` discards the
    /// requested bank outright on this board (`(void) bank;`) and always
    /// derives the filter from the sample rate, because the analog chain it has
    /// to drive is nothing like the MAX2837's. The MAX2831's narrowest coarse
    /// setting is about 10 MHz of complex baseband — narrower than that is an
    /// extra switched filter plus decimation in the FPGA, which only the
    /// firmware knows how to combine.
    ///
    /// So [`Request::BasebandFilterBandwidthSet`] is not sent at all on such a
    /// board. Sending it would be accepted and ignored, and the driver would
    /// then report a filter width the radio is not running — which feeds the
    /// zero-IF LO offset and would be a lie with consequences.
    pub fn sets_own_filter(self) -> bool {
        matches!(self, BoardKind::HackRfPro)
    }

    /// The baseband filter this board ends up at for `rate_hz` when the host
    /// does not override it.
    ///
    /// Both branches are the same 0.75-of-rate rule; they differ in where it
    /// lands. The MAX2837 boards snap to a 16-entry table, so the driver has to
    /// snap the same way to report what the radio is really doing. The Pro
    /// keeps the figure as-is — its firmware carries `bb_bandwidth = 3/4 *
    /// sample_rate` through to the FPGA and only the analog part of the chain
    /// is quantised, so the table would be the wrong answer here.
    pub fn auto_filter_bw(self, rate_hz: f64) -> u32 {
        if self.sets_own_filter() { (0.75 * rate_hz) as u32 } else { auto_filter_bw(rate_hz) }
    }

    /// The board a product id implies, for the device list — where the id is
    /// all there is, because naming a radio must not require opening it.
    ///
    /// `0x6089` answers "HackRF One" for want of anything better; use
    /// [`Self::from_product`] where the product string is to hand, because that
    /// is what separates a One from a Pro without opening anything.
    pub fn from_pid(pid: u16) -> BoardKind {
        match pid {
            PID_JAWBREAKER => BoardKind::Jawbreaker,
            PID_RAD1O => BoardKind::Rad1o,
            _ => BoardKind::HackRfOne,
        }
    }

    /// The board a product id *and* the USB product string imply.
    ///
    /// Still free — the strings come from sysfs, the registry or IOKit, so this
    /// opens nothing — and it is the only way to tell a HackRF Pro from a
    /// HackRF One before [`Request::BoardIdRead`] can be asked. The two share a
    /// product id, and the settings UI has to offer a different rate list for
    /// each, so this has to work at enumeration time rather than at open.
    ///
    /// A radio whose descriptor says nothing falls back to [`Self::from_pid`],
    /// which is the conservative answer: a Pro treated as a One works, and a
    /// One treated as a Pro would be offered rates that alias.
    pub fn from_product(pid: u16, product: Option<&str>) -> BoardKind {
        match product.map(str::trim) {
            Some(p) if pid == PID_HACKRF_ONE && p.eq_ignore_ascii_case(PRODUCT_HACKRF_PRO) => {
                BoardKind::HackRfPro
            }
            _ => BoardKind::from_pid(pid),
        }
    }
}

/// One megahertz, as libhackrf's `FREQ_ONE_MHZ`.
const FREQ_ONE_MHZ: u64 = 1_000_000;

/// The tuning payload: megahertz then the remaining hertz, both little-endian
/// `u32`.
///
/// Splitting the frequency this way rather than sending a `u64` is the wire
/// format, not a nicety — the firmware tunes the synthesiser from the MHz part
/// and the offset from the remainder. Both halves are little-endian, unlike the
/// Airspy HF+ next door whose one frequency field is big-endian, so this is
/// asserted against literals.
pub fn encode_set_freq(hz: u64) -> [u8; 8] {
    let mhz = (hz / FREQ_ONE_MHZ) as u32;
    let rem = (hz - mhz as u64 * FREQ_ONE_MHZ) as u32;
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&mhz.to_le_bytes());
    out[4..8].copy_from_slice(&rem.to_le_bytes());
    out
}

/// Split a sample rate into the `(freq_hz, divider)` pair the firmware wants.
///
/// libhackrf derives this by picking apart the IEEE-754 mantissa and adding it
/// to itself until the low bits clear; what that computes is the smallest
/// divider under 32 that makes the rate integral, which is what this does
/// legibly instead. Every rate in [`SAMPLE_RATES`] is a whole number of hertz,
/// so the divider is always 1 there and the fractional path exists only for a
/// hand-typed rate.
pub fn rate_params(hz: f64) -> (u32, u32) {
    const MAX_N: u32 = 32;
    for divider in 1..MAX_N {
        let scaled = hz * divider as f64;
        if (scaled - scaled.round()).abs() < 1e-6 {
            return (scaled.round() as u32, divider);
        }
    }
    // Nothing under 32 made it integral. libhackrf gives up the same way, and
    // rounding to whole hertz is a sub-ppm error on any rate this radio runs.
    (hz.round() as u32, 1)
}

/// The rate payload: `freq_hz` then `divider`, both little-endian `u32`.
pub fn encode_sample_rate(freq_hz: u32, divider: u32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&freq_hz.to_le_bytes());
    out[4..8].copy_from_slice(&divider.to_le_bytes());
    out
}

/// The MAX2837's baseband filter bandwidths, in Hz. Not a range — the filter
/// takes one of exactly these.
pub const FILTER_BW_TABLE: [u32; 16] = [
    1_750_000, 2_500_000, 3_500_000, 5_000_000, 5_500_000, 6_000_000, 7_000_000, 8_000_000,
    9_000_000, 10_000_000, 12_000_000, 14_000_000, 15_000_000, 20_000_000, 24_000_000, 28_000_000,
];

/// The widest table entry that does not exceed `want`, clamped at both ends.
///
/// This is libhackrf's `hackrf_compute_baseband_filter_bw` with one deliberate
/// difference. Its loop walks off the end of the table into the `{0}`
/// terminator when asked for more than 28 MHz and returns **0**, which the
/// firmware then reads as a bandwidth — so asking for too much gets you the
/// narrowest possible filter rather than the widest. Here that clamps to the
/// top entry, which is what the caller meant.
pub fn compute_filter_bw(want: u32) -> u32 {
    match FILTER_BW_TABLE.iter().position(|bw| *bw >= want) {
        // Exactly on a step, or narrower than the narrowest: take it as-is.
        Some(0) => FILTER_BW_TABLE[0],
        Some(i) if FILTER_BW_TABLE[i] == want => want,
        // The first entry at or above `want` overshoots, so step back one.
        Some(i) => FILTER_BW_TABLE[i - 1],
        None => FILTER_BW_TABLE[FILTER_BW_TABLE.len() - 1],
    }
}

/// The filter the radio picks for itself when a rate is set, from libhackrf's
/// `0.75 * rate`.
///
/// This matters beyond anti-aliasing: `sdroxide_radio`'s zero-IF policy parks
/// the LO a quarter of a span above the dial and *withdraws the offset* if it
/// would land past 45 % of the analog bandwidth. A filter chosen too narrow
/// therefore does not merely soften the band edges — it silently switches off
/// the DC-spike avoidance, which looks exactly like the offset being broken.
/// See the test below, which asserts the relationship rather than describing
/// it.
pub fn auto_filter_bw(rate_hz: f64) -> u32 {
    compute_filter_bw((0.75 * rate_hz) as u32)
}

/// Which gain stage a value is for. Named because all three go out the same
/// unusual way and the direction is the thing to get right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainSetter {
    /// 0–40 dB in 8 dB steps, receive only.
    Lna,
    /// 0–62 dB in 2 dB steps, receive only.
    Vga,
    /// 0–47 dB in 1 dB steps, transmit only.
    TxVga,
}

impl GainSetter {
    pub fn request(self) -> Request {
        match self {
            GainSetter::Lna => Request::SetLnaGain,
            GainSetter::Vga => Request::SetVgaGain,
            GainSetter::TxVga => Request::SetTxvgaGain,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            GainSetter::Lna => "LNA",
            GainSetter::Vga => "VGA",
            GainSetter::TxVga => "TXVGA",
        }
    }

    /// The range and step, in dB.
    pub fn range(self) -> (u32, u32, u32) {
        match self {
            GainSetter::Lna => (0, 40, 8),
            GainSetter::Vga => (0, 62, 2),
            GainSetter::TxVga => (0, 47, 1),
        }
    }

    /// Clamp and quantise a dB figure to what this stage can actually be set
    /// to, returning what the hardware will really do.
    ///
    /// libhackrf masks the low bits off (`value &= ~0x07` for the LNA) rather
    /// than rounding, so 15 dB of LNA is 8 dB and not 16. Reproducing the
    /// truncation matters: the UI shows this value back, and a slider that says
    /// 15 while the radio does 8 is a lie that costs someone an evening.
    pub fn snap(self, db: f64) -> u32 {
        let (min, max, step) = self.range();
        let clamped = db.clamp(min as f64, max as f64) as u32;
        clamped - clamped % step
    }
}

/// The 40-byte reply to [`Request::GetM0State`], little-endian throughout.
///
/// The only thing in this protocol that answers **"is the radio actually
/// reading what I am sending it?"** A host cannot tell a transfer the radio is
/// ignoring from one it simply has not reached yet — both look like a transfer
/// that has not completed. The radio can, and this is how it says so.
///
/// Layout and the mode numbering are libhackrf's `hackrf_m0_state`; the modes
/// are its own comment verbatim, because 3 and 4 being two *different* transmit
/// states is the distinction the whole thing is worth reading for. A radio
/// sitting in `TX_START` with `m0_count` at zero has been keyed and has never
/// been fed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct M0State {
    pub requested_mode: u16,
    pub request_flag: u16,
    pub active_mode: u32,
    /// Bytes the M0 has moved — the radio's side of the wire.
    pub m0_count: u32,
    /// Bytes the M4 has moved — the USB side.
    pub m4_count: u32,
    pub num_shortfalls: u32,
    pub longest_shortfall: u32,
    pub shortfall_limit: u32,
    pub threshold: u32,
    pub next_mode: u32,
    pub error: u32,
}

/// The length of the [`M0State`] reply.
pub const M0_STATE_LEN: u16 = 40;

impl M0State {
    pub fn parse(b: &[u8]) -> Option<M0State> {
        if b.len() < M0_STATE_LEN as usize {
            return None;
        }
        let half = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let word = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Some(M0State {
            requested_mode: half(0),
            request_flag: half(2),
            active_mode: word(4),
            m0_count: word(8),
            m4_count: word(12),
            num_shortfalls: word(16),
            longest_shortfall: word(20),
            shortfall_limit: word(24),
            threshold: word(28),
            next_mode: word(32),
            error: word(36),
        })
    }

    pub fn mode_name(mode: u32) -> &'static str {
        match mode {
            0 => "IDLE",
            1 => "WAIT",
            2 => "RX",
            3 => "TX_START",
            4 => "TX_RUN",
            _ => "unknown",
        }
    }

    /// Why the M0 gave up, if it did. A `TX timeout` here is the radio saying it
    /// was keyed and starved — which is a different fault from never having been
    /// keyed at all, and the two are indistinguishable from the host side.
    pub fn error_name(&self) -> &'static str {
        match self.error {
            0 => "none",
            1 => "RX timeout",
            2 => "TX timeout",
            3 => "missed deadline",
            _ => "unknown",
        }
    }
}

impl std::fmt::Display for M0State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "mode {} (requested {}), M0 moved {} B, M4 moved {} B, {} shortfall(s) \
             (longest {} B, limit {}), error {}",
            M0State::mode_name(self.active_mode),
            M0State::mode_name(u32::from(self.requested_mode)),
            self.m0_count,
            self.m4_count,
            self.num_shortfalls,
            self.longest_shortfall,
            self.shortfall_limit,
            self.error_name(),
        )
    }
}

/// The 24-byte reply to [`Request::BoardPartidSerialnoRead`]:
/// `{ u32 part_id[2]; u32 serial_no[4] }`, all little-endian.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PartIdSerial {
    pub part_id: [u32; 2],
    pub serial: [u32; 4],
}

impl PartIdSerial {
    pub fn parse(b: &[u8]) -> Option<PartIdSerial> {
        if b.len() < 24 {
            return None;
        }
        let word = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Some(PartIdSerial {
            part_id: [word(0), word(4)],
            serial: [word(8), word(12), word(16), word(20)],
        })
    }

    /// The serial as the 32 hex digits every HackRF tool prints, which is also
    /// what the USB `iSerial` descriptor carries.
    pub fn serial_hex(&self) -> String {
        self.serial.iter().map(|w| format!("{w:08x}")).collect()
    }
}

/// A NUL-padded ASCII reply, such as the firmware version string, as a
/// `String`.
pub fn parse_ascii(b: &[u8]) -> String {
    let end = b.iter().position(|c| *c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).trim().to_string()
}

/// Whether an operator-supplied serial names this radio.
///
/// **Suffix** matching, which is libhackrf's behaviour and the reason every
/// HackRF instruction on the internet quotes only the last few digits: the
/// serial is 32 hex characters of which the leading half is usually zeroes, so
/// nobody types the whole thing. Case-insensitive and whitespace-tolerant here
/// where libhackrf is neither, because the value in `radio.json` was typed or
/// pasted by a human and the descriptor's case is the firmware's choice.
///
/// An empty `want` matches everything — that is "no serial configured", not "a
/// serial that happens to be blank".
///
/// The rule itself lives in `sdroxide-types` so the settings UI can apply the
/// identical one — it has to pick out the selected device to know which board's
/// rates to offer — and the settings UI is shared with the wasm client, which
/// cannot depend on this crate. The tests below are still the ones that pin the
/// behaviour.
pub fn serial_matches(want: &str, found: Option<&str>) -> bool {
    sdroxide_types::hackrf_serial_matches(want, found)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::HackRfConfig;

    /// The rates the settings combo offers, which is what the driver has to
    /// cope with. Lives in `sdroxide-types` because the settings UI is shared
    /// with the wasm client and cannot depend on this crate.
    const OFFERED_RATES: [f64; 7] = HackRfConfig::SAMPLE_RATES;

    /// Both halves are little-endian. The Airspy HF+ in the crate next door
    /// sends its one frequency field big-endian, so this is asserted against
    /// literals rather than against `to_le_bytes` — a copied-across "fix" has
    /// to fail here.
    #[test]
    fn the_frequency_splits_into_megahertz_and_remainder() {
        // 100.1 MHz: 100 MHz plus 100 000 Hz.
        assert_eq!(encode_set_freq(100_100_000), [0x64, 0x00, 0x00, 0x00, 0xa0, 0x86, 0x01, 0x00]);
        // Exactly on a megahertz leaves the remainder zero.
        assert_eq!(encode_set_freq(1_000_000), [0x01, 0, 0, 0, 0, 0, 0, 0]);
        // Below a megahertz is all remainder.
        assert_eq!(encode_set_freq(999_999), [0, 0, 0, 0, 0x3f, 0x42, 0x0f, 0x00]);
        assert_eq!(encode_set_freq(0), [0; 8]);
        // The top of the range still fits both halves.
        let (mhz, rem) = (6_000u32, 0u32);
        let mut want = [0u8; 8];
        want[0..4].copy_from_slice(&mhz.to_le_bytes());
        want[4..8].copy_from_slice(&rem.to_le_bytes());
        assert_eq!(encode_set_freq(6_000_000_000), want);
    }

    /// Every rate the UI offers must come out as a whole number of hertz over a
    /// divider of one. A divider of anything else here would mean the rate is
    /// being approximated when it did not need to be.
    #[test]
    fn every_offered_rate_needs_no_divider() {
        for rate in OFFERED_RATES {
            let (hz, div) = rate_params(rate);
            assert_eq!(div, 1, "{rate} should not need a divider");
            assert_eq!(hz as f64, rate, "{rate} came back as {hz}");
        }
        assert_eq!(encode_sample_rate(2_000_000, 1), [0x80, 0x84, 0x1e, 0, 0x01, 0, 0, 0]);
    }

    /// The fractional path exists for a hand-typed rate. 8/3 Msps is the
    /// classic one — it is exactly representable as 8 000 000 over 3 and as
    /// nothing smaller.
    #[test]
    fn a_fractional_rate_finds_the_smallest_divider() {
        let (hz, div) = rate_params(8_000_000.0 / 3.0);
        assert_eq!((hz, div), (8_000_000, 3));
        let (hz, div) = rate_params(2_500_000.5);
        assert_eq!((hz, div), (5_000_001, 2));
    }

    /// libhackrf walks into its table terminator above 28 MHz and returns zero,
    /// which the firmware reads as a bandwidth — the widest request producing
    /// the narrowest filter. Clamping is the whole reason this is not a
    /// transcription.
    #[test]
    fn the_filter_rounds_down_and_clamps_at_both_ends() {
        // Exactly on a step is taken as-is.
        assert_eq!(compute_filter_bw(6_000_000), 6_000_000);
        assert_eq!(compute_filter_bw(1_750_000), 1_750_000);
        // Between steps rounds down.
        assert_eq!(compute_filter_bw(7_500_000), 7_000_000);
        assert_eq!(compute_filter_bw(5_999_999), 5_500_000);
        // Narrower than the narrowest is the narrowest, not nothing.
        assert_eq!(compute_filter_bw(0), 1_750_000);
        assert_eq!(compute_filter_bw(1_000_000), 1_750_000);
        // Wider than the widest is the widest, not zero.
        assert_eq!(compute_filter_bw(28_000_000), 28_000_000);
        assert_eq!(compute_filter_bw(40_000_000), 28_000_000);
        assert_eq!(compute_filter_bw(u32::MAX), 28_000_000);
    }

    /// The filter and the zero-IF LO offset are coupled, and the coupling is
    /// invisible at the call site. `sdroxide_radio::source::lo_offset_for`
    /// parks the LO a quarter-span above the dial and withdraws the offset
    /// entirely when it would exceed 45 % of the analog bandwidth — so a filter
    /// chosen too narrow silently disables DC-spike avoidance instead of merely
    /// softening the band edges.
    ///
    /// A quarter-span offset stays inside 45 % of the bandwidth exactly when
    /// the bandwidth is at least `rate / 1.8`, i.e. `0.5556 * rate`. Asserting
    /// that here means a future change to the table or to `auto_filter_bw`
    /// cannot quietly cost every HackRF owner their LO offset.
    #[test]
    fn the_auto_filter_never_costs_the_lo_offset() {
        for rate in OFFERED_RATES {
            let bw = auto_filter_bw(rate) as f64;
            let offset = rate * 0.25;
            assert!(
                offset <= bw * 0.45,
                "at {rate} sps the auto filter is {bw} Hz, which withdraws the \
                 {offset} Hz LO offset"
            );
        }
        // And it is genuinely the 0.75-of-rate rule, not something that happens
        // to satisfy the inequality.
        assert_eq!(auto_filter_bw(2.0e6), 1_750_000);
        assert_eq!(auto_filter_bw(8.0e6), 6_000_000);
        assert_eq!(auto_filter_bw(10.0e6), 7_000_000);
        assert_eq!(auto_filter_bw(20.0e6), 15_000_000);
    }

    /// The firmware truncates rather than rounds, so the value shown back to
    /// the operator has to truncate too.
    #[test]
    fn gains_truncate_to_their_step_and_clamp_to_their_range() {
        assert_eq!(GainSetter::Lna.snap(15.0), 8, "the LNA masks off the low 3 bits");
        assert_eq!(GainSetter::Lna.snap(16.0), 16);
        assert_eq!(GainSetter::Lna.snap(-5.0), 0);
        assert_eq!(GainSetter::Lna.snap(100.0), 40);

        assert_eq!(GainSetter::Vga.snap(15.0), 14, "the VGA masks off the low bit");
        assert_eq!(GainSetter::Vga.snap(62.0), 62);
        assert_eq!(GainSetter::Vga.snap(63.0), 62);

        // The transmit stage is the one with a 1 dB step, so nothing truncates.
        assert_eq!(GainSetter::TxVga.snap(23.0), 23);
        assert_eq!(GainSetter::TxVga.snap(47.9), 47);
        assert_eq!(GainSetter::TxVga.snap(48.0), 47);

        // Whatever a stage snaps to must be settable again unchanged, or the
        // UI oscillates between two values.
        for g in [GainSetter::Lna, GainSetter::Vga, GainSetter::TxVga] {
            for db in 0..=70 {
                let once = g.snap(db as f64);
                assert_eq!(g.snap(once as f64), once, "{g:?} at {db} dB is not stable");
            }
        }
    }

    /// The SPI-flash trio is 10/11/12 and there is no request 13. Both are easy
    /// to get wrong by pasting a neighbouring radio's table, and a wrong code
    /// here is a control transfer that does something else entirely.
    #[test]
    fn the_request_numbering_has_its_gap_in_the_right_place() {
        assert_eq!(Request::SpiFlashErase.code(), 10);
        assert_eq!(Request::SpiFlashWrite.code(), 11);
        assert_eq!(Request::SpiFlashRead.code(), 12);
        assert_eq!(Request::BoardIdRead.code(), 14);
        // 13 and 22 are not requests. Nothing in the enum may claim them.
        for absent in [13u8, 22] {
            assert!(
                ![
                    Request::SetTransceiverMode.code(),
                    Request::SpiFlashRead.code(),
                    Request::BoardIdRead.code(),
                    Request::SetTxvgaGain.code(),
                    Request::AntennaEnable.code(),
                ]
                .contains(&absent),
                "{absent} is not a HackRF request"
            );
        }
        assert_eq!(Request::SetTransceiverMode.code(), 1);
        assert_eq!(Request::SetFreq.code(), 16);
        assert_eq!(Request::AmpEnable.code(), 17);
        assert_eq!(Request::SetTxvgaGain.code(), 21);
        assert_eq!(Request::AntennaEnable.code(), 23);
    }

    /// Everything the radio needs in order to work must be mandatory; getting
    /// this wrong either refuses to open an older radio or hides a real
    /// failure.
    #[test]
    fn only_the_later_requests_are_optional() {
        for r in [
            Request::SetTransceiverMode,
            Request::SampleRateSet,
            Request::BasebandFilterBandwidthSet,
            Request::SetFreq,
            Request::AmpEnable,
            Request::SetLnaGain,
            Request::SetVgaGain,
            Request::SetTxvgaGain,
            Request::AntennaEnable,
            Request::BoardIdRead,
        ] {
            assert!(!r.is_optional(), "{r:?} is what makes the radio work");
            assert_eq!(r.since_api(), None);
        }
        for r in [Request::GetM0State, Request::BoardRevRead, Request::SetLeds] {
            assert!(r.is_optional(), "{r:?}");
            assert!(r.since_api().unwrap() >= 0x0106);
        }
    }

    /// The radio's own account of a stuck key-down, as it will arrive in a
    /// field report: keyed, and never fed.
    #[test]
    fn the_m0_state_reply_reads_a_keyed_but_unfed_transmitter() {
        let mut b = Vec::new();
        b.extend_from_slice(&3u16.to_le_bytes()); // requested TX_START
        b.extend_from_slice(&1u16.to_le_bytes()); // request_flag
        for w in [3u32, 0, 32_768, 0, 0, 0, 0, 0, 0] {
            b.extend_from_slice(&w.to_le_bytes());
        }
        assert_eq!(b.len(), M0_STATE_LEN as usize);

        let m = M0State::parse(&b).expect("a whole reply");
        assert_eq!(m.active_mode, 3);
        assert_eq!(m.m0_count, 0, "the radio has moved nothing");
        assert_eq!(m.m4_count, 32_768, "but USB delivered to it");
        assert_eq!(m.error_name(), "none");
        let shown = m.to_string();
        assert!(shown.contains("mode TX_START"), "{shown}");
        assert!(shown.contains("M0 moved 0 B"), "{shown}");

        // The distinction the whole read exists for: a radio that was keyed and
        // starved says so, and it is not the same fault as never being keyed.
        b[36] = 2;
        assert_eq!(M0State::parse(&b).unwrap().error_name(), "TX timeout");

        // A short reply is not a half-parsed state.
        assert_eq!(M0State::parse(&b[..39]), None);
        assert_eq!(M0State::parse(&[]), None);
    }

    #[test]
    fn the_part_id_reply_splits_into_two_words_and_four() {
        let mut b = Vec::new();
        for w in [0xa000_cb3cu32, 0x0058_4753, 0x0000_0000, 0x0000_0000, 0x4578_63c8, 0x267a_765f] {
            b.extend_from_slice(&w.to_le_bytes());
        }
        let p = PartIdSerial::parse(&b).expect("24 bytes is a whole reply");
        assert_eq!(p.part_id, [0xa000_cb3c, 0x0058_4753]);
        assert_eq!(p.serial_hex(), "0000000000000000457863c8267a765f");
        // A short reply is not a half-parsed serial.
        assert_eq!(PartIdSerial::parse(&b[..23]), None);
        assert_eq!(PartIdSerial::parse(&[]), None);
    }

    /// Suffix matching, because nobody types 32 hex digits and every HackRF
    /// instruction quotes the tail.
    #[test]
    fn a_serial_matches_on_its_suffix() {
        let full = Some("0000000000000000457863c8267a765f");
        assert!(serial_matches("267a765f", full), "the tail is what people type");
        assert!(serial_matches("267A765F", full), "case is the firmware's choice");
        assert!(serial_matches(" 267a765f ", full), "pasted values carry whitespace");
        assert!(serial_matches("0000000000000000457863c8267a765f", full));
        // A prefix is not a suffix, and neither is a wrong tail.
        assert!(!serial_matches("00000000", full));
        assert!(!serial_matches("267a7650", full));
        // Longer than the real serial cannot match, and must not panic.
        assert!(!serial_matches("f".repeat(64).as_str(), full));
        // No serial configured means any radio will do.
        assert!(serial_matches("", full));
        assert!(serial_matches("   ", full));
        // A radio with no descriptor cannot satisfy a specific request, but
        // still satisfies "any".
        assert!(!serial_matches("267a765f", None));
        assert!(serial_matches("", None));
    }

    /// The tuning range is the firmware's clamp, not a per-board figure: 0 Hz
    /// to 7250 MHz, the same span libhackrf documents and SoapyHackRF
    /// publishes. Narrowing it to a board's specified coverage is what issue
    /// #226 was about — a HackRF is deaf below 1 MHz and above 6 GHz, but deaf
    /// is not the same as untunable.
    #[test]
    fn the_tuning_range_is_the_firmwares_and_is_board_independent() {
        assert_eq!(TUNING_RANGE_HZ, (0.0, 7.25e9));
    }

    /// The product id names the radio in a list without opening it; the board
    /// id refines that once it is open. What the two decide is the sample rate
    /// list, the bias tee and who owns the baseband filter — no longer the
    /// tuning range, which is the same on all of them.
    #[test]
    fn each_board_is_named_by_its_product_id_and_refined_by_its_board_id() {
        assert_eq!(BoardKind::from_pid(PID_RAD1O), BoardKind::Rad1o);
        assert_eq!(BoardKind::from_pid(PID_JAWBREAKER), BoardKind::Jawbreaker);
        assert_eq!(BoardKind::from_pid(PID_HACKRF_ONE), BoardKind::HackRfOne);

        // Rev 9 and pre-rev-9 are both a HackRF One as far as this driver is
        // concerned; only the name would differ and it does not.
        assert_eq!(BoardKind::from_code(2), BoardKind::HackRfOne);
        assert_eq!(BoardKind::from_code(4), BoardKind::HackRfOne);
        assert_eq!(BoardKind::from_code(5), BoardKind::HackRfPro);
        assert_eq!(BoardKind::from_code(0xfe), BoardKind::Unknown(0xfe));
        assert_eq!(BoardKind::Unknown(0xfe).name(), "HackRF");
        // The codename is the firmware's, not the operator's: libhackrf's
        // `hackrf_board_id_name` answers "HackRF Pro" for board id 5 and so
        // does every label this driver produces.
        assert_eq!(BoardKind::HackRfPro.name(), "HackRF Pro");

        // Offering a bias tee that does not exist is a switch that silently
        // does nothing. The Pro has one — the firmware accepts
        // `AntennaEnable` for `BOARD_ID_PRALINE` alongside the two HackRF One
        // revisions and stalls it for everything else.
        assert!(BoardKind::HackRfOne.has_bias_tee());
        assert!(BoardKind::HackRfPro.has_bias_tee());
        assert!(!BoardKind::Jawbreaker.has_bias_tee());
        assert!(!BoardKind::Rad1o.has_bias_tee());
    }

    /// A HackRF Pro shares `0x6089` with a HackRF One, so the product id alone
    /// cannot separate them and the product string has to. Getting this wrong
    /// in the safe direction costs a Pro owner the bottom of their band and
    /// their narrow rates; getting it wrong the other way offers a One rates
    /// that come back aliased.
    #[test]
    fn a_pro_is_told_from_a_one_by_its_product_string() {
        let pro = BoardKind::from_product(PID_HACKRF_ONE, Some("HackRF Pro"));
        assert_eq!(pro, BoardKind::HackRfPro);
        // Descriptors are the firmware's choice of case and may arrive padded.
        assert_eq!(BoardKind::from_product(PID_HACKRF_ONE, Some(" hackrf pro ")), pro);

        // Everything else on that id is a One, including a radio that offers
        // no product string at all.
        assert_eq!(
            BoardKind::from_product(PID_HACKRF_ONE, Some("HackRF One")),
            BoardKind::HackRfOne
        );
        assert_eq!(BoardKind::from_product(PID_HACKRF_ONE, None), BoardKind::HackRfOne);
        assert_eq!(BoardKind::from_product(PID_HACKRF_ONE, Some("")), BoardKind::HackRfOne);

        // And the string never overrides an id that already says something
        // else — a rad1o calling itself a Pro is a rad1o.
        assert_eq!(BoardKind::from_product(PID_RAD1O, Some("HackRF Pro")), BoardKind::Rad1o);
        assert_eq!(
            BoardKind::from_product(PID_JAWBREAKER, Some("HackRF Pro")),
            BoardKind::Jawbreaker
        );
    }

    /// The Pro decimates in its FPGA, so it goes a decade narrower than any
    /// board built around the MAX5864 — and the driver must not clamp that
    /// away.
    #[test]
    fn only_the_pro_goes_below_two_megasamples() {
        assert_eq!(BoardKind::HackRfPro.rate_range(), (200.0e3, 20.0e6));
        for b in [BoardKind::HackRfOne, BoardKind::Jawbreaker, BoardKind::Rad1o] {
            assert_eq!(b.rate_range(), (2.0e6, 20.0e6), "{b:?}");
        }
    }

    /// The Pro's firmware discards the requested baseband bandwidth and derives
    /// its own, so the driver must not send the request and must not report the
    /// MAX2837 table's answer — that figure feeds the zero-IF LO offset.
    #[test]
    fn the_pro_picks_its_own_filter_and_says_so() {
        assert!(BoardKind::HackRfPro.sets_own_filter());
        for b in [BoardKind::HackRfOne, BoardKind::Jawbreaker, BoardKind::Rad1o] {
            assert!(!b.sets_own_filter(), "{b:?}");
        }

        // Same 0.75-of-rate rule, different landing place.
        assert_eq!(BoardKind::HackRfOne.auto_filter_bw(20.0e6), 15_000_000);
        assert_eq!(BoardKind::HackRfPro.auto_filter_bw(20.0e6), 15_000_000);
        assert_eq!(BoardKind::HackRfOne.auto_filter_bw(10.0e6), 7_000_000, "snapped to the table");
        assert_eq!(BoardKind::HackRfPro.auto_filter_bw(10.0e6), 7_500_000, "not snapped");

        // Below the MAX2837's narrowest step the two diverge completely, which
        // is the whole point: at 250 ksps the table would claim 1.75 MHz of
        // analog bandwidth on a radio running 187.5 kHz.
        assert_eq!(BoardKind::HackRfOne.auto_filter_bw(250.0e3), 1_750_000);
        assert_eq!(BoardKind::HackRfPro.auto_filter_bw(250.0e3), 187_500);

        // And the LO offset survives at every rate either board can run — the
        // same relationship `the_auto_filter_never_costs_the_lo_offset` pins
        // for the MAX2837 boards, checked across the Pro's wider span.
        for rate in [200.0e3, 250.0e3, 500.0e3, 1.0e6, 2.0e6, 8.0e6, 20.0e6] {
            let bw = BoardKind::HackRfPro.auto_filter_bw(rate) as f64;
            assert!(rate * 0.25 <= bw * 0.45, "at {rate} sps the Pro's filter is {bw} Hz");
        }
    }

    #[test]
    fn ascii_replies_stop_at_the_nul() {
        assert_eq!(parse_ascii(b"2024.02.1\0\0\0"), "2024.02.1");
        assert_eq!(parse_ascii(b"2024.02.1"), "2024.02.1");
        assert_eq!(parse_ascii(b"\0junk"), "");
        assert_eq!(parse_ascii(b""), "");
    }
}
