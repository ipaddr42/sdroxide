//! AIS domain types, shared by the native engine, the wire protocol and the UI
//! (native + WASM). Pure data + serde — the GMSK demodulator and the message
//! decoder live in the native `sdroxide-ais` crate.
//!
//! # Why a vessel table rather than a message log
//!
//! The Automatic Identification System is the same argument [`crate::adsb`]
//! makes at sea. A ship under way sends its position every two to ten seconds
//! and its name, call sign, dimensions and destination once every six minutes,
//! in a *different* message. A chronological log of what arrived is the same
//! forty ships over and over, none of them labelled, and the question a
//! watchkeeper has is "what is out there, and where is it going".
//!
//! So the decoder keeps one [`AisVessel`] per MMSI and re-sends the whole table
//! a couple of times a second. The MMSI is the stable key that lets a panel row
//! keep its place and a map symbol keep its track, and it is what joins the
//! position report to the static report that names it.
//!
//! # The clocks are longer than ADS-B's, and that is not a preference
//!
//! An airliner squitters twice a second; a ship at anchor reports **once every
//! three minutes**, and a Class B unit under way once every thirty seconds.
//! Carrying ADS-B's ten-second map window over would blank the map between
//! every pair of reports from a stationary vessel — which is most of a harbour.
//! [`AIS_DROP_MAP_S`] and [`AIS_DROP_LIST_S`] are set from the reporting
//! intervals in ITU-R M.1371 Table 1, not from taste.
//!
//! # The trail is measured in minutes, not in points
//!
//! ADS-B keeps a fixed number of history dots because every aircraft reports at
//! the same rate, so a count *is* a duration. AIS reporting rates span two
//! orders of magnitude — 2 s for a fast ship, 180 s for a moored one — so a
//! forty-point trail would be eighty seconds of a ferry and two hours of a
//! anchored tanker, drawn identically. [`AisSettings::trail_minutes`] is a
//! window in time; [`AIS_TRACK_MAX`] is only the ceiling that stops one fast
//! vessel filling the message.
//!
//! Sources: ITU-R M.1371-5 (the AIS technical characteristics) for the message
//! layouts, the reporting intervals and every code table below; IALA's
//! Guideline 1082 for the aid-to-navigation types.

use serde::{Deserialize, Serialize};

/// AIS 1 — marine VHF channel 87B. Half the traffic, worldwide.
pub const AIS_CHANNEL_A_HZ: f64 = 161_975_000.0;
/// AIS 2 — marine VHF channel 88B, the other half.
pub const AIS_CHANNEL_B_HZ: f64 = 162_025_000.0;

/// Midway between the two channels: where the dial goes for AIS.
///
/// Not a channel itself — nothing transmits here — which is exactly why it is
/// the right place to park. A receiver centred on either channel would have the
/// other 50 kHz off, and on a zero-IF front end the one on the dial would sit
/// in the DC spike.
pub const AIS_PLAN_CENTER_HZ: f64 = 162_000_000.0;

/// The marine VHF raster: one channel's slot.
pub const AIS_CHANNEL_SPACING_HZ: f64 = 25_000.0;

/// Bits a second on the air. One rate, worldwide.
pub const AIS_BIT_RATE: f64 = 9_600.0;

/// Narrowest front-end stream the lane will start on.
///
/// Enough for one channel at five samples a bit. A receiver this narrow reaches
/// only whichever of the two channels it happens to be over, which the panel is
/// told — half the traffic is a real answer, and a great deal better than a
/// refusal.
pub const AIS_MIN_RATE_HZ: f64 = 48_000.0;

/// Samples a bit below which the timing estimate has too little to work with.
///
/// GMSK at 9600 bit/s is a smooth waveform and the estimator here is a
/// non-data-aided one over the whole burst, so it needs rather less than a
/// hunting loop would — but under four samples a bit the matched filter has no
/// shape left to match and the interpolator is guessing between two samples.
pub const AIS_GOOD_SPS: f64 = 4.0;

/// Longest trail kept per vessel, whatever the settings ask for.
///
/// A ship in a traffic separation scheme reports every two seconds, so a
/// ten-minute trail is three hundred points if nothing bounds it — times five
/// hundred vessels, twice a second, on the wire.
pub const AIS_TRACK_MAX: usize = 240;

/// Default for [`AisSettings::drop_map_s`]: five minutes.
///
/// A Class A vessel at anchor reports every three minutes and a Class B one
/// every three when it is slow, so anything under that blanks the map between
/// two perfectly good reports. Five leaves a margin for one lost slot.
pub const AIS_DROP_MAP_S: u16 = 300;

/// Default for [`AisSettings::drop_list_s`]: half an hour.
///
/// The static report — the one carrying the name — comes round every six
/// minutes, so a list window shorter than a few of those would keep throwing
/// away vessels just before they told anyone what they were called.
pub const AIS_DROP_LIST_S: u16 = 1_800;

/// Default for [`AisSettings::trail_minutes`].
pub const AIS_TRAIL_MINUTES: u16 = 10;

/// Default for [`AisSettings::max_vessels`].
pub const AIS_MAX_VESSELS: u16 = 500;

/// Default for [`AisSettings::vector_minutes`]. Six minutes is the interval a
/// chart plotter's default CPA vector uses.
pub const AIS_VECTOR_MINUTES: f32 = 6.0;

/// Default for [`AisSettings::threshold_db`] — how far above the learned noise
/// floor a slot has to be before it is worth demodulating.
pub const AIS_THRESHOLD_DB: u8 = 8;

/// Both channels enabled — the two low bits of [`AisSettings::channels`].
pub const AIS_ALL_CHANNELS: u8 = 0x03;

/// What sort of station an MMSI belongs to.
///
/// Kept because the map draws them differently and because it changes what a
/// row *means*: a lighthouse that has not moved in eighty years is not a stale
/// target, and a receiver that hears nothing but base stations is hearing the
/// shore rather than the sea.
///
/// Decided from the MMSI's own format (ITU-R M.585) and from which message the
/// station sent, in that order — the format is definitive where it applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum AisKind {
    /// A ship with a Class A transponder: the mandatory fit for SOLAS vessels.
    #[default]
    ClassA,
    /// A ship with a Class B unit — smaller craft, lower power, slower
    /// reporting.
    ClassB,
    /// A shore station: `00MIDXXXX`, or anything sending message 4.
    BaseStation,
    /// A search-and-rescue aircraft: `111MIDXXX`, or message 9. It has an
    /// altitude and no draught.
    SarAircraft,
    /// An aid to navigation — a buoy, a beacon, a lighthouse, or a "virtual"
    /// mark that exists only as this transmission. `99MIDXXXX`, or message 21.
    AidToNavigation,
    /// A search-and-rescue transmitter, man-overboard beacon or AIS EPIRB:
    /// `970`, `972`, `974`. The one kind of target that is an emergency by
    /// existing.
    Sart,
    /// A craft associated with a parent ship — a lifeboat, a tender: `98MIDXXXX`.
    Craft,
}

/// The lowest and highest Maritime Identification Digits ITU has ever
/// allocated — the country code buried in almost every MMSI format.
///
/// 2xx Europe, 3xx North America, Central America and the Caribbean, 4xx Asia,
/// 5xx Oceania, 6xx Africa, 7xx South America. Nothing below 201 or above 775
/// has been assigned to anybody, and the numbers in between that are still
/// spare are a moving target not worth tabulating: what matters here is that
/// the field is a country and not three digits of noise.
///
/// Source: ITU-R M.585-9 Annex 1, and the ITU's MID list.
pub const MMSI_MID_MIN: u32 = 201;
/// See [`MMSI_MID_MIN`].
pub const MMSI_MID_MAX: u32 = 775;

/// True if `mid` is in the range ITU allocates country codes from.
fn is_mid(mid: u32) -> bool {
    (MMSI_MID_MIN..=MMSI_MID_MAX).contains(&mid)
}

/// True if `mmsi` is an identity ITU-R M.585 defines *and* one a station can
/// transmit from.
///
/// # Why a decoder needs this at all
///
/// The only thing standing between a burst of interference and a target on the
/// chart is a sixteen-bit frame check sequence, and sixteen bits is not very
/// many. AIS is self-organising TDMA with no collision detection, so two
/// stations out of range of each other reuse the same slot and go on doing so
/// for as long as they are both there — the hidden-node case the standard
/// accepts by design. A receiver in the middle hears the pair welded together
/// every time round, and once in about sixty-five thousand of those the wreck
/// checks out. Because the collision *repeats*, so does the frame: one lucky
/// check sequence becomes a station that reports for hours, drifting about the
/// map as the noise on it changes. That is what issue #400 saw — two of them,
/// steaming about in the Pacific at anchor.
///
/// What gives them away is the identity. An MMSI is nine digits with a
/// structure, and a number pulled out of a collision has no reason to have one:
/// both of the ones reported were five digits long. So a message from a number
/// that is not an identity is not a message — no station could have sent it,
/// whatever its check sequence came out as.
///
/// Source: ITU-R M.585-9 (assignment and use of maritime identities).
#[must_use]
pub fn mmsi_is_identity(mmsi: u32) -> bool {
    match mmsi {
        // 111MIDXXX — SAR aircraft.
        111_000_000..=111_999_999 => is_mid(mmsi / 1_000 % 1_000),
        // MIDXXXXXX — a ship station, the ordinary case.
        200_000_000..=799_999_999 => is_mid(mmsi / 1_000_000),
        // 8MIDXXXXX — a handheld VHF set with DSC and a GNSS receiver.
        800_000_000..=899_999_999 => is_mid(mmsi / 100_000 % 1_000),
        // 970yyxxxx / 972yyxxxx / 974yyxxxx — AIS-SART, man-overboard beacon
        // and EPIRB-AIS. These carry a manufacturer number where everything
        // else carries a country, so there is no MID in them to check; the
        // three-digit prefix is the whole of their format.
        970_000_000..=970_999_999 | 972_000_000..=972_999_999 | 974_000_000..=974_999_999 => true,
        // 98MIDXXXX — a craft associated with a parent ship. 99MIDXXXX — an
        // aid to navigation.
        980_000_000..=999_999_999 => is_mid(mmsi / 10_000 % 1_000),
        // 00MIDXXXX — a coast or base station.
        0..=9_999_999 => is_mid(mmsi / 10_000),
        // 0MIDXXXXX is a group of ships: an address to call, not a station,
        // and nothing transmits from one. Everything else — 1xx that is not
        // 111, 9xx that is not 97/98/99 — is unallocated.
        _ => false,
    }
}

impl AisKind {
    /// What the MMSI's own format says, where it says anything.
    ///
    /// ITU-R M.585 reserves whole prefixes, and those are worth more than the
    /// message a station happened to send: a base station that has only ever
    /// been heard sending a position report is still a base station.
    ///
    /// A prefix only counts where the country code inside it counts too —
    /// `000049613` matches `00MIDXXXX` on its first two digits and has `004`
    /// where the MID belongs, so calling it a base station labels a decoding
    /// accident as the shore. See [`mmsi_is_identity`].
    pub fn from_mmsi(mmsi: u32) -> Option<AisKind> {
        if !mmsi_is_identity(mmsi) {
            return None;
        }
        match mmsi {
            // 970xxxxxx / 972xxxxxx / 974xxxxxx — SART, MOB, EPIRB-AIS.
            970_000_000..=970_999_999 | 972_000_000..=972_999_999 | 974_000_000..=974_999_999 => {
                Some(AisKind::Sart)
            }
            // 99MIDXXXX — aid to navigation.
            990_000_000..=999_999_999 => Some(AisKind::AidToNavigation),
            // 98MIDXXXX — craft associated with a parent ship.
            980_000_000..=989_999_999 => Some(AisKind::Craft),
            // 111MIDXXX — SAR aircraft.
            111_000_000..=111_999_999 => Some(AisKind::SarAircraft),
            // 00MIDXXXX — coast/base station. 0MIDXXXXX is a group call, which
            // is not a station at all and never originates a report.
            0..=9_999_999 => Some(AisKind::BaseStation),
            _ => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            AisKind::ClassA => "Class A",
            AisKind::ClassB => "Class B",
            AisKind::BaseStation => "base station",
            AisKind::SarAircraft => "SAR aircraft",
            AisKind::AidToNavigation => "aid to navigation",
            AisKind::Sart => "SART / MOB / EPIRB",
            AisKind::Craft => "associated craft",
        }
    }

    /// Three characters for the table's TYPE column.
    pub fn short(self) -> &'static str {
        match self {
            AisKind::ClassA => "A",
            AisKind::ClassB => "B",
            AisKind::BaseStation => "BASE",
            AisKind::SarAircraft => "SAR",
            AisKind::AidToNavigation => "ATON",
            AisKind::Sart => "SART",
            AisKind::Craft => "CRFT",
        }
    }

    /// Whether this is something that moves. An aid to navigation and a base
    /// station both report a position and neither has a course, so the map
    /// draws them as marks rather than as vessels and leaves off the vector.
    pub fn is_underway(self) -> bool {
        matches!(self, AisKind::ClassA | AisKind::ClassB | AisKind::SarAircraft | AisKind::Craft)
    }
}

/// One station, as everything heard from it so far.
///
/// "Vessel" because that is what nearly every row is; a base station and a buoy
/// are here too, told apart by [`Self::kind`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AisVessel {
    /// The Maritime Mobile Service Identity: nine digits, the table's key.
    pub mmsi: u32,
    /// What it calls itself, from a static report (message 5, 19, 21 or 24A).
    /// Empty until one arrives, which can be six minutes — a target with no
    /// name is normal rather than broken.
    pub name: String,
    /// The radio call sign, from message 5 or 24B.
    pub call_sign: String,
    /// IMO number, from message 5. Zero and 1–999999 are both "not supplied"
    /// in practice, so it is `None` unless it looks like a real one.
    pub imo: Option<u32>,
    /// What sort of station this is.
    pub kind: AisKind,
    /// The ship and cargo type code, and the wording of it — see
    /// [`ship_type_label`]. Kept as the code as well so a panel can sort or
    /// filter on the class without re-parsing the words.
    pub ship_type: Option<u8>,
    /// The aid-to-navigation type code, where this is one.
    pub aid_type: Option<u8>,
    /// A "virtual" aid: a mark that exists only as this transmission, with
    /// nothing physically there. Worth flagging — it is the one thing on the
    /// chart that cannot be seen from the bridge.
    pub virtual_aid: bool,
    /// Navigational status (message 1/2/3), 0–15. See [`nav_status_label`].
    pub nav_status: Option<u8>,
    /// Where it says it is going, from message 5.
    pub destination: String,
    /// Estimated time of arrival as the message states it: `MM-DD HH:MM`, UTC.
    pub eta: String,
    /// Maximum present static draught, metres.
    pub draught_m: Option<f32>,
    /// Reference point of the reported position, metres from the transponder to
    /// bow, stern, port and starboard. Their sums are the vessel's length and
    /// beam, which is what makes a 300 m tanker draw larger than a pilot boat.
    pub dim_bow_m: u16,
    pub dim_stern_m: u16,
    pub dim_port_m: u16,
    pub dim_starboard_m: u16,
    /// Latest position, degrees. `None` until a position report arrives.
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    /// Where it has been, oldest first — bounded by
    /// [`AisSettings::trail_minutes`] in time and [`AIS_TRACK_MAX`] in points.
    ///
    /// `f32` for the reason the ADS-B track is: at these latitudes that is
    /// about two metres, far under what a history dot can show, and it halves
    /// what the whole table costs on a message re-sent twice a second.
    pub track: Vec<(f32, f32)>,
    /// Speed over ground, knots.
    pub sog_kt: Option<f32>,
    /// Course over ground, degrees true — where it is *going*.
    pub cog_deg: Option<f32>,
    /// True heading, degrees — where the bow is *pointing*. Not the same thing
    /// as the course in any tide or wind, which is why both are kept and why
    /// the icon is drawn to the heading where there is one.
    pub heading_deg: Option<f32>,
    /// Rate of turn, degrees a minute; negative is to port. Broadcast, unlike
    /// ADS-B's, but only by Class A and only when the sensor is fitted.
    pub turn_rate_deg_min: Option<f32>,
    /// Altitude in metres, for a SAR aircraft (message 9).
    pub altitude_m: Option<i32>,
    /// The position is a differentially corrected fix (better than 10 m) rather
    /// than an uncorrected one.
    pub accuracy: bool,
    /// The receiver's own integrity check passed at the transmitter.
    pub raim: bool,
    /// UTC as a base station stated it (message 4), `HH:MM:SS`.
    pub utc: String,
    /// Which channel the last message came in on: `'A'` or `'B'`.
    pub channel: char,
    /// Signal level of the last accepted message, dBFS — negative.
    pub rssi_dbfs: f32,
    /// ...and how far above that channel's own noise floor it was.
    pub snr_db: f32,
    /// Messages accepted from this MMSI this session.
    pub messages: u32,
    /// The type number of the most recent accepted message.
    pub last_type: u8,
    /// Unix seconds when first heard this session.
    pub first_at: i64,
    /// Unix seconds of the last accepted message of any kind.
    pub last_at: i64,
    /// Unix seconds of the last message that moved the position. Zero when
    /// there has never been one.
    pub last_pos_at: i64,
    /// The last accepted message as the `!AIVDM` sentences a receiver would
    /// put on an NMEA line — several where the message needed more than one.
    ///
    /// Here rather than a hex dump because it is the one thing in the panel an
    /// operator can *check*. This decoder was written from ITU-R M.1371 and not
    /// from a recording, so the question "is it reading the standard right"
    /// cannot be answered from inside sdroxide; a sentence every AIS tool in
    /// the world accepts can be pasted into one of them and the answers
    /// compared.
    pub nmea: String,
}

impl AisVessel {
    /// A fresh entry for an MMSI just heard.
    pub fn new(mmsi: u32, now: i64) -> AisVessel {
        AisVessel {
            mmsi,
            name: String::new(),
            call_sign: String::new(),
            imo: None,
            kind: AisKind::from_mmsi(mmsi).unwrap_or(AisKind::ClassA),
            ship_type: None,
            aid_type: None,
            virtual_aid: false,
            nav_status: None,
            destination: String::new(),
            eta: String::new(),
            draught_m: None,
            dim_bow_m: 0,
            dim_stern_m: 0,
            dim_port_m: 0,
            dim_starboard_m: 0,
            lat: None,
            lon: None,
            track: Vec::new(),
            sog_kt: None,
            cog_deg: None,
            heading_deg: None,
            turn_rate_deg_min: None,
            altitude_m: None,
            accuracy: false,
            raim: false,
            utc: String::new(),
            channel: 'A',
            rssi_dbfs: -100.0,
            snr_db: 0.0,
            messages: 0,
            last_type: 0,
            first_at: now,
            last_at: now,
            last_pos_at: 0,
            nmea: String::new(),
        }
    }

    /// What to call it on screen: the name once it has arrived, else the MMSI.
    /// Never empty — a target with no label is a target the operator cannot
    /// talk about.
    pub fn label(&self) -> String {
        let n = self.name.trim();
        if n.is_empty() { self.mmsi.to_string() } else { n.to_string() }
    }

    /// Has a position at all.
    pub fn has_position(&self) -> bool {
        self.lat.is_some() && self.lon.is_some()
    }

    /// The position is too old to draw: no position report for `drop_map_s`.
    ///
    /// Answers `true` for a station that has never had one, which is what the
    /// map wants — there is nothing to place — while the list still shows the
    /// row, because "heard, named, position not yet" is real information.
    pub fn pos_stale(&self, now: i64, drop_map_s: u16) -> bool {
        if !self.has_position() {
            return true;
        }
        now - self.last_pos_at > i64::from(drop_map_s)
    }

    /// Overall length in metres, where the dimensions have been reported.
    pub fn length_m(&self) -> Option<u32> {
        let l = u32::from(self.dim_bow_m) + u32::from(self.dim_stern_m);
        (l > 0).then_some(l)
    }

    /// Beam in metres, likewise.
    pub fn beam_m(&self) -> Option<u32> {
        let b = u32::from(self.dim_port_m) + u32::from(self.dim_starboard_m);
        (b > 0).then_some(b)
    }

    /// Which way to draw the icon: the heading where the vessel reports one,
    /// and the course over ground otherwise.
    ///
    /// Both, in that order, because they are different facts and the wrong one
    /// looks worse than none: a ferry crabbing across a tideway is *pointing*
    /// twenty degrees off its track, and a symbol drawn to the course would
    /// show it sailing sideways. A station with no heading sensor reports only
    /// the course, and for it the course is the best available answer.
    pub fn icon_deg(&self) -> Option<f32> {
        self.heading_deg.or(self.cog_deg)
    }

    /// Speed as the three characters a table column has room for.
    pub fn fmt_speed(&self) -> String {
        match self.sog_kt {
            Some(kt) if kt >= 100.0 => format!("{kt:.0}"),
            Some(kt) => format!("{kt:.1}"),
            None => "---".to_string(),
        }
    }

    /// Course as three digits, the way a compass is read.
    pub fn fmt_course(&self) -> String {
        match self.cog_deg.or(self.heading_deg) {
            Some(d) => format!("{:03.0}", d.rem_euclid(360.0)),
            None => "---".to_string(),
        }
    }

    /// Length by beam, the way a port entry writes it.
    pub fn fmt_size(&self) -> String {
        match (self.length_m(), self.beam_m()) {
            (Some(l), Some(b)) => format!("{l}×{b} m"),
            (Some(l), None) => format!("{l} m"),
            _ => String::new(),
        }
    }

    /// The worded ship type, where one was reported.
    pub fn type_label(&self) -> Option<&'static str> {
        match self.kind {
            AisKind::AidToNavigation => self.aid_type.map(aid_type_label),
            _ => self.ship_type.map(ship_type_label),
        }
    }

    /// True while this station is an emergency by its own account: a SART, a
    /// man-overboard beacon, an EPIRB, or a vessel declaring that it is not
    /// under command or aground.
    pub fn is_alarm(&self) -> bool {
        self.kind == AisKind::Sart || matches!(self.nav_status, Some(2) | Some(6))
    }
}

/// The navigational status a Class A vessel declares (ITU-R M.1371 Table 45).
///
/// Worth wording rather than numbering: "not under command" and "restricted
/// manoeuvrability" are the two that change what another vessel must do, and
/// nobody reads them as 2 and 3.
pub fn nav_status_label(code: u8) -> &'static str {
    match code {
        0 => "under way using engine",
        1 => "at anchor",
        2 => "not under command",
        3 => "restricted manoeuvrability",
        4 => "constrained by draught",
        5 => "moored",
        6 => "aground",
        7 => "fishing",
        8 => "under way sailing",
        9 => "reserved (HSC)",
        10 => "reserved (WIG)",
        11 => "towing astern",
        12 => "pushing ahead or towing alongside",
        13 => "reserved",
        14 => "AIS-SART / MOB / EPIRB",
        _ => "undefined",
    }
}

/// The ship and cargo type (ITU-R M.1371 Table 50), as words.
///
/// The first digit is the class and the second says what it is carrying or how
/// fast it goes; the pairing is a table rather than a rule, which is why this is
/// written out. Only the hazard categories are folded, because "carrying
/// dangerous goods, category A" is the same sentence whatever the hull is.
pub fn ship_type_label(code: u8) -> &'static str {
    match code {
        0 => "not available",
        1..=19 => "reserved",
        20..=29 => "wing in ground",
        30 => "fishing",
        31 => "towing",
        32 => "towing (long or wide)",
        33 => "dredging or underwater ops",
        34 => "diving ops",
        35 => "military ops",
        36 => "sailing",
        37 => "pleasure craft",
        38 | 39 => "reserved",
        40..=49 => "high-speed craft",
        50 => "pilot vessel",
        51 => "search and rescue",
        52 => "tug",
        53 => "port tender",
        54 => "anti-pollution",
        55 => "law enforcement",
        56 | 57 => "local vessel",
        58 => "medical transport",
        59 => "noncombatant",
        60..=69 => "passenger",
        70..=79 => "cargo",
        80..=89 => "tanker",
        _ => "other",
    }
}

/// Whether a ship-type code says the vessel is carrying dangerous goods,
/// harmful substances or marine pollutants — the second digit, for the classes
/// that use it that way.
pub fn ship_type_hazard(code: u8) -> Option<&'static str> {
    let ranged = matches!(code, 20..=29 | 40..=49 | 60..=69 | 70..=79 | 80..=89 | 90..=99);
    if !ranged {
        return None;
    }
    match code % 10 {
        1 => Some("hazard category A"),
        2 => Some("hazard category B"),
        3 => Some("hazard category C"),
        4 => Some("hazard category D"),
        _ => None,
    }
}

/// The aid-to-navigation type (ITU-R M.1371 Table 51 / IALA G1082), as words.
pub fn aid_type_label(code: u8) -> &'static str {
    match code {
        0 => "aid to navigation",
        1 => "reference point",
        2 => "RACON",
        3 => "offshore structure",
        4 => "spare",
        5 => "light, without sectors",
        6 => "light, with sectors",
        7 => "leading light front",
        8 => "leading light rear",
        9 => "beacon, cardinal N",
        10 => "beacon, cardinal E",
        11 => "beacon, cardinal S",
        12 => "beacon, cardinal W",
        13 => "beacon, port hand",
        14 => "beacon, starboard hand",
        15 => "beacon, preferred channel port",
        16 => "beacon, preferred channel starboard",
        17 => "beacon, isolated danger",
        18 => "beacon, safe water",
        19 => "beacon, special mark",
        20 => "cardinal mark N",
        21 => "cardinal mark E",
        22 => "cardinal mark S",
        23 => "cardinal mark W",
        24 => "port hand mark",
        25 => "starboard hand mark",
        26 => "preferred channel port",
        27 => "preferred channel starboard",
        28 => "isolated danger",
        29 => "safe water",
        30 => "special mark",
        31 => "light vessel / LANBY / rig",
        _ => "aid to navigation",
    }
}

/// What one of the two channels is doing.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AisChannelStatus {
    pub freq_hz: f64,
    /// `"A"` or `"B"`.
    pub label: String,
    /// A demodulator is running on it.
    pub live: bool,
    /// Why it is not, when it is not: outside the receiver's window, or
    /// switched off. The two look identical on screen and want completely
    /// different answers.
    pub reason: Option<String>,
    /// Slots the gate opened on, and messages that came out of them.
    pub bursts: u64,
    pub messages: u64,
    /// The learned noise floor, dBFS — what the threshold is measured from.
    pub floor_dbfs: f32,
}

/// What the engine tells the panel about the decoder itself.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AisStatus {
    /// Every station still on the list, in no particular order — the panel
    /// sorts.
    pub vessels: Vec<AisVessel>,
    /// The two channels, always both, live or not.
    pub channels: Vec<AisChannelStatus>,
    /// Why nothing is running, when nothing is running. A receiver that cannot
    /// reach 162 MHz, or cannot deliver a wide enough stream, produces an empty
    /// panel either way; only this distinguishes that from an empty sea.
    pub unavailable: Option<String>,
    /// Why the decoder will do badly here even though it is running — one
    /// channel out of two, or too few samples a bit.
    pub degraded: Option<String>,
    /// Where the operator would have to tune for the decoder to work. `None`
    /// when the dial is already right.
    pub suggest_center_hz: Option<f64>,
    /// Where the decoder's own window is, and how wide, in Hz. Shown because
    /// "your receiver is not looking at 162 MHz" is a claim about numbers the
    /// operator cannot otherwise see.
    pub window_center_hz: f64,
    pub window_rate_hz: f64,
    /// Slots the gates opened on across both channels, and how many of those
    /// produced a frame whose check sequence passed. A high burst count with no
    /// messages is a band busy with something else — worth showing rather than
    /// leaving the panel looking broken.
    pub bursts: u64,
    pub messages: u64,
    /// Frames that framed correctly and whose check sequence did not pass.
    pub bad_fcs: u64,
    /// Frames that passed their check sequence and carried a message type this
    /// decoder does not read. Counted rather than hidden: it is the difference
    /// between "the receiver is deaf" and "the receiver is fine and there is
    /// nothing here worth showing you".
    pub unsupported: u64,
    /// Samples a bit the channel demodulators are running at.
    pub samples_per_bit: f32,
    /// How far off frequency the transmissions that decoded have been, in Hz.
    ///
    /// A frequency discriminator turns a carrier offset into a DC level, and
    /// the decoder measures that level rather than assuming it — so this costs
    /// nothing and answers the question an operator otherwise has no way to
    /// ask. Every ship being three kilohertz off in the same direction is not
    /// three thousand ships with bad oscillators; it is this receiver, and the
    /// fix is the front end's frequency correction.
    ///
    /// `None` until something has decoded, because a burst that did not decode
    /// measures whatever the gate opened on rather than a ship.
    pub offset_hz: Option<f32>,
}

/// How the AIS decoder behaves. Owned by the engine (it lives in
/// [`crate::RadioState`]), edited from the panel's setup, and persisted across
/// restarts (`ais.json`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AisSettings {
    /// The decoder runs. Follows the mode — selecting AIS switches it on — but
    /// kept as a field so a front end that cannot feed it can switch it off and
    /// say so.
    pub enabled: bool,
    /// One bit per channel, A in the low bit.
    pub channels: u8,
    /// How far above the learned noise floor a slot has to be, in dB.
    pub threshold_db: u8,
    /// Seconds without a position report before a target leaves the map and its
    /// row greys.
    pub drop_map_s: u16,
    /// Seconds without any message at all before the station leaves the list.
    pub drop_list_s: u16,
    /// How much history to keep and draw behind each target, in minutes.
    pub trail_minutes: u16,
    /// How far ahead the vector reaches, in minutes at the current speed over
    /// ground.
    pub vector_minutes: f32,
    /// Ceiling on the table, so a busy estuary cannot grow the status message
    /// without bound. The stations heard longest ago are dropped first.
    pub max_vessels: u16,
}

impl Default for AisSettings {
    fn default() -> Self {
        AisSettings {
            enabled: true,
            channels: AIS_ALL_CHANNELS,
            threshold_db: AIS_THRESHOLD_DB,
            drop_map_s: AIS_DROP_MAP_S,
            drop_list_s: AIS_DROP_LIST_S,
            trail_minutes: AIS_TRAIL_MINUTES,
            vector_minutes: AIS_VECTOR_MINUTES,
            max_vessels: AIS_MAX_VESSELS,
        }
    }
}

impl AisSettings {
    /// The decoder switched off, for a front end that cannot run it.
    ///
    /// A separate value rather than `Default` with `enabled: false`, for the
    /// reason [`crate::AdsbSettings::OFF`] is: the engine forces this into the
    /// live state on an audio-mode source, and that must not be mistaken for
    /// what the operator chose.
    pub const OFF: AisSettings = AisSettings {
        enabled: false,
        channels: AIS_ALL_CHANNELS,
        threshold_db: AIS_THRESHOLD_DB,
        drop_map_s: AIS_DROP_MAP_S,
        drop_list_s: AIS_DROP_LIST_S,
        trail_minutes: AIS_TRAIL_MINUTES,
        vector_minutes: AIS_VECTOR_MINUTES,
        max_vessels: AIS_MAX_VESSELS,
    };

    /// The settings with every field inside the range the decoder can honour.
    ///
    /// Applied where they arrive rather than where they are used: these come
    /// from a config file an operator may have edited and from a remote client,
    /// and a zero trail or a million-vessel ceiling should be corrected once
    /// rather than defended against everywhere.
    pub fn sane(mut self) -> AisSettings {
        self.channels &= AIS_ALL_CHANNELS;
        self.threshold_db = self.threshold_db.clamp(3, 30);
        self.drop_map_s = self.drop_map_s.clamp(10, 3_600);
        self.drop_list_s = self.drop_list_s.clamp(self.drop_map_s, 21_600);
        self.trail_minutes = self.trail_minutes.clamp(0, 360);
        self.vector_minutes = self.vector_minutes.clamp(0.0, 60.0);
        self.max_vessels = self.max_vessels.clamp(10, 5_000);
        self
    }

    /// Whether channel `i` — 0 for A, 1 for B — is switched on.
    pub fn channel_enabled(self, i: usize) -> bool {
        i < 2 && self.channels & (1 << i) != 0
    }

    /// Whether anything at all is switched on.
    pub fn any_enabled(self) -> bool {
        self.channels & AIS_ALL_CHANNELS != 0
    }

    /// The trail window in seconds, which is what the tracker actually trims
    /// against.
    pub fn trail_secs(self) -> i64 {
        i64::from(self.trail_minutes) * 60
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The dial sits between the two channels, and both are on the marine
    /// raster — the arithmetic that says the plan is the plan.
    #[test]
    fn the_two_channels_straddle_the_plan_centre() {
        assert_eq!((AIS_CHANNEL_A_HZ + AIS_CHANNEL_B_HZ) / 2.0, AIS_PLAN_CENTER_HZ);
        assert_eq!(AIS_CHANNEL_B_HZ - AIS_CHANNEL_A_HZ, 2.0 * AIS_CHANNEL_SPACING_HZ);
    }

    /// A prefix decides what a station is, and it outranks whatever message it
    /// happened to send: `992...` is a buoy even in a position report.
    #[test]
    fn the_mmsi_format_says_what_a_station_is() {
        assert_eq!(AisKind::from_mmsi(992_111_840), Some(AisKind::AidToNavigation));
        assert_eq!(AisKind::from_mmsi(2_442_000), Some(AisKind::BaseStation));
        assert_eq!(AisKind::from_mmsi(111_232_500), Some(AisKind::SarAircraft));
        assert_eq!(AisKind::from_mmsi(970_123_456), Some(AisKind::Sart));
        assert_eq!(AisKind::from_mmsi(982_345_678), Some(AisKind::Craft));
        // An ordinary ship's MMSI says nothing beyond the flag, so the message
        // it sent is what decides Class A from Class B.
        assert_eq!(AisKind::from_mmsi(244_660_000), None);
    }

    /// Issue #400: a number with no country in it is not an identity, and a
    /// prefix alone is not enough to make one. `000049613` matches `00MIDXXXX`
    /// on its two leading zeros and carries `004` where the MID belongs.
    #[test]
    fn a_number_with_no_country_in_it_is_not_an_identity() {
        for real in [
            244_660_000, // a Dutch ship
            316_001_234, // a Canadian ship
            3_160_021,   // a Canadian coast station
            992_111_840, // a German aid to navigation
            111_232_500, // a British SAR aircraft
            982_345_678, // a British ship's tender
            824_712_345, // an Italian handheld
            970_123_456, // an AIS-SART, which has a maker's number, not a MID
        ] {
            assert!(mmsi_is_identity(real), "{real} is a station");
        }
        for wrong in [
            0,           // never programmed
            49_613,      // the first of issue #400's ghosts: 00 004 9613
            65_216,      // and the second
            100_000_000, // 1xx that is not 111MID
            920_000_000, // 92x is not allocated
            20_100_000,  // 0 201 00000 — a group of ships, which never sends
            800_000_000, // 8 000 00000 — no country
            200_000_000, // 200 is one below the first MID ITU ever handed out
        ] {
            assert!(!mmsi_is_identity(wrong), "{wrong} is not a station");
        }
    }

    /// The map's clock and the list's clock are not the same clock, and a
    /// station with no position at all is stale for the map's purposes while
    /// still being a real row.
    #[test]
    fn a_moored_ship_stays_on_the_map_between_reports() {
        let mut v = AisVessel::new(244_660_000, 1_000);
        assert!(v.pos_stale(1_000, AIS_DROP_MAP_S), "never positioned is nothing to draw");
        v.lat = Some(52.37);
        v.lon = Some(4.89);
        v.last_pos_at = 1_000;
        // Three minutes on — the interval a vessel at anchor reports at — it is
        // still drawn. Under ADS-B's ten-second window it would not be, and
        // that is the whole reason these constants are separate.
        assert!(!v.pos_stale(1_180, AIS_DROP_MAP_S));
        assert!(v.pos_stale(1_400, AIS_DROP_MAP_S));
    }

    /// The icon points where the bow points when the vessel says, and along the
    /// track when it does not.
    #[test]
    fn the_icon_follows_the_heading_before_the_course() {
        let mut v = AisVessel::new(244_660_000, 0);
        assert_eq!(v.icon_deg(), None);
        v.cog_deg = Some(275.0);
        assert_eq!(v.icon_deg(), Some(275.0));
        v.heading_deg = Some(255.0);
        assert_eq!(v.icon_deg(), Some(255.0), "a heading outranks a course");
    }

    /// A hand-edited config cannot ask for a list window shorter than the map
    /// one — that would drop a vessel before it had a chance to grey.
    #[test]
    fn the_list_window_is_never_shorter_than_the_map_window() {
        let s = AisSettings { drop_map_s: 600, drop_list_s: 30, ..AisSettings::default() }.sane();
        assert!(s.drop_list_s >= s.drop_map_s);
        let wild = AisSettings { channels: 0xff, ..AisSettings::default() }.sane();
        assert_eq!(wild.channels, AIS_ALL_CHANNELS);
        assert!(!AisSettings::OFF.enabled);
        assert!(AisSettings::default().channel_enabled(0));
        assert!(AisSettings::default().channel_enabled(1));
        assert!(!AisSettings::default().channel_enabled(2));
    }

    /// The defaults are the ones the reporting intervals ask for, and the
    /// module note is the argument: five minutes on the map, ten of trail.
    #[test]
    fn the_defaults_are_set_from_the_reporting_intervals() {
        let d = AisSettings::default();
        assert_eq!(d.drop_map_s, 300, "a vessel at anchor reports every 180 s");
        assert_eq!(d.trail_minutes, 10);
        // The message carrying a ship's name comes round every six minutes, so
        // the list window has to span several of them or a vessel keeps being
        // dropped just before it says what it is called.
        assert!(
            i64::from(d.drop_list_s) >= 3 * 360,
            "{} s is under three static reports",
            d.drop_list_s
        );
        assert_eq!(d.trail_secs(), 600);
    }

    /// A ship's type reads as words, and a tanker carrying category A goods
    /// says so — the one part of the code an operator has to act on.
    #[test]
    fn a_ship_type_reads_as_words_and_a_hazard_shows() {
        assert_eq!(ship_type_label(70), "cargo");
        assert_eq!(ship_type_label(80), "tanker");
        assert_eq!(ship_type_label(52), "tug");
        assert_eq!(ship_type_hazard(81), Some("hazard category A"));
        // The single-code classes have no second digit to read that way.
        assert_eq!(ship_type_hazard(52), None);
        assert_eq!(ship_type_hazard(70), None);
    }
}
