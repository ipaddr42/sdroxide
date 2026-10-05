//! The NCDXF/IARU International Beacon Project: which beacon is transmitting on
//! which band, right now.
//!
//! Eighteen beacons, five bands, and a schedule that is entirely deterministic:
//! a 180-second cycle of eighteen 10-second slots, aligned to UTC midnight, in
//! which exactly one beacon is on each band at any instant. No network, no
//! state — a clock and a table, which is why it lives here with the other pure
//! data.
//!
//! Why a listener cares: a beacon you can hear is a path that is open, measured
//! rather than forecast. Each beacon's site is known, so the same row also
//! answers "which direction, how far".
//!
//! Callsigns, sites and locators are the NCDXF's published list
//! (<https://www.ncdxf.org/beacon/beaconlocations.html>); a position is the
//! centre of the six-character locator, which is as precisely as the NCDXF
//! states it. The slot order and band offsets are the published schedule,
//! cross-checked against [OpenHamClock](https://github.com/accius/openhamclock)
//! (MIT).

use crate::geo::{bearing_deg, distance_km, grid_to_latlon};

/// One IBP beacon.
pub struct Beacon {
    pub callsign: &'static str,
    /// The site, as the NCDXF names it, with the country for context.
    pub location: &'static str,
    /// The six-character Maidenhead locator the NCDXF publishes.
    pub locator: &'static str,
}

impl Beacon {
    /// Latitude and longitude, degrees (north and east positive): the centre
    /// of the published locator.
    pub fn position(&self) -> (f64, f64) {
        grid_to_latlon(self.locator).expect("every beacon's locator is a valid six-character one")
    }
}

/// The eighteen beacons, in transmission order. The index into this array *is*
/// the beacon's place in the cycle.
pub const BEACONS: &[Beacon] = &[
    Beacon { callsign: "4U1UN", location: "United Nations, New York", locator: "FN30as" },
    Beacon { callsign: "VE8AT", location: "Inuvik, Canada", locator: "CP38gh" },
    Beacon { callsign: "W6WX", location: "Mt. Umunhum, California", locator: "CM97bd" },
    Beacon { callsign: "KH6RS", location: "Maui, Hawaii", locator: "BL10ts" },
    Beacon { callsign: "ZL6B", location: "Masterton, New Zealand", locator: "RE78tw" },
    Beacon { callsign: "VK6RBP", location: "Rolystone, Australia", locator: "OF87av" },
    Beacon { callsign: "JA2IGY", location: "Mt. Asama, Japan", locator: "PM84jk" },
    Beacon { callsign: "RR9O", location: "Novosibirsk, Russia", locator: "NO14kx" },
    Beacon { callsign: "VR2B", location: "Hong Kong", locator: "OL72bg" },
    Beacon { callsign: "4S7B", location: "Colombo, Sri Lanka", locator: "MJ96wv" },
    Beacon { callsign: "ZS6DN", location: "Pretoria, South Africa", locator: "KG33xi" },
    Beacon { callsign: "5Z4B", location: "Kikuyu, Kenya", locator: "KI88hr" },
    Beacon { callsign: "4X6TU", location: "Tel Aviv, Israel", locator: "KM72jb" },
    Beacon { callsign: "OH2B", location: "Lohja, Finland", locator: "KP20eh" },
    Beacon { callsign: "CS3B", location: "São Jorge, Madeira", locator: "IM12jt" },
    Beacon { callsign: "LU4AA", location: "Buenos Aires, Argentina", locator: "GF05tj" },
    Beacon { callsign: "OA4B", location: "Lima, Peru", locator: "FH17mw" },
    Beacon { callsign: "YV5B", location: "Caracas, Venezuela", locator: "FK60nd" },
];

/// One beacon band and its place in the cycle.
pub struct IbpBand {
    /// The metre band, as operators say it.
    pub label: &'static str,
    /// The beacon frequency, Hz — what the dial wants.
    pub freq_hz: f64,
    /// Added to the cycle's slot to give this band's beacon. A beacon steps
    /// *up* one band every ten seconds, so the band `n` steps above 20 m is
    /// showing the beacon that was on 20 m `n` slots ago, `(slot - n) mod 18` —
    /// stored as `18 - n` so the sum never goes negative.
    pub offset: usize,
}

/// The five beacon bands, in the order a beacon steps through them.
pub const BANDS: &[IbpBand] = &[
    IbpBand { label: "20m", freq_hz: 14_100_000.0, offset: 0 },
    IbpBand { label: "17m", freq_hz: 18_110_000.0, offset: 17 },
    IbpBand { label: "15m", freq_hz: 21_150_000.0, offset: 16 },
    IbpBand { label: "12m", freq_hz: 24_930_000.0, offset: 15 },
    IbpBand { label: "10m", freq_hz: 28_200_000.0, offset: 14 },
];

/// Seconds one beacon holds a band.
pub const SLOT_SECONDS: i64 = 10;
/// Seconds in a full pass over the eighteen beacons.
pub const CYCLE_SECONDS: i64 = SLOT_SECONDS * BEACONS.len() as i64;

/// The slot within the cycle, `0..18`, for a Unix time.
///
/// The cycle is aligned to UTC midnight, as the NCDXF's own schedule is; the
/// day length is a whole number of cycles, so only the time of day matters.
pub fn slot_at(unix_s: i64) -> usize {
    (unix_s.rem_euclid(86_400) % CYCLE_SECONDS / SLOT_SECONDS) as usize
}

/// Seconds left in the current slot, `1..=SLOT_SECONDS`.
pub fn seconds_left_in_slot(unix_s: i64) -> i64 {
    SLOT_SECONDS - unix_s.rem_euclid(SLOT_SECONDS)
}

/// A band's beacon at one instant, with its geometry from the observer.
pub struct Active {
    pub band: &'static IbpBand,
    pub beacon: &'static Beacon,
    /// The beacon's place in the cycle, `0..18`.
    pub beacon_index: usize,
    /// Bearing from the observer, degrees clockwise from north. `None` when no
    /// observer position was given.
    pub bearing_deg: Option<f64>,
    /// Great-circle distance from the observer, km. `None` when no observer
    /// position was given.
    pub distance_km: Option<f64>,
}

/// What is transmitting on each band at `unix_s`, from an optional observer
/// position `(lat, lon)` for the bearing and distance.
pub fn active_at(unix_s: i64, from: Option<(f64, f64)>) -> Vec<Active> {
    let slot = slot_at(unix_s);
    BANDS
        .iter()
        .map(|band| {
            let beacon_index = (slot + band.offset) % BEACONS.len();
            let beacon = &BEACONS[beacon_index];
            let (bearing, distance) = match from {
                Some(here) => (
                    Some(bearing_deg(here, beacon.position())),
                    Some(distance_km(here, beacon.position())),
                ),
                None => (None, None),
            };
            Active { band, beacon, beacon_index, bearing_deg: bearing, distance_km: distance }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ymd_hms_to_unix;

    fn at(h: u32, m: u32, s: u32) -> i64 {
        ymd_hms_to_unix(2026, 6, 1, h, m, s)
    }

    /// The cycle starts at UTC midnight and turns over every three minutes.
    #[test]
    fn the_cycle_is_aligned_to_utc_midnight() {
        assert_eq!(slot_at(at(0, 0, 0)), 0);
        assert_eq!(slot_at(at(0, 0, 9)), 0);
        assert_eq!(slot_at(at(0, 0, 10)), 1);
        assert_eq!(slot_at(at(0, 2, 50)), 17);
        assert_eq!(slot_at(at(0, 3, 0)), 0, "the cycle did not restart at 3 minutes");
        assert_eq!(slot_at(at(23, 59, 59)), 17);
        // And on the next day, the same.
        let next = ymd_hms_to_unix(2026, 6, 2, 0, 0, 0);
        assert_eq!(slot_at(next), 0);
    }

    #[test]
    fn seconds_left_counts_down_within_the_slot() {
        assert_eq!(seconds_left_in_slot(at(0, 0, 0)), 10);
        assert_eq!(seconds_left_in_slot(at(0, 0, 1)), 9);
        assert_eq!(seconds_left_in_slot(at(0, 0, 9)), 1);
        assert_eq!(seconds_left_in_slot(at(0, 0, 10)), 10);
    }

    /// At the start of a cycle the 20 m beacon is the first in the list and the
    /// higher bands are the ones that started earlier — the offsets, not a
    /// guess.
    #[test]
    fn the_band_offsets_place_the_right_beacon_at_slot_zero() {
        let a = active_at(at(0, 0, 0), None);
        assert_eq!(a[0].beacon.callsign, "4U1UN", "20m at slot 0");
        assert_eq!(a[1].beacon.callsign, "YV5B", "17m is one slot behind the list");
        assert_eq!(a[2].beacon.callsign, "OA4B", "15m two slots behind");
        assert_eq!(a[3].beacon.callsign, "LU4AA", "12m three slots behind");
        assert_eq!(a[4].beacon.callsign, "CS3B", "10m four slots behind");
    }

    /// Over one full cycle every beacon reaches every band exactly once — the
    /// property the whole schedule rests on.
    #[test]
    fn every_beacon_works_every_band_once_a_cycle() {
        for band_ix in 0..BANDS.len() {
            let mut seen = vec![0u32; BEACONS.len()];
            for slot in 0..BEACONS.len() {
                let t = at(0, 0, 0) + slot as i64 * SLOT_SECONDS;
                let a = active_at(t, None);
                seen[a[band_ix].beacon_index] += 1;
            }
            assert!(
                seen.iter().all(|&n| n == 1),
                "band {} did not visit every beacon once: {seen:?}",
                BANDS[band_ix].label
            );
        }
    }

    /// The geometry is from the observer: no distance to a beacon from its own
    /// site, and a beacon due north of the observer bears north — and due
    /// south, south.
    #[test]
    fn the_geometry_is_measured_from_the_observer() {
        let (lat, lon) = BEACONS[0].position();
        let a = active_at(at(0, 0, 0), Some((lat, lon)));
        assert!(a[0].distance_km.unwrap() < 1.0, "distance to the beacon itself");

        let south_of_it = active_at(at(0, 0, 0), Some((lat - 10.0, lon)));
        let bearing = south_of_it[0].bearing_deg.unwrap();
        assert!(bearing < 0.5 || bearing > 359.5, "due north read as {bearing}°");
        let km = south_of_it[0].distance_km.unwrap();
        assert!((km - 1112.0).abs() < 10.0, "ten degrees of latitude read as {km} km");

        let north_of_it = active_at(at(0, 0, 0), Some((lat + 10.0, lon)));
        let bearing = north_of_it[0].bearing_deg.unwrap();
        assert!((bearing - 180.0).abs() < 0.5, "due south read as {bearing}°");
    }

    /// Every published locator parses, and lands where the site is: a few
    /// sites checked against where they are on the map, so a transposed letter
    /// in the table shows up here.
    #[test]
    fn the_locators_place_the_beacons_on_their_sites() {
        for b in BEACONS {
            let (lat, lon) = b.position();
            assert!(
                (-90.0..=90.0).contains(&lat) && (-180.0..=180.0).contains(&lon),
                "{}",
                b.callsign
            );
        }
        let near = |call: &str, lat: f64, lon: f64| {
            let b = BEACONS.iter().find(|b| b.callsign == call).unwrap();
            let km = distance_km(b.position(), (lat, lon));
            assert!(km < 25.0, "{call} is {km:.0} km from its site");
        };
        near("4U1UN", 40.749, -73.968); // the UN building
        near("KH6RS", 20.8, -156.33); // Maui, not Oahu
        near("VE8AT", 68.36, -133.72); // Inuvik
        near("OA4B", -12.05, -77.04); // Lima
    }
}
