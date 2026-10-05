//! Annual meteor showers: which are active, how strong, and where the radiant
//! is for the observer right now.
//!
//! A shower is a fixed date window and a fixed radiant, so the whole module is
//! a static table and two bits of arithmetic — no network, no state, no I/O.
//! That is deliberate: it is the one propagation forecast that does not depend
//! on the ionosphere, and it is the same every year, so it is worth carrying
//! rather than fetching.
//!
//! The shower dates, radiants, rates and velocities are the International
//! Meteor Organization's working list, at the peak. The radiant drifts a little
//! through a shower's window (the IMO publishes a degree or so a day); that
//! drift is well inside the "is the radiant up" question this answers, and the
//! table carries the peak position rather than pretending to more precision.
//!
//! Why a listener cares: a meteor leaves an ionised trail, and a trail that
//! crosses a path briefly opens it — meteor scatter is worked on 6 m and 2 m
//! (and heard as the "pings" on the low VHF broadcast band), and long
//! over-the-horizon paths on 10 m can come alive for seconds at a time. The useful question is not "how many meteors" but "is the radiant
//! above the horizon, and is the shower near its peak".

use crate::ephem::{gmst_deg, julian_day};
use crate::vec3::{Vec3, vec3};
use sdroxide_types::utc_ymd_hms;

/// One annual meteor shower, as the IMO publishes it.
pub struct Shower {
    /// The published name.
    pub name: &'static str,
    /// IMO's three-letter code, stable across years and used by observers.
    pub code: &'static str,
    /// First day of activity, `(month, day)`, inclusive. A window may wrap the
    /// new year — the Quadrantids run December into January — so containment is
    /// not a plain range.
    pub start: (u32, u32),
    /// The peak date. The rate below is the peak rate.
    pub peak: (u32, u32),
    /// Last day of activity, inclusive.
    pub end: (u32, u32),
    /// Zenithal hourly rate at the peak: meteors one observer would see with
    /// the radiant overhead under a perfect sky. A brightness figure, not a
    /// count to expect — the observed rate is usually a small fraction of it.
    pub zhr: u32,
    /// Radiant right ascension at the peak, degrees.
    pub ra_deg: f64,
    /// Radiant declination at the peak, degrees.
    pub dec_deg: f64,
    /// Geocentric velocity, km/s. Fast showers (above ~40) leave longer-lived
    /// trails and better meteor-scatter returns.
    pub velocity_kms: u32,
    /// Parent body, or the accepted best guess where none is certain.
    pub parent: &'static str,
}

/// The major showers, ordered by the calendar. Dates are the IMO working
/// list; the whole table is one screen and is meant to be read.
pub const SHOWERS: &[Shower] = &[
    Shower {
        name: "Quadrantids",
        code: "QUA",
        start: (12, 28),
        peak: (1, 3),
        end: (1, 12),
        zhr: 80,
        ra_deg: 230.0,
        dec_deg: 49.0,
        velocity_kms: 41,
        parent: "2003 EH1",
    },
    Shower {
        name: "April Lyrids",
        code: "LYR",
        start: (4, 14),
        peak: (4, 22),
        end: (4, 30),
        zhr: 18,
        ra_deg: 271.0,
        dec_deg: 34.0,
        velocity_kms: 49,
        parent: "C/1861 G1 Thatcher",
    },
    Shower {
        name: "Eta Aquariids",
        code: "ETA",
        start: (4, 19),
        peak: (5, 6),
        end: (5, 28),
        zhr: 50,
        ra_deg: 338.0,
        dec_deg: -1.0,
        velocity_kms: 66,
        parent: "1P/Halley",
    },
    // A daytime shower: its radiant is a few degrees from the Sun, so it is
    // seen by radar and heard by meteor scatter rather than watched — the
    // strongest of the year for radio, and the rate is the radio-derived one.
    Shower {
        name: "Daytime Arietids",
        code: "ARI",
        start: (5, 14),
        peak: (6, 7),
        end: (6, 24),
        zhr: 30,
        ra_deg: 44.0,
        dec_deg: 24.0,
        velocity_kms: 38,
        parent: "96P/Machholz complex (?)",
    },
    Shower {
        name: "Southern Delta Aquariids",
        code: "SDA",
        start: (7, 12),
        peak: (7, 30),
        end: (8, 23),
        zhr: 25,
        ra_deg: 340.0,
        dec_deg: -16.0,
        velocity_kms: 41,
        parent: "96P/Machholz",
    },
    Shower {
        name: "Perseids",
        code: "PER",
        start: (7, 17),
        peak: (8, 12),
        end: (8, 24),
        zhr: 100,
        ra_deg: 48.0,
        dec_deg: 58.0,
        velocity_kms: 59,
        parent: "109P/Swift-Tuttle",
    },
    Shower {
        name: "Kappa Cygnids",
        code: "KCG",
        start: (8, 3),
        peak: (8, 17),
        end: (8, 25),
        zhr: 3,
        ra_deg: 286.0,
        dec_deg: 59.0,
        velocity_kms: 25,
        parent: "2008 ED69 (?)",
    },
    Shower {
        name: "Aurigids",
        code: "AUR",
        start: (8, 28),
        peak: (9, 1),
        end: (9, 5),
        zhr: 6,
        ra_deg: 91.0,
        dec_deg: 39.0,
        velocity_kms: 66,
        parent: "C/1911 N1 Kiess",
    },
    Shower {
        name: "September Epsilon Perseids",
        code: "SPE",
        start: (9, 5),
        peak: (9, 9),
        end: (9, 21),
        zhr: 5,
        ra_deg: 48.0,
        dec_deg: 40.0,
        velocity_kms: 64,
        parent: "unknown",
    },
    Shower {
        name: "Draconids",
        code: "DRA",
        start: (10, 6),
        peak: (10, 8),
        end: (10, 10),
        zhr: 10,
        ra_deg: 262.0,
        dec_deg: 54.0,
        velocity_kms: 20,
        parent: "21P/Giacobini-Zinner",
    },
    Shower {
        name: "Orionids",
        code: "ORI",
        start: (10, 2),
        peak: (10, 21),
        end: (11, 7),
        zhr: 20,
        ra_deg: 95.0,
        dec_deg: 16.0,
        velocity_kms: 66,
        parent: "1P/Halley",
    },
    Shower {
        name: "Southern Taurids",
        code: "STA",
        start: (9, 10),
        peak: (10, 10),
        end: (11, 20),
        zhr: 5,
        ra_deg: 32.0,
        dec_deg: 9.0,
        velocity_kms: 27,
        parent: "2P/Encke",
    },
    Shower {
        name: "Northern Taurids",
        code: "NTA",
        start: (10, 20),
        peak: (11, 12),
        end: (12, 10),
        zhr: 5,
        ra_deg: 58.0,
        dec_deg: 22.0,
        velocity_kms: 29,
        parent: "2P/Encke",
    },
    Shower {
        name: "Leonids",
        code: "LEO",
        start: (11, 6),
        peak: (11, 17),
        end: (11, 30),
        zhr: 15,
        ra_deg: 152.0,
        dec_deg: 22.0,
        velocity_kms: 71,
        parent: "55P/Tempel-Tuttle",
    },
    Shower {
        name: "Geminids",
        code: "GEM",
        start: (12, 4),
        peak: (12, 14),
        end: (12, 20),
        zhr: 150,
        ra_deg: 112.0,
        dec_deg: 33.0,
        velocity_kms: 35,
        parent: "3200 Phaethon",
    },
    Shower {
        name: "Ursids",
        code: "URS",
        start: (12, 17),
        peak: (12, 22),
        end: (12, 26),
        zhr: 10,
        ra_deg: 217.0,
        dec_deg: 76.0,
        velocity_kms: 33,
        parent: "8P/Tuttle",
    },
];

/// A shower that is active right now, with its radiant placed for the observer.
pub struct ActiveShower {
    pub shower: &'static Shower,
    /// Radiant altitude above the observer's horizon, degrees. Below zero the
    /// shower is still meteors, but its trails arrive from below and are largely
    /// hidden by the Earth.
    pub alt_deg: f64,
    /// Radiant azimuth, degrees clockwise from north.
    pub az_deg: f64,
    /// Signed days to the peak: zero on the peak date, negative after it.
    /// Wrapped into `[-182, 182]` so a December shower's peak reads as days
    /// away, not most of a year.
    pub days_to_peak: i32,
}

impl ActiveShower {
    /// True on the peak date itself.
    pub fn at_peak(&self) -> bool {
        self.days_to_peak == 0
    }

    /// True while the radiant is above the horizon.
    pub fn radiant_up(&self) -> bool {
        self.alt_deg > 0.0
    }
}

/// Cumulative days before the first of each month in a non-leap year, for the
/// approximate day-of-year the peak distance uses. February's leap day is
/// irrelevant here: no shower peaks near it.
const MONTH_START: [i32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];

/// Approximate day of the year for `(month, day)`, non-leap.
fn doy(md: (u32, u32)) -> i32 {
    MONTH_START.get(md.0.saturating_sub(1) as usize).copied().unwrap_or(0) + md.1 as i32
}

/// Is `md` inside a window that may wrap the new year?
fn in_window(md: (u32, u32), start: (u32, u32), end: (u32, u32)) -> bool {
    if start <= end { md >= start && md <= end } else { md >= start || md <= end }
}

/// Signed distance in days from `md` to `target`, folded into `[-182, 182]` so
/// the shorter way round the year wins.
fn days_between(md: (u32, u32), target: (u32, u32)) -> i32 {
    let mut diff = doy(target) - doy(md);
    if diff > 182 {
        diff -= 365;
    } else if diff < -182 {
        diff += 365;
    }
    diff
}

/// Place a radiant at `(ra_deg, dec_deg)` for an observer, as
/// `(altitude, azimuth)` in degrees, azimuth clockwise from north.
///
/// The radiant is effectively at infinite range, so the geocentric direction is
/// the one that matters — the topocentric correction is far below the daily
/// drift of the radiant itself and the width of a trail.
pub fn radiant_altaz(
    lat_deg: f64,
    lon_deg: f64,
    ra_deg: f64,
    dec_deg: f64,
    unix_s: i64,
) -> (f64, f64) {
    let rad = radiant_ecef(ra_deg, dec_deg, julian_day(unix_s as f64));
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    let up = vec3(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin());
    let east = vec3(-lo.sin(), lo.cos(), 0.0);
    let north = vec3(-la.sin() * lo.cos(), -la.sin() * lo.sin(), la.cos());
    let alt = rad.dot(up).clamp(-1.0, 1.0).asin().to_degrees();
    let az = rad.dot(east).atan2(rad.dot(north)).to_degrees();
    (alt, (az + 360.0) % 360.0)
}

/// The Earth-fixed unit vector of a direction at right ascension `ra_deg` and
/// declination `dec_deg`, at Julian date `jd`.
///
/// The same frame [`crate::ephem::subsolar_point`] works in: rotating the
/// celestial vector by −GMST about the pole. That the two agree is one of the
/// tests, because an off-by-one here would put every radiant on the wrong side
/// of the sky while still looking plausible.
fn radiant_ecef(ra_deg: f64, dec_deg: f64, jd: f64) -> Vec3 {
    let h = (ra_deg - gmst_deg(jd)).to_radians();
    let dec = dec_deg.to_radians();
    vec3(dec.cos() * h.cos(), dec.cos() * h.sin(), dec.sin())
}

/// The showers active at `unix_s`, with each radiant placed for the observer,
/// strongest first.
///
/// "Active" is the IMO's own window, not a fitted rate curve: outside the
/// window there is nothing, and inside it the rate is the peak figure the table
/// carries. The daily-rate shape is a refinement this does not claim.
pub fn active_at(lat_deg: f64, lon_deg: f64, unix_s: i64) -> Vec<ActiveShower> {
    let (_, mo, da, _, _, _) = utc_ymd_hms(unix_s);
    let md = (mo, da);
    let mut out: Vec<ActiveShower> = SHOWERS
        .iter()
        .filter(|s| in_window(md, s.start, s.end))
        .map(|s| {
            let (alt, az) = radiant_altaz(lat_deg, lon_deg, s.ra_deg, s.dec_deg, unix_s);
            ActiveShower {
                shower: s,
                alt_deg: alt,
                az_deg: az,
                days_to_peak: days_between(md, s.peak),
            }
        })
        .collect();
    // Strongest first: the peak rate is what decides whether a shower is worth
    // listening for, and the radiant altitude is shown beside it.
    out.sort_by_key(|a| std::cmp::Reverse(a.shower.zhr));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephem::{geodetic_to_body, obliquity_deg, subsolar_point, sun_geocentric};
    use sdroxide_types::ymd_hms_to_unix;

    fn at(y: i64, mo: u32, d: u32, h: u32) -> i64 {
        ymd_hms_to_unix(y, mo, d, h, 0, 0)
    }

    /// A window that wraps the new year must still contain its peak, or the
    /// Quadrantids would be the one shower never seen.
    #[test]
    fn every_window_contains_its_own_peak() {
        for s in SHOWERS {
            assert!(
                in_window(s.peak, s.start, s.end),
                "{}'s peak {:?} is outside {:?}..{:?}",
                s.code,
                s.peak,
                s.start,
                s.end
            );
        }
    }

    #[test]
    fn the_codes_and_names_are_unique() {
        for (i, a) in SHOWERS.iter().enumerate() {
            for b in &SHOWERS[i + 1..] {
                assert_ne!(a.code, b.code, "two showers share the code {}", a.code);
                assert_ne!(a.name, b.name, "two showers share the name {}", a.name);
            }
        }
    }

    /// A date inside the Perseids finds them; a date in February finds nothing
    /// as strong, and the Quadrantid window is a wrapped one rather than a
    /// month-long mistake.
    #[test]
    fn the_right_showers_are_active_on_a_given_day() {
        let codes =
            |t: i64| active_at(50.0, 0.0, t).into_iter().map(|a| a.shower.code).collect::<Vec<_>>();
        assert!(codes(at(2026, 8, 12, 22)).contains(&"PER"));
        // The Quadrantids run 28 Dec – 12 Jan: active on both sides of the
        // year, and not in high summer.
        assert!(codes(at(2026, 1, 3, 1)).contains(&"QUA"));
        assert!(codes(at(2026, 12, 30, 1)).contains(&"QUA"));
        assert!(!codes(at(2026, 7, 1, 1)).contains(&"QUA"));
        // The Daytime Arietids, the radio shower, peak in early June — and
        // their radiant is up in daylight, which is the point of them.
        let june = active_at(50.0, 0.0, at(2026, 6, 7, 10));
        let ari = june.iter().find(|a| a.shower.code == "ARI").expect("ARI active on 7 June");
        assert!(ari.at_peak() && ari.radiant_up(), "the Arietid radiant is up mid-morning");
        // The Geminids run to the 20th, the Northern Taurids into December.
        assert!(codes(at(2026, 12, 19, 1)).contains(&"GEM"));
        assert!(codes(at(2026, 12, 8, 1)).contains(&"NTA"));
        assert!(!codes(at(2026, 10, 15, 1)).contains(&"NTA"));
    }

    /// The pole sits at the latitude and due north, whatever the date — the
    /// simplest check that the frame is the right way up.
    #[test]
    fn the_pole_sits_at_the_latitude_and_due_north() {
        let (alt, az) = radiant_altaz(50.0, 0.0, 0.0, 90.0, at(2026, 3, 20, 12));
        assert!((alt - 50.0).abs() < 1e-6, "pole altitude {alt}");
        assert!(az.abs() < 1e-6 || (az - 360.0).abs() < 1e-6, "pole azimuth {az}");
    }

    /// The Earth-fixed radiant frame has to be the one the subsolar point is
    /// expressed in, or the shower circles are misplaced without ever looking
    /// obviously wrong. The Sun's own right ascension and declination are
    /// recovered from its ecliptic longitude, which is what `sun_geocentric`
    /// publishes.
    #[test]
    fn the_radiant_frame_agrees_with_the_subsolar_point() {
        for t in [at(2026, 1, 1, 0), at(2026, 6, 1, 6), at(2026, 9, 15, 18)] {
            let jd = julian_day(t as f64);
            let (lambda, _) = sun_geocentric(jd);
            let eps = obliquity_deg(jd).to_radians();
            let (l, e) = (lambda.to_radians(), eps);
            let ra = (l.sin() * e.cos()).atan2(l.cos()).to_degrees();
            let dec = (l.sin() * e.sin()).clamp(-1.0, 1.0).asin().to_degrees();

            let rad = radiant_ecef(ra, dec, jd);
            let (sub_lat, sub_lon) = subsolar_point(jd);
            let sun = geodetic_to_body(sub_lat, sub_lon);
            assert!((rad - sun).len() < 1e-6, "radiant frame disagrees at {t}: {rad:?} vs {sun:?}");
        }
    }

    /// The Perseid radiant rises and sets at a latitude where it is not
    /// circumpolar — which is the whole "is it worth listening now" question,
    /// so it is worth pinning.
    #[test]
    fn the_perseid_radiant_rises_and_sets_over_a_day() {
        let base = at(2026, 8, 12, 0);
        let mut hi = f64::MIN;
        let mut lo = f64::MAX;
        for k in 0..(24 * 12) {
            let (alt, _) = radiant_altaz(30.0, 0.0, 48.0, 58.0, base + k * 300);
            hi = hi.max(alt);
            lo = lo.min(alt);
        }
        // Dec 58 at lat 30 culminates at 90 − (58 − 30) = 62°.
        assert!(hi > 55.0, "the radiant never rose: peak {hi}");
        assert!(lo < 0.0, "the radiant never set: floor {lo}");
    }

    /// The peak distance reads the short way round the year.
    #[test]
    fn days_to_the_peak_fold_across_the_new_year() {
        // 31 Dec is three days before the 3 Jan peak.
        assert_eq!(days_between((12, 31), (1, 3)), 3);
        // 1 Jan is two days after it.
        assert_eq!(days_between((1, 1), (1, 3)), 2);
        // 14 Dec is the Geminids' peak day.
        assert_eq!(days_between((12, 14), (12, 14)), 0);
    }
}
