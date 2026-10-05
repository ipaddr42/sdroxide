//! Positions of the Sun, Earth and Moon, Earth's rotation, and the orientation
//! of the Sun's rotation axis.
//!
//! Algorithms are Meeus, *Astronomical Algorithms* (2nd ed.) — the low-accuracy
//! solar theory (ch. 25), the abridged ELP 2000-82 lunar series (ch. 47), mean
//! sidereal time (ch. 12), mean obliquity (ch. 22) and the solar-disk elements
//! (ch. 29). Accuracy is far finer than a view that exaggerates body radii by
//! 20× can express — except for the Moon, whose full ch. 47 precision the
//! eclipse shadow genuinely uses; see `accuracy` on each function.
//!
//! # Frame
//!
//! Heliocentric ecliptic **of date**, right-handed: +X toward the vernal
//! equinox, +Z toward ecliptic north. Distances are in gigametres (10⁶ km), so
//! 1 AU ≈ 149.6 and the Earth's radius ≈ 0.0064 — the whole scene fits in
//! `[1e-3, 1e3]`, which stays exact in the f32 the GPU wants.
//!
//! Coordinates "of date" rather than J2000 because the low-accuracy solar
//! theory produces them naturally; precession is 0.014°/yr, invisible here, and
//! skipping the reduction removes a class of sign errors.

use crate::vec3::{Basis, Vec3, vec3};

/// Astronomical unit, gigametres.
pub const AU: f64 = 149.597_870_7;
/// Solar radius, gigametres.
pub const SUN_R: f64 = 0.695_700;
/// Earth mean radius, gigametres.
pub const EARTH_R: f64 = 0.006_371;
/// Moon mean radius, gigametres.
pub const MOON_R: f64 = 0.001_737_4;

/// Inclination of the Sun's equator to the ecliptic (Carrington).
const SUN_INCL_DEG: f64 = 7.25;

/// Sign of Stonyhurst longitude along `SunFrame::y`.
///
/// Stonyhurst longitude is **West-positive**, and the sub-Earth meridian drifts
/// slower than the Sun rotates, so a feature's longitude grows with time. With
/// `y = z × x` and prograde rotation about `+z`, that growth is along `+y`.
/// Verified by `stonyhurst_longitude_drifts_west` below, which propagates a
/// feature a day forward and checks the increase equals the synodic rate.
const STONYHURST_WEST_SIGN: f64 = 1.0;

/// Julian Day from a Unix timestamp (both UTC; the ~70 s of TT−UTC is well
/// below every tolerance here).
pub fn julian_day(unix_s: f64) -> f64 {
    unix_s / 86_400.0 + 2_440_587.5
}

/// Unix timestamp from a Julian Day — the inverse of [`julian_day`], for the
/// places that solve for a date (a comet's next perihelion) and then have to
/// print it.
pub fn unix_from_julian_day(jd: f64) -> f64 {
    (jd - 2_440_587.5) * 86_400.0
}

/// Julian centuries since J2000.0.
pub fn centuries(jd: f64) -> f64 {
    (jd - 2_451_545.0) / 36_525.0
}

pub fn wrap360(d: f64) -> f64 {
    let r = d % 360.0;
    if r < 0.0 { r + 360.0 } else { r }
}

/// Wrap to `(-180, 180]`.
pub fn wrap180(d: f64) -> f64 {
    let r = wrap360(d);
    if r > 180.0 { r - 360.0 } else { r }
}

/// Mean obliquity of the ecliptic, degrees (Meeus 22.2). Accuracy 0.01″ over
/// ±2000 years of J2000.
pub fn obliquity_deg(jd: f64) -> f64 {
    let t = centuries(jd);
    23.439_291_111_1 - 0.013_004_166_7 * t - 1.638_9e-7 * t * t + 5.036_1e-7 * t * t * t
}

/// Greenwich *mean* sidereal time, degrees (Meeus 12.4).
///
/// Mean rather than apparent: the equation of the equinoxes is at most 1.1″,
/// which is 34 m at the equator — far below the resolution of the land mask.
pub fn gmst_deg(jd: f64) -> f64 {
    let t = centuries(jd);
    wrap360(
        280.460_618_37 + 360.985_647_366_29 * (jd - 2_451_545.0) + 0.000_387_933 * t * t
            - t * t * t / 38_710_000.0,
    )
}

/// Apparent geocentric longitude of the Sun (degrees) and the Earth–Sun
/// distance (AU). Meeus ch. 25 "low accuracy": ±0.01° in longitude.
pub fn sun_geocentric(jd: f64) -> (f64, f64) {
    let t = centuries(jd);
    // Geometric mean longitude and mean anomaly.
    let l0 = 280.466_46 + 36_000.769_83 * t + 0.000_303_2 * t * t;
    let m = (357.529_11 + 35_999.050_29 * t - 0.000_153_7 * t * t).to_radians();
    let e = 0.016_708_634 - 0.000_042_037 * t - 0.000_000_126_7 * t * t;
    // Equation of the centre.
    let c = (1.914_602 - 0.004_817 * t - 0.000_014 * t * t) * m.sin()
        + (0.019_993 - 0.000_101 * t) * (2.0 * m).sin()
        + 0.000_289 * (3.0 * m).sin();
    let true_lon = l0 + c;
    let true_anom = (m.to_degrees() + c).to_radians();
    let r = 1.000_001_018 * (1.0 - e * e) / (1.0 + e * true_anom.cos());
    // Reduce to the apparent longitude: aberration plus the longitude term of
    // nutation, both folded into Meeus's two-term correction.
    let omega = (125.04 - 1934.136 * t).to_radians();
    let lambda = true_lon - 0.005_69 - 0.004_78 * omega.sin();
    (wrap360(lambda), r)
}

/// Heliocentric position of the Earth, gigametres.
pub fn earth_heliocentric(jd: f64) -> Vec3 {
    let (lambda, r) = sun_geocentric(jd);
    // The Earth sits opposite the Sun's geocentric direction. The Earth's
    // ecliptic latitude as seen from the Sun is under 1″, so it is dropped.
    Vec3::from_lon_lat_deg(lambda + 180.0, 0.0) * (r * AU)
}

/// ΔT = TT − UT, seconds: the drift of uniform (ephemeris) time against the
/// Earth's rotation. The 2005–2050 polynomial from Espenak & Meeus, *Five
/// Millennium Canon of Solar Eclipses*; extrapolated it is a minute or two
/// wrong by ±2100, which at the Moon's 0.55″/s is still far inside the lunar
/// series' own truncation.
fn delta_t_s(jd: f64) -> f64 {
    let t = (jd - 2_451_545.0) / 365.25;
    62.92 + 0.322_17 * t + 0.005_589 * t * t
}

/// Meeus table 47.A: multiples of (D, M, M′, F), then the sine coefficient of
/// the longitude series (10⁻⁶ degree) and the cosine coefficient of the
/// distance series (10⁻³ km).
const MOON_LON_DIST: &[(i32, i32, i32, i32, f64, f64)] = &[
    (0, 0, 1, 0, 6_288_774.0, -20_905_355.0),
    (2, 0, -1, 0, 1_274_027.0, -3_699_111.0),
    (2, 0, 0, 0, 658_314.0, -2_955_968.0),
    (0, 0, 2, 0, 213_618.0, -569_925.0),
    (0, 1, 0, 0, -185_116.0, 48_888.0),
    (0, 0, 0, 2, -114_332.0, -3_149.0),
    (2, 0, -2, 0, 58_793.0, 246_158.0),
    (2, -1, -1, 0, 57_066.0, -152_138.0),
    (2, 0, 1, 0, 53_322.0, -170_733.0),
    (2, -1, 0, 0, 45_758.0, -204_586.0),
    (0, 1, -1, 0, -40_923.0, -129_620.0),
    (1, 0, 0, 0, -34_720.0, 108_743.0),
    (0, 1, 1, 0, -30_383.0, 104_755.0),
    (2, 0, 0, -2, 15_327.0, 10_321.0),
    (0, 0, 1, 2, -12_528.0, 0.0),
    (0, 0, 1, -2, 10_980.0, 79_661.0),
    (4, 0, -1, 0, 10_675.0, -34_782.0),
    (0, 0, 3, 0, 10_034.0, -23_210.0),
    (4, 0, -2, 0, 8_548.0, -21_636.0),
    (2, 1, -1, 0, -7_888.0, 24_208.0),
    (2, 1, 0, 0, -6_766.0, 30_824.0),
    (1, 0, -1, 0, -5_163.0, -8_379.0),
    (1, 1, 0, 0, 4_987.0, -16_675.0),
    (2, -1, 1, 0, 4_036.0, -12_831.0),
    (2, 0, 2, 0, 3_994.0, -10_445.0),
    (4, 0, 0, 0, 3_861.0, -11_650.0),
    (2, 0, -3, 0, 3_665.0, 14_403.0),
    (0, 1, -2, 0, -2_689.0, -7_003.0),
    (2, 0, -1, 2, -2_602.0, 0.0),
    (2, -1, -2, 0, 2_390.0, 10_056.0),
    (1, 0, 1, 0, -2_348.0, 6_322.0),
    (2, -2, 0, 0, 2_236.0, -9_884.0),
    (0, 1, 2, 0, -2_120.0, 5_751.0),
    (0, 2, 0, 0, -2_069.0, 0.0),
    (2, -2, -1, 0, 2_048.0, -4_950.0),
    (2, 0, 1, -2, -1_773.0, 4_130.0),
    (2, 0, 0, 2, -1_595.0, 0.0),
    (4, -1, -1, 0, 1_215.0, -3_958.0),
    (0, 0, 2, 2, -1_110.0, 0.0),
    (3, 0, -1, 0, -892.0, 3_258.0),
    (2, 1, 1, 0, -810.0, 2_616.0),
    (4, -1, -2, 0, 759.0, -1_897.0),
    (0, 2, -1, 0, -713.0, -2_117.0),
    (2, 2, -1, 0, -700.0, 2_354.0),
    (2, 1, -2, 0, 691.0, 0.0),
    (2, -1, 0, -2, 596.0, 0.0),
    (4, 0, 1, 0, 549.0, -1_423.0),
    (0, 0, 4, 0, 537.0, -1_117.0),
    (4, -1, 0, 0, 520.0, -1_571.0),
    (1, 0, -2, 0, -487.0, -1_739.0),
    (2, 1, 0, -2, -399.0, 0.0),
    (0, 0, 2, -2, -381.0, -4_421.0),
    (1, 1, 1, 0, 351.0, 0.0),
    (3, 0, -2, 0, -340.0, 0.0),
    (4, 0, -3, 0, 330.0, 0.0),
    (2, -1, 2, 0, 327.0, 0.0),
    (0, 2, 1, 0, -323.0, 1_165.0),
    (1, 1, -1, 0, 299.0, 0.0),
    (2, 0, 3, 0, 294.0, 0.0),
    (2, 0, -1, -2, 0.0, 8_752.0),
];

/// Meeus table 47.B: multiples of (D, M, M′, F) and the sine coefficient of
/// the latitude series (10⁻⁶ degree).
const MOON_LAT: &[(i32, i32, i32, i32, f64)] = &[
    (0, 0, 0, 1, 5_128_122.0),
    (0, 0, 1, 1, 280_602.0),
    (0, 0, 1, -1, 277_693.0),
    (2, 0, 0, -1, 173_237.0),
    (2, 0, -1, 1, 55_413.0),
    (2, 0, -1, -1, 46_271.0),
    (2, 0, 0, 1, 32_573.0),
    (0, 0, 2, 1, 17_198.0),
    (2, 0, 1, -1, 9_266.0),
    (0, 0, 2, -1, 8_822.0),
    (2, -1, 0, -1, 8_216.0),
    (2, 0, -2, -1, 4_324.0),
    (2, 0, 1, 1, 4_200.0),
    (2, 1, 0, -1, -3_359.0),
    (2, -1, -1, 1, 2_463.0),
    (2, -1, 0, 1, 2_211.0),
    (2, -1, -1, -1, 2_065.0),
    (0, 1, -1, -1, -1_870.0),
    (4, 0, -1, -1, 1_828.0),
    (0, 1, 0, 1, -1_794.0),
    (0, 0, 0, 3, -1_749.0),
    (0, 1, -1, 1, -1_565.0),
    (1, 0, 0, 1, -1_491.0),
    (0, 1, 1, 1, -1_475.0),
    (0, 1, 1, -1, -1_410.0),
    (0, 1, 0, -1, -1_344.0),
    (1, 0, 0, -1, -1_335.0),
    (0, 0, 3, 1, 1_107.0),
    (4, 0, 0, -1, 1_021.0),
    (4, 0, -1, 1, 833.0),
    (0, 0, 1, -3, 777.0),
    (4, 0, -2, 1, 671.0),
    (2, 0, 0, -3, 607.0),
    (2, 0, 2, -1, 596.0),
    (2, -1, 1, -1, 491.0),
    (2, 0, -2, 1, -451.0),
    (0, 0, 3, -1, 439.0),
    (2, 0, 2, 1, 422.0),
    (2, 0, -3, -1, 421.0),
    (2, 1, -1, 1, -366.0),
    (2, 1, 0, 1, -351.0),
    (4, 0, 0, 1, 331.0),
    (2, -1, 1, 1, 315.0),
    (2, -2, 0, -1, 302.0),
    (0, 0, 1, 3, -283.0),
    (2, 1, 1, -1, -229.0),
    (1, 1, 0, -1, 223.0),
    (1, 1, 0, 1, 223.0),
    (0, 1, -2, -1, -220.0),
    (2, 1, -1, -1, -220.0),
    (1, 0, 1, 1, -185.0),
    (2, -1, -2, -1, 181.0),
    (0, 1, 2, 1, -177.0),
    (4, 0, -2, -1, 176.0),
    (4, -1, -1, -1, 166.0),
    (1, 0, 1, -1, -164.0),
    (4, 0, 1, -1, 132.0),
    (1, 0, -1, -1, -119.0),
    (4, -1, 0, -1, 115.0),
    (2, -2, 0, 1, 107.0),
];

/// Geocentric ecliptic longitude and latitude of the Moon (degrees) and its
/// distance (gigametres).
///
/// The abridged ELP 2000-82 series (Meeus ch. 47, all sixty terms of tables
/// 47.A and 47.B): ±10″ in longitude, ±4″ in latitude, a few kilometres in
/// distance. The eclipse shadow the 3D view casts on the Earth is where that
/// precision is spent — with the Almanac's ±0.3° short series that stood here
/// before, the umbra landed a thousand kilometres along its track from where
/// the eclipse actually is.
///
/// The argument is UT, as [`julian_day`] yields. The Moon is the one body
/// here fast enough (0.55″/s) for the ~70 s of TT − UT to matter against its
/// own series' accuracy, so that conversion happens inside.
pub fn moon_geocentric(jd: f64) -> (f64, f64, f64) {
    let t = centuries(jd + delta_t_s(jd) / 86_400.0);
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t3 * t;

    // Fundamental arguments (Meeus 47.1–47.7), degrees: the Moon's mean
    // longitude, its mean elongation from the Sun, the Sun's and the Moon's
    // mean anomalies, and the Moon's argument of latitude.
    let lp = 218.316_447_7 + 481_267.881_234_21 * t - 0.001_578_6 * t2 + t3 / 538_841.0
        - t4 / 65_194_000.0;
    let d = 297.850_192_1 + 445_267.111_403_4 * t - 0.001_881_9 * t2 + t3 / 545_868.0
        - t4 / 113_065_000.0;
    let m = 357.529_109_2 + 35_999.050_290_9 * t - 0.000_153_6 * t2 + t3 / 24_490_000.0;
    let mp = 134.963_396_4 + 477_198.867_505_5 * t + 0.008_741_4 * t2 + t3 / 69_699.0
        - t4 / 14_712_000.0;
    let f = 93.272_095_0 + 483_202.017_523_3 * t - 0.003_653_9 * t2 - t3 / 3_526_000.0
        + t4 / 863_310_000.0;

    // Venus, Jupiter and the Earth's flattening, plus the shrinking of the
    // Earth's orbital eccentricity that scales every term carrying M.
    let a1 = (119.75 + 131.849 * t).to_radians();
    let a2 = (53.09 + 479_264.290 * t).to_radians();
    let a3 = (313.45 + 481_266.484 * t).to_radians();
    let e = 1.0 - 0.002_516 * t - 0.000_007_4 * t2;

    let (lpr, dr, mr, mpr, fr) =
        (lp.to_radians(), d.to_radians(), m.to_radians(), mp.to_radians(), f.to_radians());

    let mut sum_l = 3_958.0 * a1.sin() + 1_962.0 * (lpr - fr).sin() + 318.0 * a2.sin();
    let mut sum_r = 0.0;
    for &(td, tm, tmp, tf, sl, sr) in MOON_LON_DIST {
        let arg = dr * td as f64 + mr * tm as f64 + mpr * tmp as f64 + fr * tf as f64;
        let scale = e.powi(tm.abs());
        sum_l += sl * arg.sin() * scale;
        sum_r += sr * arg.cos() * scale;
    }
    let mut sum_b = -2_235.0 * lpr.sin()
        + 382.0 * a3.sin()
        + 175.0 * (a1 - fr).sin()
        + 175.0 * (a1 + fr).sin()
        + 127.0 * (lpr - mpr).sin()
        - 115.0 * (lpr + mpr).sin();
    for &(td, tm, tmp, tf, sb) in MOON_LAT {
        let arg = dr * td as f64 + mr * tm as f64 + mpr * tmp as f64 + fr * tf as f64;
        sum_b += sb * arg.sin() * e.powi(tm.abs());
    }

    // Reduce to the *apparent* longitude with the same one-term nutation the
    // solar theory folds in — Sun−Moon elongation, the whole eclipse question,
    // has to be formed from a consistent pair. (Nutation in longitude is a
    // rotation about the ecliptic pole, so β is untouched.)
    let omega = (125.04 - 1_934.136 * t).to_radians();
    let lambda = lp + sum_l * 1.0e-6 - 0.004_78 * omega.sin();
    (wrap360(lambda), sum_b * 1.0e-6, (385_000.56 + sum_r * 1.0e-3) / 1.0e6)
}

/// Heliocentric position of the Moon, gigametres.
pub fn moon_heliocentric(jd: f64) -> Vec3 {
    earth_heliocentric(jd) + moon_geocentric_vec(jd)
}

/// Earth→Moon vector in ecliptic coordinates, gigametres.
pub fn moon_geocentric_vec(jd: f64) -> Vec3 {
    let (lambda, beta, dist) = moon_geocentric(jd);
    Vec3::from_lon_lat_deg(lambda, beta) * dist
}

/// Longitude of the ascending node of the Sun's equator on the ecliptic,
/// degrees (Carrington, Meeus ch. 29).
fn sun_node_deg(jd: f64) -> f64 {
    73.666_667 + 1.395_833_3 * (jd - 2_396_758.0) / 36_525.0
}

/// Unit vector along the Sun's north rotational pole, in ecliptic coordinates.
///
/// The pole sits at ecliptic longitude `K − 90°`, latitude `90° − i`.
pub fn solar_north_ecliptic(jd: f64) -> Vec3 {
    let k = sun_node_deg(jd);
    Vec3::from_lon_lat_deg(k - 90.0, 90.0 - SUN_INCL_DEG)
}

/// The solar-disk elements as seen from Earth (Meeus ch. 29), degrees:
/// `(P, B0, L0)` — position angle of the rotation axis, heliographic latitude
/// of the disk centre, and its Carrington longitude.
pub fn solar_p_b0_l0(jd: f64) -> (f64, f64, f64) {
    let theta = wrap360((jd - 2_398_220.0) * 360.0 / 25.38);
    let i = SUN_INCL_DEG.to_radians();
    let k = sun_node_deg(jd);
    let (lambda, _) = sun_geocentric(jd);
    // Meeus uses the *geometric* longitude corrected for aberration only.
    let lambda_p = lambda - 0.005_69;
    let eps = obliquity_deg(jd).to_radians();

    let x = (-lambda_p.to_radians().cos() * eps.tan()).atan();
    let y = (-(lambda - k).to_radians().cos() * i.tan()).atan();
    let p = (x + y).to_degrees();

    let b0 = ((lambda - k).to_radians().sin() * i.sin()).asin().to_degrees();

    // η is the angle, measured prograde about the solar axis, from the equator's
    // ascending node on the ecliptic to the disk centre. Doing it with vectors
    // rather than `arctan(tan(λ−K)·cos I)` avoids the classic half-turn error:
    // the disk centre is the *Sun→Earth* direction, which is opposite the Sun's
    // geocentric longitude, and the naive arctangent lands 180° away.
    let node = Vec3::from_lon_lat_deg(k, 0.0);
    let north = solar_north_ecliptic(jd);
    let prograde = north.cross(node);
    let to_earth = Vec3::from_lon_lat_deg(lambda + 180.0, 0.0);
    let eta = to_earth.dot(prograde).atan2(to_earth.dot(node)).to_degrees();
    let l0 = wrap360(eta - theta);

    (p, b0, l0)
}

/// An orthonormal Stonyhurst basis for the Sun at a given instant.
///
/// `z` is solar north, `x` points at the sub-Earth point on the solar equator
/// (heliographic longitude 0), and `y = z × x` is heliographic **West**.
#[derive(Debug, Clone, Copy)]
pub struct SunFrame {
    pub basis: Basis,
    /// Unit vector from the Sun toward the Earth.
    pub to_earth: Vec3,
}

pub fn sun_frame(jd: f64) -> SunFrame {
    let z = solar_north_ecliptic(jd);
    let to_earth = earth_heliocentric(jd).normalize();
    // Project the Sun→Earth direction onto the solar equatorial plane: that is
    // the central meridian as seen from Earth.
    let x = (to_earth - z * z.dot(to_earth)).normalize();
    let y = z.cross(x);
    SunFrame { basis: Basis { x, y, z }, to_earth }
}

impl SunFrame {
    /// Unit direction of a heliographic (Stonyhurst) coordinate, in ecliptic
    /// coordinates. `lon_west_deg` is West-positive — the DONKI convention.
    pub fn direction(&self, lat_deg: f64, lon_west_deg: f64) -> Vec3 {
        let b = lat_deg.to_radians();
        let l = lon_west_deg.to_radians() * STONYHURST_WEST_SIGN;
        self.basis.x * (b.cos() * l.cos())
            + self.basis.y * (b.cos() * l.sin())
            + self.basis.z * b.sin()
    }

    /// Inverse of [`Self::direction`]: `(lat, lon_west)` in degrees.
    pub fn stonyhurst_of(&self, dir: Vec3) -> (f64, f64) {
        let v = self.basis.unapply(dir.normalize());
        let lat = v.z.clamp(-1.0, 1.0).asin().to_degrees();
        let lon = v.y.atan2(v.x).to_degrees() * STONYHURST_WEST_SIGN;
        (lat, lon)
    }

    /// Heliographic latitude of the disk centre, from the frame itself.
    pub fn b0_deg(&self) -> f64 {
        self.to_earth.dot(self.basis.z).clamp(-1.0, 1.0).asin().to_degrees()
    }
}

/// Sidereal solar rotation rate at a heliographic latitude, degrees/day
/// (Snodgrass & Ulrich 1990, magnetic-feature tracers).
pub fn sidereal_rotation_deg_per_day(lat_deg: f64) -> f64 {
    let s2 = lat_deg.to_radians().sin().powi(2);
    14.713 - 2.396 * s2 - 1.787 * s2 * s2
}

/// Mean rate at which the Earth's heliocentric longitude advances, degrees/day.
/// Subtract this from the sidereal rate to get the synodic (as-seen-from-Earth)
/// rate that Stonyhurst longitudes actually drift at.
pub const EARTH_MEAN_MOTION_DEG_PER_DAY: f64 = 0.985_647;

/// Orientation of the Earth: an orthonormal basis whose columns are the ECEF
/// axes expressed in ecliptic coordinates.
///
/// With this basis the mesh's body frame *is* ECEF, so the land-mask UVs, the
/// QTH marker and the terminator all share one coordinate system.
/// Rotate an equatorial-of-date vector into ecliptic-of-date coordinates.
///
/// A rotation by −ε about the X axis. Shared with the satellite tracker, whose
/// SGP4 output is in the equatorial TEME frame.
pub fn equatorial_to_ecliptic(v: Vec3, jd: f64) -> Vec3 {
    let eps = obliquity_deg(jd).to_radians();
    vec3(v.x, v.y * eps.cos() + v.z * eps.sin(), -v.y * eps.sin() + v.z * eps.cos())
}

pub fn earth_basis(jd: f64) -> Basis {
    let theta = gmst_deg(jd).to_radians();
    // ECEF → equatorial of date is a rotation by GMST about the polar axis;
    // equatorial → ecliptic is the obliquity rotation.
    let eq_to_ecl = |v: Vec3| equatorial_to_ecliptic(v, jd);
    Basis {
        x: eq_to_ecl(vec3(theta.cos(), theta.sin(), 0.0)),
        y: eq_to_ecl(vec3(-theta.sin(), theta.cos(), 0.0)),
        z: eq_to_ecl(vec3(0.0, 0.0, 1.0)),
    }
}

/// Orientation of the Moon: the mean Earth/polar-axis frame, as columns in
/// ecliptic coordinates.
///
/// The Moon is tidally locked, so its body frame is defined by where it is:
/// `x` points at the Earth (selenographic 0°, the centre of the near side),
/// `z` is the rotation axis, and `y` completes the pair, which puts
/// selenographic **east** — the Mare Crisium limb — along the direction of
/// orbital motion, exactly as it appears from the ground.
///
/// Cassini's laws put the axis within 1.54° of the ecliptic pole, so that is
/// what is used for `z`. Physical and optical libration (±8° in longitude,
/// ±7° in latitude) is not modelled: it rocks the near side by a few degrees
/// about a face that is otherwise always the same one.
pub fn moon_basis(jd: f64) -> Basis {
    let to_earth = (-moon_geocentric_vec(jd)).normalize();
    let pole = vec3(0.0, 0.0, 1.0);
    // Orthogonalise, so the frame stays exactly orthonormal as the Moon moves
    // through its 5° of ecliptic latitude.
    let z = (pole - to_earth * pole.dot(to_earth)).normalize();
    Basis { x: to_earth, y: z.cross(to_earth), z }
}

/// Unit vector of a geodetic latitude/longitude in the Earth's body (ECEF)
/// frame. Spherical: WGS-84 flattening is 0.3%, invisible at any exaggeration
/// this view offers.
pub fn geodetic_to_body(lat_deg: f64, lon_deg: f64) -> Vec3 {
    let (la, lo) = (lat_deg.to_radians(), lon_deg.to_radians());
    vec3(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin())
}

/// Geographic latitude/longitude of the point where the Sun is overhead.
pub fn subsolar_point(jd: f64) -> (f64, f64) {
    let basis = earth_basis(jd);
    // Earth → Sun in ecliptic coordinates, expressed in the ECEF frame.
    let to_sun = basis.unapply((-earth_heliocentric(jd)).normalize());
    (to_sun.z.clamp(-1.0, 1.0).asin().to_degrees(), to_sun.y.atan2(to_sun.x).to_degrees())
}

/// The Sun's elevation above the horizon at a place, in degrees.
///
/// Negative below. Geometric: no refraction, no solar radius, no terrain — the
/// half-degree or so those are worth is well inside what anything here asks of
/// it.
pub fn solar_elevation_deg(lat_deg: f64, lon_deg: f64, unix_s: f64) -> f64 {
    let (sub_lat, sub_lon) = subsolar_point(julian_day(unix_s));
    let here = geodetic_to_body(lat_deg, lon_deg);
    let sun = geodetic_to_body(sub_lat, sub_lon);
    // Both are unit vectors from the centre, so their dot product is the cosine
    // of the angle between them — and the Sun's elevation at `here` is ninety
    // degrees less that angle.
    90.0 - here.dot(sun).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Is the Sun up at this place?
///
/// The dividing line an HF band-conditions table means by "day" and "night".
/// Taken at the geometric horizon rather than at civil twilight: the D layer
/// that decides whether 80 m is a local band or a DX one follows the Sun
/// itself, and picking a twilight definition would be inventing precision the
/// underlying verdicts do not have.
pub fn is_daylight_at(lat_deg: f64, lon_deg: f64, unix_s: i64) -> bool {
    solar_elevation_deg(lat_deg, lon_deg, unix_s as f64) > 0.0
}

/// Solar elevation above which the flat map carries no night shade, in degrees.
/// A little above the horizon, so the shade sits on the shadow rather than
/// through the middle of the terminator.
const NIGHT_DAY_TOP_DEG: f64 = 2.0;
/// Solar elevation at which the shade reaches full strength, in degrees —
/// between the end of nautical twilight (−12°) and of astronomical (−18°), where
/// the sky is dark to the eye. The band between this and [`NIGHT_DAY_TOP_DEG`]
/// is the twilight the grey line is drawn in.
const NIGHT_FULL_DEG: f64 = -14.0;
/// Alpha of the darkest shade, as a fraction. Strong enough that the terminator
/// reads across land as well as sea, but not opaque: the coastlines, borders and
/// any propagation heat stay readable under it. This is what the texture
/// carries; a light map paints it weaker — see the UI's `MapPalette::night_max`.
pub const NIGHT_MAX_ALPHA: f32 = 0.78;
/// Night ink — a deep dusk blue rather than black, so the map is shaded rather
/// than erased.
const NIGHT_INK: [u8; 3] = [8, 12, 28];

/// How dark the grey-line overlay is at a solar elevation: `0.0` in daylight,
/// rising through twilight to `1.0` at full night.
///
/// Pure and monotonic, which is what the tests pin. The eye can check the
/// boundary that matters — the subsolar point is unshaded, its antipode is
/// not — without a graphics context.
pub fn night_shade(elev_deg: f64) -> f32 {
    if elev_deg >= NIGHT_DAY_TOP_DEG {
        0.0
    } else if elev_deg <= NIGHT_FULL_DEG {
        1.0
    } else {
        ((NIGHT_DAY_TOP_DEG - elev_deg) / (NIGHT_DAY_TOP_DEG - NIGHT_FULL_DEG)) as f32
    }
}

/// The grey-line overlay for an equirectangular world image: straight RGBA8,
/// row-major from the north pole, `width` cells across 360° of longitude and
/// `height` down 180° of latitude.
///
/// The same Sun the band-conditions table is taken at ([`solar_elevation_deg`])
/// and the same frame the flat maps project, so the shade and the "day"/"night"
/// a verdict calls a band cannot disagree. Pure arithmetic — no textures, no
/// I/O — so it compiles for the browser and can be unit-tested.
pub fn night_shade_rgba(width: usize, height: usize, unix_s: i64) -> Vec<u8> {
    let mut px = vec![0u8; width * height * 4];
    if width == 0 || height == 0 {
        return px;
    }
    // The subsolar point once, not once per cell: the elevation of every cell is
    // the angle to that same unit vector.
    let (sub_lat, sub_lon) = subsolar_point(julian_day(unix_s as f64));
    let sun = geodetic_to_body(sub_lat, sub_lon);
    let alpha_full = (NIGHT_MAX_ALPHA * 255.0).round() as u8;
    for y in 0..height {
        let lat = 90.0 - (y as f64 + 0.5) * 180.0 / height as f64;
        for x in 0..width {
            let lon = -180.0 + (x as f64 + 0.5) * 360.0 / width as f64;
            let here = geodetic_to_body(lat, lon);
            let elev = 90.0 - here.dot(sun).clamp(-1.0, 1.0).acos().to_degrees();
            let a = (night_shade(elev) * alpha_full as f32).round() as u8;
            let i = (y * width + x) * 4;
            px[i..i + 3].copy_from_slice(&NIGHT_INK);
            px[i + 3] = a;
        }
    }
    px
}

#[cfg(test)]
mod tests {
    use super::*;

    /// JD → the Unix seconds our public API takes.
    fn unix_of_jd(jd: f64) -> f64 {
        (jd - 2_440_587.5) * 86_400.0
    }

    #[test]
    fn julian_day_round_trips() {
        // 2000-01-01T12:00:00Z is J2000.0 exactly.
        assert!((julian_day(946_728_000.0) - 2_451_545.0).abs() < 1e-9);
        assert!((julian_day(0.0) - 2_440_587.5).abs() < 1e-9);
        let jd = 2_446_895.5;
        assert!((julian_day(unix_of_jd(jd)) - jd).abs() < 1e-9);
    }

    #[test]
    fn obliquity_matches_meeus_22a() {
        // 1987 April 10.0 TD → ε0 = 23°26′27.407″.
        let e = obliquity_deg(2_446_895.5);
        assert!((e - 23.440_946).abs() < 1e-5, "ε = {e}");
    }

    #[test]
    fn gmst_matches_meeus_12b() {
        // 1987 April 10, 0h UT → 13h10m46.3668s = 197.693195°.
        let g = gmst_deg(2_446_895.5);
        assert!((g - 197.693_195).abs() < 1e-4, "gmst = {g}");
    }

    #[test]
    fn gmst_advances_one_sidereal_turn_per_day() {
        let a = gmst_deg(2_451_545.0);
        let b = gmst_deg(2_451_546.0);
        // A sidereal day is ~3m56s short of a solar day → +0.9856°/day.
        assert!((wrap180(b - a) - 0.985_647).abs() < 1e-3, "{a} → {b}");
    }

    #[test]
    fn sun_matches_meeus_25a() {
        // 1992 October 13.0 TD, JD 2448908.5. The reference values are from
        // Meeus's high-accuracy example 25.b (λ 199.90895°, R 0.99760775 AU);
        // the low-accuracy theory used here is allowed to differ, and does so
        // by 0.0001° in longitude and 5e-5 AU (8000 km, i.e. 0.005%) in radius.
        let (lambda, r) = sun_geocentric(2_448_908.5);
        assert!((lambda - 199.908_95).abs() < 0.01, "λ = {lambda}");
        assert!((r - 0.997_607_75).abs() < 1e-4, "R = {r}");
    }

    #[test]
    fn earth_distance_stays_within_the_real_orbit() {
        // Perihelion 0.9833 AU, aphelion 1.0167 AU.
        let mut lo = f64::MAX;
        let mut hi = f64::MIN;
        for d in 0..366 {
            let r = earth_heliocentric(2_451_545.0 + d as f64).len() / AU;
            lo = lo.min(r);
            hi = hi.max(r);
        }
        assert!((0.9832..0.9836).contains(&lo), "perihelion {lo}");
        assert!((1.0165..1.0169).contains(&hi), "aphelion {hi}");
    }

    #[test]
    fn moon_matches_meeus_example_47a() {
        // 1992 April 12.0 TD → mean λ 133.162655°, β −3.229126°,
        // Δ 368409.7 km; apparent λ 133.167265° after the book's nutation of
        // +16.595″. The example is stated in dynamical time and the function
        // takes UT, so hand it the UT instant whose TT is the book's.
        let jd_td = 2_448_724.5;
        let (lambda, beta, dist) = moon_geocentric(jd_td - delta_t_s(jd_td) / 86_400.0);
        // 0.001° ≈ 3.6″: room for the one-term nutation standing in for the
        // book's full value, nothing more.
        assert!((wrap180(lambda - 133.167_265)).abs() < 0.001, "λ = {lambda}");
        assert!((beta + 3.229_126).abs() < 0.000_5, "β = {beta}");
        assert!((dist * 1.0e6 - 368_409.7).abs() < 1.0, "Δ = {} km", dist * 1.0e6);
    }

    #[test]
    fn moon_orbit_stays_physical() {
        // Perigee ≈ 356 400 km, apogee ≈ 406 700 km; |β| ≤ 5.3°; the mean
        // longitude rate is the sidereal month, 13.176°/day.
        let (mut lo, mut hi, mut max_beta) = (f64::MAX, f64::MIN, 0.0f64);
        for d in 0..800 {
            let (_, beta, dist) = moon_geocentric(2_451_545.0 + d as f64);
            lo = lo.min(dist * 1.0e6);
            hi = hi.max(dist * 1.0e6);
            max_beta = max_beta.max(beta.abs());
        }
        assert!((352_000.0..362_000.0).contains(&lo), "perigee {lo}");
        assert!((402_000.0..411_000.0).contains(&hi), "apogee {hi}");
        assert!((4.9..5.6).contains(&max_beta), "max |β| {max_beta}");

        // Mean motion over a century, so the ±0.3° periodic wobble at each
        // endpoint contributes under 0.00002°/day to the difference.
        const SIDEREAL_MONTH_DEG_PER_DAY: f64 = 13.176_358;
        let n = 36_525.0;
        let l0 = moon_geocentric(2_451_545.0).0;
        let l1 = moon_geocentric(2_451_545.0 + n).0;
        let whole = ((n * SIDEREAL_MONTH_DEG_PER_DAY - (l1 - l0)) / 360.0).round();
        let rate = ((l1 - l0) + 360.0 * whole) / n;
        // 5e-4 °/day, not tighter, for two honest reasons: each endpoint still
        // carries the ±8° of periodic terms (up to 4e-4 °/day across a single
        // century), and the longitude is measured from the equinox of date, so
        // its rate is the *tropical* one — precession's 4e-5 °/day above the
        // sidereal constant compared against here.
        assert!((rate - SIDEREAL_MONTH_DEG_PER_DAY).abs() < 5e-4, "mean motion {rate}");
    }

    #[test]
    fn solar_disk_elements_match_meeus_29a() {
        // 1992 October 13.0 TD → P = 26.27°, B0 = 5.99°, L0 = 238.63°.
        let (p, b0, l0) = solar_p_b0_l0(2_448_908.5);
        assert!((p - 26.27).abs() < 0.02, "P = {p}");
        assert!((b0 - 5.99).abs() < 0.02, "B0 = {b0}");
        assert!((wrap180(l0 - 238.63)).abs() < 0.05, "L0 = {l0}");
    }

    /// Independent ground truth for the Carrington longitude of the disk
    /// centre, which is the easiest quantity here to get 180° wrong.
    ///
    /// NOAA SWPC's solar-region summary publishes both the Stonyhurst and the
    /// Carrington longitude of every sunspot group, and `L_carrington =
    /// L0 + L_stonyhurst_west`. Two regions from the 2026-07-24 report:
    /// `S03E14` at Carrington 99 → L0 = 99 − (−14) = 113, and `N05W46` at
    /// Carrington 159 → L0 = 159 − 46 = 113. The report's positions are for
    /// 2400 UT, i.e. 2026-07-25T00:00Z.
    #[test]
    fn l0_matches_swpc_carrington_longitudes() {
        let jd = julian_day(1_784_937_600.0); // 2026-07-25T00:00:00Z
        let (_, _, l0) = solar_p_b0_l0(jd);
        assert!(wrap180(l0 - 113.0).abs() < 1.5, "L0 = {l0}");
    }

    #[test]
    fn l0_regresses_at_the_synodic_carrington_rate() {
        // The disk centre's Carrington longitude runs backwards as the Sun
        // turns under it: one synodic Carrington rotation is 27.2753 days.
        let jd = 2_460_500.0;
        let a = solar_p_b0_l0(jd).2;
        let b = solar_p_b0_l0(jd + 1.0).2;
        assert!((wrap180(b - a) + 360.0 / 27.2753).abs() < 0.1, "{a} → {b}");
    }

    /// The vector form of the solar axis and Meeus's independent B0 formula are
    /// derived differently; if the node longitude's sign were wrong, the annual
    /// B0 curve would invert and this would fail immediately.
    #[test]
    fn solar_axis_vector_agrees_with_meeus_b0() {
        for i in 0..24 {
            let jd = 2_460_000.0 + i as f64 * 15.2;
            let (_, b0_meeus, _) = solar_p_b0_l0(jd);
            let b0_vec = sun_frame(jd).b0_deg();
            assert!((b0_vec - b0_meeus).abs() < 0.05, "jd {jd}: {b0_vec} vs {b0_meeus}");
        }
    }

    #[test]
    fn b0_peaks_in_september_and_troughs_in_march() {
        // Earth crosses the Sun's equatorial plane in early June and early
        // December; B0 reaches +7.25° around 7 September.
        let mut best = (f64::MIN, 0.0);
        let mut worst = (f64::MAX, 0.0);
        // 2024-01-01T00:00Z .. one year.
        for d in 0..366 {
            let jd = 2_460_310.5 + d as f64;
            let b0 = sun_frame(jd).b0_deg();
            if b0 > best.0 {
                best = (b0, d as f64);
            }
            if b0 < worst.0 {
                worst = (b0, d as f64);
            }
        }
        assert!((best.0 - 7.25).abs() < 0.1, "max B0 {}", best.0);
        assert!((worst.0 + 7.25).abs() < 0.1, "min B0 {}", worst.0);
        // Day 250 of 2024 is 6 September; day 64 is 4 March.
        assert!((best.1 - 250.0).abs() < 6.0, "max B0 on day {}", best.1);
        assert!((worst.1 - 64.0).abs() < 6.0, "min B0 on day {}", worst.1);
    }

    /// The sign exam for [`STONYHURST_WEST_SIGN`]: rotate a feature prograde
    /// about the solar axis for a day, then re-measure it in the *new* frame.
    /// Its longitude must have grown by the synodic rate — growing toward West
    /// is what "West-positive" means.
    #[test]
    fn stonyhurst_longitude_drifts_west() {
        for lat in [-30.0, -10.0, 0.0, 15.0, 40.0] {
            let jd = 2_460_500.0;
            let f0 = sun_frame(jd);
            let start_lon = -20.0;
            let dir0 = f0.direction(lat, start_lon);

            // One day of prograde rotation about the solar north pole.
            let n = solar_north_ecliptic(jd);
            let ang = sidereal_rotation_deg_per_day(lat).to_radians();
            let dir1 = dir0 * ang.cos()
                + n.cross(dir0) * ang.sin()
                + n * (n.dot(dir0) * (1.0 - ang.cos()));

            let f1 = sun_frame(jd + 1.0);
            let (lat1, lon1) = f1.stonyhurst_of(dir1);
            let expected = sidereal_rotation_deg_per_day(lat) - EARTH_MEAN_MOTION_DEG_PER_DAY;
            assert!((lat1 - lat).abs() < 0.05, "latitude drifted: {lat} → {lat1}");
            assert!(
                (wrap180(lon1 - start_lon) - expected).abs() < 0.05,
                "lat {lat}: drift {} expected {expected}",
                wrap180(lon1 - start_lon)
            );
        }
    }

    #[test]
    fn stonyhurst_round_trips() {
        let f = sun_frame(2_460_500.0);
        for (lat, lon) in [(0.0, 0.0), (25.0, 60.0), (-12.0, -140.0), (60.0, 179.0)] {
            let (lat2, lon2) = f.stonyhurst_of(f.direction(lat, lon));
            assert!((lat2 - lat).abs() < 1e-9 && wrap180(lon2 - lon).abs() < 1e-9);
        }
        // Longitude 0 is the sub-Earth meridian by construction.
        let (lat0, lon0) = f.stonyhurst_of(f.to_earth);
        assert!(lon0.abs() < 1e-9, "sub-Earth longitude {lon0}");
        assert!((lat0 - f.b0_deg()).abs() < 1e-9);
    }

    #[test]
    fn earth_basis_is_orthonormal_and_polar() {
        let b = earth_basis(2_460_500.3);
        for v in [b.x, b.y, b.z] {
            assert!((v.len() - 1.0).abs() < 1e-12);
        }
        assert!(b.x.dot(b.y).abs() < 1e-12 && b.x.dot(b.z).abs() < 1e-12);
        // Right-handed: x × y = z.
        assert!((b.x.cross(b.y) - b.z).len() < 1e-12);
        // The north pole sits at ecliptic latitude 90° − ε ≈ 66.56°.
        let e = obliquity_deg(2_460_500.3);
        assert!((b.z.lat_deg() - (90.0 - e)).abs() < 1e-9, "pole at {}", b.z.lat_deg());
        // ...and at ecliptic longitude 90°.
        assert!((wrap180(b.z.lon_deg() - 90.0)).abs() < 1e-9);
    }

    /// Day and night are opposite sides of the planet, and the Sun is directly
    /// overhead at the subsolar point. Checked against the geometry rather than
    /// against a table, so this stays true whatever the ephemeris is refined to.
    #[test]
    fn the_sun_is_up_where_it_is_overhead_and_down_opposite() {
        for unix in [1_784_937_600i64, 1_772_000_000, 1_800_000_000, 1_760_500_000] {
            let (lat, lon) = subsolar_point(julian_day(unix as f64));
            let el = solar_elevation_deg(lat, lon, unix as f64);
            assert!((el - 90.0).abs() < 0.01, "overhead elevation was {el} at {unix}");
            assert!(is_daylight_at(lat, lon, unix));
            // The antipode is midnight.
            let (alat, alon) = (-lat, wrap180(lon + 180.0));
            let ael = solar_elevation_deg(alat, alon, unix as f64);
            assert!((ael + 90.0).abs() < 0.01, "antipodal elevation was {ael} at {unix}");
            assert!(!is_daylight_at(alat, alon, unix));
        }
    }

    /// The polar night is the sharpest case a day/night split has to get right.
    #[test]
    fn the_poles_follow_the_season_rather_than_the_clock() {
        // Northern midsummer and midwinter. Whatever the hour, the North Pole
        // is lit in June and dark in December.
        for h in [0i64, 6, 12, 18] {
            let june = 1_718_884_800 + h * 3600; // 2024-06-20
            let dec = 1_734_696_000 + h * 3600; // 2024-12-20
            assert!(is_daylight_at(89.5, 0.0, june), "north pole dark at midsummer +{h}h");
            assert!(!is_daylight_at(89.5, 0.0, dec), "north pole lit at midwinter +{h}h");
            assert!(!is_daylight_at(-89.5, 0.0, june), "south pole lit at its midwinter +{h}h");
            assert!(is_daylight_at(-89.5, 0.0, dec), "south pole dark at its midsummer +{h}h");
        }
    }

    #[test]
    fn subsolar_point_tracks_noon() {
        // At 12:00 UTC the Sun is overhead near the Greenwich meridian; the
        // equation of time keeps it within ±4° of it all year.
        for d in 0..365 {
            let jd = julian_day(1_704_110_400.0 + d as f64 * 86_400.0); // 2024-01-01T12:00Z
            let (lat, lon) = subsolar_point(jd);
            assert!(lon.abs() < 4.5, "day {d}: subsolar longitude {lon}");
            assert!(lat.abs() < 23.5, "day {d}: subsolar latitude {lat}");
        }
        // The declination extremes are the solstices.
        let june = subsolar_point(julian_day(1_718_884_800.0)).0; // 2024-06-20T12:00Z
        let dec = subsolar_point(julian_day(1_734_696_000.0)).0; // 2024-12-20T12:00Z
        assert!((june - 23.44).abs() < 0.1, "June solstice {june}");
        assert!((dec + 23.42).abs() < 0.15, "December solstice {dec}");
    }

    #[test]
    fn subsolar_longitude_sweeps_westward_with_the_clock() {
        // Six hours later the Sun is overhead 90° further west.
        let jd = julian_day(1_704_110_400.0);
        let a = subsolar_point(jd).1;
        let b = subsolar_point(jd + 0.25).1;
        assert!((wrap180(b - a) + 90.0).abs() < 0.5, "{a} → {b}");
    }

    /// The one thing everybody knows about the Moon: the same face, always.
    #[test]
    fn the_moon_keeps_one_face_towards_the_earth() {
        for d in 0..60 {
            let jd = 2_460_500.0 + d as f64 * 0.5;
            let b = moon_basis(jd);
            for v in [b.x, b.y, b.z] {
                assert!((v.len() - 1.0).abs() < 1e-12);
            }
            assert!(b.x.dot(b.y).abs() < 1e-12 && b.x.dot(b.z).abs() < 1e-12);
            assert!((b.x.cross(b.y) - b.z).len() < 1e-12, "left-handed lunar frame");

            // Selenographic (0°, 0°) is the sub-Earth point...
            let near_side = b.apply(geodetic_to_body(0.0, 0.0));
            let to_earth = (-moon_geocentric_vec(jd)).normalize();
            assert!(near_side.dot(to_earth) > 0.999_999, "the near side has turned away");
            // ...and the far side is the far side.
            let far = b.apply(geodetic_to_body(0.0, 180.0));
            assert!(far.dot(to_earth) < -0.999_999);
        }
    }

    /// Which way round the map goes, settled by something anyone can check
    /// with binoculars: a few days after new moon the crescent lights the
    /// **eastern** limb, so Mare Crisium (17°N 59°E) catches the Sun days
    /// before Oceanus Procellarum (18°N 57°W) does.
    ///
    /// This is the test that pins the sign of `y`. Mirror the frame and every
    /// lunar feature lands on the wrong limb while the Moon still, misleadingly,
    /// keeps one face towards the Earth.
    #[test]
    fn the_waxing_crescent_lights_the_eastern_limb() {
        // Elongation 45–75° east of the Sun: three or four days old, by which
        // point the terminator has moved past Crisium.
        let jd = (0..120)
            .map(|k| 2_460_500.0 + k as f64 * 0.5)
            .find(|jd| {
                let elong = wrap180(moon_geocentric(*jd).0 - sun_geocentric(*jd).0);
                (45.0..75.0).contains(&elong)
            })
            .expect("no waxing crescent in two months");

        let b = moon_basis(jd);
        let to_sun = (-(earth_heliocentric(jd) + moon_geocentric_vec(jd))).normalize();
        let lit = |lat: f64, lon: f64| b.apply(geodetic_to_body(lat, lon)).dot(to_sun);
        assert!(lit(17.0, 59.0) > 0.15, "Mare Crisium is dark on a waxing crescent");
        assert!(lit(18.0, -57.0) < -0.15, "Oceanus Procellarum is lit on a waxing crescent");
    }

    #[test]
    fn geodetic_to_body_is_ecef() {
        // 0°N 0°E is the +X axis; 0°N 90°E is +Y; the north pole is +Z.
        assert!((geodetic_to_body(0.0, 0.0) - vec3(1.0, 0.0, 0.0)).len() < 1e-12);
        assert!((geodetic_to_body(0.0, 90.0) - vec3(0.0, 1.0, 0.0)).len() < 1e-12);
        assert!((geodetic_to_body(90.0, 0.0) - vec3(0.0, 0.0, 1.0)).len() < 1e-12);
    }

    /// The shade is a ramp, not a step: daylight at the top, night at the
    /// bottom, and monotonic all the way with nothing out of range.
    #[test]
    fn the_shade_ramps_from_day_to_night() {
        assert_eq!(night_shade(45.0), 0.0);
        assert_eq!(night_shade(-45.0), 1.0);
        let twilight = night_shade(0.0);
        assert!(twilight > 0.0 && twilight < 1.0, "the horizon is not a step: {twilight}");
        let mut prev = 1.0;
        for k in -90..=90 {
            let a = night_shade(k as f64);
            assert!((0.0..=1.0).contains(&a), "shade {a} out of range at {k}");
            // Shade only ever lightens as the Sun climbs; never darkens.
            assert!(a <= prev, "the shade darkened as the Sun rose at {k}: {prev} then {a}");
            prev = a;
        }
    }

    /// The picture has to agree with the ephemeris it was drawn from: the cell
    /// under the Sun is in daylight, and the one opposite it is in full night.
    #[test]
    fn the_subsolar_cell_is_lit_and_its_antipode_is_not() {
        let (w, h) = (360usize, 180usize);
        let unix = 1_800_000_000i64;
        let px = night_shade_rgba(w, h, unix);
        assert_eq!(px.len(), w * h * 4);
        let (sub_lat, sub_lon) = subsolar_point(julian_day(unix as f64));
        let alpha_at = |lat: f64, lon: f64| {
            let y = (((90.0 - lat) / 180.0) * h as f64) as usize;
            let x = ((((lon + 180.0) % 360.0 + 360.0) % 360.0) / 360.0 * w as f64) as usize;
            px[(y.min(h - 1) * w + x.min(w - 1)) * 4 + 3]
        };
        assert_eq!(alpha_at(sub_lat, sub_lon), 0, "the cell under the Sun is shaded");
        assert_eq!(
            alpha_at(-sub_lat, sub_lon + 180.0),
            (NIGHT_MAX_ALPHA * 255.0).round() as u8,
            "the antipode is not in full night"
        );
    }
}
