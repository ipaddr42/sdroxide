//! The auroral oval: where it is now, how much power is going into it, and
//! whether it is about to get better.
//!
//! Three NOAA SWPC products, none of which needs an API key:
//!
//! * **OVATION** — a 1°×1° grid of the probability of seeing aurora, issued
//!   every five minutes and valid about forty minutes ahead. This is the oval
//!   itself, and it is what wraps the globe in the 3D view.
//! * **Hemispheric power** — the gigawatts being deposited in each hemisphere,
//!   which is the single number that says how big the event is.
//! * **The planetary K forecast** — three days of predicted Kp, which is how
//!   far in advance an operator can know a storm is coming.
//!
//! The grid is the only large payload in this crate after the solar imagery
//! (about 900 kB), so it is fetched on a much slower cadence than it is issued;
//! [`AuroraOval::forecast_unix`] says what the picture is actually valid for
//! rather than implying it is instantaneous.

use serde::{Deserialize, Serialize};

use crate::timefmt;

pub const OVATION_URL: &str = "https://services.swpc.noaa.gov/json/ovation_aurora_latest.json";
pub const HEMI_POWER_URL: &str =
    "https://services.swpc.noaa.gov/text/aurora-nowcast-hemi-power.txt";
pub const KP_FORECAST_URL: &str =
    "https://services.swpc.noaa.gov/products/noaa-planetary-k-index-forecast.json";

/// Grid width: one cell per degree of longitude.
pub const GRID_W: usize = 360;
/// Grid height: one cell per degree of latitude, poles included.
pub const GRID_H: usize = 181;

/// Below this the feed's own numerical noise dominates; a cell under it is not
/// aurora, it is a rounding artefact.
pub const NOISE_FLOOR_PCT: f64 = 1.0;
/// The contour drawn on the globe as "the edge". A tenth of a chance is about
/// where a dark-sky observer with a camera starts to have something to look at.
pub const EDGE_PCT: f64 = 10.0;

/// One OVATION issue: the probability of visible aurora over the whole globe.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuroraOval {
    /// When the inputs were observed.
    pub observed_unix: i64,
    /// What the grid is a forecast *for* — typically 40–50 minutes after the
    /// observation. This is the honest timestamp for the picture.
    pub forecast_unix: i64,
    /// Probability of visible aurora, percent, row-major from the north pole
    /// down and from 180° west eastward — the same equirectangular layout as
    /// the land mask, so it uploads as a texture the globe shader samples with
    /// the sphere's own UVs.
    pub grid: Vec<u8>,
}

impl AuroraOval {
    /// Cell index for a geodetic position, or `None` if the grid is unusable.
    fn cell(&self, lat: f64, lon: f64) -> Option<usize> {
        if self.grid.len() != GRID_W * GRID_H || !lat.is_finite() || !lon.is_finite() {
            return None;
        }
        let row = (90.0 - lat).round().clamp(0.0, (GRID_H - 1) as f64) as usize;
        let col = ((lon + 180.0).round() as i64).rem_euclid(GRID_W as i64) as usize;
        Some(row * GRID_W + col)
    }

    /// Probability of visible aurora at a point, percent.
    pub fn probability(&self, lat: f64, lon: f64) -> f64 {
        self.cell(lat, lon).map_or(0.0, |i| self.grid[i] as f64)
    }

    /// The strongest cell in one hemisphere, percent.
    pub fn peak(&self, north: bool) -> f64 {
        self.rows(north).flat_map(|(_, row)| row.iter().copied()).max().unwrap_or(0) as f64
    }

    /// The latitude closest to the equator at which the oval reaches
    /// `threshold` percent, signed. The "how far towards the equator can it be
    /// seen" number, read off the grid rather than guessed from Kp.
    pub fn equatorward_edge(&self, north: bool, threshold: f64) -> Option<f64> {
        let t = threshold.clamp(0.0, 255.0) as u8;
        // `rows` runs pole-first, so the last row holding anything is the one
        // nearest the equator.
        self.rows(north)
            .filter(|(_, row)| row.iter().any(|v| *v >= t))
            .map(|(lat, _)| lat)
            .next_back()
    }

    /// The equatorward edge as a function of longitude, for drawing the contour
    /// on the globe: one entry every `step_deg` degrees starting at 180° west,
    /// `None` where that meridian has no aurora at all.
    ///
    /// Each sample is the mean of a few neighbouring columns. A 1° grid of
    /// probabilities is ragged at its own edge, and an unsmoothed contour reads
    /// as noise rather than as the boundary it is meant to be.
    pub fn edge_profile(&self, north: bool, threshold: f64, step_deg: usize) -> Vec<Option<f64>> {
        let step = step_deg.clamp(1, 90);
        if self.grid.len() != GRID_W * GRID_H {
            return Vec::new();
        }
        let rows: Vec<(f64, &[u8])> = self.rows(north).collect();
        (0..GRID_W / step)
            .map(|k| {
                let col = k * step;
                rows.iter()
                    .filter(|(_, row)| smoothed(row, col) >= threshold)
                    .map(|(lat, _)| *lat)
                    .next_back()
            })
            .collect()
    }

    /// Whether there is anything worth drawing.
    pub fn is_empty(&self) -> bool {
        self.grid.iter().all(|v| (*v as f64) < NOISE_FLOOR_PCT)
    }

    /// Rows of one hemisphere with their latitudes, ordered from the pole
    /// towards the equator. Row 90 is the equator itself and belongs to
    /// neither hemisphere.
    fn rows(&self, north: bool) -> impl DoubleEndedIterator<Item = (f64, &[u8])> {
        // Rows 0..90 run north pole first, which is already the order wanted;
        // rows 91..181 run equator first, so they are walked backwards.
        let len = if self.grid.len() == GRID_W * GRID_H { GRID_H / 2 } else { 0 };
        (0..len).map(move |k| {
            let r = if north { k } else { GRID_H - 1 - k };
            (90.0 - r as f64, &self.grid[r * GRID_W..(r + 1) * GRID_W])
        })
    }
}

/// Mean of a cell and its immediate longitude neighbours, wrapping at the
/// antimeridian.
fn smoothed(row: &[u8], col: usize) -> f64 {
    const SPAN: i64 = 2;
    let mut sum = 0.0;
    for d in -SPAN..=SPAN {
        let c = (col as i64 + d).rem_euclid(GRID_W as i64) as usize;
        sum += row[c] as f64;
    }
    sum / (2 * SPAN + 1) as f64
}

#[derive(Deserialize)]
struct RawOvation {
    #[serde(rename = "Observation Time")]
    observation_time: Option<String>,
    #[serde(rename = "Forecast Time")]
    forecast_time: Option<String>,
    /// `[longitude east, latitude, probability percent]`, one per whole degree.
    coordinates: Vec<(i32, i32, i32)>,
}

/// Parse an OVATION issue. `None` for anything that is not a usable grid — an
/// empty or malformed one must leave the previous picture on screen rather than
/// blanking the oval.
pub fn parse_ovation(json: &str) -> Option<AuroraOval> {
    let raw: RawOvation =
        serde_json::from_str(json).map_err(|e| tracing::warn!("OVATION parse failed: {e}")).ok()?;
    if raw.coordinates.is_empty() {
        return None;
    }

    let mut grid = vec![0u8; GRID_W * GRID_H];
    for (lon, lat, value) in raw.coordinates {
        if !(-90..=90).contains(&lat) {
            continue;
        }
        // The feed publishes 0–360 east; the texture starts at 180° west.
        let col = (lon as i64 + 180).rem_euclid(GRID_W as i64) as usize;
        let row = (90 - lat) as usize;
        grid[row * GRID_W + col] = value.clamp(0, 100) as u8;
    }

    let observed_unix = raw.observation_time.as_deref().and_then(timefmt::parse_unix).unwrap_or(0);
    Some(AuroraOval {
        observed_unix,
        // A grid with no forecast stamp is valid for when it was observed;
        // inventing an offset would overstate how current it is.
        forecast_unix: raw
            .forecast_time
            .as_deref()
            .and_then(timefmt::parse_unix)
            .unwrap_or(observed_unix),
        grid,
    })
}

/// The power being deposited into each auroral zone, gigawatts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HemisphericPower {
    pub observed_unix: i64,
    pub forecast_unix: i64,
    pub north_gw: f64,
    pub south_gw: f64,
}

impl HemisphericPower {
    pub fn hemisphere(&self, north: bool) -> f64 {
        if north { self.north_gw } else { self.south_gw }
    }

    /// NOAA's Hemispheric Power Index, 1–10, from the gigawatt figure — the
    /// activity level their own auroral products are graded on.
    pub fn index(gw: f64) -> u8 {
        match gw {
            g if g >= 150.0 => 10,
            g if g >= 100.0 => 9,
            g if g >= 50.0 => 8,
            g if g >= 30.0 => 7,
            g if g >= 20.0 => 6,
            g if g >= 15.0 => 5,
            g if g >= 10.0 => 4,
            g if g >= 7.0 => 3,
            g if g >= 5.0 => 2,
            _ => 1,
        }
    }

    /// That index in words.
    pub fn words(gw: f64) -> &'static str {
        match HemisphericPower::index(gw) {
            1..=2 => "quiet",
            3..=4 => "low",
            5..=6 => "moderate",
            7 => "active",
            8 => "high",
            _ => "extreme",
        }
    }
}

/// Parse the tabular hemispheric-power file, taking the newest row.
///
/// ```text
/// # Observation        Forecast            North ...   South ...
/// 2026-07-25_14:55    2026-07-25_15:44      19      20
/// ```
///
/// The file holds a whole day at five-minute cadence; only the last line
/// describes the aurora as it is about to be.
pub fn parse_hemispheric_power(text: &str) -> Option<HemisphericPower> {
    text.lines()
        .rev()
        .filter(|l| !l.trim_start().starts_with('#') && !l.trim().is_empty())
        .find_map(|line| {
            let mut f = line.split_whitespace();
            let observed_unix = timefmt::parse_unix(f.next()?)?;
            let forecast_unix = timefmt::parse_unix(f.next()?)?;
            // "n/a" is what a gap in the data looks like; skip to an older row
            // rather than reporting zero gigawatts, which means something else.
            let north_gw: f64 = f.next()?.parse().ok()?;
            let south_gw: f64 = f.next()?.parse().ok()?;
            if !(0.0..=2000.0).contains(&north_gw) || !(0.0..=2000.0).contains(&south_gw) {
                return None;
            }
            Some(HemisphericPower { observed_unix, forecast_unix, north_gw, south_gw })
        })
}

/// One three-hour bin of the planetary K series.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KpPoint {
    /// Start of the bin.
    pub unix: i64,
    pub kp: f64,
    /// False for the observed and estimated part of the series, true for the
    /// forecast. The distinction is the whole point of the product.
    pub predicted: bool,
}

impl KpPoint {
    /// NOAA's G-scale storm level, 0–5.
    pub fn storm_level(&self) -> u8 {
        crate::indices::GeomagneticIndex { kp: self.kp, a_index: 0.0, observed_unix: self.unix }
            .storm_level()
    }
}

#[derive(Deserialize)]
struct RawKpForecast {
    time_tag: Option<String>,
    kp: Option<f64>,
    observed: Option<String>,
}

/// Parse the planetary K forecast: a week of observations followed by three
/// days of predictions, in three-hour bins.
pub fn parse_kp_forecast(json: &str) -> Vec<KpPoint> {
    let raw: Vec<RawKpForecast> = match serde_json::from_str(json) {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!("Kp forecast parse failed: {e}");
            return Vec::new();
        }
    };
    let mut out: Vec<KpPoint> = raw
        .into_iter()
        .filter_map(|r| {
            let kp = r.kp?;
            (0.0..=9.0).contains(&kp).then_some(KpPoint {
                unix: timefmt::parse_unix(r.time_tag.as_deref()?)?,
                kp,
                predicted: r.observed.as_deref() == Some("predicted"),
            })
        })
        .collect();
    out.sort_by_key(|p| p.unix);
    out
}

/// The forecast bins covering the next `horizon_s` seconds.
pub fn upcoming(points: &[KpPoint], now: i64, horizon_s: i64) -> impl Iterator<Item = &KpPoint> {
    // The bin containing `now` is still ahead of us for most of its three
    // hours, so it counts; anything wholly in the past does not.
    points.iter().filter(move |p| p.unix + 10_800 > now && p.unix <= now + horizon_s)
}

/// The observed bins covering the last `horizon_s` seconds, oldest first.
///
/// The other half of the series the same product carries: `upcoming` is what
/// is predicted, this is what was measured, and a trend chart needs both. Only
/// observed bins are returned — the forecast half is [`upcoming`]'s — so the
/// two never overlap in the middle.
pub fn recent(points: &[KpPoint], now: i64, horizon_s: i64) -> impl Iterator<Item = &KpPoint> {
    // The bin containing `now` has not finished, so it belongs to the forecast
    // half ([`upcoming`]); requiring the bin to have *ended* is what keeps the
    // two from both showing it.
    points.iter().filter(move |p| {
        !p.predicted && p.unix + 10_800 <= now && p.unix + 10_800 > now - horizon_s
    })
}

/// The worst bin in the next `horizon_s` seconds.
pub fn peak_forecast(points: &[KpPoint], now: i64, horizon_s: i64) -> Option<&KpPoint> {
    upcoming(points, now, horizon_s).max_by(|a, b| a.kp.total_cmp(&b.kp))
}

/// Roughly how far towards the equator the aurora can be seen at a given Kp, as
/// a **geomagnetic** latitude.
///
/// This is a straight-line fit to SWPC's published viewline table (66.5° at
/// Kp 0 falling about 2° per unit of Kp), and it is a rule of thumb: it says
/// nothing about cloud, moonlight or how dark the sky is, and geomagnetic
/// latitude is several degrees from geographic at most longitudes. The oval
/// drawn on the globe comes from the OVATION grid instead, which needs none of
/// those caveats.
pub fn viewline_geomagnetic_lat(kp: f64) -> f64 {
    (66.5 - 2.05 * kp.clamp(0.0, 9.0)).clamp(48.0, 66.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    const OVATION: &str = include_str!("../tests/fixtures/ovation.json");
    const HEMI: &str = include_str!("../tests/fixtures/hemi_power.txt");
    const KPF: &str = include_str!("../tests/fixtures/kp_forecast.json");

    /// The fixture is a real issue trimmed to the 110–139° east band, which is
    /// where the oval was strongest when it was taken.
    #[test]
    fn parses_a_real_ovation_issue() {
        let o = parse_ovation(OVATION).expect("fixture must parse");
        assert_eq!(o.grid.len(), GRID_W * GRID_H);
        assert_eq!(o.observed_unix, 1_784_991_060); // 2026-07-25T14:51:00Z
        // The forecast is for later than the observation, which is what makes
        // it a forecast.
        assert!(o.forecast_unix > o.observed_unix);
        assert!(o.forecast_unix - o.observed_unix < 2 * 3600);
        assert!(!o.is_empty());

        // Every value is a percentage.
        assert!(o.grid.iter().all(|v| *v <= 100));
        // ...and outside the trimmed band there is nothing at all, so a partial
        // feed leaves zeros rather than wrapping into the wrong longitudes.
        assert_eq!(o.probability(70.0, -80.0), 0.0);
    }

    /// The one thing that cannot be checked by inspection: that a cell ends up
    /// at the latitude and longitude it was published for.
    #[test]
    fn the_grid_lands_where_the_feed_put_it() {
        let o = parse_ovation(OVATION).expect("fixture must parse");
        // The fixture's strongest cell, verbatim from the file: 122°E, 53°S.
        assert_eq!(o.probability(-53.0, 122.0), 22.0);
        assert_eq!(o.peak(false), 22.0);
        // ...and the northern band's strongest, at 110°E, 70°N.
        assert_eq!(o.probability(70.0, 110.0), 16.0);
        assert_eq!(o.peak(true), 16.0);

        // A degree away is a different cell, so this is not just a saturated
        // neighbourhood.
        assert!(o.probability(-30.0, 122.0) < 22.0);
        // Longitude wraps rather than clamping.
        assert_eq!(o.probability(-53.0, 122.0 - 360.0), 22.0);
    }

    /// The oval is a polar phenomenon: an edge in the tropics would mean the
    /// rows had been indexed from the wrong end.
    #[test]
    fn the_edges_sit_at_auroral_latitudes() {
        let o = parse_ovation(OVATION).expect("fixture must parse");
        let north = o.equatorward_edge(true, EDGE_PCT).expect("northern oval");
        let south = o.equatorward_edge(false, EDGE_PCT).expect("southern oval");
        assert_eq!(north, 67.0);
        assert_eq!(south, -49.0);

        // A lower threshold reaches further towards the equator, never less far.
        let loose = o.equatorward_edge(true, 3.0).expect("northern oval");
        assert!(loose <= north, "threshold 3 % gave {loose}, threshold 10 % gave {north}");
        // Nothing on the planet is at 100 %.
        assert_eq!(o.equatorward_edge(true, 100.0), None);
    }

    #[test]
    fn the_edge_contour_follows_the_band_it_was_trimmed_to() {
        let o = parse_ovation(OVATION).expect("fixture must parse");
        let profile = o.edge_profile(true, EDGE_PCT, 4);
        assert_eq!(profile.len(), GRID_W / 4);
        // Sample k covers longitude −180 + 4k, so the 110–139° band is samples
        // 72..80. Those have an edge; the empty ocean of zeros elsewhere has
        // none.
        assert!(profile[74].is_some(), "no edge at 116° east, where the fixture has aurora");
        assert!(profile[10].is_none(), "an edge at 140° west, where the fixture has no data");
        for (k, lat) in profile.iter().enumerate() {
            if let Some(lat) = lat {
                assert!((50.0..=90.0).contains(lat), "sample {k} put the edge at {lat}°");
            }
        }
    }

    #[test]
    fn an_unusable_grid_is_rejected_rather_than_blanking_the_oval() {
        assert_eq!(parse_ovation(""), None);
        assert_eq!(parse_ovation("[]"), None);
        assert_eq!(parse_ovation(r#"{"coordinates": []}"#), None);
        // A grid with no timestamps is still a grid.
        let o = parse_ovation(r#"{"coordinates": [[0, 60, 40]]}"#).expect("grid without times");
        assert_eq!(o.probability(60.0, 0.0), 40.0);
        assert_eq!(o.observed_unix, 0);
        assert_eq!(o.forecast_unix, 0);
        // Nonsense latitudes are dropped, not written outside the grid.
        let o = parse_ovation(r#"{"coordinates": [[0, 900, 40], [0, 60, 7]]}"#).unwrap();
        assert_eq!(o.grid.len(), GRID_W * GRID_H);
        assert_eq!(o.probability(60.0, 0.0), 7.0);
    }

    #[test]
    fn an_empty_oval_reports_itself_empty() {
        let o = parse_ovation(r#"{"coordinates": [[0, 60, 0]]}"#).unwrap();
        assert!(o.is_empty());
        assert_eq!(o.peak(true), 0.0);
        assert_eq!(o.equatorward_edge(true, EDGE_PCT), None);
        assert!(o.edge_profile(true, EDGE_PCT, 4).iter().all(|e| e.is_none()));
    }

    #[test]
    fn parses_the_real_hemispheric_power_file() {
        let p = parse_hemispheric_power(HEMI).expect("fixture must parse");
        // The last row of the fixture.
        assert_eq!(p.north_gw, 19.0);
        assert_eq!(p.south_gw, 21.0);
        assert_eq!(p.observed_unix, 1_784_991_600); // 2026-07-25T15:00Z
        assert_eq!(p.forecast_unix, 1_784_994_540); // 2026-07-25T15:49Z
        assert_eq!(p.hemisphere(true), 19.0);
        assert_eq!(p.hemisphere(false), 21.0);
    }

    #[test]
    fn a_gap_in_the_power_file_falls_back_to_the_last_good_row() {
        let text = "# header\n\
                    2026-07-25_14:55    2026-07-25_15:44      19      20\n\
                    2026-07-25_15:00    2026-07-25_15:49     n/a     n/a\n";
        let p = parse_hemispheric_power(text).expect("the older row is still usable");
        assert_eq!(p.north_gw, 19.0);
        assert_eq!(parse_hemispheric_power("# only a header\n"), None);
        assert_eq!(parse_hemispheric_power(""), None);
        assert_eq!(parse_hemispheric_power("garbage"), None);
    }

    #[test]
    fn the_power_index_matches_noaas_activity_table() {
        for (gw, level) in [
            (0.0, 1),
            (4.9, 1),
            (5.0, 2),
            (7.0, 3),
            (12.0, 4),
            (16.0, 5),
            (25.0, 6),
            (40.0, 7),
            (60.0, 8),
            (120.0, 9),
            (200.0, 10),
        ] {
            assert_eq!(HemisphericPower::index(gw), level, "{gw} GW");
        }
        assert_eq!(HemisphericPower::words(3.0), "quiet");
        assert_eq!(HemisphericPower::words(25.0), "moderate");
        assert_eq!(HemisphericPower::words(200.0), "extreme");
    }

    #[test]
    fn parses_the_real_kp_forecast_and_keeps_the_two_halves_apart() {
        let points = parse_kp_forecast(KPF);
        assert!(points.len() > 40, "only {} bins", points.len());
        assert!(points.windows(2).all(|w| w[0].unix <= w[1].unix), "bins are out of order");
        assert!(points.iter().any(|p| p.predicted), "no forecast bins");
        assert!(points.iter().any(|p| !p.predicted), "no observed bins");
        // Every prediction is later than every observation.
        let last_observed = points.iter().filter(|p| !p.predicted).map(|p| p.unix).max().unwrap();
        assert!(points.iter().filter(|p| p.predicted).all(|p| p.unix > last_observed));
        // Three-hour bins. The real series does drop one occasionally — this
        // fixture is missing 12:00 UT — so the spacing is a multiple of three
        // hours rather than exactly three.
        assert!(points.windows(2).all(|w| (w[1].unix - w[0].unix) % 10_800 == 0));
        assert!(points.windows(2).all(|w| w[1].unix > w[0].unix));
        assert!(points.iter().all(|p| (0.0..=9.0).contains(&p.kp)));
    }

    #[test]
    fn the_peak_forecast_looks_forward_only() {
        let points = [
            KpPoint { unix: 1_000_000, kp: 8.0, predicted: false },
            KpPoint { unix: 1_010_800, kp: 3.0, predicted: true },
            KpPoint { unix: 1_021_600, kp: 6.0, predicted: true },
            KpPoint { unix: 1_200_000, kp: 9.0, predicted: true },
        ];
        let now = 1_015_000;
        // The Kp 8 bin ended before `now`, so the worst still to come is 6.
        let peak = peak_forecast(&points, now, 24 * 3600).expect("something ahead");
        assert_eq!(peak.kp, 6.0);
        assert_eq!(peak.storm_level(), 2);
        // The bin containing `now` counts, so three bins are in range.
        assert_eq!(upcoming(&points, now, 24 * 3600).count(), 2);
        // Nothing ahead is not an error.
        assert_eq!(peak_forecast(&points, 2_000_000, 24 * 3600), None);
        assert_eq!(peak_forecast(&[], now, 24 * 3600), None);
    }

    #[test]
    fn recent_returns_observed_history_oldest_first() {
        let points = [
            KpPoint { unix: 1_000_000, kp: 2.0, predicted: false },
            KpPoint { unix: 1_010_800, kp: 3.0, predicted: false },
            KpPoint { unix: 1_021_600, kp: 4.0, predicted: false },
            KpPoint { unix: 1_032_400, kp: 5.0, predicted: true },
        ];
        let now = 1_025_000;
        let seen: Vec<i64> = recent(&points, now, 24 * 3600).map(|p| p.unix).collect();
        // The in-progress bin (1_021_600) belongs to `upcoming`, not here; the
        // predicted bin is never history.
        assert_eq!(seen, vec![1_000_000, 1_010_800]);
        // The two halves meet without overlapping: no bin in both.
        let ahead: Vec<i64> = upcoming(&points, now, 24 * 3600).map(|p| p.unix).collect();
        assert!(seen.iter().all(|u| !ahead.contains(u)), "a bin is in both halves");
    }

    #[test]
    fn kp_forecast_survives_a_broken_feed() {
        assert!(parse_kp_forecast("not json").is_empty());
        assert!(parse_kp_forecast("[]").is_empty());
        // Out-of-range and undated entries are dropped, not clamped in.
        let json = r#"[{"time_tag":"2026-07-26T00:00:00","kp":42.0,"observed":"predicted"},
                       {"kp":3.0,"observed":"predicted"},
                       {"time_tag":"2026-07-26T03:00:00","kp":3.0,"observed":"predicted"}]"#;
        let p = parse_kp_forecast(json);
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].kp, 3.0);
    }

    #[test]
    fn the_viewline_moves_towards_the_equator_as_kp_rises() {
        assert!((viewline_geomagnetic_lat(0.0) - 66.5).abs() < 0.01);
        assert!((viewline_geomagnetic_lat(5.0) - 56.25).abs() < 0.01);
        // Monotonic, and never absurd at either end.
        let mut prev = 90.0;
        for k in 0..=90 {
            let lat = viewline_geomagnetic_lat(k as f64 / 10.0);
            assert!(lat <= prev, "Kp {} moved the viewline poleward", k as f64 / 10.0);
            assert!((48.0..=66.5).contains(&lat));
            prev = lat;
        }
    }
}
