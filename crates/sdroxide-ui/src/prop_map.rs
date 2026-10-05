//! Turning a propagation field into pixels.
//!
//! The store itself is [`sdroxide_types::PropStore`] — it has to be, because
//! the server builds one too for the browser's solar tab. What is here is the
//! part that needs a graphics stack: resolving cells to colours, and holding
//! the egui texture the flat map paints.

use sdroxide_types::{Band, PropField};

pub use sdroxide_types::{PropSources, PropStore};

// ── Drawing it ──────────────────────────────────────────────────────────────

/// Which propagation picture to draw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PropMode {
    /// One band, through the blue → green → yellow → red ramp.
    PerBand,
    /// Every selected band at once, one hue each, summed. The view that answers
    /// "what are conditions like" without being configured first.
    #[default]
    AllBands,
}

/// A propagation field rendered to pixels.
///
/// The heat is a continuous field, so it is drawn as its own bilinear-filtered
/// image *beneath* the continents rather than by tinting the dot matrix. The
/// dots are how the map answers a categorical question — land or sea — and a
/// stippled field would read as a scatter of cells rather than as the smooth
/// thing it is. Bilinear interpolation of a Gaussian-splatted grid is what
/// turns 2.5° cells into soft regions with no visible edges, and it is also
/// what makes two overlapping bands mix continuously instead of one winning
/// per cell.
pub struct PropHeat {
    tex: Option<eframe::egui::TextureHandle>,
    /// What the texture was built from, so it is rebuilt only when it changed.
    built: Option<(u64, PropMode, u8, u32)>,
    /// Absolute path count the top of the ramp corresponds to, for the legend.
    /// Without it the colours are relative and the map lies by omission.
    pub peak_paths: f32,
    /// Bands actually present in the picture, for the legend's swatches.
    pub bands: Vec<Band>,
}

impl Default for PropHeat {
    fn default() -> Self {
        PropHeat { tex: None, built: None, peak_paths: 0.0, bands: Vec::new() }
    }
}

/// Alpha ceiling. A heat map is paint on the map, not a replacement for it: the
/// coastline has to stay readable underneath.
const MAX_ALPHA: f32 = 0.62;

impl PropHeat {
    /// Rebuild if anything it depends on has moved, and hand back the texture.
    ///
    /// `band_mask` selects which bands the all-bands view sums, one bit per
    /// index into [`Band::ALL`]; `band` is the one the per-band view draws.
    pub fn texture(
        &mut self,
        ctx: &eframe::egui::Context,
        field: &PropField,
        mode: PropMode,
        band: Band,
        band_mask: u32,
    ) -> Option<&eframe::egui::TextureHandle> {
        let band_ix = Band::ALL.iter().position(|b| *b == band).unwrap_or(0) as u8;
        let key = (field.generation, mode, band_ix, band_mask);
        if self.built != Some(key) || self.tex.is_none() {
            let (img, peak, bands) = Self::image(field, mode, band, band_mask);
            self.peak_paths = peak;
            self.bands = bands;
            match self.tex.as_mut() {
                // LINEAR is the whole point: it is what turns cells into shapes.
                Some(t) => t.set(img, eframe::egui::TextureOptions::LINEAR),
                None => {
                    self.tex = Some(ctx.load_texture(
                        "prop-heat",
                        img,
                        eframe::egui::TextureOptions::LINEAR,
                    ))
                }
            }
            self.built = Some(key);
        }
        self.tex.as_ref().filter(|_| self.peak_paths > 0.0)
    }

    /// Resolve the field to pixels.
    ///
    /// Returns the image, the absolute path count the ramp's top means, and the
    /// bands that contributed.
    pub fn image(
        field: &PropField,
        mode: PropMode,
        band: Band,
        band_mask: u32,
    ) -> (eframe::egui::ColorImage, f32, Vec<Band>) {
        use sdroxide_types::{PROP_GRID_H, PROP_GRID_W};
        let (rgba, peak, bands) = Self::rgba(field, mode, band, band_mask);
        let px: Vec<eframe::egui::Color32> = rgba
            .chunks_exact(4)
            .map(|c| eframe::egui::Color32::from_rgba_unmultiplied(c[0], c[1], c[2], c[3]))
            .collect();
        let img = eframe::egui::ColorImage {
            size: [PROP_GRID_W, PROP_GRID_H],
            pixels: px,
            source_size: eframe::egui::vec2(PROP_GRID_W as f32, PROP_GRID_H as f32),
        };
        (img, peak, bands)
    }

    /// Resolve the field to straight (non-premultiplied) RGBA8, row-major from
    /// the north pole.
    ///
    /// The single place a propagation cell becomes a colour. The flat map turns
    /// these bytes into an egui texture and the globe uploads them to wgpu
    /// unchanged, so the two views are not merely consistent — they are the
    /// same pixels. Nothing in the shader picks a colour, which is the only way
    /// to be sure the two cannot drift apart.
    ///
    /// Pure, so the colour rules are testable without a graphics context.
    pub fn rgba(
        field: &PropField,
        mode: PropMode,
        band: Band,
        band_mask: u32,
    ) -> (Vec<u8>, f32, Vec<Band>) {
        use sdroxide_types::{PROP_GRID_H, PROP_GRID_W};

        let bands: Vec<Band> = match mode {
            PropMode::PerBand => field.live_bands().into_iter().filter(|b| *b == band).collect(),
            PropMode::AllBands => field
                .live_bands()
                .into_iter()
                .filter(|b| {
                    Band::ALL.iter().position(|x| x == b).is_some_and(|i| band_mask & (1 << i) != 0)
                })
                .collect(),
        };
        // Normalised against the strongest cell in the picture, so a quiet band
        // still shows its structure. The caller shows `peak` beside the ramp;
        // without that the same colour would mean different things on different
        // evenings and the map would be lying by omission.
        let peak =
            bands.iter().filter_map(|b| field.plane(*b)).map(|p| p.peak()).fold(0.0f32, f32::max);

        let mut px = vec![0u8; PROP_GRID_W * PROP_GRID_H * 4];
        if peak > 0.0 {
            for i in 0..PROP_GRID_W * PROP_GRID_H {
                let c = match mode {
                    PropMode::PerBand => {
                        let Some(p) = field.plane(band) else { continue };
                        let t = (p.weight[i] / peak).clamp(0.0, 1.0);
                        if t <= 0.0 {
                            continue;
                        }
                        let [r, g, b] = crate::colormap::prop_ramp_at(t);
                        // Alpha carries intensity too, so a weak cell fades out
                        // rather than sitting there as a flat blue tint.
                        let a = (t.sqrt() * MAX_ALPHA * 255.0) as u8;
                        [r, g, b, a]
                    }
                    PropMode::AllBands => {
                        // Additive in colour, so two bands over one place mix to
                        // something between their hues instead of one winning.
                        let (mut rs, mut gs, mut bs, mut ws) = (0.0f32, 0.0, 0.0, 0.0);
                        for band in &bands {
                            let Some(p) = field.plane(*band) else { continue };
                            let w = p.weight[i] / peak;
                            if w <= 0.0 {
                                continue;
                            }
                            let c = crate::colormap::band_color(*band);
                            rs += c[0] as f32 * w;
                            gs += c[1] as f32 * w;
                            bs += c[2] as f32 * w;
                            ws += w;
                        }
                        if ws <= 0.0 {
                            continue;
                        }
                        let a = (ws.min(1.0).sqrt() * MAX_ALPHA * 255.0) as u8;
                        [(rs / ws) as u8, (gs / ws) as u8, (bs / ws) as u8, a]
                    }
                };
                px[i * 4..i * 4 + 4].copy_from_slice(&c);
            }
        }
        (px, peak, bands)
    }
}

/// Cells across the whole world in the grey-line overlay. 1° is finer than the
/// terminator's own blur — the twilight band is 16° wide — and the texture's
/// LINEAR filtering smooths the edges between cells.
const NIGHT_SHADE_W: usize = 360;
const NIGHT_SHADE_H: usize = 180;

/// The grey-line overlay for the flat maps: where the Sun is up, where it is
/// down, and the twilight between.
///
/// One equirectangular texture, rebuilt only when the minute changes. The
/// terminator moves about 0.25° of longitude a minute, a quarter of a cell, so
/// a per-frame rebuild would upload pixels that differ by nothing. It is
/// geography rather than data: drawn whether or not the propagation heat is on,
/// and independent of it.
#[derive(Default)]
pub struct NightShade {
    tex: Option<eframe::egui::TextureHandle>,
    built_minute: Option<i64>,
}

impl NightShade {
    /// The terminator texture for this instant, rebuilt at most once a minute.
    pub fn texture(&mut self, ctx: &eframe::egui::Context, unix: i64) -> eframe::egui::TextureId {
        let minute = unix.div_euclid(60);
        if self.built_minute != Some(minute) || self.tex.is_none() {
            let (w, h) = (NIGHT_SHADE_W, NIGHT_SHADE_H);
            let rgba = sdroxide_solar::night_shade_rgba(w, h, minute * 60);
            let px: Vec<eframe::egui::Color32> = rgba
                .as_chunks::<4>()
                .0
                .iter()
                .map(|&[r, g, b, a]| eframe::egui::Color32::from_rgba_unmultiplied(r, g, b, a))
                .collect();
            let img = eframe::egui::ColorImage {
                size: [w, h],
                pixels: px,
                source_size: eframe::egui::vec2(w as f32, h as f32),
            };
            match self.tex.as_mut() {
                // LINEAR is what keeps the 1° cells from showing as banding.
                Some(t) => t.set(img, eframe::egui::TextureOptions::LINEAR),
                None => {
                    self.tex = Some(ctx.load_texture(
                        "map-night",
                        img,
                        eframe::egui::TextureOptions::LINEAR,
                    ))
                }
            }
            self.built_minute = Some(minute);
        }
        self.tex.as_ref().map(|t| t.id()).expect("the texture was just built")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::{Decode, PropSource};

    const NOW: i64 = 1_800_000_000;
    const HOME: &str = "IO91";
    const FAR: &str = "KO85";

    fn decode(grid: &str, slot: i64, snr: i16) -> Decode {
        Decode {
            slot_utc: slot,
            snr_db: snr,
            dt: 0.2,
            audio_hz: 1500.0,
            message: format!("CQ AB1CD {grid}"),
            to: None,
            from: Some("AB1CD".into()),
            grid: Some(grid.into()),
            is_cq: true,
            cq_to: None,
            free_text: false,
            rr73_to: None,
        }
    }

    // ── drawing ────────────────────────────────────────────────────────────

    /// An empty field must produce nothing at all rather than a wash of the
    /// ramp's cold end over the whole world.
    #[test]
    fn an_empty_field_draws_nothing() {
        let f = PropField::default();
        let (img, peak, bands) = PropHeat::image(&f, PropMode::AllBands, Band::M20, u32::MAX);
        assert_eq!(peak, 0.0);
        assert!(bands.is_empty());
        assert!(img.pixels.iter().all(|p| p.a() == 0), "an empty field painted something");
    }

    /// Two bands over the same cell have to mix, not fight. This is the whole
    /// difference between "colour-blended shapes" and one band winning per cell.
    #[test]
    fn two_bands_over_one_place_mix_rather_than_one_winning() {
        let mut s = PropStore::default();
        s.observe_decodes(&[decode(FAR, NOW, -12)], PropSource::Ft8, 14_074_000.0, HOME, NOW);
        s.observe_decodes(&[decode(FAR, NOW, -12)], PropSource::Ft8, 28_074_000.0, HOME, NOW);
        let f = s.peek();
        assert_eq!(f.live_bands().len(), 2);

        let (img, _, bands) = PropHeat::image(&f, PropMode::AllBands, Band::M20, u32::MAX);
        assert_eq!(bands.len(), 2);
        let hottest = img.pixels.iter().max_by_key(|p| p.a()).copied().expect("a pixel");
        let m20 = crate::colormap::band_color(Band::M20);
        let m10 = crate::colormap::band_color(Band::M10);
        // Strictly between the two hues on at least one channel, which a
        // winner-takes-all blend could never produce.
        let mixed = (0..3).any(|c| {
            let v = [hottest.r(), hottest.g(), hottest.b()][c] as i32;
            let (a, b) = (m20[c] as i32, m10[c] as i32);
            v > a.min(b) + 2 && v < a.max(b) - 2
        });
        assert!(mixed, "{hottest:?} is not between {m20:?} and {m10:?}");
    }

    /// The per-band view draws the band it was asked for and nothing else — the
    /// point of having it is to look at one band without the others on top.
    #[test]
    fn the_per_band_view_shows_only_the_band_it_was_asked_for() {
        let mut s = PropStore::default();
        s.observe_decodes(&[decode(FAR, NOW, -12)], PropSource::Ft8, 14_074_000.0, HOME, NOW);
        let f = s.peek();
        let (lit, _, _) = PropHeat::image(&f, PropMode::PerBand, Band::M20, u32::MAX);
        assert!(lit.pixels.iter().any(|p| p.a() > 0), "the band with traffic drew nothing");
        let (dark, peak, _) = PropHeat::image(&f, PropMode::PerBand, Band::M10, u32::MAX);
        assert_eq!(peak, 0.0, "a band with no traffic reported a peak");
        assert!(dark.pixels.iter().all(|p| p.a() == 0), "a band with no traffic drew something");
    }

    /// Deselecting a band in the combined view has to remove it.
    #[test]
    fn the_band_mask_keeps_deselected_bands_out_of_the_combined_view() {
        let mut s = PropStore::default();
        s.observe_decodes(&[decode(FAR, NOW, -12)], PropSource::Ft8, 14_074_000.0, HOME, NOW);
        let f = s.peek();
        let m20 = Band::ALL.iter().position(|b| *b == Band::M20).unwrap();
        let (_, peak, bands) = PropHeat::image(&f, PropMode::AllBands, Band::M20, !(1 << m20));
        assert_eq!(peak, 0.0);
        assert!(bands.is_empty());
    }

    /// The heat has to be readable over the map rather than covering it.
    #[test]
    fn the_hottest_cell_never_becomes_opaque() {
        let mut s = PropStore::default();
        for i in 0..50 {
            s.observe_decodes(
                &[decode(FAR, NOW + i * 15, -12)],
                PropSource::Ft8,
                14_074_000.0,
                HOME,
                NOW,
            );
        }
        let f = s.peek();
        let (img, _, _) = PropHeat::image(&f, PropMode::AllBands, Band::M20, u32::MAX);
        let max_a = img.pixels.iter().map(|p| p.a()).max().unwrap_or(0);
        assert!(max_a > 100, "even the hottest cell is barely visible ({max_a})");
        assert!(max_a <= (MAX_ALPHA * 255.0) as u8 + 1, "the map was painted over ({max_a})");
    }
}
