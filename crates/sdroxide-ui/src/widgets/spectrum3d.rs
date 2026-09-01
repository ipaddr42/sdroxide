//! The spectrum strip drawn as a receding surface: the newest spectrum across
//! the front, the ones before it flowing away from the viewer.
//!
//! The same numbers the flat trace is drawn from, with time given the depth
//! axis instead of being thrown away — which is what makes a carrier that
//! comes and goes read as a ridge rather than as a line that twitches. The
//! waterfall says the same thing in colour; this says it in shape, and a weak
//! signal a couple of dB over the floor is a bump you can see the length of.
//!
//! Everything here is plain egui geometry — a mesh per row and, in the line
//! rendering, a polyline over it. Deliberately: the waterfall's history is a
//! wgpu texture behind a paint callback, and a second GPU path would have to
//! be written twice over (native and WebGL) and would put the browser client's
//! panadapter on a lane the native one does not use. The cost of doing it this
//! way is a vertex buffer rather than a texture upload — a quarter of a million
//! vertices a frame for [`DEPTH`] rows across a 1696-point panadapter — which
//! is what [`VERTEX_BUDGET`] is there to bound.
//!
//! Measured on a release build at 60 fps against the flat trace on the same
//! synthetic band: 35% of a core flat, 70% solid, 52% lines. Both renderings
//! are held to the same budget, and the line one spends more of it per column
//! (it strokes a polyline over the same fill), so it is handed fewer columns
//! and comes out the cheaper of the two — the budget is the knob, not the
//! rendering. This is the only display in the client that can double its own
//! CPU, which is why it is off until it is asked for.
//!
//! Hidden surface removal is the painter's algorithm and nothing else: rows go
//! down back to front, each filled from its own curve to its own baseline, so
//! a near ridge covers the far ones behind it and a flat near row leaves them
//! showing over the top. That is why the fills are opaque even in the line
//! rendering, where the fill is the background colour and its whole job is to
//! be something for the lines behind to disappear into.

use std::collections::VecDeque;

use eframe::egui::{self, Align2, Color32, FontId, Rect, Shape, Stroke, pos2};

use crate::view::ViewState;

/// Spectra the surface remembers, and so the depth of the picture. Rows are
/// clocked off the wall clock at the operator's **flow** rate
/// ([`sdroxide_types::UiSettings::spectrum_3d_rows_per_sec`]), so this is a
/// fixed number of rows and the time it covers is that rate: eight seconds at
/// Slow, one at Faster. Time is not labelled on this axis and does not need to
/// be — the depth is there to show how a signal is changing, and the waterfall
/// next to it is the instrument with a clock on it.
///
/// Every one of them is drawn, every frame, which is what fixes the number:
/// deciding row by row which are worth drawing is what made the far half of
/// the surface shiver (see `draw`). Forty-eight is what a 1696-point-wide
/// panadapter can carry at one column per point inside [`VERTEX_BUDGET`], and
/// it puts a row every few points down an ordinary strip's depth axis — close
/// enough that the surface between them reads as a surface.
pub const DEPTH: usize = 48;

/// Rows the flow may advance in one frame. A tab left in the background, or a
/// hitch, would otherwise come back and rebuild the whole surface out of the
/// one spectrum that happened to be current — the same clamp the waterfall's
/// own scroll puts on `dt`.
const MAX_STEP: usize = 8;

/// Vertices the whole surface is allowed, near enough. What it buys differs by
/// rendering: the solid one spends two a column (the top of the curtain and its
/// foot), the line one spends those plus egui's own polyline tessellation over
/// the top, which is about five more. Both are counted in [`columns_for`], so
/// the two renderings cost about the same and the line one simply comes out
/// with fewer columns.
///
/// The number is what a 1696-point-wide panadapter needs to be drawn at one
/// vertex per point in the solid rendering — i.e. it is set so that the display
/// most people have is not compromised at all, and only a very wide window
/// starts trading columns for the budget.
const VERTEX_BUDGET: usize = 260_000;

/// How the strip's height is split between the receding axis and the amplitude
/// the newest row is drawn at: this much depth to `1 - DEPTH_FRAC` of
/// amplitude. A ratio rather than a fraction of the strip, because [`FILL`]
/// then scales the pair of them up together until the picture reaches the top
/// edge.
///
/// Nearly half, because the two compete: what the depth axis is given is what
/// separates one row from the next, and a surface whose noise floor alone is
/// taller than its whole depth reads as a picket fence rather than as a
/// landscape.
const DEPTH_FRAC: f32 = 0.46;

/// How hard the perspective narrows: the oldest row comes out `1/(1 + PERSP)`
/// as wide as the newest, and its amplitude is foreshortened by the same
/// factor, which is what makes the rows sit on one another rather than beside
/// one another.
const PERSP: f32 = 0.85;

/// How far the far end of the surface is dimmed towards the background, so
/// depth reads as depth and not as a signal that got weaker.
const FOG: f32 = 0.55;

/// The share of the strip a picture built straight out of [`DEPTH_FRAC`] and
/// [`PERSP`] would actually reach into, and so the divisor that takes the
/// slack back out again.
///
/// The tallest thing the surface can draw is the *oldest* row at full scale:
/// it stands on the back of the floor, [`DEPTH_FRAC`] up the strip, and adds
/// its own amplitude foreshortened by `1/(1 + PERSP)` on top. That comes to
/// about three quarters of the height, which left the top quarter of the strip
/// unreachable — dead in every picture, and a quarter of a *bigger* number
/// every time the operator dragged the separator down, so the display answered
/// being given more room by growing its own margin.
///
/// Dividing both axes by it scales the picture up until the far row at the
/// ceiling touches the top edge, which fixes the waste at nothing whatever the
/// strip's height. The two axes keep their ratio to each other, so this is the
/// same picture drawn larger and not a different balance between depth and
/// amplitude.
const FILL: f32 = DEPTH_FRAC + (1.0 - DEPTH_FRAC) / (1.0 + PERSP);

/// One remembered spectrum, with the window it was taken over.
///
/// Its own centre and span rather than an index into the current view: a
/// retune, a pan or a zoom changes where these bins belong on screen, and a
/// row that carried only its samples would be redrawn as if it had been
/// measured on the band that is on screen now. Keeping the axis with the
/// samples means an older row simply slides — and runs off the edge, reading
/// as the floor — exactly as it should.
#[derive(Default)]
struct Row {
    center_hz: f64,
    span_hz: f64,
    /// Levels in the frame's own u8 mapping over `[db_floor, db_ceil]`, the
    /// same scale the flat trace and the waterfall are drawn from.
    bins: Vec<u8>,
}

impl Row {
    /// Take a spectrum, keeping whatever allocation this row already had.
    fn set(&mut self, center_hz: f64, span_hz: f64, values: &[f32]) {
        self.center_hz = center_hz;
        self.span_hz = span_hz;
        self.bins.clear();
        self.bins.extend(values.iter().map(|v| v.clamp(0.0, 255.0) as u8));
    }
}

/// The remembered spectra, newest last, and the live one in front of them.
#[derive(Default)]
pub struct Surface {
    rows: VecDeque<Row>,
    /// The spectrum as of this frame, which is what the *front* of the surface
    /// is drawn from — not the newest remembered row.
    ///
    /// Without it the front row is however old the flow rate makes it, and the
    /// surface arrives in steps: a sliver of empty floor opens at the front,
    /// grows for a whole row, and is snapped shut by a row appearing out of
    /// nothing. At Faster that happens 64 times a second and reads as motion;
    /// at Slow it happens 8 times a second and reads as a stutter. Drawing the
    /// live spectrum at the front and letting the remembered rows glide back
    /// behind it makes the flow continuous at every rate — a row is laid down
    /// as an exact copy of the live one, in the same place, so the moment it
    /// joins the history is invisible.
    live: Row,
    /// Wall clock the flow was last advanced at, and the fraction of a row
    /// carried over from it. The rate is absolute — rows a *second*, not rows a
    /// frame — so the surface flows at the speed the operator picked whatever
    /// the frame rate is, and a browser client at 30 fps and a desktop at 60
    /// show the same picture.
    ///
    /// The carry is not only bookkeeping: it is how far the remembered rows
    /// have travelled since the last one was laid down, and the drawing adds it
    /// to every row's depth. That is what turns the flow from a step per row
    /// into a glide.
    last_now: f64,
    accum: f32,
}

impl Surface {
    /// Advance the flow to `now` and lay down however many rows are due, from
    /// the spectrum in `values` — the UI-smoothed bins the flat trace would
    /// have been drawn from, so the *reaction* setting steadies this surface
    /// exactly as it steadies the line.
    ///
    /// `rows_per_sec` of zero holds the surface still. That is what a stalled
    /// stream comes through as (see `WfTuning::surface_rows_per_sec`): rows of
    /// a spectrum that is no longer being measured would be time that never
    /// happened, which is the same reason the waterfall stops scrolling.
    ///
    /// Above the frame rate the extra rows are copies of the same spectrum,
    /// exactly as a waterfall clocked faster than its front end repeats lines.
    pub fn push(
        &mut self,
        center_hz: f64,
        span_hz: f64,
        values: &[f32],
        now: f64,
        rows_per_sec: f32,
    ) {
        if values.is_empty() || span_hz <= 0.0 {
            return;
        }
        // First call, or the clock jumped: start counting from here rather than
        // treating the gap as time the surface owes rows for.
        let dt = if self.last_now > 0.0 { (now - self.last_now).clamp(0.0, 0.3) } else { 0.0 };
        self.last_now = now;
        if rows_per_sec <= 0.0 {
            self.accum = 0.0;
            return;
        }
        self.accum += dt as f32 * rows_per_sec;
        let due = (self.accum.floor() as usize).min(MAX_STEP);
        self.accum -= self.accum.floor();
        self.live.set(center_hz, span_hz, values);
        for _ in 0..due {
            // Reuse the oldest row's allocation rather than freeing and asking
            // for it again a few kilobytes at a time, sixty times a second.
            let mut row = if self.rows.len() >= DEPTH {
                self.rows.pop_front().unwrap_or_default()
            } else {
                Row::default()
            };
            row.set(center_hz, span_hz, values);
            self.rows.push_back(row);
        }
    }

    /// Forget everything. Called while the flat trace is the one being drawn:
    /// coming back to the surface with minutes-old rows still in it would draw
    /// a band that has since been left as though it were the last few seconds,
    /// which is the same trap the full-band strip clears its history for.
    pub fn clear(&mut self) {
        self.rows.clear();
        self.live.bins.clear();
        self.last_now = 0.0;
        self.accum = 0.0;
    }
}

/// The height the newest row is drawn in: the strip's amplitude axis, from the
/// bottom edge upwards. What the markers that annotate the *current* spectrum
/// — the passband, the filter edges, the tuning lines — are kept inside, so
/// they mark the front plane rather than cutting through the rows behind it.
pub fn front_plane_h(strip_h: f32) -> f32 {
    strip_h * (1.0 - DEPTH_FRAC) / FILL
}

/// One-point perspective onto the strip: the newest row across the front at
/// full width, the vanishing point centred above it.
struct Proj {
    cx: f32,
    front_y: f32,
    depth_h: f32,
    amp_h: f32,
}

impl Proj {
    fn new(rect: &Rect) -> Self {
        Proj {
            cx: rect.center().x,
            front_y: rect.bottom(),
            depth_h: rect.height() * DEPTH_FRAC / FILL,
            amp_h: front_plane_h(rect.height()),
        }
    }

    /// Foreshortening at depth `d` — 0 the newest row, 1 the oldest.
    fn scale(&self, d: f32) -> f32 {
        1.0 / (1.0 + PERSP * d)
    }

    /// The baseline the row at depth `d` stands on. Spaced by the same
    /// division the width is, so the rows crowd together towards the back the
    /// way the columns narrow.
    fn base_y(&self, d: f32) -> f32 {
        let far = 1.0 / (1.0 + PERSP);
        self.front_y - self.depth_h * (1.0 - self.scale(d)) / (1.0 - far)
    }

    /// Where a point that would be at `x_lin` on the front plane lands on the
    /// plane at foreshortening `s`.
    fn x(&self, x_lin: f32, s: f32) -> f32 {
        self.cx + (x_lin - self.cx) * s
    }
}

/// `c` dimmed towards black by `f` (1 leaves it alone, 0 blacks it out).
fn dim(c: [u8; 3], f: f32) -> Color32 {
    let f = f.clamp(0.0, 1.0);
    Color32::from_rgb((c[0] as f32 * f) as u8, (c[1] as f32 * f) as u8, (c[2] as f32 * f) as u8)
}

/// The floor the rows stand on: the frequency gridlines converging on the
/// vanishing point, the back edge, and the two sides. Drawn before the rows,
/// so a ridge in front covers the floor behind it.
fn draw_floor(painter: &egui::Painter, view: &ViewState, rect: &Rect, p: &Proj) {
    let grid = Stroke::new(0.5, crate::theme::scope_gray(38));
    let s_far = p.scale(1.0);
    let back_y = p.base_y(1.0);
    let step = super::spectrum_view::freq_grid_step_for_width(
        view.view_lo_hz,
        view.view_hi_hz,
        rect.width(),
    );
    for hz in super::spectrum_view::gridlines_at(view.view_lo_hz, view.view_hi_hz, step) {
        let x_lin = view.freq_to_x(hz, rect);
        painter.line_segment(
            [pos2(p.x(x_lin, 1.0), p.front_y), pos2(p.x(x_lin, s_far), back_y)],
            grid,
        );
    }
    // The far edge and the two rails, which are what say the floor is a plane
    // and not a fan of unrelated lines.
    let edge = Stroke::new(0.5, crate::theme::scope_gray(52));
    let (bl, br) = (p.x(rect.left(), s_far), p.x(rect.right(), s_far));
    painter.line_segment([pos2(bl, back_y), pos2(br, back_y)], edge);
    painter.line_segment([pos2(rect.left(), p.front_y), pos2(bl, back_y)], edge);
    painter.line_segment([pos2(rect.right(), p.front_y), pos2(br, back_y)], edge);

    // The amplitude axis, on the front plane where the newest row is drawn at
    // full height. Every 20 dB of the display range, as the flat grid does.
    let range = view.db_ceil - view.db_floor;
    if range > 1.0 {
        let mut db = (view.db_floor / 20.0).ceil() * 20.0;
        while db < view.db_ceil {
            let y = p.front_y - (db - view.db_floor) / range * p.amp_h;
            painter.line_segment([pos2(rect.left(), y), pos2(rect.left() + 5.0, y)], grid);
            painter.text(
                pos2(rect.left() + 7.0, y),
                Align2::LEFT_CENTER,
                format!("{db:.0}"),
                FontId::monospace(9.0 * crate::theme::panadapter_font_scale()),
                crate::theme::scope_gray(100),
            );
            db += 20.0;
        }
    }
}

/// Draw the surface into `rect`.
///
/// `solid` picks the rendering: a filled surface coloured by the waterfall
/// palette `palette`, or traces over an opaque fill. `palette` is ignored by
/// the line rendering, which draws every row in the flat trace's own colour so
/// that switching between the two displays does not also switch the ink.
pub fn draw(
    painter: &egui::Painter,
    rect: &Rect,
    view: &ViewState,
    surface: &Surface,
    solid: bool,
    palette: usize,
) {
    if rect.height() < 24.0 || rect.width() < 16.0 {
        return;
    }
    let p = Proj::new(rect);
    draw_floor(painter, view, rect, &p);
    if surface.live.bins.is_empty() && surface.rows.is_empty() {
        return;
    }

    // Which rows to draw, front to back for the choosing and back to front for
    // the drawing.
    //
    // The live spectrum is the front of the surface and the remembered rows sit
    // behind it, each one `accum` further back than the last row laid down —
    // see [`Surface::live`]. Depth is counted in rows here and turned into a
    // fraction of the axis at the end, so the carry is simply added.
    //
    // *Every* remembered row, and no rule about which ones are worth drawing.
    //
    // A rule that reads the row's position on screen cannot be stable, because
    // the position is what is moving: a row that clears its neighbour by a
    // pixel this frame does not clear it the next, so the set being drawn
    // alternates and the surface shivers between two versions of itself from
    // the depth where the rows first crowd together — with the onset visible as
    // a line across the picture. A rule that reads the row's *index* is no
    // better: every row laid down renumbers them all, so an every-other-one
    // stride swaps its two halves once a row.
    //
    // So the count is fixed instead, and [`DEPTH`] is chosen small enough that
    // all of them can be afforded — the trade is made in [`columns_for`],
    // where spending fewer columns costs a little sharpness and nothing else.
    //
    // Normalised by `DEPTH` rather than `DEPTH - 1` so the carry has somewhere
    // to go: at `DEPTH - 1` the oldest row is pushed past the end of the axis
    // the instant the flow moves at all, and popping out and back once a row is
    // the same flicker in the one place it is least worth having.
    let carry = surface.accum.clamp(0.0, 1.0);
    let far = DEPTH as f32;
    let newest = surface.rows.len().saturating_sub(1);
    let live = (!surface.live.bins.is_empty()).then_some((&surface.live, 0.0));
    let history =
        (0..surface.rows.len()).rev().map(|i| (&surface.rows[i], (newest - i) as f32 + carry));
    let mut drawn: Vec<(&Row, f32)> =
        live.into_iter().chain(history).map(|(row, back)| (row, back / far)).collect();
    // Back to front: the painter's algorithm is the depth buffer.
    drawn.reverse();
    let cols = columns_for(rect.width(), drawn.len(), solid);

    // The screen x of each column and the band it covers, worked out once for
    // the whole surface: they are the same for every row (the rows differ only
    // in where they stand and how wide they are drawn), and `x_to_freq` is f64.
    // Half a column either side rather than the centre alone, because what is
    // sampled is a *range* — see the peak below.
    let mut xs = Vec::with_capacity(cols);
    let mut edges = Vec::with_capacity(cols);
    let last = (cols - 1) as f32;
    let half = rect.width() / last * 0.5;
    for c in 0..cols {
        let x = rect.left() + rect.width() * c as f32 / last;
        xs.push(x);
        edges.push((view.x_to_freq(x - half, rect), view.x_to_freq(x + half, rect)));
    }

    let lut = crate::colormap::lut(palette);
    // The crest of the row behind, kept so this one can be joined to it. A
    // height field is a mesh in both directions and not a stack of independent
    // profiles: a carrier is one column wide, perspective puts that column at a
    // different x on every row, and without the strip between them a steady
    // carrier comes out as a row of separate needles receding to the vanishing
    // point instead of the wall it is.
    let mut behind: Vec<egui::epaint::Vertex> = Vec::new();
    let mut line = Vec::with_capacity(cols);
    let uv = egui::epaint::WHITE_UV;

    for (row, d) in drawn {
        let n = row.bins.len();
        if n == 0 || row.span_hz <= 0.0 {
            continue;
        }
        let s = p.scale(d);
        let base_y = p.base_y(d);
        let fog = 1.0 - FOG * d;
        let lo = row.center_hz - row.span_hz / 2.0;

        // The row's own bin axis as a line: bin = hz * k + b. Two multiplies a
        // column instead of a divide, and it is the row's axis and not the
        // view's, which is what lets an older row slide with the band.
        let k = n as f64 / row.span_hz;
        let b = -lo * k;

        // Three runs of vertices: the crest of the row behind, this row's own
        // crest, and its feet. The strip between the first two is the surface
        // itself; the strip between the last two is the curtain that hides
        // everything the surface stands in front of.
        let joined = behind.len() == cols;
        let mut mesh = egui::epaint::Mesh::default();
        mesh.vertices.reserve(cols * 3);
        mesh.indices.reserve((cols - 1) * 12);
        if joined {
            mesh.vertices.extend_from_slice(&behind);
        }
        line.clear();
        for c in 0..cols {
            // The strongest bin in the slice of band this column covers, not
            // the one that happens to land under its centre. Zoomed out there
            // are several bins to a column, and point-sampling them drops a
            // carrier the moment it falls between two columns — it flickers,
            // and on a surface where every row samples the same axis it
            // flickers as a hole punched through the ridge. Adjacent columns
            // tile the axis, so the whole row still costs one pass over the
            // bins however many columns there are.
            let (f0, f1) = edges[c];
            let (i0, i1) = (f0 * k + b, f1 * k + b);
            let raw = peak(&row.bins, i0, i1);
            let v = raw as f32 / 255.0;
            let x = p.x(xs[c], s);
            let top = pos2(x, base_y - v * p.amp_h * s);
            let (c_top, c_base) = if solid {
                let rgb =
                    [lut[raw as usize * 4], lut[raw as usize * 4 + 1], lut[raw as usize * 4 + 2]];
                // The curtain under each column falls away to almost black,
                // which is what separates one row from the row behind without
                // a line drawn between them. Nearly black rather than merely
                // darker on purpose: the rows overlap several deep at any
                // useful amplitude, so a crest read against a dim curtain is
                // the only thing that makes the surface a surface and not a
                // block of colour with a ragged top edge.
                (dim(rgb, fog), dim(rgb, fog * 0.10))
            } else {
                // The background, lifting a shade towards the back: the fills
                // are here to hide what is behind them, and the lift is the
                // only thing telling a far row's blank from a near one's.
                let g = (6.0 + 14.0 * d) as u8;
                let c = Color32::from_gray(g);
                (c, c)
            };
            mesh.vertices.push(egui::epaint::Vertex { pos: top, uv, color: c_top });
            mesh.vertices.push(egui::epaint::Vertex { pos: pos2(x, base_y), uv, color: c_base });
            if !solid {
                line.push(top);
            }
        }
        // Where the three runs start. Interleaved crest/foot for this row, so
        // its two vertices for column `c` are adjacent.
        let (back0, own0) = if joined { (0u32, cols as u32) } else { (0, 0) };
        for c in 0..cols - 1 {
            let (a, e) = (own0 + (c * 2) as u32, own0 + ((c + 1) * 2) as u32);
            // The surface between this row and the one behind it, first: it
            // lies further away than anything else this row draws, and the
            // painter's algorithm is the only depth test there is.
            if joined {
                let (q, r) = (back0 + c as u32, back0 + c as u32 + 1);
                mesh.indices.extend_from_slice(&[q, r, a, a, r, e]);
            }
            // Then the curtain from this row's crest down to its own feet.
            mesh.indices.extend_from_slice(&[a, a + 1, e, e, a + 1, e + 1]);
        }
        // This row's crest becomes the row behind for the next one round.
        behind.clear();
        behind.extend(mesh.vertices[own0 as usize..].iter().step_by(2).copied());
        painter.add(Shape::mesh(mesh));
        if !solid {
            let c = TRACE;
            painter.add(Shape::line(
                line.clone(),
                Stroke::new(1.0, dim([c.r(), c.g(), c.b()], 0.35 + 0.65 * fog)),
            ));
        }
    }
}

/// The flat trace's own colour, so the line rendering reads as that trace
/// given a depth axis rather than as a different instrument.
const TRACE: Color32 = Color32::from_rgb(120, 220, 255);

/// Columns to draw each row with: one per point of width, cut back only if
/// [`VERTEX_BUDGET`] says the surface cannot afford that many.
///
/// One vertex per *point* rather than per device pixel, which is where the
/// flat trace samples. The trace is one polyline and this is dozens, so the
/// last half-pixel of sharpness is the most expensive thing on offer here and
/// the least visible: a row is a couple of points tall.
fn columns_for(width: f32, rows: usize, solid: bool) -> usize {
    // Three vertices a column — the crest, its foot, and the copy of the crest
    // that joins this row to the one behind — plus about five more for the
    // polyline egui tessellates over the top in the line rendering.
    let per_col = if solid { 3 } else { 8 };
    let affordable = VERTEX_BUDGET / per_col / rows.max(1);
    (width.round() as usize).clamp(48, affordable.max(48))
}

/// The largest bin between `i0` and `i1` on a row's bin axis, with the range
/// clamped into the row and an empty one answered by the single bin it lands
/// in. Anything wholly outside the row reads as the floor — which is what the
/// band outside a frame's own span always is, exactly as the flat trace has it.
fn peak(bins: &[u8], i0: f64, i1: f64) -> u8 {
    let n = bins.len();
    if n == 0 || i1 <= 0.0 || i0 >= n as f64 {
        return 0;
    }
    let a = (i0.max(0.0) as usize).min(n - 1);
    let b = (i1.max(0.0).ceil() as usize).clamp(a + 1, n);
    bins[a..b].iter().copied().max().unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run the flow for `secs` at `rate`, in sixtieth-of-a-second frames.
    fn flow(s: &mut Surface, secs: f64, rate: f32) {
        let mut t = 100.0;
        s.push(14_100_000.0, 100_000.0, &[10.0, 200.0, 30.0], t, rate);
        for _ in 0..(secs * 60.0) as usize {
            t += 1.0 / 60.0;
            s.push(14_100_000.0, 100_000.0, &[10.0, 200.0, 30.0], t, rate);
        }
    }

    /// The history is a ring: it fills to [`DEPTH`] and then drops the oldest,
    /// rather than growing for as long as the radio is on.
    #[test]
    fn the_surface_keeps_a_fixed_depth() {
        let mut s = Surface::default();
        flow(&mut s, 30.0, 16.0);
        assert_eq!(s.rows.len(), DEPTH);
    }

    /// The flow is rows a *second*, not rows a frame: that is what makes the
    /// picture the same on a browser client at 30 fps as on a desktop at 60,
    /// and what makes the **flow** chips mean seconds of band.
    #[test]
    fn the_flow_runs_on_the_clock_and_not_on_the_frame_rate() {
        // Two seconds at sixteen rows a second is thirty-two rows, at either
        // frame rate. One row of slack for where the fraction lands.
        for fps in [30.0, 60.0] {
            let mut s = Surface::default();
            let mut t = 100.0;
            s.push(14_100_000.0, 100_000.0, &[1.0], t, 16.0);
            for _ in 0..(2.0 * fps) as usize {
                t += 1.0 / fps;
                s.push(14_100_000.0, 100_000.0, &[1.0], t, 16.0);
            }
            let n = s.rows.len();
            assert!((31..=33).contains(&n), "at {fps} fps the flow laid down {n} rows, not 32");
        }
    }

    /// The front of the surface is the *live* spectrum, and it is there from
    /// the first frame — before the flow has had time to lay down a single
    /// remembered row. Without it the surface is blank for an eighth of a
    /// second at Slow, and its front row is always a row out of date.
    #[test]
    fn the_live_spectrum_is_the_front_of_the_surface_at_once() {
        let mut s = Surface::default();
        s.push(14_100_000.0, 100_000.0, &[7.0, 9.0], 100.0, 6.0);
        assert!(s.rows.is_empty(), "a row was laid down before any time had passed");
        assert_eq!(s.live.bins, vec![7, 9], "the live row was not taken");
        // And it follows every frame, not every row: at eight rows a second
        // sixty frames go by between one row and the next.
        s.push(14_100_000.0, 100_000.0, &[1.0, 2.0], 100.008, 6.0);
        assert_eq!(s.live.bins, vec![1, 2], "the live row went stale between rows");
    }

    /// The carry is what makes the flow a glide rather than a step: between one
    /// row and the next it climbs through every value from 0 to 1, and the
    /// drawing adds it to the depth of every remembered row. A carry that only
    /// ever read 0 would put the whole surface back a whole row at a time —
    /// which at Slow is eight visible jerks a second.
    #[test]
    fn the_carry_glides_between_one_row_and_the_next() {
        let mut s = Surface::default();
        let (rate, mut t) = (6.0f32, 100.0f64);
        s.push(14_100_000.0, 100_000.0, &[1.0], t, rate);
        let mut seen = Vec::new();
        for _ in 0..60 {
            t += 1.0 / 60.0;
            s.push(14_100_000.0, 100_000.0, &[1.0], t, rate);
            seen.push(s.accum);
        }
        assert!(seen.iter().all(|&a| (0.0..1.0).contains(&a)), "the carry left [0, 1): {seen:?}");
        // A whole second at six rows a second: six rows laid down, and the
        // carry seen at a spread of values on the way rather than sitting at 0.
        assert_eq!(s.rows.len(), 6);
        let distinct = seen.iter().filter(|a| **a > 0.05).count();
        assert!(distinct > 40, "the carry only moved {distinct} of 60 frames");
    }

    /// Whatever the carry, every remembered row still lands on the axis.
    /// Normalised by `DEPTH - 1` the oldest one is pushed off the end the
    /// instant the flow moves at all and pops back a row later — flicker in the
    /// one place it is least worth having, and the reason for the `DEPTH` here.
    #[test]
    fn the_carry_never_pushes_a_row_off_the_back() {
        for carry in [0.0f32, 0.01, 0.5, 0.99] {
            let d = ((DEPTH - 1) as f32 + carry) / DEPTH as f32;
            assert!((0.0..1.0).contains(&d), "a carry of {carry} put the oldest row at {d}");
        }
    }

    /// A rate of zero is how a stalled stream reaches the surface. It must hold
    /// still: rows of a spectrum nobody is measuring any more would be time
    /// that never happened, which is exactly what the waterfall stops for.
    #[test]
    fn a_stalled_stream_holds_the_surface_still() {
        let mut s = Surface::default();
        flow(&mut s, 1.0, 16.0);
        let before = s.rows.len();
        assert!(before > 0);
        let mut t = 200.0;
        for _ in 0..120 {
            t += 1.0 / 60.0;
            s.push(14_100_000.0, 100_000.0, &[1.0], t, 0.0);
        }
        assert_eq!(s.rows.len(), before, "a stalled stream kept filling the surface");
    }

    /// A tab left in the background comes back with a long gap on the clock.
    /// The surface must not rebuild itself out of the one spectrum that happens
    /// to be current when it does.
    #[test]
    fn a_long_gap_does_not_rebuild_the_whole_surface() {
        let mut s = Surface::default();
        s.push(14_100_000.0, 100_000.0, &[1.0], 100.0, 64.0);
        s.push(14_100_000.0, 100_000.0, &[1.0], 400.0, 64.0);
        assert!(s.rows.len() <= MAX_STEP, "{} rows from one gap", s.rows.len());
    }

    /// Clearing has to reset the clock as well, or the first frame after coming
    /// back to the surface is owed every row of the gap since it was left.
    #[test]
    fn a_cleared_surface_starts_its_clock_again() {
        let mut s = Surface::default();
        flow(&mut s, 1.0, 16.0);
        s.clear();
        s.push(14_100_000.0, 100_000.0, &[1.0], 500.0, 16.0);
        assert_eq!(s.rows.len(), 0, "the gap across the clear was charged as rows");
    }

    /// A column stands for a slice of band, not for the one bin under its
    /// centre. Point-sampling drops a carrier the moment it falls between two
    /// columns; on a surface, where every row samples the same axis, that is a
    /// hole punched clean through the ridge.
    #[test]
    fn a_column_takes_the_strongest_bin_it_covers() {
        // One hot bin in a hundred, and a column covering ten of them.
        let mut bins = vec![10u8; 100];
        bins[43] = 250;
        assert_eq!(peak(&bins, 40.0, 50.0), 250, "the carrier fell between two columns");
        assert_eq!(peak(&bins, 50.0, 60.0), 10);
        // A range narrower than a bin still reads the bin it lands in.
        assert_eq!(peak(&bins, 43.2, 43.6), 250);
        // And the band outside the row is the floor, from either end.
        assert_eq!(peak(&bins, -20.0, -1.0), 0);
        assert_eq!(peak(&bins, 100.0, 120.0), 0);
        // A range that only half overlaps still sees what it covers.
        assert_eq!(peak(&bins, -5.0, 44.0), 250);
        assert_eq!(peak(&[], 0.0, 4.0), 0);
    }

    /// The columns follow the width, and the vertex budget is what stops a very
    /// wide panadapter from spending the frame on them. The line rendering pays
    /// egui's polyline tessellation as well, so it gets fewer.
    #[test]
    fn the_column_count_follows_the_width_until_the_budget_bites() {
        // An ordinary panadapter is drawn at one vertex per point, uncut.
        assert_eq!(columns_for(1696.0, 47, true), 1696);
        // A very wide one is cut back rather than allowed to cost the frame.
        let wide = columns_for(3440.0, 47, true);
        assert!(wide < 3440, "a 3440-point surface was drawn at full width");
        assert!(wide > 1000, "the budget cut a wide surface to {wide} columns");
        // The line rendering is more expensive a column, so it gets fewer of
        // them at the same width and row count — never more.
        assert!(columns_for(1696.0, 47, false) <= columns_for(1696.0, 47, true));
        // Whatever the budget says, a surface is never drawn as a handful of
        // columns: a floor keeps the picture readable on a tall thin strip.
        assert!(columns_for(1696.0, DEPTH, false) >= 48);
    }

    /// The front row is drawn at full width on the bottom edge, and the oldest
    /// stands a depth's height above it, narrowed. Everything the projection
    /// is for, asserted at its two ends.
    #[test]
    fn the_newest_row_is_across_the_front() {
        let rect = Rect::from_min_size(pos2(10.0, 20.0), egui::vec2(200.0, 100.0));
        let p = Proj::new(&rect);
        assert_eq!(p.scale(0.0), 1.0);
        assert_eq!(p.base_y(0.0), rect.bottom());
        assert_eq!(p.x(rect.left(), 1.0), rect.left());
        assert_eq!(p.x(rect.right(), 1.0), rect.right());

        let far = p.scale(1.0);
        assert!(far < 1.0, "the oldest row is not foreshortened");
        let depth_h = rect.height() * DEPTH_FRAC / FILL;
        assert!((p.base_y(1.0) - (rect.bottom() - depth_h)).abs() < 1e-3);
        assert!(p.x(rect.left(), far) > rect.left(), "the back edge is not inset");
        assert!(p.x(rect.right(), far) < rect.right(), "the back edge is not inset");
    }

    /// The rows must never reach past the strip they are drawn in: a full-scale
    /// signal on the oldest row is the highest anything can go.
    #[test]
    fn a_full_scale_signal_stays_inside_the_strip() {
        let rect = Rect::from_min_size(pos2(0.0, 0.0), egui::vec2(300.0, 120.0));
        let p = Proj::new(&rect);
        for step in 0..=10 {
            let d = step as f32 / 10.0;
            let top = p.base_y(d) - p.amp_h * p.scale(d);
            assert!(top >= rect.top() - 0.001, "depth {d} drew above the strip: {top}");
            assert!(top <= rect.bottom(), "depth {d} drew below the strip: {top}");
        }
    }

    /// ...and it must reach it. The picture is built from the two axes and the
    /// perspective, and left to itself it stopped a quarter of the way down
    /// from the top edge — a band nothing could ever be drawn in, which grew
    /// with the strip, so dragging the separator down bought the operator more
    /// empty sky rather than more surface. [`FILL`] is what takes it out, and
    /// it has to take *all* of it out at *every* height.
    #[test]
    fn the_surface_reaches_the_top_of_the_strip_at_any_height() {
        for h in [40.0, 120.0, 400.0, 900.0, 1440.0] {
            let rect = Rect::from_min_size(pos2(0.0, 7.0), egui::vec2(300.0, h));
            let p = Proj::new(&rect);
            // The tallest the surface can draw, which is the oldest row at the
            // ceiling: it stands furthest back and its amplitude is stacked on
            // top of the whole depth axis.
            let top = (0..=20)
                .map(|s| {
                    let d = s as f32 / 20.0;
                    p.base_y(d) - p.amp_h * p.scale(d)
                })
                .fold(f32::INFINITY, f32::min);
            let waste = top - rect.top();
            assert!(waste.abs() < 0.01, "a {h}-point strip left {waste} points above the surface");
        }
    }

    /// The front plane the passband and the filter edges are drawn on is the
    /// amplitude axis, so it has to be the *scaled* one — marks laid out on the
    /// unscaled height would sit a third of the way down the row they annotate.
    #[test]
    fn the_marks_plane_is_the_amplitude_axis() {
        let rect = Rect::from_min_size(pos2(0.0, 0.0), egui::vec2(300.0, 500.0));
        let p = Proj::new(&rect);
        assert!((front_plane_h(rect.height()) - p.amp_h).abs() < 1e-3);
    }
}
