//! The QO-100 BEACON window: calibrate the station's frequency chain against
//! Es'hail-2's narrowband beacon.
//!
//! The beacon this tracks — [`QO100_BEACON_HZ`] — is not a plain carrier: it
//! is a 400 baud differential+Manchester BPSK telemetry signal (AO-40
//! "uncoded" framing), which is why a magnitude peak search has no purpose
//! here — Manchester encoding leaves a *null* at the carrier frequency, not a
//! peak. The actual demodulator lives engine-side, in `sdroxide_qo100`
//! (raw IQ and real phase information, neither of which the UI has); this
//! window is a thin front end onto [`sdroxide_types::Qo100Settings`] (ON/OFF,
//! search width) and [`sdroxide_types::Qo100Status`] (lock, measured
//! frequency, decoded text) — the same split the ISM window keeps with
//! [`sdroxide_types::IsmSettings`]/[`sdroxide_types::IsmStatus`].
//!
//! When the decoder locks, the frequency it had to assume for the sync word
//! and CRC to check out *is* the station's whole frequency error — the LNB's
//! real LO plus whatever the SDR's own clock is off by, lumped together the
//! way an operator would correct them by hand. APPLY writes that correction
//! into [`sdroxide_types::RadioConfig::converter_offset_hz`] and reopens the
//! front end — the same round trip Settings ▸ Radio ▸ Apply makes — rather
//! than doing it silently on every lock, so a bad reading never yanks a
//! running receiver.
//!
//! The mini waterfall is visual context only now, not a measurement: it reads
//! the same [`sdroxide_types::SpectrumFrame`] everything else on screen
//! already gets, cropped to a window around the beacon, so the operator can
//! see the Manchester null (and the search width buttons' effect) even
//! though nothing here measures anything off it.

use std::collections::VecDeque;

use eframe::egui::{
    self, Color32, ColorImage, Pos2, Rect, RichText, Sense, Stroke, TextureHandle, Vec2,
};
use sdroxide_types::{Command, QO100_BEACON_HZ, Qo100Status, SpectrumFrame, Vfo};

use crate::app::SdroxideApp;
use crate::{colormap, theme};

/// Columns the mini waterfall's history is resampled to — independent of the
/// widget's pixel width, like [`crate::widgets::wide_spectrum`]'s `COLS`.
const COLS: usize = 240;
/// Rows of history kept.
const ROWS: usize = 90;
/// Height of the strip, in points.
const STRIP_H: f32 = 130.0;

/// The search-width step the width buttons move by, and the ends of the
/// range they're clamped to — "5 kHz and its multiples", wide enough at the
/// top for a station that has never been calibrated and narrow enough at the
/// bottom that the search is not paying to cover more band than any real
/// LNB drifts.
const WIDTH_STEP_HZ: f64 = 5_000.0;
const MIN_HALF_WIDTH_HZ: f64 = WIDTH_STEP_HZ;
const MAX_HALF_WIDTH_HZ: f64 = 50_000.0;

/// Everything the window remembers between frames that is not a setting the
/// engine already tracks — purely the mini waterfall's own drawing state,
/// and the last correction actually applied.
#[derive(Default)]
pub(in crate::app) struct Qo100WinState {
    /// Newest row last, each [`COLS`] bytes of 0..=255 magnitude, resampled
    /// from whatever of the live [`SpectrumFrame`] falls in the current
    /// window.
    rows: VecDeque<Vec<u8>>,
    tex: Option<TextureHandle>,
    last_seq: u32,
    /// The (lo_hz, hi_hz) the stored rows were drawn against — a width change
    /// invalidates them outright rather than trying to rescale history drawn
    /// at a different span.
    window: Option<(f64, f64)>,
    /// The last correction actually written: old offset, new offset, and the
    /// wall-clock second it was applied, so the operator sees what happened
    /// even after the numbers above have moved on.
    applied: Option<(f64, f64, i64)>,
}

fn fmt_hz_signed(hz: f64) -> String {
    if hz.abs() >= 1000.0 { format!("{:+.2} kHz", hz / 1000.0) } else { format!("{hz:+.0} Hz") }
}

/// New [`sdroxide_types::RadioConfig::converter_offset_hz`] that puts the
/// beacon — currently read at `measured_hz` where it should read `target_hz`
/// — exactly on `target_hz`, for the same physical LNB and receiver.
///
/// Derived from the converter's own convention
/// (`sdroxide_radio::converter_open_hz`: `hardware_hz = dial_hz + offset`, so
/// `dial_hz = hardware_hz - offset`). The same physical signal read under two
/// offsets gives `measured_hz - target_hz = new_offset - old_offset`.
pub(in crate::app) fn corrected_offset_hz(
    old_offset_hz: f64,
    measured_hz: f64,
    target_hz: f64,
) -> f64 {
    old_offset_hz + (measured_hz - target_hz)
}

/// The bin range of `frame` overlapping `[lo_hz, hi_hz)`, clamped to what the
/// frame actually covers. `None` when the frame has no bins, no span, or does
/// not reach the requested window at all.
fn bin_range(frame: &SpectrumFrame, lo_hz: f64, hi_hz: f64) -> Option<std::ops::Range<usize>> {
    let n = frame.bins.len();
    if n == 0 || frame.span_hz <= 0.0 {
        return None;
    }
    let flo = frame.center_hz - frame.span_hz / 2.0;
    let fhi = frame.center_hz + frame.span_hz / 2.0;
    if hi_hz <= flo || lo_hz >= fhi {
        return None;
    }
    let bin_hz = frame.span_hz / n as f64;
    let b0 = (((lo_hz.max(flo) - flo) / bin_hz).floor() as isize).clamp(0, n as isize - 1) as usize;
    let b1 = (((hi_hz.min(fhi) - flo) / bin_hz).ceil() as isize).clamp(1, n as isize) as usize;
    (b1 > b0).then_some(b0..b1)
}

/// Fold a new frame into the history, resampled onto [`COLS`] against the
/// fixed `(lo, hi)` window. A width change clears the history outright rather
/// than rescaling it — cheap, and correct, since nothing here promises a
/// scrolling record of *other* widths.
fn push_row(win: &mut Qo100WinState, frame: &SpectrumFrame, lo: f64, hi: f64) {
    if frame.seq == win.last_seq && win.window == Some((lo, hi)) {
        return;
    }
    if win.window != Some((lo, hi)) {
        win.rows.clear();
        win.window = Some((lo, hi));
    }
    win.last_seq = frame.seq;

    let flo = frame.center_hz - frame.span_hz / 2.0;
    let fhi = frame.center_hz + frame.span_hz / 2.0;
    let n = frame.bins.len();
    let mut row = vec![0u8; COLS];
    if n > 0 && frame.span_hz > 0.0 && hi > flo && lo < fhi {
        let bin_hz = frame.span_hz / n as f64;
        for (c, slot) in row.iter_mut().enumerate() {
            let x0 = lo + (hi - lo) * c as f64 / COLS as f64;
            let x1 = lo + (hi - lo) * (c + 1) as f64 / COLS as f64;
            let (ox0, ox1) = (x0.max(flo), x1.min(fhi));
            if ox1 <= ox0 {
                continue; // this column falls outside what the receiver currently covers
            }
            let b0 = (((ox0 - flo) / bin_hz).floor() as isize).clamp(0, n as isize - 1) as usize;
            let b1 = (((ox1 - flo) / bin_hz).ceil() as isize).clamp(1, n as isize) as usize;
            if b1 > b0 {
                *slot = frame.bins[b0..b1].iter().copied().max().unwrap_or(0);
            }
        }
    }
    win.rows.push_back(row);
    while win.rows.len() > ROWS {
        win.rows.pop_front();
    }
    win.tex = None; // rebuilt on the next draw
}

fn texture<'a>(
    win: &'a mut Qo100WinState,
    ctx: &egui::Context,
    palette: usize,
) -> Option<&'a TextureHandle> {
    if win.tex.is_none() && !win.rows.is_empty() {
        let lut = colormap::lut(palette);
        // Newest row first, so the strip scrolls downward like every other
        // waterfall in the app.
        let mut pixels = Vec::with_capacity(COLS * win.rows.len());
        for row in win.rows.iter().rev() {
            pixels.extend(row.iter().map(|v| {
                let i = *v as usize * 4;
                Color32::from_rgb(lut[i], lut[i + 1], lut[i + 2])
            }));
        }
        let img = ColorImage::new([COLS, win.rows.len()], pixels);
        win.tex = Some(ctx.load_texture("qo100-waterfall", img, egui::TextureOptions::LINEAR));
    }
    win.tex.as_ref()
}

/// Draw the strip: the target line, and — while the decoder has a lock —
/// where it actually found the beacon. Purely visual; nothing here is read
/// back, unlike the version of this window that used to hunt a magnitude
/// peak in it (see the module doc for why that never worked on this signal).
fn paint_strip(
    ui: &mut egui::Ui,
    win: &mut Qo100WinState,
    frame: Option<&SpectrumFrame>,
    palette: usize,
    lo: f64,
    hi: f64,
    measured_hz: Option<f64>,
) {
    let (rect, resp) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), STRIP_H), Sense::hover());
    if !ui.is_rect_visible(rect) {
        return;
    }
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, Color32::BLACK);

    if let Some(f) = frame {
        push_row(win, f, lo, hi);
    }
    let ctx = ui.ctx().clone();
    if let Some(tex) = texture(win, &ctx, palette) {
        painter.image(
            tex.id(),
            rect,
            Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );
    }

    let x_of = |hz: f64| -> f32 {
        rect.left() + ((hz - lo) / (hi - lo)).clamp(0.0, 1.0) as f32 * rect.width()
    };
    // The target: where the beacon belongs.
    let tx = x_of(QO100_BEACON_HZ);
    painter.line_segment(
        [Pos2::new(tx, rect.top()), Pos2::new(tx, rect.bottom())],
        Stroke::new(1.0, theme::CYAN()),
    );
    // Where the decoder actually locked, while that lock is still fresh.
    if let Some(m) = measured_hz
        && (lo..=hi).contains(&m)
    {
        let mx = x_of(m);
        painter.line_segment(
            [Pos2::new(mx, rect.top()), Pos2::new(mx, rect.bottom())],
            Stroke::new(1.6, theme::GREEN()),
        );
    }

    let font = egui::FontId::monospace(9.0);
    let dim = Color32::from_white_alpha(200);
    painter.text(
        Pos2::new(rect.left() + 3.0, rect.top() + 1.0),
        egui::Align2::LEFT_TOP,
        format!("{:.3} MHz", lo / 1e6),
        font.clone(),
        dim,
    );
    painter.text(
        Pos2::new(rect.right() - 3.0, rect.top() + 1.0),
        egui::Align2::RIGHT_TOP,
        format!("{:.3} MHz", hi / 1e6),
        font,
        dim,
    );
    crate::chrome::paint_cut_border(&painter, rect, theme::LINE_LIT(), theme::PANEL());
    let _ = resp; // hover-only; kept so `ui.allocate_exact_size` reserves layout the usual way
}

impl SdroxideApp {
    pub(in crate::app) fn qo100_window(&mut self, ctx: &egui::Context, cmds: &mut Vec<Command>) {
        if !self.show_qo100 {
            return;
        }
        let mut win = std::mem::take(&mut self.qo100_win);
        let mut open = self.show_qo100;
        let resp = egui::Window::new("QO-100 BEACON")
            .id(crate::layout::salted_id(ctx, "Qo100"))
            .open(&mut open)
            .frame(crate::chrome::window_frame())
            .resizable(true)
            .default_width(crate::layout::window_w(ctx, 440.0))
            .show(ctx, |ui| {
                crate::chrome::window_body_bg(ui);
                self.qo100_body(ui, &mut win, cmds)
            });
        if let Some(r) = &resp {
            crate::chrome::paint_window_border(ctx, &r.response);
        }
        self.show_qo100 = open;
        self.qo100_win = win;
    }

    fn qo100_body(&mut self, ui: &mut egui::Ui, win: &mut Qo100WinState, cmds: &mut Vec<Command>) {
        // `may_rx_hz`, not `can_rx_hz`: a driver that publishes no tuning
        // ranges (SoapySDR makes `getFrequencyRange` optional, and plenty of
        // backends skip it) has not said it *cannot* reach the beacon. Gating
        // on `can_rx_hz` there greys ON out on a correctly-set-up LNB station
        // and tells it to configure a converter offset it already has.
        let reachable = self.caps.as_ref().is_none_or(|c| c.may_rx_hz(QO100_BEACON_HZ));
        let frame = self.frame.clone();
        // Edited in place and sent whole on any change, the same convention
        // the ISM window follows for `IsmSettings` — the engine persists this
        // and echoes it back, so there is no separate apply step.
        let mut cfg = self.state.qo100;
        let (lo, hi) = (
            QO100_BEACON_HZ - cfg.search_half_width_hz,
            QO100_BEACON_HZ + cfg.search_half_width_hz,
        );
        // Whether whatever the receiver is *actually* capturing right now
        // reaches anywhere near the beacon at all — as against `reachable`,
        // which asks whether it ever could. A capable receiver still shows a
        // blank strip while it is parked on 144 MHz, and that is the second
        // most likely reason (after no converter at all) this window opens on
        // an empty waterfall. This only judges the *mini waterfall*'s own
        // picture — the decoder itself reads the raw IQ straight from the
        // hardware and does not depend on the main dial being anywhere near
        // the beacon at all, only on the beacon being inside what the
        // hardware actually captures.
        let in_view = frame.as_deref().is_some_and(|f| bin_range(f, lo, hi).is_some());
        let dial_hz = self.state.active_freq_hz();
        let status = self.qo100_status.clone();

        ui.horizontal(|ui| {
            let run = crate::chrome::chip_enabled(
                ui,
                reachable,
                cfg.enabled,
                if cfg.enabled { "ON" } else { "OFF" },
            );
            if run.clicked() {
                cfg.enabled = !cfg.enabled;
                if cfg.enabled {
                    // Visual convenience only — the decoder reads raw IQ
                    // straight off the hardware and works regardless of
                    // where the main dial happens to be, as long as the
                    // beacon is inside what the hardware actually captures.
                    cmds.push(Command::SetVfo { vfo: Vfo::A, hz: QO100_BEACON_HZ });
                }
            }
            if !reachable {
                run.on_hover_text(
                    "This receiver cannot reach 10489.750 MHz on its own — set up an \
                     LNB/converter offset first (Settings ▸ Radio)",
                );
            } else {
                run.on_hover_text(
                    "Demodulate the beacon's own BPSK-400 telemetry and measure exactly how far \
                     it sits from 10489.750 MHz",
                );
            }

            ui.add_space(8.0);
            ui.label(RichText::new("width").size(10.0).color(theme::CYAN_DIM()));
            if ui.small_button("−").on_hover_text("Narrower — search a smaller slice").clicked()
            {
                cfg.search_half_width_hz =
                    (cfg.search_half_width_hz - WIDTH_STEP_HZ).max(MIN_HALF_WIDTH_HZ);
            }
            ui.label(
                RichText::new(format!("±{:.0} kHz", cfg.search_half_width_hz / 1000.0))
                    .size(11.0)
                    .monospace(),
            );
            if ui
                .small_button("+")
                .on_hover_text("Wider — for when the beacon isn't found at the current width")
                .clicked()
            {
                cfg.search_half_width_hz =
                    (cfg.search_half_width_hz + WIDTH_STEP_HZ).min(MAX_HALF_WIDTH_HZ);
            }
        });

        // Unmissable, not just a hover: a fresh station has no converter set
        // up yet, so ON stays disabled and the strip would otherwise sit
        // there blank with no clue why — the single most likely reason
        // anyone ever opens this window and sees nothing.
        if !reachable {
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new(
                        "This radio's own tuning range doesn't reach 10489.750 MHz — it needs an \
                         LNB/converter offset (Settings ▸ Radio ▸ Converter) before this window can \
                         do anything.",
                    )
                    .size(10.5)
                    .color(theme::YELLOW()),
                );
                if ui.small_button("Open Settings ▸ Radio").clicked() {
                    self.open_radio_settings();
                }
            });
        }

        // The receiver *can* reach the beacon but currently is not — parked
        // on some other band, most often. Only the *mini waterfall* has
        // nothing to show either way; the decoder itself is unaffected (see
        // `in_view`'s own doc), so this is about the picture, not the search.
        if reachable && !in_view {
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new(format!(
                        "The receiver is currently listening around {:.6} MHz — nowhere near the \
                         10489.750 MHz beacon, so there is nothing here to draw. The decoder keeps \
                         searching regardless.",
                        dial_hz / 1e6
                    ))
                    .size(10.5)
                    .color(theme::YELLOW()),
                );
                if ui.small_button("Tune to 10489.750 MHz").clicked() {
                    cmds.push(Command::SetVfo { vfo: Vfo::A, hz: QO100_BEACON_HZ });
                }
            });
        }

        if let Some(f) = self.frame.as_ref() {
            let capped = f.span_hz / 2.0 < cfg.search_half_width_hz;
            if capped && f.span_hz > 0.0 {
                ui.label(
                    RichText::new(format!(
                        "receiver currently covers only ±{:.0} kHz here — the rest of the strip stays blank",
                        f.span_hz / 2e3
                    ))
                    .size(9.5)
                    .color(theme::YELLOW()),
                );
            }
        }

        ui.add_space(4.0);
        let measured_hz = locked_freq(status.as_ref());
        paint_strip(
            ui,
            win,
            frame.as_deref(),
            self.ui_settings.waterfall_palette,
            lo,
            hi,
            measured_hz,
        );

        ui.add_space(6.0);
        ui.label(
            RichText::new(status_line(cfg.enabled, status.as_ref(), self.ctrl.engine_is_remote()))
                .size(9.5)
                .color(theme::CYAN_DIM()),
        );

        let radio_cfg = self.ctrl.radio_config();
        let old_offset = radio_cfg.as_ref().map(|c| c.converter_offset_hz).unwrap_or(0.0);

        egui::Grid::new("qo100-grid").num_columns(2).spacing([16.0, 3.0]).show(ui, |ui| {
            let dim = |s: &str| RichText::new(s).size(9.5).color(theme::CYAN_DIM());
            // What the receiver is actually listening to right now — the
            // direct answer to "why is the strip empty", always on screen
            // rather than only when it explains a problem.
            ui.label(dim("RECEIVER"));
            ui.label(
                RichText::new(format!("{:.6} MHz", dial_hz / 1e6))
                    .size(11.0)
                    .monospace()
                    .color(if in_view { theme::TEXT() } else { theme::YELLOW() }),
            );
            ui.end_row();

            ui.label(dim("TARGET"));
            ui.label(
                RichText::new(format!("{:.6} MHz", QO100_BEACON_HZ / 1e6)).size(12.0).monospace(),
            );
            ui.end_row();

            ui.label(dim("MEASURED"));
            match measured_hz {
                Some(hz) => ui.label(
                    RichText::new(format!("{:.6} MHz", hz / 1e6)).size(12.0).monospace().strong(),
                ),
                None => ui.label(
                    RichText::new(if cfg.enabled { "not locked yet" } else { "—" })
                        .size(11.0)
                        .color(theme::CYAN_DIM()),
                ),
            };
            ui.end_row();

            ui.label(dim("DRIFT"));
            match measured_hz {
                Some(hz) => {
                    let err = hz - QO100_BEACON_HZ;
                    let colour = if err.abs() < 200.0 {
                        theme::GREEN()
                    } else if err.abs() < 3000.0 {
                        theme::YELLOW()
                    } else {
                        theme::TEXT()
                    };
                    ui.label(RichText::new(fmt_hz_signed(err)).size(12.0).monospace().color(colour))
                }
                None => ui.label(RichText::new("—").size(11.0).color(theme::CYAN_DIM())),
            };
            ui.end_row();

            ui.label(dim("CONVERTER OFFSET"));
            ui.label(RichText::new(format!("{old_offset:.0} Hz")).size(11.0).monospace());
            ui.end_row();
        });

        // The decoded telemetry text — the beacon's own status report, shown
        // for its own sake and as independent confirmation the decode is
        // real: garbage here despite a "locked" CRC would be a red flag no
        // number above could catch.
        if let Some(s) = status.as_ref().filter(|s| !s.text.is_empty()) {
            ui.add_space(4.0);
            ui.label(RichText::new("TELEMETRY").size(9.5).color(theme::CYAN_DIM()));
            egui::Frame::new().fill(theme::INPUT_BG()).inner_margin(6.0).show(ui, |ui| {
                ui.add(
                    egui::Label::new(
                        RichText::new(&s.text).monospace().size(10.5).color(theme::GREEN()),
                    )
                    .wrap(),
                );
            });
        }

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let confirmed = apply_is_confirmed(status.as_ref());
            let can_apply = measured_hz.is_some() && radio_cfg.is_some() && confirmed;
            let apply = ui.add_enabled(
                can_apply,
                egui::Button::new(RichText::new(" APPLY CORRECTION ").strong()),
            );
            let apply = apply.on_hover_text(if measured_hz.is_some() && !confirmed {
                "Waiting for a second CRC-valid frame before offering to write this — one lock \
                 alone could be a chance match"
            } else {
                "Write the corrected converter/LNB offset and reopen the receiver — a brief \
                 interruption, the same one Settings ▸ Radio ▸ Apply makes"
            });
            if apply.clicked()
                && let (Some(mut c), Some(measured)) = (radio_cfg.clone(), measured_hz)
            {
                let new_offset =
                    corrected_offset_hz(c.converter_offset_hz, measured, QO100_BEACON_HZ);
                c.converter_offset_hz = new_offset;
                self.ctrl.set_radio_config(c.clone());
                self.ctrl.reopen_source();
                self.radio_cfg = Some(c);
                win.applied = Some((old_offset, new_offset, crate::time::now_unix()));
            }
            if let Some((old, new, at)) = win.applied {
                let ago = crate::time::now_unix() - at;
                ui.label(
                    RichText::new(format!(
                        "last applied {ago}s ago: {old:.0} → {new:.0} Hz ({:+.0} Hz)",
                        new - old
                    ))
                    .size(9.5)
                    .color(theme::CYAN_DIM()),
                );
            }
        });

        if cfg != self.state.qo100 {
            cmds.push(Command::SetQo100Config(cfg));
        }
    }
}

/// The beacon's dial-domain frequency while the decoder's lock is still
/// fresh — `None` once it has gone stale (see
/// [`sdroxide_types::Qo100Status::locked`]'s own doc for how long that grace
/// period is) or if the decoder has never locked at all.
fn locked_freq(status: Option<&Qo100Status>) -> Option<f64> {
    status.filter(|s| s.locked).map(|s| QO100_BEACON_HZ + s.offset_hz)
}

/// Whether a measured offset is safe to offer for `APPLY CORRECTION`, which
/// writes it straight into the converter/LNB setting.
///
/// A 32-bit sync word matched within three bit errors, times a 16-bit CRC,
/// turns up by chance roughly once every couple of hours of searching, so a
/// single lock is not enough to act on. The button stays disabled until a
/// second CRC-valid frame lands (`blocks_locked >= 2`) carrying a non-empty
/// decoded payload — a false positive that clears both is vanishingly
/// unlikely, and the operator can still eyeball the TELEMETRY panel.
fn apply_is_confirmed(status: Option<&Qo100Status>) -> bool {
    status.is_some_and(|s| s.blocks_locked >= 2 && !s.text.is_empty())
}

/// The one-line summary under the strip: off, searching (with how many
/// blocks it has tried), or locked — the same "attempted vs. succeeded"
/// distinction `IsmStatus`'s bursts/decodes line exists for, so a search
/// that is running but has not found the beacon yet reads differently from
/// one that never started.
///
/// `remote` is the client-does-not-own-the-engine case: the decoder runs on
/// the station and its status has no path to a remote client yet (see
/// `RadioEvent::Qo100Status`), so rather than sit on "starting…" forever the
/// line says plainly that this readout is local to the receiving station.
fn status_line(enabled: bool, status: Option<&Qo100Status>, remote: bool) -> String {
    if !enabled {
        return String::new();
    }
    if remote {
        return "decoder runs on the receiving station — its readout is not sent to remote clients"
            .to_string();
    }
    match status {
        None => "starting…".to_string(),
        Some(s) if s.locked => {
            format!(
                "locked — {} block{} tried, {} locked",
                s.blocks_tried,
                if s.blocks_tried == 1 { "" } else { "s" },
                s.blocks_locked
            )
        }
        Some(s) if s.blocks_tried == 0 => {
            "searching — the first window fills after about 24 s, then repeats".to_string()
        }
        Some(s) => format!(
            "searching — {} block{} tried, none locked yet",
            s.blocks_tried,
            if s.blocks_tried == 1 { "" } else { "s" }
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_beacon_read_high_needs_the_offset_raised() {
        // Beacon really sits on QO100_BEACON_HZ but reads 5 kHz high — the
        // LNB's LO is 5 kHz low, which the software currently under-subtracts.
        let old = -9_750_000_000.0;
        let measured = QO100_BEACON_HZ + 5_000.0;
        assert_eq!(corrected_offset_hz(old, measured, QO100_BEACON_HZ), old + 5_000.0);
    }

    #[test]
    fn a_beacon_read_low_needs_the_offset_lowered() {
        let old = -9_750_000_000.0;
        let measured = QO100_BEACON_HZ - 1_200.0;
        assert_eq!(corrected_offset_hz(old, measured, QO100_BEACON_HZ), old - 1_200.0);
    }

    #[test]
    fn a_correctly_calibrated_station_gets_the_same_offset_back() {
        let old = -9_750_000_000.0;
        assert_eq!(corrected_offset_hz(old, QO100_BEACON_HZ, QO100_BEACON_HZ), old);
    }

    fn frame_with(center_hz: f64, span_hz: f64, bins: Vec<u8>) -> SpectrumFrame {
        SpectrumFrame {
            seq: 1,
            center_hz,
            span_hz,
            db_floor: -120.0,
            db_ceil: -20.0,
            bins,
            rows: Vec::new(),
            rows_clocked: false,
        }
    }

    #[test]
    fn bin_range_refuses_a_window_the_frame_never_reaches() {
        let frame = frame_with(14_000_000.0, 100_000.0, vec![0u8; 64]);
        assert!(bin_range(&frame, QO100_BEACON_HZ - 5_000.0, QO100_BEACON_HZ + 5_000.0).is_none());
    }

    #[test]
    fn bin_range_clamps_to_what_the_frame_actually_covers() {
        // Requested window straddles the frame's edge; the range returned
        // must stay inside 0..bins.len().
        let frame = frame_with(QO100_BEACON_HZ + 3_000.0, 10_000.0, vec![0u8; 100]);
        let r = bin_range(&frame, QO100_BEACON_HZ - 5_000.0, QO100_BEACON_HZ + 5_000.0)
            .expect("overlaps");
        assert!(r.end <= 100);
    }

    fn status(locked: bool, offset_hz: f64) -> Qo100Status {
        Qo100Status { running: true, locked, offset_hz, ..Default::default() }
    }

    #[test]
    fn locked_freq_reads_off_the_target_plus_the_measured_offset() {
        let s = status(true, 1_234.0);
        assert_eq!(locked_freq(Some(&s)), Some(QO100_BEACON_HZ + 1_234.0));
    }

    #[test]
    fn locked_freq_is_none_when_not_locked_or_absent() {
        assert_eq!(locked_freq(None), None);
        assert_eq!(locked_freq(Some(&status(false, 0.0))), None);
    }

    #[test]
    fn status_line_is_blank_while_switched_off() {
        assert_eq!(status_line(false, Some(&status(true, 0.0)), false), "");
        assert_eq!(status_line(false, None, true), "");
    }

    #[test]
    fn status_line_distinguishes_locked_from_still_searching() {
        assert!(status_line(true, Some(&status(true, 0.0)), false).starts_with("locked"));
        let mut searching = status(false, 0.0);
        searching.blocks_tried = 3;
        assert!(status_line(true, Some(&searching), false).starts_with("searching"));
    }

    #[test]
    fn status_line_tells_a_remote_client_the_readout_is_local() {
        // No status will ever arrive on a remote client (the server drops
        // the event), so the line must not sit on "starting…".
        let line = status_line(true, None, true);
        assert!(line.contains("receiving station"), "{line:?}");
        assert_ne!(line, "starting…");
    }

    #[test]
    fn status_line_before_the_first_window_says_how_long_it_takes() {
        // A search that is running but has not filled a window yet — distinct
        // from one that never started (`None` → "starting…").
        let mut s = status(false, 0.0);
        s.blocks_tried = 0;
        let line = status_line(true, Some(&s), false);
        assert!(line.starts_with("searching"), "{line:?}");
        assert!(line.contains("24 s"), "{line:?}");
    }

    #[test]
    fn apply_stays_disabled_until_a_second_crc_valid_frame_with_text() {
        assert!(!apply_is_confirmed(None), "nothing decoded yet");

        let mut s = status(true, 1_000.0);
        s.blocks_locked = 1;
        s.text = "QO-100 XX".into();
        assert!(!apply_is_confirmed(Some(&s)), "one lock could be a chance match");

        s.blocks_locked = 2;
        s.text = String::new();
        assert!(!apply_is_confirmed(Some(&s)), "a blank payload is not a confirmation");

        s.text = "QO-100 XX".into();
        assert!(apply_is_confirmed(Some(&s)), "two locks and real text: safe to offer");
    }
}
