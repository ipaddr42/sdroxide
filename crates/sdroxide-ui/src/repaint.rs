//! Asking for the next frame at a rate that is the rate asked for.
//!
//! [`egui::Context::request_repaint_after`] does not schedule a wake-up in the
//! duration handed to it. It first subtracts
//! [`egui::InputState::predicted_dt`] — "make it less likely we over-shoot the
//! target", `egui/src/context.rs` — on the theory that a repaint asked for in
//! `d` should be *on screen* by `d`, so the pass has to start a frame early.
//!
//! That reasoning only holds while `predicted_dt` is a measurement. It is not:
//! neither `eframe` nor `egui-winit` ever writes the field, so it keeps its
//! `RawInput` default of 1/60 s on every native and web build, and the
//! subtraction is a flat 16.67 ms taken off every request. Measured against
//! egui 0.35 (an eleven-line eframe app asking for one fixed delay, X11 and
//! Wayland alike, at both 60 Hz and 144 Hz output):
//!
//! | asked | got |
//! |---|---|
//! | 200 ms | 184 ms |
//! | 66 ms | 50 ms |
//! | 33 ms | 16.5 ms |
//! | ≤ 17 ms | *no delay at all* |
//!
//! The last row is the one that mattered. sdroxide's frame scheduler asks for
//! `1000 / frame_rate_fps` ms, so the default 60 fps asked for 16 ms, the
//! subtraction floored it at zero, and the UI thread stopped being paced at
//! all: it redrew as fast as the machine would let it, which is the definition
//! of one saturated core. On a fast desktop that looked like a harmless 85 fps;
//! on a thin laptop it looked like the whole application living on one core
//! (issue: 8-core Lunar Lake, `sdroxide` pegged at 99 %).
//!
//! So every place that wants a *cadence* — a meter that creeps, a waterfall
//! that scrolls, the frame scheduler itself — goes through here, which adds
//! back exactly what egui is about to take off. Reading `predicted_dt` rather
//! than hard-coding 16.67 ms keeps the compensation exact if egui ever starts
//! filling the field in for real.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use eframe::egui;

/// One frame at the rate the operator chose, in milliseconds. Written once a
/// frame by the app; read by every animation that wants the next frame.
///
/// Starts at the default 60 fps so anything drawn before the first frame sets
/// it — the sign-in screen's globe — is paced rather than free.
static FRAME_PERIOD_MS: AtomicU64 = AtomicU64::new(1000 / 60);

/// Publish the frame period Settings → UI is asking for. Called once per frame,
/// before anything draws.
pub fn set_frame_period_ms(ms: u64) {
    FRAME_PERIOD_MS.store(ms.max(1), Ordering::Relaxed);
}

fn frame_period() -> Duration {
    Duration::from_millis(FRAME_PERIOD_MS.load(Ordering::Relaxed))
}

/// What [`egui::Context::request_repaint_after`] is about to subtract.
/// `predicted_dt` is seconds as `f32` and always positive, so the conversion
/// cannot fail — but a zero on the error path only leaves egui's own behaviour
/// in place.
fn bias(ctx: &egui::Context) -> Duration {
    Duration::try_from_secs_f32(ctx.input(|i| i.predicted_dt)).unwrap_or(Duration::ZERO)
}

/// Ask for the next pass in `delay`, and never sooner than one frame.
///
/// Drop-in for [`egui::Context::request_repaint_after`] everywhere except the
/// frame scheduler itself — see the module docs for why calling egui's version
/// does not do what it says, and [`animate`] for why the floor is here.
pub fn after(ctx: &egui::Context, delay: Duration) {
    ctx.request_repaint_after(delay.max(frame_period()) + bias(ctx));
}

/// [`after`] in whole milliseconds, which is how most callers say it.
pub fn after_ms(ctx: &egui::Context, ms: u64) {
    after(ctx, Duration::from_millis(ms));
}

/// The next frame, for something that is animating: a fade, a scroll, a needle
/// settling, a fling coasting.
///
/// egui's own `request_repaint()` means *immediately*, and immediately is not a
/// rate — a widget that calls it every frame while it animates pins the whole
/// window at whatever the machine can draw, which is how the S-meter needle and
/// the waterfall scroll went on running flat out after the panadapter started
/// honouring Settings → UI. An animation wants the *next* frame, and which
/// frame that is belongs to the frame rate, not to the widget.
pub fn animate(ctx: &egui::Context) {
    after(ctx, Duration::ZERO);
}

/// The frame scheduler's own request: the cadence itself, not something
/// following it, so this is the one call that is not floored at a frame.
///
/// [`crate::app`]'s scheduler already computes the period from the same
/// setting, and shortens it deliberately for a settle timer that has to land
/// between frames.
pub fn schedule(ctx: &egui::Context, delay: Duration) {
    ctx.request_repaint_after(delay + bias(ctx));
}

/// [`schedule`] in whole milliseconds.
pub fn schedule_ms(ctx: &egui::Context, ms: u64) {
    schedule(ctx, Duration::from_millis(ms));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive one headless pass that asks for a repaint, and report the delay
    /// egui actually scheduled for the root viewport.
    fn scheduled(ask: impl Fn(&egui::Context)) -> Duration {
        let ctx = egui::Context::default();
        // A fresh context repaints immediately for its first few passes while
        // fonts and layout settle, so run several and read the last — by then
        // the only thing asking for a repaint is `ask`.
        let mut delay = None;
        for _ in 0..4 {
            let out = ctx.run_ui(egui::RawInput::default(), |ui| ask(ui.ctx()));
            delay = out.viewport_output.get(&egui::ViewportId::ROOT).map(|v| v.repaint_delay);
            // Nothing here paints, so the texture deltas go unapplied — and
            // since egui 0.36 dropping them unhandled is a debug assertion.
            out.drop_without_applying_deltas();
        }
        delay.expect("no root viewport output")
    }

    /// The bug this module exists for: egui takes `predicted_dt` — a hard-coded
    /// 1/60 s, since no backend ever fills the field in — off every request.
    /// Pinned here so an egui upgrade that changes it is visible rather than
    /// silent, in either direction: the compensation is written against this.
    #[test]
    fn egui_shortens_a_bare_request_by_one_sixtieth_of_a_second() {
        let got = scheduled(|ctx| ctx.request_repaint_after(Duration::from_millis(33)));
        assert!(
            (16..=18).contains(&got.as_millis()),
            "expected 33 ms minus the 16.7 ms bias, got {got:?}"
        );
        // Anything at or under the bias loses its delay entirely, which is what
        // turned the 60 fps default into an unthrottled render loop.
        let got = scheduled(|ctx| ctx.request_repaint_after(Duration::from_millis(16)));
        assert_eq!(got, Duration::ZERO, "a 16 ms request should be floored to no delay at all");
    }

    #[test]
    fn the_helper_schedules_the_delay_it_was_asked_for() {
        for ms in [16_u64, 33, 66, 200] {
            let got = scheduled(|ctx| after_ms(ctx, ms));
            let got_ms = got.as_millis() as i64;
            assert!((got_ms - ms as i64).abs() <= 1, "asked {ms} ms, egui scheduled {got_ms} ms");
        }
    }
}
