//! Running eframe on an event loop of our own, so a Wayland session does not
//! burn a core doing nothing.
//!
//! eframe drives winit with `ControlFlow`, and it sets `ControlFlow::Poll` —
//! "come straight back, do not sleep" — in exactly one place: the moment it
//! decides a repaint is due, in `check_redraw_requests`
//! (`eframe/src/native/run.rs`). It asks winit for the redraw, drops the
//! window's entry from its pending-repaint map, and because that map is now
//! empty there is no deadline left to turn the control flow back into a
//! `WaitUntil`. Nothing else restores it either: the wrapper eframe runs under
//! `run_native` does not implement `about_to_wait`. The loop is left spinning
//! until the `RedrawRequested` it asked for comes back.
//!
//! On X11 that is the very next iteration, so the spin is invisible. On
//! **Wayland it is not**: winit will not emit `RedrawRequested` while a frame
//! callback is outstanding —
//!
//! ```ignore
//! if window.frame_callback_state() == FrameCallbackState::Requested {
//!     return None;
//! }
//! ```
//!
//! (`winit/src/platform_impl/linux/wayland/event_loop/mod.rs`) — so the redraw
//! waits on the compositor, and the loop hammers `pump_events` for as long as
//! it takes. A compositor that throttles the callbacks of a window it is not
//! showing at full rate — occluded, on another workspace, behind a terminal —
//! withholds them for a long time, and the spin is then more or less
//! permanent. Measured on this tree, one radio on a synthetic source at 60 fps:
//!
//! | session | main thread |
//! |---|---|
//! | X11 | 8 % |
//! | headless wlroots (no real vblank, callbacks never throttled) | 9 % |
//! | a Wayland desktop | **98 %** — 53 % user, 45 % system |
//!
//! and the profile is `winit::…::EventLoop::pump_events`,
//! `pthread_mutex_lock`, `__vdso_clock_gettime`, which is the loop and nothing
//! else. It is unrelated to the frame rate: 5 fps still cost 80 %, because the
//! spin is not per-frame work, it is the wait between frames.
//!
//! So the loop is ours, and [`Paced`] refuses to leave it in `Poll`. The
//! redraw eframe asked for is already queued with winit; a wait does not lose
//! it, and any event — the compositor's frame callback above all — ends the
//! wait immediately. The bound keeps a platform that somehow never delivers
//! the redraw ticking over instead of frozen, and matches the interval eframe
//! itself falls back to for a window it cannot paint.
//!
//! See [`crate::repaint`] for the other half of the frame pacing, which is
//! about how often a frame is *asked* for rather than what happens while
//! waiting for one.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use eframe::UserEvent;
use egui_winit::winit::application::ApplicationHandler;
use egui_winit::winit::event::{DeviceEvent, DeviceId, StartCause, WindowEvent};
use egui_winit::winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use egui_winit::winit::window::WindowId;

/// How long the loop may sleep with a redraw outstanding before it wakes to
/// look again. Only reached when the window system never answers the redraw
/// request at all — normally an event arrives long before this — and it is the
/// same interval eframe uses for a window it cannot paint.
const REDRAW_WAIT: Duration = Duration::from_millis(100);

/// Run `creator`'s app to completion on our own winit loop.
///
/// Drop-in for [`eframe::run_native`], including its behaviour of returning
/// the app-creation error to the caller.
pub fn run(
    app_name: &str,
    mut options: eframe::NativeOptions,
    creator: eframe::AppCreator<'_>,
) -> eframe::Result<()> {
    // eframe's own wrapper keeps the creation error and hands it back from
    // `run_native`; ours cannot reach that field, so the error is caught on
    // the way past instead. `--connect` to a server that is not there is the
    // case that depends on it.
    let failure: Rc<RefCell<Option<String>>> = Rc::default();
    let slot = failure.clone();
    let creator: eframe::AppCreator<'_> = Box::new(move |cc| match creator(cc) {
        Ok(app) => Ok(app),
        Err(e) => {
            *slot.borrow_mut() = Some(e.to_string());
            Err(e)
        }
    });

    // `centered` is a first-start convenience, and eframe applies it *after*
    // it has restored the geometry the last session saved — so leaving it set
    // walks a remembered window back to the middle of the screen on every
    // start (issue #256; the centring itself is issue #234). It is asked for
    // only where there is genuinely nothing remembered.
    if options.centered && remembers_geometry(app_name) {
        options.centered = false;
    }

    let mut builder = EventLoop::<UserEvent>::with_user_event();
    // The hook is part of `NativeOptions`, and building the loop here is what
    // takes it out of eframe's hands — so it has to be applied here too.
    if let Some(hook) = std::mem::take(&mut options.event_loop_builder) {
        hook(&mut builder);
    }
    let event_loop = builder.build().map_err(eframe::Error::WinitEventLoop)?;

    let inner = eframe::create_native(app_name, options, creator, &event_loop);
    let result = event_loop.run_app(&mut Paced { inner });

    if let Some(e) = failure.borrow_mut().take() {
        return Err(eframe::Error::AppCreation(e.into()));
    }
    result.map_err(eframe::Error::WinitEventLoop)
}

/// Whether a previous session left window geometry behind for eframe to
/// restore.
///
/// Read the way eframe reads it — its own storage directory, its own `app.ron`,
/// its own `"window"` key — but only for the key's presence, so nothing here
/// has to agree with `WindowSettings`' layout. Absent, unreadable or
/// unrecognisable all mean "nothing remembered", which is the answer that
/// leaves the window centred; the worst a wrong guess can do is centre a
/// window that had a position, exactly as before this existed.
fn remembers_geometry(app_name: &str) -> bool {
    let Some(dir) = eframe::storage_dir(app_name) else { return false };
    std::fs::read_to_string(dir.join("app.ron")).is_ok_and(|s| s.contains("\"window\""))
}

/// eframe's own handler, with the one thing it does not do: put the loop back
/// to sleep. Every other method is eframe's, unchanged.
struct Paced<'a> {
    inner: eframe::EframeWinitApplication<'a>,
}

impl ApplicationHandler<UserEvent> for Paced<'_> {
    fn about_to_wait(&mut self, el: &ActiveEventLoop) {
        self.inner.about_to_wait(el);
        let now = Instant::now();
        // A deadline already in the past is the same spin by another name:
        // winit returns from the wait immediately and comes straight back.
        // Anything due has already been acted on in `new_events`, above.
        let spinning = match el.control_flow() {
            ControlFlow::Poll => true,
            ControlFlow::WaitUntil(at) => at <= now,
            ControlFlow::Wait => false,
        };
        if spinning {
            el.set_control_flow(ControlFlow::WaitUntil(now + REDRAW_WAIT));
        }
    }

    fn resumed(&mut self, el: &ActiveEventLoop) {
        self.inner.resumed(el);
    }

    fn window_event(&mut self, el: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        self.inner.window_event(el, id, event);
    }

    fn new_events(&mut self, el: &ActiveEventLoop, cause: StartCause) {
        self.inner.new_events(el, cause);
    }

    fn user_event(&mut self, el: &ActiveEventLoop, event: UserEvent) {
        self.inner.user_event(el, event);
    }

    fn device_event(&mut self, el: &ActiveEventLoop, id: DeviceId, event: DeviceEvent) {
        self.inner.device_event(el, id, event);
    }

    fn suspended(&mut self, el: &ActiveEventLoop) {
        self.inner.suspended(el);
    }

    fn exiting(&mut self, el: &ActiveEventLoop) {
        self.inner.exiting(el);
    }

    fn memory_warning(&mut self, el: &ActiveEventLoop) {
        self.inner.memory_warning(el);
    }
}
