//! Which layout the window wears, and the handful of metrics that follow from
//! it: how big a chip has to be to hit with a finger, how far from a filter
//! edge counts as grabbing it, how large the frequency digits may grow.
//!
//! The UI is the same immediate-mode code natively and in the browser, and
//! eframe pins the web zoom factor to 1.0 — so one egui point is one CSS pixel
//! and every hardcoded size in this crate is literally a pixel on a phone. The
//! control strip is eight boxes that reserve a fixed width before they draw
//! (see [`crate::chrome::module_bare_h`]), so on a narrow screen they wrap to
//! a row each and the widest still overflow. That is what the tiers exist to
//! fix.

use eframe::egui;
use sdroxide_types::LayoutMode;

/// The layout in force: how much room there is to lay controls out in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tier {
    /// The full eight-module control strip.
    Desktop,
    /// Frequency readout and S-meter at full size, everything else in menus.
    /// On a screen too short for those two rows (see [`short_tablet`]) the
    /// bar collapses further, to the single-row strip.
    Tablet,
    /// Compact readout and meter, menus, and the waterfall alone below.
    Phone,
}

impl Tier {
    /// Menus instead of the full module strip.
    pub fn compact(self) -> bool {
        self != Tier::Desktop
    }

    /// Finger-sized targets, no hover to rely on, no popups that fade by
    /// themselves. True for tablets as well as phones: both are touched.
    pub fn touch(self) -> bool {
        self != Tier::Desktop
    }

    /// The waterfall and nothing else — no spectrum line, no full-band strip.
    /// A spectrum trace in a 360 pt-wide window is a strip too thin to read
    /// that costs the waterfall a third of its height.
    pub fn waterfall_only(self) -> bool {
        self == Tier::Phone
    }

    /// How far from a filter edge or a marker still counts as grabbing it.
    /// A mouse lands where it is pointed; a fingertip covers about 9 mm.
    pub fn grab_px(self) -> f32 {
        match self {
            Tier::Desktop => 6.0,
            Tier::Tablet => 11.0,
            Tier::Phone => 13.0,
        }
    }

    /// Largest frequency-readout digit this tier will use. The readout is sized
    /// to the space it is given (see `freq_module`); this only caps it, so a
    /// phone in landscape doesn't spend its width on 40 pt digits.
    pub fn digit_cap(self) -> f32 {
        match self {
            Tier::Desktop | Tier::Tablet => crate::widgets::freq_display::DIGIT_SIZE,
            Tier::Phone => 30.0,
        }
    }
}

/// The tier a viewport of `size` earns, with the operator's override folded in.
///
/// Phone below 600 wide because the frequency readout alone measures ~512:
/// under 600 something is clipped however the strip wraps. Phone below 440
/// *tall* as well, for a phone in landscape — 852×393 or 932×430 is wide
/// enough for a desktop row, but a 150 pt stack of control strip would take
/// 40% of the height. No tablet lands there: every one of them is 768 tall or
/// more in landscape.
///
/// Tablet up to 1400 wide. The condensed desktop strip packs into two balanced
/// rows from roughly 1250 pt of content width (one row from ~2450), so 1400 is
/// no longer forced by overflow the way it was when the strip needed 2600 for
/// a single row — but a row of menus still reads better than a wall of boxes
/// on a small screen, and the breakpoint also guards the touch metrics, so
/// lowering it is a separate decision from the strip's packing. (768 portrait,
/// the tightest tablet, leaves 732 pt of content width once the top panel's
/// 8+8 margin and `angled_frame`'s 10+10 are off it, while the full-size
/// readout and S-meter together want 770.)
pub fn tier_for(size: egui::Vec2, mode: LayoutMode) -> Tier {
    match mode {
        LayoutMode::Desktop => Tier::Desktop,
        LayoutMode::Tablet | LayoutMode::Small => Tier::Tablet,
        LayoutMode::Phone => Tier::Phone,
        LayoutMode::Auto => {
            let (w, h) = (size.x, size.y);
            if w < 600.0 || h < 440.0 {
                Tier::Phone
            } else if w < 1400.0 || h < 620.0 {
                Tier::Tablet
            } else {
                Tier::Desktop
            }
        }
    }
}

/// Below this height a tablet-tier viewport is "short": the tablet top bar —
/// a full-width frequency box stacked over a row of meter and menu chips,
/// some 170 pt of chrome — would take a quarter of a 1280x720 panel before
/// the waterfall got anything. Short viewports get the single-row strip
/// instead; everything taller keeps the stacked layout and its full-size
/// readout. Between 720 (the screen the strip was asked for) and 768 (the
/// most common laptop height, which has the room for the stack).
pub const SHORT_H: f32 = 740.0;

/// Whether the tablet tier's top bar should be the single-row strip rather
/// than the stacked layout — see [`SHORT_H`]. Pure function of the size and
/// override so it can be tested beside [`tier_for`].
pub fn short_tablet_for(size: egui::Vec2, mode: LayoutMode) -> bool {
    // `Small` is the tablet tier with this said out loud: it exists for a
    // screen that fits the stacked bar and cannot afford it.
    mode == LayoutMode::Small || (tier_for(size, mode) == Tier::Tablet && size.y < SHORT_H)
}

/// Whether the operating panels should pull their chips and spacing in.
///
/// The screen this answers is 1366×768: wide enough that the tablet tier fits
/// and short enough that the panel and the waterfall are fighting over about
/// six hundred points. Desktop-sized chips then spend two rows of that on the
/// controls above a decode list with nothing left to list (issue #211).
///
/// A question about *height*, not about the tier, and deliberately a different
/// threshold from [`SHORT_H`]: that one asks whether the stacked top bar fits,
/// and at 768 it does. This asks what is left underneath it once it has, and at
/// 768 that is about six hundred points for a waterfall and an operating panel
/// both. A tall narrow window — a tablet in portrait, a docked column — is the
/// tablet tier with height to spare and is not tight.
///
/// Always true under [`LayoutMode::Small`], which is the operator saying so on
/// a screen of any size, and always on a phone.
pub fn tight_for(size: egui::Vec2, mode: LayoutMode) -> bool {
    mode == LayoutMode::Small
        || tier_for(size, mode) == Tier::Phone
        || (tier_for(size, mode) != Tier::Desktop && size.y < TIGHT_H)
}

/// Below this height an operating panel pulls its chips in — see
/// [`tight_for`]. Above 768 so the commonest small laptop is caught, below the
/// 900-odd a 1080 screen leaves a maximised window, so a full desktop is not.
pub const TIGHT_H: f32 = 820.0;

/// Where the tight flag lives between [`set_tight`] and [`tight`].
fn tight_id() -> egui::Id {
    egui::Id::new("sdroxide-tight")
}

/// Publish [`tight_for`] for this frame, beside [`set_tier`] and for the same
/// reason: the panels that read it are too deep to thread it into.
pub fn set_tight(ctx: &egui::Context, tight: bool) {
    ctx.data_mut(|d| d.insert_temp(tight_id(), tight));
}

/// Whether this frame is drawing a small screen — see [`tight_for`].
pub fn tight(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp(tight_id()))
        .unwrap_or_else(|| tight_for(ctx.content_rect().size(), LayoutMode::Auto))
}

/// Pull a `Ui`'s chip padding and spacing in where the screen is small.
///
/// One call at the top of the operating-panel area rather than a `tight` test
/// at every chip: the panels are thousands of lines of `chrome::chip`, and a
/// switch at each of them would be missed at the next one added. egui sizes a
/// button from its padding, so this shrinks every control below it at once —
/// about eight points a row, which over the four rows of chips an FT8 panel
/// carries is a decode list's worth of height.
pub fn tighten(ui: &mut egui::Ui) {
    // A phone's panel is already one pane at a time and has no rows to save;
    // shrinking there would only cost the fingertip its target.
    if !tight(ui.ctx()) || tier(ui.ctx()) == Tier::Phone {
        return;
    }
    let s = ui.spacing_mut();
    // Vertical only, and `interact_size` untouched: what a short screen is
    // short of is height, and the tier's minimum target size is what keeps a
    // chip hittable on a touched one. Taking the padding out of a row is worth
    // about eight points; taking the target out of it is worth a bug report.
    s.button_padding.y = s.button_padding.y.min(2.0);
    s.item_spacing.y = s.item_spacing.y.min(3.0);
}

/// Where the short-bar decision lives between [`set_short_tablet`] and
/// [`short_tablet`].
fn short_id() -> egui::Id {
    egui::Id::new("sdroxide-short-tablet")
}

/// Publish [`short_tablet_for`] for this frame, beside [`set_tier`].
///
/// Published rather than recomputed, for the reason the tier is: the operator's
/// override lives in settings the context cannot see, and a `Small screen`
/// asked for on a tall display would otherwise be measured back to "not short"
/// at the one place that draws the bar.
pub fn set_short_tablet(ctx: &egui::Context, short: bool) {
    ctx.data_mut(|d| d.insert_temp(short_id(), short));
}

/// Whether this frame's top bar is the single-row strip — see
/// [`short_tablet_for`].
pub fn short_tablet(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp(short_id()))
        .unwrap_or_else(|| tier(ctx) == Tier::Tablet && ctx.content_rect().height() < SHORT_H)
}

/// Where the tier lives between [`set_tier`] and [`tier`].
fn tier_id() -> egui::Id {
    egui::Id::new("sdroxide-tier")
}

/// Publish the tier for this frame. Called once, at the top of the app's
/// `update`, because the operator's override lives in `UiSettings` — which the
/// context cannot see and the widgets that need the tier cannot reach.
pub fn set_tier(ctx: &egui::Context, tier: Tier) {
    ctx.data_mut(|d| d.insert_temp(tier_id(), tier));
}

/// The tier in force this frame.
///
/// Read from context memory rather than passed as an argument: the widgets that
/// need it are as deep as [`crate::widgets::spectrum_view::show_ext`], which
/// already takes nineteen parameters, and every one of them already holds a
/// `Context`. The fallback covers the frame before the first [`set_tier`] and
/// the solar view, which has no settings of its own.
pub fn tier(ctx: &egui::Context) -> Tier {
    ctx.data(|d| d.get_temp(tier_id()))
        .unwrap_or_else(|| tier_for(ctx.content_rect().size(), LayoutMode::Auto))
}

/// Where the radio-id salt lives between [`set_radio_salt`] and [`salted_id`].
fn radio_salt_id() -> egui::Id {
    egui::Id::new("sdroxide-radio-salt")
}

/// Publish which radio's UI the frame is drawing. Split view draws several
/// complete apps in one frame, so every *fixed* egui id — floating windows,
/// the top-bar panel, the frequency readout — must differ per radio or the
/// instances share state and egui reports id clashes. Same publication
/// pattern as [`set_tier`], and for the same reason: the sites that mint
/// those ids are too deep to thread another argument into.
pub fn set_radio_salt(ctx: &egui::Context, radio_id: u32) {
    ctx.data_mut(|d| d.insert_temp(radio_salt_id(), radio_id));
}

/// The radio whose UI is being drawn right now (0 outside a multi-radio
/// session, and for the station radio).
pub fn radio_salt(ctx: &egui::Context) -> u32 {
    ctx.data(|d| d.get_temp::<u32>(radio_salt_id())).unwrap_or(0)
}

/// A fixed id, kept distinct per radio. Radio 0 keeps the historical unsalted
/// id — the same bargain `view_key` strikes — so an existing installation
/// keeps its saved window placements.
pub fn salted_id(ctx: &egui::Context, name: &str) -> egui::Id {
    match radio_salt(ctx) {
        0 => egui::Id::new(name),
        n => egui::Id::new((name, n)),
    }
}

/// A window width that fits the viewport, with a margin so the cut-corner
/// border stays visible. egui persists window sizes, so without this a keyer
/// opened at 600 pt on a desktop stays 600 pt wide on the phone that later
/// loads the same storage.
pub fn window_w(ctx: &egui::Context, want: f32) -> f32 {
    want.min(ctx.content_rect().width() - 16.0).max(160.0)
}

/// A window height that fits the viewport. See [`window_w`].
pub fn window_h(ctx: &egui::Context, want: f32) -> f32 {
    want.min(ctx.content_rect().height() - 16.0).max(120.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::vec2;

    #[test]
    fn viewports_land_in_the_tier_they_were_measured_for() {
        let auto = LayoutMode::Auto;
        // Phones, portrait and landscape.
        assert_eq!(tier_for(vec2(360.0, 800.0), auto), Tier::Phone);
        assert_eq!(tier_for(vec2(393.0, 852.0), auto), Tier::Phone);
        assert_eq!(tier_for(vec2(412.0, 915.0), auto), Tier::Phone);
        assert_eq!(tier_for(vec2(852.0, 393.0), auto), Tier::Phone, "landscape phone is short");
        assert_eq!(tier_for(vec2(932.0, 430.0), auto), Tier::Phone);
        // Tablets, both ways up — and the laptop-sized windows that get the
        // same treatment, because the full strip would wrap over three rows.
        assert_eq!(tier_for(vec2(768.0, 1024.0), auto), Tier::Tablet);
        assert_eq!(tier_for(vec2(1024.0, 768.0), auto), Tier::Tablet);
        assert_eq!(tier_for(vec2(820.0, 1180.0), auto), Tier::Tablet);
        assert_eq!(tier_for(vec2(1280.0, 800.0), auto), Tier::Tablet);
        // Desktops.
        assert_eq!(tier_for(vec2(1440.0, 900.0), auto), Tier::Desktop);
        assert_eq!(tier_for(vec2(1920.0, 1080.0), auto), Tier::Desktop);
    }

    #[test]
    fn only_short_tablet_viewports_get_the_single_row_strip() {
        let auto = LayoutMode::Auto;
        // The screen the strip was asked for, and a squat ultrawide window.
        assert!(short_tablet_for(vec2(1280.0, 720.0), auto));
        assert!(short_tablet_for(vec2(1920.0, 600.0), auto));
        // Mid-sized viewports keep the stacked tablet layout.
        assert!(!short_tablet_for(vec2(1280.0, 800.0), auto), "800 pt tall is mid-sized");
        assert!(!short_tablet_for(vec2(1024.0, 768.0), auto), "a 768 line laptop has the room");
        assert!(!short_tablet_for(vec2(768.0, 1024.0), auto), "portrait tablet");
        // The other tiers have layouts of their own, however short.
        assert!(!short_tablet_for(vec2(852.0, 393.0), auto), "landscape phone");
        assert!(!short_tablet_for(vec2(1920.0, 1080.0), auto), "desktop");
    }

    #[test]
    fn the_override_wins_over_the_viewport() {
        let big = vec2(1920.0, 1080.0);
        assert_eq!(tier_for(big, LayoutMode::Phone), Tier::Phone);
        assert_eq!(tier_for(big, LayoutMode::Tablet), Tier::Tablet);
        let small = vec2(360.0, 800.0);
        assert_eq!(tier_for(small, LayoutMode::Desktop), Tier::Desktop);
    }

    /// The screen issue #211 is about: 1366×768 fits the tablet tier and
    /// cannot afford its chrome, so `Auto` already pulls the panels in — and
    /// `Small screen` says so on any size, for an operator running a window
    /// rather than a whole screen.
    #[test]
    fn a_small_screen_is_tight_whether_it_was_asked_for_or_measured() {
        let small = egui::vec2(1366.0, 768.0);
        assert_eq!(tier_for(small, LayoutMode::Auto), Tier::Tablet, "it is still a tablet");
        assert!(tight_for(small, LayoutMode::Auto), "…and a short one");
        // The stacked bar still fits at 768 and is still worth having (see
        // `SHORT_H`); it is what is *under* it that has to be pulled in.
        assert!(!short_tablet_for(small, LayoutMode::Auto));
        // A window rather than a whole screen — a title bar and a taskbar off
        // it — loses the stack as well.
        assert!(short_tablet_for(egui::vec2(1366.0, 700.0), LayoutMode::Auto));

        // A full desktop is not, however the operator sizes their window.
        let desktop = egui::vec2(2560.0, 1440.0);
        assert!(!tight_for(desktop, LayoutMode::Auto));
        assert!(!tight_for(desktop, LayoutMode::Desktop));
        // Forced, it is tight on any screen — the point of the setting.
        assert!(tight_for(desktop, LayoutMode::Small));
        assert!(short_tablet_for(desktop, LayoutMode::Small));
        assert_eq!(tier_for(desktop, LayoutMode::Small), Tier::Tablet);
        // …and the tablet setting alone is not: an operator who picked that
        // asked for the menus, not for smaller chips.
        assert!(!tight_for(desktop, LayoutMode::Tablet));

        // A phone is tighter than either and stays that way.
        assert!(tight_for(egui::vec2(390.0, 844.0), LayoutMode::Auto));
        assert!(tight_for(desktop, LayoutMode::Phone));
    }
}
