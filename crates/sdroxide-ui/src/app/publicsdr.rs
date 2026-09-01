//! The "PUBLIC SDRS" window: browse the receivers other people have published
//! and open one as a radio.
//!
//! Built on the same bones as [`crate::app::spots`], because it is the same
//! problem — a long list from the internet, filter chips, a fuzzy search, and a
//! row that does something when it is clicked.
//!
//! # Where the list comes from
//!
//! Not from here. The window asks
//! [`sdroxide_types::DeviceProbe::PublicSdrs`], which is answered by the
//! machine the radio is attached to — the same lane the device enumerations
//! use. That is what makes this work in a browser: the web client has no HTTP
//! client at all, and could not read either directory across origins if it had.
//! It is also the right end to ask, because it is the end that will hold the
//! connection: a receiver this screen can reach is no use if the station
//! cannot.
//!
//! # Being a guest
//!
//! A receiver that cannot be used is shown greyed with the reason rather than
//! hidden. Two of them matter and neither is obvious from outside: a KiwiSDR
//! whose operator has not opened any channels to non-browser apps will refuse
//! sdroxide however many are free, and a receiver whose channels are all in use
//! is somebody else's for the moment. Being told which is far better than a
//! connection that fails for no visible reason.

use eframe::egui::{self, RichText};
use sdroxide_types::{Command, PublicSdrEntry, PublicSdrNetwork};

use crate::theme::ThemedScroll;
use crate::time::now_unix;

use crate::app::util::fmt_age;
use crate::app::{RadioTabRequest, SdroxideApp};

/// Rows drawn at once. The directories run to about eleven hundred receivers
/// between them and every row is a handful of laid-out labels; past this the
/// list is not a list any more, and the search box is the way through it.
const MAX_ROWS: usize = 300;

/// One receiver row: network badge, name, what it covers, how busy it is,
/// where it is, and the two ways to take it.
///
/// Columns are allocated rather than laid out by content, the way
/// [`crate::app::spots`]'s rows are: a directory is a table, and a name that
/// pushed the frequency column sideways would make it unreadable. The last one
/// takes whatever is left over so a wide window shows more of the place name
/// rather than more empty space.
///
/// Returns what the operator pressed, if anything.
fn entry_row(
    ui: &mut egui::Ui,
    e: &PublicSdrEntry,
    distance_km: Option<f64>,
) -> Option<PickAction> {
    let blocked = e.blocked_reason();
    let mut action = None;
    let dim = crate::theme::gray(if blocked.is_some() { 100 } else { 170 });
    let net_col = match e.network {
        PublicSdrNetwork::KiwiSdr => crate::theme::CYAN(),
        PublicSdrNetwork::SpyServer => crate::theme::PINK(),
    };
    /// What the buttons (or the refusal text) need on the right.
    const ACTIONS_W: f32 = 172.0;
    /// The five fixed columns ahead of the place, and the gaps between them.
    const FIXED_W: f32 = 60.0 + 176.0 + 80.0 + 46.0 + 4.0 * 6.0;
    /// The distance column, which is only drawn when a station grid is set.
    const DISTANCE_W: f32 = 56.0 + 6.0;
    /// Narrower than this and a place name is an ellipsis, which is worth less
    /// than the space. The row's tooltip has it either way.
    const PLACE_MIN_W: f32 = 74.0;

    egui::Frame::new()
        .fill(crate::theme::ROW_BG())
        .inner_margin(egui::Margin { left: 8, right: 6, top: 2, bottom: 2 })
        .show(ui, |ui| {
            let row_w = ui.available_width();
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                let col = |ui: &mut egui::Ui, w: f32, lbl: egui::Label| {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(w, 20.0), egui::Sense::hover());
                    ui.new_child(
                        egui::UiBuilder::new()
                            .max_rect(rect)
                            .layout(egui::Layout::left_to_right(egui::Align::Center)),
                    )
                    .add(lbl);
                };
                col(
                    ui,
                    60.0,
                    egui::Label::new(
                        RichText::new(e.network.label()).size(10.0).strong().color(net_col),
                    ),
                );
                let name_col = if blocked.is_some() {
                    crate::theme::gray(130)
                } else {
                    crate::theme::TEXT_STRONG()
                };
                col(
                    ui,
                    176.0,
                    egui::Label::new(RichText::new(&e.name).size(13.0).color(name_col)).truncate(),
                );
                col(
                    ui,
                    80.0,
                    egui::Label::new(RichText::new(e.range_label()).size(11.0).color(dim)),
                );
                // Users, and the one number that decides whether a receiver is
                // available at all.
                let busy = if e.max_users == 0 {
                    "—".to_string()
                } else {
                    format!("{}/{}", e.users, e.max_users)
                };
                let busy_col = match () {
                    _ if blocked.is_some() => crate::theme::ALERT(),
                    _ if e.max_users > 0 && e.users * 2 >= e.max_users => crate::theme::YELLOW(),
                    _ => crate::theme::GREEN(),
                };
                col(ui, 46.0, egui::Label::new(RichText::new(busy).size(11.0).color(busy_col)));
                // The distance stands in where the operator never wrote a
                // place, which on a SpyServer is most of them — it is the only
                // thing the directory knows about where that receiver is.
                // Its own column rather than appended to the place: a name
                // long enough to truncate would otherwise take the distance
                // with it, and on a SpyServer — where the operator usually
                // wrote no place at all — the distance is the only thing the
                // directory knows about where the receiver is.
                // Only where there is one to show: an operator who has not set
                // a grid would otherwise pay a column's width for a run of
                // blanks, and that width is the place name's.
                if let Some(km) = distance_km {
                    col(
                        ui,
                        56.0,
                        egui::Label::new(
                            RichText::new(if km >= 1000.0 {
                                format!("{:.1}k km", km / 1000.0)
                            } else {
                                format!("{km:.0} km")
                            })
                            .size(11.0)
                            .color(dim),
                        ),
                    );
                }
                // The one column that flexes, from the row's own width — which
                // has to be captured before anything is allocated out of it,
                // because `available_width` here describes what is left of the
                // frame rather than the row.
                //
                // On a narrow window it is dropped rather than squeezed: the
                // fixed columns come to more than a small window has, and a
                // place name rendered as a bare ellipsis is worth less than the
                // space it costs. Widen the window and it comes back.
                let used = FIXED_W + if distance_km.is_some() { DISTANCE_W } else { 0.0 };
                let place_w = (row_w - used - ACTIONS_W).min(340.0);
                if place_w >= PLACE_MIN_W {
                    col(
                        ui,
                        place_w,
                        egui::Label::new(RichText::new(&e.location).size(11.0).color(dim))
                            .truncate(),
                    );
                }

                ui.with_layout(
                    egui::Layout::right_to_left(egui::Align::Center),
                    |ui| match &blocked {
                        Some(why) => {
                            ui.add(
                                egui::Label::new(
                                    RichText::new(why).size(10.0).color(crate::theme::ALERT()),
                                )
                                .truncate(),
                            );
                        }
                        None => {
                            if crate::chrome::chip(ui, false, "+ TAB")
                                .on_hover_text("Open this receiver as a new radio, in its own tab")
                                .clicked()
                            {
                                action = Some(PickAction::NewRadio);
                            }
                            if crate::chrome::chip(ui, false, "USE")
                                .on_hover_text(
                                    "Point *this* radio at the receiver, replacing whatever \
                                     interface it is on now — asked again first, because that \
                                     is a whole radio's worth of setting up",
                                )
                                .clicked()
                            {
                                action = Some(PickAction::ThisRadio);
                            }
                        }
                    },
                );
            });
        })
        .response
        // Everything that did not earn a column of its own: the address to
        // connect to, the antenna, and what the receiver says it is.
        .on_hover_text(format!(
            "{}\n{}\nantenna: {}\n{}",
            e.address,
            e.device,
            if e.antenna.is_empty() { "not stated" } else { &e.antenna },
            match e.snr_db {
                Some(snr) => format!("noise-floor score {snr}"),
                None => format!("up to {:.0} kHz of I/Q", e.max_iq_rate / 1e3),
            },
        ));
    ui.add_space(1.0);
    action
}

/// What a row's buttons asked for.
#[derive(Clone, Copy, PartialEq)]
enum PickAction {
    NewRadio,
    ThisRadio,
}

/// What the operator answered the **USE** confirmation with.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Confirm {
    /// Go ahead: this radio becomes the receiver.
    Replace,
    /// The way out that keeps both — the receiver arrives in a tab of its own
    /// and the radio the operator is on is left alone.
    NewTab,
    Cancel,
}

/// The question **USE** asks before it replaces a configured radio, and the
/// three answers to it.
///
/// Drawn across the top of the list rather than in a window of its own: the
/// row that raised it is still on screen behind, and a second window over a
/// list of eleven hundred rows would have to be dismissed before the operator
/// could look at what they were about to lose.
fn confirm_panel(ui: &mut egui::Ui, blurb: &str) -> Option<Confirm> {
    let mut answer = None;
    crate::chrome::red_panel(ui, |ui| {
        ui.label(RichText::new(blurb).color(crate::theme::TEXT_STRONG()).size(12.0));
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if crate::chrome::chip(ui, false, "REPLACE")
                .on_hover_text("Point the radio you are on at this receiver")
                .clicked()
            {
                answer = Some(Confirm::Replace);
            }
            if crate::chrome::chip(ui, false, "+ TAB INSTEAD")
                .on_hover_text("Open the receiver as another radio and leave this one alone")
                .clicked()
            {
                answer = Some(Confirm::NewTab);
            }
            if crate::chrome::chip(ui, false, "CANCEL").clicked() {
                answer = Some(Confirm::Cancel);
            }
        });
    });
    answer
}

impl SdroxideApp {
    /// Everything the search and the chips let through, ranked.
    fn public_sdr_rows<'a>(
        &self,
        entries: &'a [PublicSdrEntry],
        dial_hz: f64,
    ) -> Vec<(&'a PublicSdrEntry, i32)> {
        let query = self.public_sdr_search.trim();
        let visible: Vec<&PublicSdrEntry> = entries
            .iter()
            .filter(|e| {
                let net_on = match e.network {
                    PublicSdrNetwork::SpyServer => self.public_sdr_nets_shown[0],
                    PublicSdrNetwork::KiwiSdr => self.public_sdr_nets_shown[1],
                };
                net_on
                    && (!self.public_sdr_free_only || e.blocked_reason().is_none())
                    && (!self.public_sdr_in_band || e.covers(dial_hz))
            })
            .collect();
        let mut rows: Vec<(&PublicSdrEntry, i32)> = visible
            .iter()
            .filter_map(|e| crate::fuzzy::score_terms(&e.haystack(), query).map(|s| (*e, s)))
            .collect();
        if !query.is_empty() {
            rows.sort_by_key(|r| std::cmp::Reverse(r.1));
        }
        rows
    }

    /// Browse the public-SDR directories and open one as a radio.
    pub(in crate::app) fn public_sdrs_window(
        &mut self,
        ctx: &egui::Context,
        _cmds: &mut [Command],
    ) {
        if !self.show_public_sdrs {
            // Asked again the next time it opens, so a window reopened after an
            // hour is not showing an hour-old list.
            self.public_sdrs_asked = false;
            return;
        }
        // The cached answer on first open, so the window paints at once; the
        // ⟳ chip is what goes to the network.
        if !self.public_sdrs_asked {
            self.public_sdrs_asked = true;
            self.ask_device(ctx, sdroxide_types::DeviceProbe::PublicSdrs { refresh: false });
        }

        let dial_hz = self.state.active_freq_hz();
        let my_pos = sdroxide_types::grid_to_latlon(&self.my_grid());
        let directory = self.public_sdrs.clone();
        let mut open = self.show_public_sdrs;
        let mut refresh = false;
        let mut picked: Option<(PublicSdrEntry, PickAction)> = None;
        // Worked out before the window borrows `self`: it reads the roster and
        // the radio's own configuration, neither of which the list does.
        let blurb = self.public_sdr_confirm.as_deref().map(|e| self.replace_blurb(e));
        let mut answer: Option<Confirm> = None;

        let resp = egui::Window::new("PUBLIC SDRS")
            .id(crate::layout::salted_id(ctx, "PUBLIC SDRS"))
            .open(&mut open)
            .frame(crate::chrome::window_frame())
            .resizable(true)
            .default_width(crate::layout::window_w(ctx, 980.0))
            .default_height(crate::layout::window_h(ctx, 520.0))
            .show(ctx, |ui| {
                crate::chrome::window_body_bg(ui);
                if let Some(blurb) = &blurb {
                    answer = confirm_panel(ui, blurb);
                    ui.add_space(6.0);
                }
                ui.horizontal(|ui| {
                    for (i, net) in PublicSdrNetwork::ALL.iter().enumerate() {
                        if crate::chrome::chip(ui, self.public_sdr_nets_shown[i], net.label())
                            .clicked()
                        {
                            self.public_sdr_nets_shown[i] = !self.public_sdr_nets_shown[i];
                        }
                    }
                    if crate::chrome::chip(ui, self.public_sdr_free_only, "AVAILABLE")
                        .on_hover_text(
                            "Hide receivers that are full, and the ones whose operator has \
                             not opened any channels to apps other than a browser",
                        )
                        .clicked()
                    {
                        self.public_sdr_free_only = !self.public_sdr_free_only;
                    }
                    if crate::chrome::chip(ui, self.public_sdr_in_band, "IN BAND")
                        .on_hover_text("Only receivers that cover the current dial frequency")
                        .clicked()
                    {
                        self.public_sdr_in_band = !self.public_sdr_in_band;
                    }
                    if crate::chrome::chip(ui, self.public_sdr_low_bw, "LOW BW")
                        .on_hover_text(
                            "Take a SpyServer in its low-bandwidth shape: a narrow I/Q window \
                             that follows the dial plus the server's own band view, instead of \
                             megabits of wideband I/Q. No effect on a KiwiSDR, which has only \
                             the one shape.",
                        )
                        .clicked()
                    {
                        self.public_sdr_low_bw = !self.public_sdr_low_bw;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if crate::chrome::chip(ui, false, "⟳ REFRESH")
                            .on_hover_text("Fetch both directories again")
                            .clicked()
                        {
                            refresh = true;
                        }
                    });
                });
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 5.0;
                    ui.label(RichText::new("⌕").color(crate::theme::CYAN_DIM()).size(14.0));
                    crate::chrome::field(
                        ui,
                        egui::TextEdit::singleline(&mut self.public_sdr_search)
                            .desired_width(240.0)
                            .hint_text("name, place, antenna, band")
                            .text_color(crate::theme::TEXT_STRONG()),
                    );
                    if !self.public_sdr_search.trim().is_empty()
                        && ui.button("✕").on_hover_text("Clear the search").clicked()
                    {
                        self.public_sdr_search.clear();
                    }
                });

                let Some(dir) = &directory else {
                    ui.separator();
                    ui.add_space(8.0);
                    ui.label(
                        RichText::new(if self.probes_answered {
                            "Fetching the receiver lists…"
                        } else {
                            "The machine this radio is attached to does not answer \
                             device questions, so it cannot fetch the lists either."
                        })
                        .color(crate::theme::gray(140)),
                    );
                    return;
                };

                // Age and any source that failed, on one line: a directory that
                // is quietly an hour old looks exactly like a fresh one.
                ui.horizontal_wrapped(|ui| {
                    let age = if dir.fetched_unix > 0 {
                        fmt_age(now_unix() - dir.fetched_unix)
                    } else {
                        "—".to_string()
                    };
                    ui.label(
                        RichText::new(format!(
                            "{} receivers · {} SpyServer · {} KiwiSDR · fetched {age} ago",
                            dir.entries.len(),
                            dir.count(PublicSdrNetwork::SpyServer),
                            dir.count(PublicSdrNetwork::KiwiSdr),
                        ))
                        .size(11.0)
                        .color(crate::theme::gray(150)),
                    );
                    for note in &dir.notes {
                        ui.label(RichText::new(note).size(11.0).color(crate::theme::ALERT()));
                    }
                });
                ui.separator();

                let rows = self.public_sdr_rows(&dir.entries, dial_hz);
                if !self.public_sdr_search.trim().is_empty() || rows.len() > MAX_ROWS {
                    let shown = rows.len().min(MAX_ROWS);
                    let (text, colour) = match rows.len() {
                        0 => ("no match".to_string(), crate::theme::ALERT()),
                        n if n > MAX_ROWS => (
                            format!("showing {shown} of {n} — search to narrow it"),
                            crate::theme::YELLOW(),
                        ),
                        n => (format!("{n} match"), crate::theme::YELLOW()),
                    };
                    ui.label(RichText::new(text).color(colour).size(10.0));
                }

                egui::ScrollArea::vertical().auto_shrink([false, false]).show_themed(ui, |ui| {
                    for (e, _) in rows.iter().take(MAX_ROWS) {
                        let km = match (my_pos, e.lat, e.lon) {
                            (Some(me), Some(lat), Some(lon)) => Some(sdroxide_types::distance_km(
                                me,
                                (f64::from(lat), f64::from(lon)),
                            )),
                            _ => None,
                        };
                        if let Some(a) = entry_row(ui, e, km) {
                            picked = Some(((*e).clone(), a));
                        }
                    }
                    if rows.is_empty() {
                        ui.add_space(8.0);
                        ui.label(
                            RichText::new("nothing matches — try turning a filter chip back on")
                                .color(crate::theme::gray(120)),
                        );
                    }
                });
            });
        if let Some(r) = &resp {
            crate::chrome::paint_window_border(ctx, &r.response);
        }
        self.show_public_sdrs = open;

        if refresh {
            self.public_sdrs = None;
            self.ask_device(ctx, sdroxide_types::DeviceProbe::PublicSdrs { refresh: true });
        }
        // The question, then the answer to it. Closing the window is a "no":
        // an armed confirmation that outlived the list it was raised from
        // would fire on whatever radio the operator had moved to by then.
        if !open {
            self.public_sdr_confirm = None;
        }
        if let Some(choice) = answer {
            // Taken only once an answer is in — a `take` in the pattern would
            // run on every frame the window is drawn and disarm the question
            // before it could be read.
            if let Some(entry) = self.public_sdr_confirm.take() {
                match choice {
                    Confirm::Replace => self.take_public_sdr(&entry, PickAction::ThisRadio),
                    Confirm::NewTab => self.take_public_sdr(&entry, PickAction::NewRadio),
                    Confirm::Cancel => {}
                }
            }
        }
        if let Some((entry, action)) = picked {
            // A fresh press supersedes a question still on screen, whichever
            // row it came from.
            self.public_sdr_confirm = None;
            match action {
                // Nothing to lose only when there is no radio there to lose:
                // an interface never chosen. Anything else — an address typed,
                // a rig dialled, sound cards picked — is asked about first.
                PickAction::ThisRadio if self.radio_is_configured() => {
                    self.public_sdr_confirm = Some(Box::new(entry));
                }
                _ => self.take_public_sdr(&entry, action),
            }
        }
    }

    /// Whether this radio has an interface at all, which is what decides
    /// whether **USE** has anything to destroy.
    ///
    /// Reads the configuration if it has not been read yet: `radio_cfg` is
    /// otherwise filled in only by opening the settings dialog, and whether
    /// USE asks first must not depend on whether anybody has.
    fn radio_is_configured(&mut self) -> bool {
        if self.radio_cfg.is_none() {
            self.radio_cfg = self.ctrl.radio_config();
        }
        self.radio_cfg.as_ref().is_some_and(|c| c.backend != sdroxide_types::Backend::None)
    }

    /// What the confirmation says: which radio is about to become which
    /// receiver, what that costs, and what it does not.
    ///
    /// Written out in full rather than left as "are you sure?", because the
    /// two things an operator gets wrong here are both invisible otherwise —
    /// that the tab goes on carrying its old name, and that the receiver's
    /// published coverage becomes the dial's limit. Both were the report.
    fn replace_blurb(&self, entry: &PublicSdrEntry) -> String {
        let chip = self.radio_roster.iter().find(|c| c.id == self.radio_id);
        let this = match chip {
            Some(c) if !c.name.is_empty() => c.name.clone(),
            Some(c) => c.default_name.clone(),
            None => "this radio".to_string(),
        };
        let iface = self
            .radio_cfg
            .as_ref()
            .map_or_else(|| "its interface".to_string(), |c| c.backend.label().to_string());
        format!(
            "Replace {this} with {}?\n\n\
             {this} is on {iface}. Taking this receiver points the same tab at {} instead, \
             renames it “{}”, and holds the dial to the {} the receiver \
             publishes — nothing here transmits.\n\n\
             The {iface} settings stay where they are, and so do the ranges stated for them: \
             switching the interface back in Settings → Radio brings the radio back as it was.",
            entry.name,
            entry.address,
            entry.name,
            entry.range_label(),
        )
    }

    /// Act on a picked receiver.
    ///
    /// Both routes go through [`PublicSdrEntry::radio_config`], so the same
    /// receiver is configured identically however it was taken — otherwise
    /// "open it in a tab" and "use it here" would drift into two subtly
    /// different radios.
    fn take_public_sdr(&mut self, entry: &PublicSdrEntry, action: PickAction) {
        match action {
            PickAction::ThisRadio => {
                // Built on this radio's own configuration, so pointing it at a
                // receiver keeps its converter offset, its audio devices and
                // everything else the operator had set — and refused outright
                // where there is none to build on, because the alternative is
                // a *default* radio written over theirs. `radio_cfg` is filled
                // in by opening the settings dialog, which nobody need ever
                // have done.
                if self.radio_cfg.is_none() {
                    self.radio_cfg = self.ctrl.radio_config();
                }
                let Some(base) = self.radio_cfg.clone() else {
                    self.show_notice(
                        "This radio's configuration has not arrived from the machine it is on \
                         yet, so there is nothing to point at the receiver — take it with + TAB \
                         instead."
                            .into(),
                    );
                    return;
                };
                let cfg = entry.radio_config(&base, self.public_sdr_low_bw);
                // The settings dialog edits its own copy of `radio_cfg` and
                // writes it back whenever the two differ, so that copy has to
                // move too — otherwise a dialog left open would push the radio
                // that was here a moment ago straight back over this one. The
                // typed range boxes go with it, for the same reason they are
                // reseeded when the interface picker is used.
                self.radio_cfg = Some(cfg.clone());
                self.range_edit = None;
                self.ctrl.set_radio_config(cfg);
                self.ctrl.reopen_source();
                // A tab named after the transceiver that used to be in it,
                // now running somebody else's receiver on the other side of
                // the world, is the half of issue #254 that no amount of
                // correct range checking would have made sense of: the dial
                // refused 144.8 MHz as out of range while the tab, its mode
                // tag and its packet monitor all still said IC-9700. The
                // confirmation says the rename is coming.
                self.radio_tab_requests
                    .push(RadioTabRequest::Rename { id: self.radio_id, name: entry.name.clone() });
                self.show_notice(format!("Pointing this radio at {}…", entry.name));
                self.show_public_sdrs = false;
            }
            PickAction::NewRadio => {
                // A brand-new radio has none of this radio's settings, and
                // should not inherit them: it is a different receiver.
                let fresh = entry
                    .radio_config(&sdroxide_types::RadioConfig::default(), self.public_sdr_low_bw);
                self.radio_tab_requests.push(RadioTabRequest::Add {
                    station: self.station_key(),
                    preset: Some(Box::new(fresh)),
                });
                self.show_notice(format!("Opening {} as another radio…", entry.name));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The question the panel is drawn with in these tests. Its text does not
    /// matter to what is being pinned, only that there is some.
    const BLURB: &str = "Replace Radio 1 with Twente?";

    /// Draw the same chips [`confirm_panel`] draws, in the same frame and the
    /// same order, and report where the one named ended up.
    ///
    /// The panel returns the answer rather than the responses, so a press has
    /// to be aimed by redrawing its layout. Keeping this beside it is the
    /// point: a chip renamed in one and not the other stops the test dead
    /// instead of quietly pressing nothing.
    fn chip_pos(ui: &mut egui::Ui, label: &str) -> egui::Pos2 {
        let mut at = egui::Pos2::ZERO;
        crate::chrome::red_panel(ui, |ui| {
            ui.label(RichText::new(BLURB).size(12.0));
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                for l in ["REPLACE", "+ TAB INSTEAD", "CANCEL"] {
                    let r = crate::chrome::chip(ui, false, l);
                    if l == label {
                        at = r.rect.center();
                    }
                }
            });
        });
        at
    }

    /// Press one of the confirmation's chips and report what the panel
    /// answered. Two passes, the way `crate::app`'s own chip test does it: the
    /// first tells egui where everything is, the second aims at it.
    fn press(label: &str) -> Option<Confirm> {
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 300.0));
        let mut at = egui::Pos2::ZERO;
        ctx.run_ui(egui::RawInput { screen_rect: Some(screen), ..Default::default() }, |ui| {
            at = chip_pos(ui, label);
        })
        .drop_without_applying_deltas();
        assert_ne!(at, egui::Pos2::ZERO, "{label} is not one of the panel's chips");

        let button = |pressed| egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        let mut answer = None;
        for events in [vec![egui::Event::PointerMoved(at), button(true)], vec![button(false)]] {
            let input = egui::RawInput { screen_rect: Some(screen), events, ..Default::default() };
            ctx.run_ui(input, |ui| answer = answer.or(confirm_panel(ui, BLURB)))
                .drop_without_applying_deltas();
        }
        answer
    }

    /// Each of the three answers has to come back as itself. A confirmation
    /// whose CANCEL replaced the radio anyway would be worse than none at all
    /// — issue #254 is somebody losing a configured IC-9700 to one click.
    #[test]
    fn every_answer_to_the_use_confirmation_comes_back_as_itself() {
        assert_eq!(press("REPLACE"), Some(Confirm::Replace));
        assert_eq!(press("+ TAB INSTEAD"), Some(Confirm::NewTab));
        assert_eq!(press("CANCEL"), Some(Confirm::Cancel));
    }

    /// ...and a panel nobody has pressed answers nothing, or the question
    /// would answer itself on the frame it was raised.
    #[test]
    fn an_unpressed_confirmation_answers_nothing() {
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 300.0));
        let mut answer = Some(Confirm::Replace);
        ctx.run_ui(egui::RawInput { screen_rect: Some(screen), ..Default::default() }, |ui| {
            answer = confirm_panel(ui, BLURB);
        })
        .drop_without_applying_deltas();
        assert_eq!(answer, None);
    }
}
