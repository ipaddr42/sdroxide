//! The external transmit/receive switch: the relay that gets the SDR out of the
//! way while the station transmits.
//!
//! A tab of its own rather than another section under Servers, where the
//! rotator lives. The panel is large — a channel table, the sequencer's
//! timings, a test button apiece — and it is the one page in this dialog whose
//! being hard to find could cost somebody a front end.
//!
//! Three things this panel has to say, and it says them in this order because
//! that is the order they matter:
//!
//! 1. what the switch will actually do, spelled out, because the ordering is
//!    *derived* from the timings rather than typed;
//! 2. that the polarity is a wiring decision and not a preference, because it
//!    is the only fail-safe that survives the program dying;
//! 3. that a transceiver keyed at its own microphone is seen late, and what to
//!    do about it.

use eframe::egui::{self, Color32, ComboBox, RichText};
use sdroxide_types::{
    FailSafe, RelayChannel, RelayConfig, RelayDevice, RelayFamily, RelayLink, RelayRole,
    RelayStatus, SenseLine,
};

use crate::chrome::StyledCombo;

const GOOD: Color32 = Color32::from_rgb(90, 200, 110);
const BAD: Color32 = Color32::from_rgb(230, 90, 80);
const WARN: Color32 = Color32::from_rgb(230, 180, 80);

#[allow(clippy::too_many_arguments)]
pub(in crate::app) fn settings_relay_tab(
    ui: &mut egui::Ui,
    cfg: &mut RelayConfig,
    seeded: bool,
    status: &RelayStatus,
    serial_ports: &[String],
    devices: &[RelayDevice],
    apply: &mut bool,
    test: &mut Option<u8>,
) {
    ui.label(
        RichText::new("Transmit/receive switch").size(14.0).strong().color(crate::theme::CYAN()),
    );
    ui.add_space(4.0);
    if !seeded {
        ui.label(RichText::new("Waiting for the station's T/R switch configuration…").weak());
        return;
    }
    ui.label(
        RichText::new(
            "Closes a contact while this station transmits — to disconnect and ground the SDR's \
             antenna input, and to key an amplifier or an outboard T/R relay in sequence with it. \
             Works with the cheap USB relay boards (LCUS, KMtronic, Numato), with a serial \
             RTS/DTR line into any interface that wants a PTT closure, and with anything else \
             through a command hook.",
        )
        .weak(),
    );
    ui.add_space(8.0);

    // ── the link ────────────────────────────────────────────────────────────
    egui::Grid::new("relay-link-grid").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
        ui.label("Hardware");
        ComboBox::from_id_salt("relay-link")
            .width(260.0)
            .selected_text(cfg.link.label())
            .show_styled(ui, |ui| {
                for l in [
                    RelayLink::Off,
                    RelayLink::Serial,
                    RelayLink::SerialLines,
                    RelayLink::Hid,
                    RelayLink::Cm108,
                    RelayLink::Gpio,
                    RelayLink::Command,
                ] {
                    if ui.selectable_label(cfg.link == l, l.label()).clicked() {
                        cfg.link = l;
                    }
                }
            });
        ui.end_row();

        if cfg.link == RelayLink::Serial {
            ui.label("Board");
            ComboBox::from_id_salt("relay-family")
                .width(260.0)
                .selected_text(cfg.family.label())
                .show_styled(ui, |ui| {
                    for f in [RelayFamily::Lcus, RelayFamily::KMtronic, RelayFamily::Numato] {
                        if ui.selectable_label(cfg.family == f, f.label()).clicked() {
                            cfg.family = f;
                            cfg.serial.baud = f.baud();
                        }
                    }
                })
                .response
                .on_hover_text(
                    "LCUS is the four-byte A0-command board sold under a dozen names; KMtronic \
                     and Numato can also be asked what they are set to",
                );
            ui.end_row();
        }

        if cfg.link.uses_serial_port() {
            ui.label("Port");
            let shown = if cfg.serial.path.is_empty() {
                "— choose —".to_string()
            } else {
                cfg.serial.path.clone()
            };
            // The list is of the *engine's* machine. Where that is elsewhere the
            // stored path is still worth showing — it says which port the switch
            // is on — but there is nothing here to choose from.
            ComboBox::from_id_salt("relay-port").width(260.0).selected_text(shown).show_styled(
                ui,
                |ui| {
                    for p in serial_ports {
                        if ui.selectable_label(&cfg.serial.path == p, p).clicked() {
                            cfg.serial.path = p.clone();
                        }
                    }
                },
            );
            ui.end_row();
        }

        if cfg.link.uses_device_list() {
            ui.label("Device");
            let shown = if cfg.device.is_empty() {
                "— choose —".to_string()
            } else {
                devices
                    .iter()
                    .find(|d| d.key == cfg.device)
                    .map(|d| d.label.clone())
                    .unwrap_or_else(|| cfg.device.clone())
            };
            ComboBox::from_id_salt("relay-device").width(260.0).selected_text(shown).show_styled(
                ui,
                |ui| {
                    for d in devices.iter().filter(|d| d.link == cfg.link) {
                        if ui.selectable_label(cfg.device == d.key, &d.label).clicked() {
                            cfg.device = d.key.clone();
                        }
                    }
                },
            );
            ui.end_row();
        }

        if cfg.link == RelayLink::Cm108 {
            ui.label("GPIO pin");
            ui.add(egui::DragValue::new(&mut cfg.cm108_pin).range(1..=8)).on_hover_text(
                "Pin 3 on every homebrew plan and every commercial board — it is the one at the \
                 end of the package that a wire can be tacked to. If the relay does not click, \
                 the card is a clone that does not implement these pins at all.",
            );
            ui.end_row();
        }

        if cfg.link == RelayLink::Gpio {
            ui.label("GPIO chip");
            crate::chrome::field(
                ui,
                egui::TextEdit::singleline(&mut cfg.device)
                    .desired_width(260.0)
                    .hint_text("/dev/gpiochip0"),
            )
            .on_hover_text("Linux only. `gpiodetect` lists the chips on the machine.");
            ui.end_row();
        }

        if cfg.link == RelayLink::Command {
            ui.label("On transmit");
            crate::chrome::field(
                ui,
                egui::TextEdit::singleline(&mut cfg.tx_cmd)
                    .desired_width(260.0)
                    .hint_text("usbrelay 1_1=1"),
            );
            ui.end_row();
            ui.label("On receive");
            crate::chrome::field(
                ui,
                egui::TextEdit::singleline(&mut cfg.rx_cmd)
                    .desired_width(260.0)
                    .hint_text("usbrelay 1_1=0"),
            );
            ui.end_row();
        }
    });

    if cfg.link == RelayLink::Off {
        ui.add_space(8.0);
        ui.label(RichText::new("Nothing is switched.").weak());
        ui.add_space(8.0);
        apply_button(ui, apply);
        return;
    }

    if cfg.link == RelayLink::Command {
        ui.add_space(4.0);
        ui.label(
            RichText::new(
                "Starting a program costs an unpredictable amount of time, and none of it is \
                 counted in the lead — so leave a wider margin here than you would for a relay \
                 board. The contact table's numbers mean nothing for this link: your commands \
                 are run once on key-down and once on key-up.",
            )
            .weak(),
        );
    }

    // ── the channels ────────────────────────────────────────────────────────
    ui.add_space(10.0);
    ui.label(RichText::new("Contacts").strong());
    ui.add_space(4.0);
    if cfg.link == RelayLink::SerialLines {
        ui.label(RichText::new("Contact 1 is RTS and contact 2 is DTR.").weak());
        ui.add_space(4.0);
    }
    let busy = status.keyed;
    let gpio = cfg.link == RelayLink::Gpio;
    // One line per contact *number*, so contact 3 drives `gpio_lines[2]` — the
    // driver indexes it that way, and a table with a gap in it would otherwise
    // operate the wrong pin.
    if gpio {
        let want = cfg.channels.iter().map(|c| usize::from(c.index)).max().unwrap_or(0);
        if cfg.gpio_lines.len() < want {
            cfg.gpio_lines.resize(want, 0);
        }
    }
    let mut remove: Option<usize> = None;
    let cols = if gpio { 8 } else { 7 };
    egui::Grid::new("relay-channels").num_columns(cols).spacing([10.0, 6.0]).show(ui, |ui| {
        ui.label(RichText::new("No.").weak());
        if gpio {
            ui.label(RichText::new("GPIO").weak());
        }
        ui.label(RichText::new("Name").weak());
        ui.label(RichText::new("Job").weak());
        ui.label(RichText::new("TX closes").weak());
        ui.label(RichText::new("Lead").weak());
        ui.label(RichText::new("Hold").weak());
        ui.label("");
        ui.end_row();

        let count = cfg.channels.len();
        let lines = &mut cfg.gpio_lines;
        for (i, ch) in cfg.channels.iter_mut().enumerate() {
            ui.add_enabled_ui(cfg.link.has_numbered_channels(), |ui| {
                ui.add(egui::DragValue::new(&mut ch.index).range(1..=32));
            });
            if gpio {
                // Bounds-checked above; a fresh row is filled in on the frame
                // after it is added.
                match lines.get_mut(usize::from(ch.index).saturating_sub(1)) {
                    Some(line) => {
                        ui.add(egui::DragValue::new(line).range(0..=255)).on_hover_text(
                            "The line's offset on this chip — the BCM number on a Raspberry Pi \
                             header, which is not the physical pin number",
                        );
                    }
                    None => {
                        ui.label("");
                    }
                }
            }
            crate::chrome::field(
                ui,
                egui::TextEdit::singleline(&mut ch.label)
                    .desired_width(120.0)
                    .hint_text("SDR antenna"),
            );
            ComboBox::from_id_salt(("relay-role", i))
                .width(180.0)
                .selected_text(ch.role.label())
                .show_styled(ui, |ui| {
                    for r in [
                        RelayRole::SdrAntenna,
                        RelayRole::Amplifier,
                        RelayRole::Aux,
                        RelayRole::Unused,
                    ] {
                        if ui.selectable_label(ch.role == r, r.label()).clicked() {
                            ch.role = r;
                            let (lead, hold) = r.default_timing();
                            ch.lead_ms = lead;
                            ch.hold_ms = hold;
                        }
                    }
                })
                .response
                .on_hover_text("Only decides the default timings — and what the log calls it");
            let mut high = ch.active_high;
            if ui
                .checkbox(&mut high, "")
                .on_hover_text(
                    "On: transmitting energises the coil. Off: transmitting releases it.\n\nThis \
                     is a wiring decision before it is a setting — see the note below.",
                )
                .changed()
            {
                ch.active_high = high;
            }
            ui.add(egui::DragValue::new(&mut ch.lead_ms).range(0..=200).suffix(" ms"))
                .on_hover_text(
                    "Closed this long before RF. A small coax relay throws in 5 to 15 ms. \
                     Transmit waits for the longest of these, so a large value is an audible \
                     gap in the receive audio; anything over 250 ms is ignored.",
                );
            ui.add(egui::DragValue::new(&mut ch.hold_ms).range(0..=2000).suffix(" ms"))
                .on_hover_text(
                    "Opened this long after RF stops. Longer than the lead on purpose: letting \
                     the antenna back a moment late costs nothing, and letting it back early \
                     costs a front end. Zero is a real answer, and the right one for an \
                     amplifier's key line.",
                );
            ui.horizontal(|ui| {
                if crate::chrome::chip_accent_enabled(
                    ui,
                    !busy,
                    false,
                    " TEST ",
                    None,
                    crate::theme::CYAN(),
                    crate::theme::INK_ON_CYAN(),
                )
                .on_hover_text(if busy {
                    "Not while the station is on the air"
                } else {
                    "Close this contact for half a second, so you can hear the relay and check \
                     the wiring with the transmitter cold"
                })
                .clicked()
                {
                    *test = Some(ch.index);
                }
                if count > 1 && ui.small_button("×").on_hover_text("Remove").clicked() {
                    remove = Some(i);
                }
            });
            ui.end_row();
        }
    });
    if let Some(i) = remove {
        cfg.channels.remove(i);
    }
    ui.add_space(4.0);
    if ui.small_button("+ add a contact").clicked() {
        let next = cfg.channels.iter().map(|c| c.index).max().unwrap_or(0).saturating_add(1);
        let (lead, hold) = RelayRole::Aux.default_timing();
        cfg.channels.push(RelayChannel {
            index: next.max(1),
            role: RelayRole::Aux,
            label: String::new(),
            active_high: true,
            lead_ms: lead,
            hold_ms: hold,
        });
    }

    // ── the fail-safe ───────────────────────────────────────────────────────
    ui.add_space(10.0);
    egui::Grid::new("relay-timing").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
        ui.label("If it will not answer");
        ComboBox::from_id_salt("relay-failsafe")
            .width(260.0)
            .selected_text(cfg.fail_safe.label())
            .show_styled(ui, |ui| {
                for f in [FailSafe::RefuseTx, FailSafe::WarnOnly] {
                    if ui.selectable_label(cfg.fail_safe == f, f.label()).clicked() {
                        cfg.fail_safe = f;
                    }
                }
            })
            .response
            .on_hover_text(
                "A switch that exists to protect a receiver is not worth having if it silently \
                 stops protecting it — so refusing is the default. Choose the other where a \
                 loose USB cable ending your contest is the worse outcome.",
            );
        ui.end_row();
    });

    // ── what it will actually do ────────────────────────────────────────────
    ui.add_space(10.0);
    ui.label(RichText::new(cfg.sequence_note()).color(crate::theme::CYAN()));
    ui.add_space(4.0);
    ui.label(
        RichText::new(
            "The order is the timings: the longest lead closes first, and the longest hold opens \
             last. So give the antenna relay the longer lead and the longer hold, and the \
             amplifier the shorter of each — then the antenna is always switched before the \
             amplifier is keyed, and the amplifier always unkeys before the antenna comes back.",
        )
        .weak(),
    );

    // ── the transmit sense input ────────────────────────────────────────────
    ui.add_space(12.0);
    ui.label(RichText::new("Transmit sense input").strong());
    ui.add_space(4.0);
    ui.label(
        RichText::new(
            "When a transceiver keys itself — its own microphone button, foot switch, VOX or \
             keyer — sdroxide finds out by asking over CAT, and that answer arrives a few \
             hundred milliseconds into the over. Wire the rig's SEND line, through an \
             opto-isolator, into a handshake input on this same port and it is seen in a few \
             milliseconds instead. The meter and the transmit interlock get faster with it.",
        )
        .weak(),
    );
    ui.add_space(6.0);
    ui.add_enabled_ui(cfg.link.uses_serial_port(), |ui| {
        egui::Grid::new("relay-sense").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
            ui.label("Line");
            ComboBox::from_id_salt("relay-sense-line")
                .width(260.0)
                .selected_text(cfg.sense.line.label())
                .show_styled(ui, |ui| {
                    for l in [SenseLine::Off, SenseLine::Cts, SenseLine::Dsr, SenseLine::Dcd] {
                        if ui.selectable_label(cfg.sense.line == l, l.label()).clicked() {
                            cfg.sense.line = l;
                        }
                    }
                });
            ui.end_row();

            if cfg.sense.line != SenseLine::Off {
                ui.label("Transmitting is");
                let mut high = cfg.sense.active_high;
                if ui
                    .checkbox(&mut high, "a high line")
                    .on_hover_text(
                        "Most opto-isolated interfaces pull the line down when the rig keys, so \
                         this is usually off",
                    )
                    .changed()
                {
                    cfg.sense.active_high = high;
                }
                ui.end_row();

                ui.label("Belongs to radio");
                ui.add(
                    egui::DragValue::new(&mut cfg.sense.radio)
                        .range(0..=15)
                        .custom_formatter(|n, _| format!("{}", n as u32 + 1)),
                )
                .on_hover_text(
                    "Which radio tab the sensed transceiver is. On a station with one radio this \
                     is 1 and nothing depends on it.",
                );
                ui.end_row();
            }
        });
    });
    if !cfg.link.uses_serial_port() {
        ui.label(RichText::new("Needs a serial port — the handshake lines are on it.").weak());
    }

    // ── the two things that are not settings ────────────────────────────────
    ui.add_space(12.0);
    ui.label(
        RichText::new("Wiring is the only fail-safe that survives this program.")
            .strong()
            .color(WARN),
    );
    ui.label(
        RichText::new(
            "Choose “TX closes” so that the *de-energised* contact is the state you want when \
             the board is unplugged, the computer is off, or sdroxide is not running. Wired so \
             that de-energised means grounded, a dead relay leaves the SDR deaf — annoying, and \
             safe. Wired the other way, a dead relay plus one transmission is a dead front end. \
             With a real amplifier behind it, take the first.",
        )
        .weak(),
    );
    ui.add_space(6.0);
    ui.label(
        RichText::new(
            "This cannot protect the first instant of an over you start at the radio. Key the \
             rig from sdroxide and the contacts always lead the RF; wire the sense input above \
             and they follow within milliseconds. For a front end genuinely worth protecting, \
             use an RF-sensed hardware T/R switch as well — no program on a computer can be the \
             whole answer.",
        )
        .weak(),
    );

    // ── status and apply ────────────────────────────────────────────────────
    ui.add_space(10.0);
    if let Some(why) = cfg.refusal() {
        ui.label(RichText::new(format!("Not usable yet: {why}")).color(WARN));
    } else if let Some(e) = status.error.as_ref() {
        ui.label(RichText::new(format!("● {e}")).color(BAD));
    } else if status.configured && status.present {
        ui.label(
            RichText::new(format!(
                "● {} — {}",
                status.describe,
                if status.keyed { "contacts in TRANSMIT" } else { "receiving" }
            ))
            .color(GOOD),
        );
    } else if status.configured {
        ui.label(RichText::new("Not open.").weak());
    } else {
        ui.label(RichText::new("Status unknown — press APPLY.").weak());
    }

    ui.add_space(8.0);
    if busy {
        ui.label(
            RichText::new("The station is on the air — APPLY will take effect after this over.")
                .color(WARN),
        );
        ui.add_space(4.0);
    }
    apply_button(ui, apply);
}

fn apply_button(ui: &mut egui::Ui, apply: &mut bool) {
    if crate::chrome::chip_accent(
        ui,
        false,
        RichText::new(" APPLY ").strong(),
        crate::theme::GREEN(),
        crate::theme::INK_ON_CYAN(),
    )
    .on_hover_text("Persist and (re)open the switch")
    .clicked()
    {
        *apply = true;
    }
}
