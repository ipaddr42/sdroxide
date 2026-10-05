//! The words for an audible alert, when it is answered with the voice.
//!
//! Pure: the callsign arrives already spelled by the [`Speaker`], so the
//! wording is testable without a voice, a sink or a clock.
//!
//! [`Speaker`]: crate::text::Speaker

use sdroxide_types::AlertEvent;

/// One spoken alert. `call` is the already-spoken callsign (see
/// [`Speaker::callsign`]), `band` the adif band the decode arrived on, and
/// `country` the resolved entity name when the country file knows it.
///
/// The callsign leads every phrase: it is the one thing the operator needs to
/// act on, so a clipped or odd-sounding country name is the part that can be
/// spared.
///
/// [`Speaker::callsign`]: crate::text::Speaker::callsign
pub fn phrase(event: AlertEvent, call: &str, band: &str, country: Option<&str>) -> String {
    match event {
        AlertEvent::Called => format!("{call}, calling you"),
        AlertEvent::Cq => format!("{call}, calling C Q"),
        AlertEvent::NewDxcc => match country {
            Some(c) => format!("{call}, new D X C C, {c}"),
            None => format!("{call}, new D X C C"),
        },
        AlertEvent::NewDxccBand => format!("{call}, new one on {}", band_words(band)),
        AlertEvent::NewGrid => format!("{call}, new grid"),
    }
}

/// An adif band name as words: `20m` becomes `20 metres`, `70cm` becomes `70
/// centimetres`. Anything that is not `<digits>m` or `<digits>cm` is returned
/// unchanged, since a wrong reading is worse than none.
pub fn band_words(band: &str) -> String {
    let b = band.trim();
    if let Some(n) = b.strip_suffix("cm")
        && !n.is_empty()
        && n.chars().all(|c| c.is_ascii_digit())
    {
        return format!("{n} centimetres");
    }
    if let Some(n) = b.strip_suffix('m')
        && !n.is_empty()
        && n.chars().all(|c| c.is_ascii_digit())
    {
        return format!("{n} metres");
    }
    b.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_event_has_its_own_line_and_the_call_leads() {
        assert_eq!(
            phrase(AlertEvent::Called, "Juliett Alfa One", "20m", None),
            "Juliett Alfa One, calling you"
        );
        assert_eq!(
            phrase(AlertEvent::Cq, "Juliett Alfa One", "20m", None),
            "Juliett Alfa One, calling C Q"
        );
        assert_eq!(
            phrase(AlertEvent::NewDxcc, "Juliett Alfa One", "20m", Some("Japan")),
            "Juliett Alfa One, new D X C C, Japan"
        );
        // No country resolved: still names the station.
        assert_eq!(
            phrase(AlertEvent::NewDxcc, "Juliett Alfa One", "20m", None),
            "Juliett Alfa One, new D X C C"
        );
        assert_eq!(
            phrase(AlertEvent::NewDxccBand, "Juliett Alfa One", "20m", None),
            "Juliett Alfa One, new one on 20 metres"
        );
        assert_eq!(
            phrase(AlertEvent::NewGrid, "Juliett Alfa One", "20m", None),
            "Juliett Alfa One, new grid"
        );
    }

    #[test]
    fn bands_read_as_metres_and_centimetres() {
        assert_eq!(band_words("160m"), "160 metres");
        assert_eq!(band_words("20m"), "20 metres");
        assert_eq!(band_words("6m"), "6 metres");
        assert_eq!(band_words("70cm"), "70 centimetres");
        // Not a metre band: left alone rather than mispronounced.
        assert_eq!(band_words("hf"), "hf");
        assert_eq!(band_words(""), "");
    }
}
