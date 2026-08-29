//! An I/Q capture as a WAV file the rest of the world can open.
//!
//! Two channels of 32-bit float — I in the left, Q in the right — at the
//! receiver's own sample rate, which is what SDR#, SDRuno, HDSDR, SDRangel and
//! GNU Radio all expect from a baseband recording. The centre frequency travels
//! in an `auxi` chunk *and* in the file name, because those two are what the
//! programs above actually read: SDR# and SDRuno take the name apart, HDSDR
//! reads the chunk.
//!
//! # Why RF64
//!
//! A RIFF file's sizes are 32-bit, so a WAV stops at 4 GB. At 2.4 Msps that is
//! **three and a half minutes** — a limit an operator would meet on their first
//! capture and read as a bug. RF64 (EBU Tech 3306) is the standard answer and
//! is what every one of the programs above opens.
//!
//! Written the way the specification prescribes rather than as a separate
//! format: the header carries a 28-byte `JUNK` chunk ahead of `fmt `, the file
//! is a perfectly ordinary WAV while it fits in one, and only a capture that
//! outgrows 4 GB is promoted on close — `RIFF`→`RF64`, the two sizes to
//! `0xFFFFFFFF`, and the reserved `JUNK` becomes the `ds64` chunk that carries
//! the real 64-bit ones. Nothing has to be decided when the recording starts,
//! which matters because nothing knows then how long it will run.

use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

/// Bytes one I/Q frame occupies: two channels of `f32`.
const FRAME_BYTES: u64 = 8;

/// Where the 32-bit `RIFF` size lives, and where the `data` size does.
const RIFF_SIZE_AT: u64 = 4;

/// The reserved chunk's payload size: `ds64` carries three 64-bit sizes and a
/// table count, and `JUNK` has to be big enough to become one in place.
const DS64_PAYLOAD: usize = 28;

/// A capture in progress.
///
/// Buffered, and the buffer is deliberately large: at 2.4 Msps this is
/// 19 MB a second, and the write happens on the engine's block thread.
pub struct IqWavWriter {
    file: BufWriter<File>,
    path: PathBuf,
    /// Byte offset of the `data` chunk's own 32-bit size field.
    data_size_at: u64,
    /// Frames written so far.
    frames: u64,
    scratch: Vec<u8>,
}

impl IqWavWriter {
    /// Open `path` and write the header for a stereo float32 stream at
    /// `rate_hz`, tuned to `center_hz`.
    pub fn create(path: &Path, rate_hz: u32, center_hz: f64) -> std::io::Result<IqWavWriter> {
        let file = File::create(path)?;
        let mut w = BufWriter::with_capacity(1 << 21, file);
        let header = header_bytes(rate_hz, center_hz);
        w.write_all(&header.bytes)?;
        Ok(IqWavWriter {
            file: w,
            path: path.to_path_buf(),
            data_size_at: header.data_size_at,
            frames: 0,
            scratch: Vec::new(),
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Seconds of signal written so far — what the operator is shown.
    pub fn seconds(&self, rate_hz: f64) -> f64 {
        if rate_hz > 0.0 { self.frames as f64 / rate_hz } else { 0.0 }
    }

    pub fn bytes(&self) -> u64 {
        self.frames * FRAME_BYTES
    }

    /// Append a block of complex samples.
    pub fn write(&mut self, iq: &[crate::Complex32]) -> std::io::Result<()> {
        self.scratch.clear();
        self.scratch.reserve(iq.len() * FRAME_BYTES as usize);
        for z in iq {
            self.scratch.extend_from_slice(&z.re.to_le_bytes());
            self.scratch.extend_from_slice(&z.im.to_le_bytes());
        }
        self.file.write_all(&self.scratch)?;
        self.frames += iq.len() as u64;
        Ok(())
    }

    /// Close the file, patching the sizes in — and promoting it to RF64 where
    /// they no longer fit in 32 bits.
    ///
    /// A failure here leaves a file whose header says it is empty, which every
    /// player reads as a zero-length recording. That is worth saying out loud
    /// rather than dropping quietly, so this returns rather than being a `Drop`.
    pub fn finish(mut self) -> std::io::Result<PathBuf> {
        self.file.flush()?;
        let data_bytes = self.frames * FRAME_BYTES;
        let riff_bytes = self.data_size_at + 4 + data_bytes - 8;
        let f = self.file.get_mut();
        if riff_bytes > u64::from(u32::MAX) || data_bytes > u64::from(u32::MAX) {
            // Promote in place: the reserved chunk was put there for this.
            f.seek(SeekFrom::Start(0))?;
            f.write_all(b"RF64")?;
            f.write_all(&u32::MAX.to_le_bytes())?;
            f.seek(SeekFrom::Start(12))?;
            f.write_all(b"ds64")?;
            f.write_all(&(DS64_PAYLOAD as u32).to_le_bytes())?;
            f.write_all(&riff_bytes.to_le_bytes())?;
            f.write_all(&data_bytes.to_le_bytes())?;
            // Sample count per channel, and no chunk-size table.
            f.write_all(&self.frames.to_le_bytes())?;
            f.write_all(&0u32.to_le_bytes())?;
            f.seek(SeekFrom::Start(self.data_size_at))?;
            f.write_all(&u32::MAX.to_le_bytes())?;
        } else {
            f.seek(SeekFrom::Start(RIFF_SIZE_AT))?;
            f.write_all(&(riff_bytes as u32).to_le_bytes())?;
            f.seek(SeekFrom::Start(self.data_size_at))?;
            f.write_all(&(data_bytes as u32).to_le_bytes())?;
        }
        f.flush()?;
        Ok(self.path)
    }
}

struct Header {
    bytes: Vec<u8>,
    data_size_at: u64,
}

/// The whole header, up to and including the `data` chunk's size field.
fn header_bytes(rate_hz: u32, center_hz: f64) -> Header {
    let mut b: Vec<u8> = Vec::with_capacity(128);
    let channels: u16 = 2;
    let bits: u16 = 32;
    let block_align = u32::from(channels) * u32::from(bits) / 8;
    let byte_rate = rate_hz * block_align;

    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&0u32.to_le_bytes()); // patched on close
    b.extend_from_slice(b"WAVE");

    // The RF64 reservation, first of the chunks exactly as Tech 3306 requires.
    b.extend_from_slice(b"JUNK");
    b.extend_from_slice(&(DS64_PAYLOAD as u32).to_le_bytes());
    b.extend_from_slice(&[0u8; DS64_PAYLOAD]);

    b.extend_from_slice(b"fmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&3u16.to_le_bytes()); // WAVE_FORMAT_IEEE_FLOAT
    b.extend_from_slice(&channels.to_le_bytes());
    b.extend_from_slice(&rate_hz.to_le_bytes());
    b.extend_from_slice(&byte_rate.to_le_bytes());
    b.extend_from_slice(&(block_align as u16).to_le_bytes());
    b.extend_from_slice(&bits.to_le_bytes());

    b.extend_from_slice(b"auxi");
    let auxi = auxi_payload(center_hz);
    b.extend_from_slice(&(auxi.len() as u32).to_le_bytes());
    b.extend_from_slice(&auxi);

    b.extend_from_slice(b"data");
    let data_size_at = b.len() as u64;
    b.extend_from_slice(&0u32.to_le_bytes()); // patched on close
    Header { bytes: b, data_size_at }
}

/// The `auxi` chunk SDR#/HDSDR write: two `SYSTEMTIME`s, then the tuning.
///
/// Sixteen little-endian `u16`s of start and stop time (year, month, day of
/// week, day, hour, minute, second, millisecond — Windows' `SYSTEMTIME`, which
/// is what the format is), then centre, dial and low-edge frequencies, a
/// timestamp pair and the ADC width. Only the centre frequency is read by
/// anything that matters, and it is the whole reason to write the chunk; the
/// rest is filled in so the layout is the one those programs parse.
fn auxi_payload(center_hz: f64) -> Vec<u8> {
    let mut b = Vec::with_capacity(164);
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let st = systemtime_fields(now);
    for _ in 0..2 {
        for v in st {
            b.extend_from_slice(&v.to_le_bytes());
        }
    }
    let hz = center_hz.max(0.0) as u32;
    b.extend_from_slice(&hz.to_le_bytes()); // centre
    b.extend_from_slice(&hz.to_le_bytes()); // dial ("frequency")
    b.extend_from_slice(&0u32.to_le_bytes()); // IF frequency
    b.extend_from_slice(&0u32.to_le_bytes()); // bandwidth
    b.extend_from_slice(&0u32.to_le_bytes()); // IQ offset
    b.extend_from_slice(&0u32.to_le_bytes()); // db offset
    b.extend_from_slice(&0u32.to_le_bytes()); // max value
    b.extend_from_slice(&32u32.to_le_bytes()); // significant bits
    b
}

/// A Unix time as the eight `u16` fields of a Windows `SYSTEMTIME`.
///
/// Written out rather than taken from a date library because the only date
/// arithmetic here is "which day is this", and pulling a dependency into
/// `sdroxide-radio` for it would be the larger change. Civil-from-days is
/// Howard Hinnant's algorithm.
fn systemtime_fields(unix: u64) -> [u16; 8] {
    let secs_of_day = (unix % 86_400) as u32;
    let days = (unix / 86_400) as i64;
    // 1970-01-01 was a Thursday; `SYSTEMTIME` counts Sunday as 0.
    let dow = ((days + 4) % 7) as u16;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u16;
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u16;
    let y = (y + i64::from(m <= 2)) as u16;
    [
        y,
        m,
        dow,
        d,
        (secs_of_day / 3600) as u16,
        (secs_of_day % 3600 / 60) as u16,
        (secs_of_day % 60) as u16,
        0,
    ]
}

/// The file name a capture is given.
///
/// SDR#'s convention, because SDR# and SDRuno both *parse* it: a recording
/// dropped on either one comes back tuned to where it was made, and that is the
/// difference between a file somebody can use and a file of numbers.
pub fn capture_name(unix: u64, center_hz: f64, rate_hz: u32) -> String {
    let t = systemtime_fields(unix);
    format!(
        "SDRoxide_{:04}{:02}{:02}_{:02}{:02}{:02}Z_{}Hz_{}sps_IQ.wav",
        t[0],
        t[1],
        t[3],
        t[4],
        t[5],
        t[6],
        center_hz.max(0.0) as u64,
        rate_hz,
    )
}

/// What a capture file says about itself.
pub struct IqWavInfo {
    /// Byte offset of the first sample.
    pub data_start: u64,
    /// Bytes of samples, or `u64::MAX` where the header does not say (a capture
    /// whose writer was killed before it could patch the size in).
    pub data_len: u64,
    pub rate_hz: f64,
    /// From the `auxi` chunk, where there is one.
    pub center_hz: Option<f64>,
}

/// Read the header of a stereo float32 WAV or RF64 capture.
///
/// `None` for anything else, including a WAV that is not the shape this writes
/// — 16-bit captures from other programs are a conversion rather than a read,
/// and answering `Some` for one would play noise at the wrong scale rather than
/// say so.
///
/// The point of it is `--file`: a capture made here plays back here, at the
/// rate and on the frequency it was made, with nothing to type.
pub fn probe(path: &Path) -> Option<IqWavInfo> {
    use std::io::Read;
    let mut f = File::open(path).ok()?;
    let mut head = [0u8; 12];
    f.read_exact(&mut head).ok()?;
    let rf64 = match &head[0..4] {
        b"RIFF" => false,
        b"RF64" => true,
        _ => return None,
    };
    if &head[8..12] != b"WAVE" {
        return None;
    }
    let mut at = 12u64;
    let mut rate = None;
    let mut center = None;
    let mut ds64_data = None;
    loop {
        let mut hdr = [0u8; 8];
        f.seek(SeekFrom::Start(at)).ok()?;
        if f.read_exact(&mut hdr).is_err() {
            return None;
        }
        let id: [u8; 4] = hdr[0..4].try_into().ok()?;
        let size = u32::from_le_bytes(hdr[4..8].try_into().ok()?) as u64;
        let body = at + 8;
        match &id {
            b"ds64" if rf64 => {
                let mut b = [0u8; 16];
                f.read_exact(&mut b).ok()?;
                ds64_data = Some(u64::from_le_bytes(b[8..16].try_into().ok()?));
            }
            b"fmt " => {
                let mut b = [0u8; 16];
                f.read_exact(&mut b).ok()?;
                let format = u16::from_le_bytes(b[0..2].try_into().ok()?);
                let channels = u16::from_le_bytes(b[2..4].try_into().ok()?);
                let bits = u16::from_le_bytes(b[14..16].try_into().ok()?);
                // Only the shape this module writes. Anything else is a
                // different file that happens to share a container.
                if format != 3 || channels != 2 || bits != 32 {
                    return None;
                }
                rate = Some(f64::from(u32::from_le_bytes(b[4..8].try_into().ok()?)));
            }
            b"auxi" if size >= 36 => {
                let mut b = [0u8; 36];
                f.read_exact(&mut b).ok()?;
                let hz = u32::from_le_bytes(b[32..36].try_into().ok()?);
                if hz > 0 {
                    center = Some(f64::from(hz));
                }
            }
            b"data" => {
                let len = if size == u64::from(u32::MAX) {
                    ds64_data.or_else(|| f.metadata().ok().map(|m| m.len().saturating_sub(body)))?
                } else {
                    size
                };
                return Some(IqWavInfo {
                    data_start: body,
                    data_len: len,
                    rate_hz: rate?,
                    center_hz: center,
                });
            }
            _ => {}
        }
        // Odd-sized chunks are padded to even, which is a RIFF rule and not an
        // optional one: skipping the pad byte reads the next chunk id off by one.
        at = body + size + (size & 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A short capture is an ordinary WAV, and the sizes in its header are the
    /// ones a player will use to find the samples.
    #[test]
    fn a_short_capture_is_a_plain_wav() {
        let dir = std::env::temp_dir().join(format!("sdroxide-iqwav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("short.wav");
        let mut w = IqWavWriter::create(&path, 48_000, 14_074_000.0).unwrap();
        let block: Vec<crate::Complex32> =
            (0..1000).map(|i| crate::Complex32::new(i as f32 * 1e-4, -0.5)).collect();
        w.write(&block).unwrap();
        assert_eq!(w.bytes(), 8000);
        w.finish().unwrap();

        let raw = std::fs::read(&path).unwrap();
        assert_eq!(&raw[..4], b"RIFF", "a capture that fits stays a WAV");
        assert_eq!(&raw[8..12], b"WAVE");
        // The RIFF size is everything after it.
        let riff = u32::from_le_bytes(raw[4..8].try_into().unwrap()) as usize;
        assert_eq!(riff, raw.len() - 8);
        // The data chunk holds every sample and nothing else.
        let at = find_chunk(&raw, b"data").expect("data chunk");
        let data = u32::from_le_bytes(raw[at + 4..at + 8].try_into().unwrap()) as usize;
        assert_eq!(data, 8000);
        assert_eq!(at + 8 + data, raw.len());

        // Float stereo at the rate it was opened with.
        let fmt = find_chunk(&raw, b"fmt ").expect("fmt chunk");
        assert_eq!(u16::from_le_bytes(raw[fmt + 8..fmt + 10].try_into().unwrap()), 3);
        assert_eq!(u16::from_le_bytes(raw[fmt + 10..fmt + 12].try_into().unwrap()), 2);
        assert_eq!(u32::from_le_bytes(raw[fmt + 12..fmt + 16].try_into().unwrap()), 48_000);

        // The centre frequency is in the chunk the players read it from.
        let auxi = find_chunk(&raw, b"auxi").expect("auxi chunk");
        let payload = auxi + 8;
        assert_eq!(
            u32::from_le_bytes(raw[payload + 32..payload + 36].try_into().unwrap()),
            14_074_000
        );
        let _ = std::fs::remove_file(&path);
    }

    /// The reservation for RF64 is there from the first byte, ahead of `fmt `,
    /// so a capture that outgrows 4 GB can be promoted without moving anything.
    #[test]
    fn the_rf64_reservation_is_written_ahead_of_the_format() {
        let dir = std::env::temp_dir().join(format!("sdroxide-iqwav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("junk.wav");
        IqWavWriter::create(&path, 2_400_000, 100e6).unwrap().finish().unwrap();
        let raw = std::fs::read(&path).unwrap();
        assert_eq!(&raw[12..16], b"JUNK", "the ds64 reservation has to be the first chunk");
        assert_eq!(u32::from_le_bytes(raw[16..20].try_into().unwrap()), DS64_PAYLOAD as u32);
        assert!(find_chunk(&raw, b"fmt ").unwrap() > 12);
        let _ = std::fs::remove_file(&path);
    }

    /// The name is the one SDR# and SDRuno parse, so a capture opens where it
    /// was made rather than at whatever the program was last tuned to.
    #[test]
    fn the_name_carries_the_tuning() {
        // 2026-08-29T13:45:07Z
        let name = capture_name(1_788_011_107, 14_074_000.0, 2_400_000);
        assert_eq!(name, "SDRoxide_20260829_134507Z_14074000Hz_2400000sps_IQ.wav");
    }

    /// The civil date behind that name, at the boundaries it is easiest to get
    /// wrong: an epoch, a leap day, and a century that is not a leap year.
    #[test]
    fn the_calendar_is_the_calendar() {
        assert_eq!(systemtime_fields(0)[..4], [1970, 1, 4, 1], "1970-01-01 was a Thursday");
        assert_eq!(systemtime_fields(951_782_400)[..4], [2000, 2, 2, 29], "2000 is a leap year");
        assert_eq!(systemtime_fields(1_709_164_800)[..4], [2024, 2, 4, 29], "so is 2024");
        assert_eq!(systemtime_fields(1_788_011_107)[..4], [2026, 8, 6, 29]);
        let t = systemtime_fields(1_788_011_107);
        assert_eq!([t[4], t[5], t[6]], [13, 45, 7]);
    }

    /// A capture made here reads back here: the rate and the frequency come out
    /// of the file, so `--file` needs neither typed at it.
    #[test]
    fn a_capture_reads_its_own_header_back() {
        let dir = std::env::temp_dir().join(format!("sdroxide-iqwav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("probe.wav");
        let mut w = IqWavWriter::create(&path, 2_400_000, 145_500_000.0).unwrap();
        w.write(&[crate::Complex32::new(0.25, -0.75); 64]).unwrap();
        w.finish().unwrap();

        let info = probe(&path).expect("our own capture");
        assert_eq!(info.rate_hz, 2_400_000.0);
        assert_eq!(info.center_hz, Some(145_500_000.0));
        assert_eq!(info.data_len, 64 * 8);
        // …and the offset really is where the samples start.
        let raw = std::fs::read(&path).unwrap();
        let at = info.data_start as usize;
        assert_eq!(f32::from_le_bytes(raw[at..at + 4].try_into().unwrap()), 0.25);
        assert_eq!(f32::from_le_bytes(raw[at + 4..at + 8].try_into().unwrap()), -0.75);
        let _ = std::fs::remove_file(&path);
    }

    /// Something that is not one of ours is refused rather than half-read: a
    /// 16-bit capture from another program would play as noise at the wrong
    /// scale, which is worse than saying no.
    #[test]
    fn a_file_that_is_not_ours_is_refused() {
        let dir = std::env::temp_dir().join(format!("sdroxide-iqwav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("pcm16.wav");
        let mut b: Vec<u8> = Vec::new();
        b.extend_from_slice(b"RIFF");
        b.extend_from_slice(&36u32.to_le_bytes());
        b.extend_from_slice(b"WAVEfmt ");
        b.extend_from_slice(&16u32.to_le_bytes());
        b.extend_from_slice(&1u16.to_le_bytes()); // PCM
        b.extend_from_slice(&2u16.to_le_bytes());
        b.extend_from_slice(&48_000u32.to_le_bytes());
        b.extend_from_slice(&192_000u32.to_le_bytes());
        b.extend_from_slice(&4u16.to_le_bytes());
        b.extend_from_slice(&16u16.to_le_bytes());
        b.extend_from_slice(b"data");
        b.extend_from_slice(&0u32.to_le_bytes());
        std::fs::write(&path, &b).unwrap();
        assert!(probe(&path).is_none());
        // …and neither is a raw CF32 stream, which has no header at all.
        let raw = dir.join("raw.cf32");
        std::fs::write(&raw, [0u8; 64]).unwrap();
        assert!(probe(&raw).is_none());
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&raw);
    }

    fn find_chunk(raw: &[u8], id: &[u8; 4]) -> Option<usize> {
        let mut at = 12;
        while at + 8 <= raw.len() {
            let size = u32::from_le_bytes(raw[at + 4..at + 8].try_into().unwrap()) as usize;
            if &raw[at..at + 4] == id {
                return Some(at);
            }
            at += 8 + size + (size & 1);
        }
        None
    }
}
