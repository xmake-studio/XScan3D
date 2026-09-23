//! Host-side mirror of src/protocol.h: the scanner's USB stream.
//!
//! Three record types share the 55 AA 03 xx magic, where xx is the record
//! length (0x09 marks the variable-length event record). Anything that is not a
//! valid record is skipped a byte at a time, so boot chatter and dropouts cost
//! at most one record. Change the firmware's layouts, change these.

pub const MAGIC: [u8; 3] = [0x55, 0xAA, 0x03];

pub const SAMPLE_TAG: u8 = 0x22;
pub const SAMPLE_LEN: usize = 34;
/// Samples from before the firmware forwarded the lidar's end angle. Still
/// decoded so older .bin captures open; they get `END_ANGLE_UNKNOWN`.
pub const LEGACY_SAMPLE_TAG: u8 = 0x20;
pub const LEGACY_SAMPLE_LEN: usize = 32;
pub const TELEM_TAG: u8 = 0x10;
pub const TELEM_LEN: usize = 16;
pub const CONFIG_TAG: u8 = 0x14;
pub const CONFIG_LEN: usize = 20;
pub const EVENT_TAG: u8 = 0x09;
/// A chunk of the calibration blob stored in the scanner's flash.
pub const CALIB_TAG: u8 = 0x0C;
pub const CALIB_HDR_LEN: usize = 13;
/// The longest blob the firmware will store (CALIB_MAX_LEN).
pub const CALIB_MAX_LEN: usize = 2048;

/// Raw lidar angle -> degrees: (raw - 0xA000) / 64, one turn over
/// 40960..64000.
pub const RAW_ANGLE_MIN: u16 = 40960;
pub const RAW_ANGLE_MAX: u16 = 64000;
pub const SPEED_SCALE: f64 = 64.0;
/// end_angle of a legacy sample. Below RAW_ANGLE_MIN, so no real angle is it.
pub const END_ANGLE_UNKNOWN: u16 = 0;

pub const POINTS: usize = 8;
pub const DIST_INVALID: u16 = 0x8000;
pub const DIST_MASK: u16 = 0x7FFF;

// Host -> MCU commands: one letter, an optional decimal argument, newline.
pub const CMD_START: char = 's';
pub const CMD_ABORT: char = 'x';
pub const CMD_HOME: char = 'h';
pub const CMD_STATUS: char = '?';
pub const CMD_ANGLE: char = 'a';
pub const CMD_TIME: char = 't';
pub const CMD_MODE: char = 'm';
pub const CMD_STEPS: char = 'n';
pub const CMD_DWELL: char = 'd';
pub const CMD_UNWRAP: char = 'u';
pub const CMD_BEEP: char = 'b';
pub const CMD_CALIB_READ: char = 'c';
pub const CMD_CALIB_WRITE: char = 'w';

pub const MODE_CONTINUOUS: u8 = 0;
pub const MODE_STEPPED: u8 = 1;

// Scanner state machine (scanner.h).
pub const STATE_IDLE: u8 = 0;
pub const STATE_PARKING: u8 = 1;
pub const STATE_SETTLING: u8 = 2;
pub const STATE_SWEEPING: u8 = 3;
pub const STATE_DONE: u8 = 4;
pub const STATE_STEP_MOVE: u8 = 5;
pub const STATE_STEP_SETTLE: u8 = 6;
pub const STATE_STEP_CAPTURE: u8 = 7;
pub const STATE_UNWRAP: u8 = 8;
pub const STATE_HOMING: u8 = 9;

/// States whose lidar frames carry an angle worth trusting.
pub fn is_capture_state(s: u8) -> bool {
    s == STATE_SWEEPING || s == STATE_STEP_CAPTURE
}

/// States in which the platform is working its way across the sweep.
pub fn is_sweep_state(s: u8) -> bool {
    is_capture_state(s) || s == STATE_STEP_MOVE || s == STATE_STEP_SETTLE
}

/// One lidar frame plus the shaft angle it was taken at.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t_us: u32,
    pub platform: f32,
    pub speed: u16,
    pub raw_angle: u16,
    pub dist: [u16; POINTS],
    pub end_angle: u16,
    /// Whether the scanner was in a capture state when this frame arrived:
    /// the 10 Hz telemetry state held forward onto the sample stream.
    pub capturing: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Telem {
    pub t_us: u32,
    pub platform: f32,
    pub state: u8,
    pub dropped: u16,
    /// How many samples had been stored when this record arrived, so a sweep
    /// can be cut out by arrival order instead of by a timestamp that wraps.
    pub sample_index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Config {
    pub degrees: f32,
    pub time: f32,
    pub mode: u8,
    pub steps: u16,
    pub settle_ms: u16,
    pub capture_ms: u16,
}

/// One piece of the stored calibration; `total` 0 means nothing is stored.
#[derive(Clone, Debug)]
pub struct CalibChunk {
    pub total: u16,
    pub offset: u16,
    pub crc: u32,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug)]
pub enum Record {
    Sample(Sample),
    Telem(Telem),
    Config(Config),
    Event(String),
    Calib(CalibChunk),
}

#[inline]
fn rd16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
#[inline]
fn rd32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
#[inline]
fn rdf32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// Incremental decoder that keeps its buffer between feeds.
///
/// Reads off a serial port land on arbitrary boundaries, so a record is
/// routinely split across two of them; the partial tail is kept for the next
/// feed.
pub struct StreamParser {
    buf: Vec<u8>,
    /// Drop frames that arrive outside a capture state instead of storing
    /// them. The firmware streams lidar frames in every state, so a connected
    /// link would otherwise append samples forever.
    pub sweep_only: bool,
    /// Last state telemetry reported.
    pub state: Option<u8>,
    /// Keep one frame past the end of each run: a legacy frame's angular width
    /// is the gap to its successor.
    grace: bool,
    pub bytes_in: u64,
    pub junk: u64,
    pub skipped: u64,
    /// Samples handed out so far, for Telem::sample_index.
    samples_out: usize,
}

impl StreamParser {
    pub fn new(sweep_only: bool) -> Self {
        StreamParser {
            buf: Vec::with_capacity(1 << 16),
            sweep_only,
            state: None,
            grace: false,
            bytes_in: 0,
            junk: 0,
            skipped: 0,
            samples_out: 0,
        }
    }

    /// Resets the arrival counter (the consumer started a fresh capture).
    pub fn restart_count(&mut self) {
        self.samples_out = 0;
    }

    /// Absorbs bytes and hands every completed record to `sink`.
    pub fn feed(&mut self, chunk: &[u8], mut sink: impl FnMut(Record)) {
        if !chunk.is_empty() {
            self.buf.extend_from_slice(chunk);
            self.bytes_in += chunk.len() as u64;
        }
        let n = self.buf.len();
        let mut i = 0usize;
        loop {
            // Find the next magic.
            let j = match find_magic(&self.buf, i) {
                Some(j) => j,
                None => {
                    let keep = i.max(n.saturating_sub(MAGIC.len() - 1));
                    self.junk += (keep - i) as u64;
                    i = keep;
                    break;
                }
            };
            if j + 4 > n {
                i = j;
                break;
            }
            let tag = self.buf[j + 3];
            match tag {
                SAMPLE_TAG | LEGACY_SAMPLE_TAG => {
                    let len = if tag == SAMPLE_TAG { SAMPLE_LEN } else { LEGACY_SAMPLE_LEN };
                    if j + len > n {
                        i = j;
                        break;
                    }
                    let capturing = self.state.map_or(true, is_capture_state);
                    if self.sweep_only && !capturing && !self.grace {
                        self.skipped += 1;
                    } else {
                        self.grace = false;
                        let b = &self.buf[j..j + len];
                        let mut dist = [0u16; POINTS];
                        for (k, d) in dist.iter_mut().enumerate() {
                            *d = rd16(b, 16 + 2 * k);
                        }
                        let end_angle = if tag == SAMPLE_TAG { rd16(b, 32) } else { END_ANGLE_UNKNOWN };
                        let s = Sample {
                            t_us: rd32(b, 4),
                            platform: rdf32(b, 8),
                            speed: rd16(b, 12),
                            raw_angle: rd16(b, 14),
                            dist,
                            end_angle,
                            capturing,
                        };
                        self.samples_out += 1;
                        sink(Record::Sample(s));
                    }
                    i = j + len;
                }
                TELEM_TAG => {
                    if j + TELEM_LEN > n {
                        i = j;
                        break;
                    }
                    let b = &self.buf[j..j + TELEM_LEN];
                    let t = Telem {
                        t_us: rd32(b, 4),
                        platform: rdf32(b, 8),
                        state: b[12],
                        dropped: rd16(b, 14),
                        sample_index: self.samples_out,
                    };
                    let was = self.state;
                    self.state = Some(t.state);
                    if was.map_or(false, is_capture_state) && !is_capture_state(t.state) {
                        self.grace = true;
                    }
                    sink(Record::Telem(t));
                    i = j + TELEM_LEN;
                }
                CONFIG_TAG => {
                    if j + CONFIG_LEN > n {
                        i = j;
                        break;
                    }
                    let b = &self.buf[j..j + CONFIG_LEN];
                    let c = Config {
                        degrees: rdf32(b, 4),
                        time: rdf32(b, 8),
                        mode: b[12],
                        steps: rd16(b, 14),
                        settle_ms: rd16(b, 16),
                        capture_ms: rd16(b, 18),
                    };
                    sink(Record::Config(c));
                    i = j + CONFIG_LEN;
                }
                EVENT_TAG => {
                    if j + 5 > n {
                        i = j;
                        break;
                    }
                    let ln = self.buf[j + 4] as usize;
                    if j + 5 + ln > n {
                        i = j;
                        break;
                    }
                    let text = String::from_utf8_lossy(&self.buf[j + 5..j + 5 + ln]).into_owned();
                    sink(Record::Event(text));
                    i = j + 5 + ln;
                }
                CALIB_TAG => {
                    if j + CALIB_HDR_LEN > n {
                        i = j;
                        break;
                    }
                    let ln = self.buf[j + 12] as usize;
                    if j + CALIB_HDR_LEN + ln > n {
                        i = j;
                        break;
                    }
                    let b = &self.buf[j..j + CALIB_HDR_LEN + ln];
                    sink(Record::Calib(CalibChunk {
                        total: rd16(b, 4),
                        offset: rd16(b, 6),
                        crc: rd32(b, 8),
                        data: b[CALIB_HDR_LEN..].to_vec(),
                    }));
                    i = j + CALIB_HDR_LEN + ln;
                }
                _ => {
                    // Not one of ours; the magic was a coincidence in a payload.
                    self.junk += 1;
                    i = j + 1;
                }
            }
        }
        self.buf.drain(..i);
    }
}

fn find_magic(buf: &[u8], from: usize) -> Option<usize> {
    let n = buf.len();
    let mut i = from;
    while i + 2 < n {
        match buf[i..n - 2].iter().position(|&b| b == MAGIC[0]) {
            None => return None,
            Some(p) => {
                let j = i + p;
                if buf[j + 1] == MAGIC[1] && buf[j + 2] == MAGIC[2] {
                    return Some(j);
                }
                i = j + 1;
            }
        }
    }
    None
}

/// Raw angle -> degrees in [0, 360).
#[inline]
pub fn to_degrees(raw: u16) -> f64 {
    let span = (RAW_ANGLE_MAX - RAW_ANGLE_MIN) as f64;
    ((raw as f64 - RAW_ANGLE_MIN as f64) * 360.0 / span).rem_euclid(360.0)
}

/// Encodes one host -> MCU command line.
pub fn command(letter: char, arg: Option<String>) -> Vec<u8> {
    let mut s = String::new();
    s.push(letter);
    if let Some(a) = arg {
        s.push_str(&a);
    }
    s.push('\n');
    s.into_bytes()
}

/// Encodes a calibration write: the 'w' line, then the blob itself raw.
pub fn calib_write(blob: &[u8]) -> Vec<u8> {
    let mut v = command(CMD_CALIB_WRITE, Some(format!("{},{}", blob.len(), crc32(blob))));
    v.extend_from_slice(blob);
    v
}

/// CRC-32 (IEEE, reflected), as the firmware computes it.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ (0xEDB8_8320 & (c & 1).wrapping_neg());
        }
    }
    !c
}

/// Puts the chunks of one calibration read back together.
///
/// Chunks arrive in order, but a read can be cut short (a new read or write
/// restarts it) or overlap another, so each is placed by its offset and the
/// blob only counts once every byte is there and the CRC agrees.
#[derive(Default)]
pub struct CalibAssembler {
    crc: u32,
    buf: Vec<u8>,
    have: Vec<bool>,
}

impl CalibAssembler {
    /// Some(blob) once complete: an empty blob means the device holds none.
    pub fn push(&mut self, c: &CalibChunk) -> Option<Vec<u8>> {
        let total = c.total as usize;
        if total == 0 {
            *self = CalibAssembler::default();
            return Some(Vec::new());
        }
        if self.buf.len() != total || self.crc != c.crc {
            self.crc = c.crc;
            self.buf = vec![0; total];
            self.have = vec![false; total];
        }
        let off = c.offset as usize;
        let end = (off + c.data.len()).min(total);
        if off >= end {
            return None;
        }
        self.buf[off..end].copy_from_slice(&c.data[..end - off]);
        self.have[off..end].iter_mut().for_each(|h| *h = true);
        if !self.have.iter().all(|&h| h) {
            return None;
        }
        let blob = std::mem::take(&mut self.buf);
        *self = CalibAssembler::default();
        (crc32(&blob) == c.crc).then_some(blob)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_bytes(t: u32, plat: f32) -> Vec<u8> {
        let mut v = vec![0x55, 0xAA, 0x03, SAMPLE_TAG];
        v.extend_from_slice(&t.to_le_bytes());
        v.extend_from_slice(&plat.to_le_bytes());
        v.extend_from_slice(&23053u16.to_le_bytes());
        v.extend_from_slice(&41000u16.to_le_bytes());
        for k in 0..8u16 {
            v.extend_from_slice(&(1000 + k).to_le_bytes());
        }
        v.extend_from_slice(&41300u16.to_le_bytes());
        v
    }

    fn telem_bytes(t: u32, state: u8) -> Vec<u8> {
        let mut v = vec![0x55, 0xAA, 0x03, TELEM_TAG];
        v.extend_from_slice(&t.to_le_bytes());
        v.extend_from_slice(&1.5f32.to_le_bytes());
        v.push(state);
        v.push(0);
        v.extend_from_slice(&0u16.to_le_bytes());
        v
    }

    fn calib_bytes(total: u16, offset: u16, crc: u32, data: &[u8]) -> Vec<u8> {
        let mut v = vec![0x55, 0xAA, 0x03, CALIB_TAG];
        v.extend_from_slice(&total.to_le_bytes());
        v.extend_from_slice(&offset.to_le_bytes());
        v.extend_from_slice(&crc.to_le_bytes());
        v.push(data.len() as u8);
        v.extend_from_slice(data);
        v
    }

    #[test]
    fn crc_matches_zlib() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn calibration_round_trip() {
        let blob: Vec<u8> = (0..150u32).map(|k| (k * 7) as u8).collect();
        let crc = crc32(&blob);
        let mut data = b"junk".to_vec();
        for off in (0..blob.len()).step_by(64) {
            let end = (off + 64).min(blob.len());
            data.extend(calib_bytes(blob.len() as u16, off as u16, crc, &blob[off..end]));
            data.extend(telem_bytes(1, STATE_IDLE));
        }
        let mut p = StreamParser::new(true);
        let mut asm = CalibAssembler::default();
        let mut got = None;
        for c in data.chunks(5) {
            p.feed(c, |r| {
                if let Record::Calib(ch) = r {
                    if let Some(b) = asm.push(&ch) {
                        got = Some(b);
                    }
                }
            });
        }
        assert_eq!(got.as_deref(), Some(blob.as_slice()));
        // Nothing stored.
        assert_eq!(asm.push(&CalibChunk { total: 0, offset: 0, crc: 0, data: vec![] }), Some(vec![]));
        // A corrupt blob is not handed out.
        let bad = CalibChunk { total: 3, offset: 0, crc: 1, data: vec![1, 2, 3] };
        assert_eq!(asm.push(&bad), None);
        let w = calib_write(b"abc");
        assert_eq!(&w[..], format!("w3,{}\nabc", crc32(b"abc")).as_bytes());
    }

    #[test]
    fn split_records_and_sweep_filter() {
        let mut data = b"boot junk".to_vec();
        data.extend(telem_bytes(10, STATE_SWEEPING));
        data.extend(sample_bytes(20, 3.0));
        data.extend(sample_bytes(30, 4.0));
        data.extend(telem_bytes(40, STATE_DONE));
        data.extend(sample_bytes(50, 5.0)); // grace frame
        data.extend(sample_bytes(60, 6.0)); // dropped
        let mut p = StreamParser::new(true);
        let mut got = Vec::new();
        // Feed in awkward pieces.
        for c in data.chunks(7) {
            p.feed(c, |r| got.push(r));
        }
        let samples: Vec<_> = got
            .iter()
            .filter_map(|r| if let Record::Sample(s) = r { Some(*s) } else { None })
            .collect();
        assert_eq!(samples.len(), 3);
        assert!(samples[0].capturing && samples[1].capturing);
        assert!(!samples[2].capturing);
        assert_eq!(samples[0].dist[7], 1007);
        assert_eq!(p.skipped, 1);
    }
}
