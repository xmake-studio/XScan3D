//! Everything decoded from one scan's stream.

use crate::protocol::{is_capture_state, Config, Record, Sample, StreamParser, Telem};

#[derive(Clone, Default)]
pub struct Capture {
    pub samples: Vec<Sample>,
    pub telem: Vec<Telem>,
    /// Latest config the device reported.
    pub config: Option<Config>,
    pub events: Vec<String>,
}

impl Capture {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn push(&mut self, rec: Record) {
        match rec {
            Record::Sample(s) => self.samples.push(s),
            Record::Telem(t) => self.telem.push(t),
            Record::Config(c) => self.config = Some(c),
            Record::Event(e) => self.events.push(e),
            Record::Calib(_) => {}
        }
    }

    /// Decodes a whole recorded stream, keeping only the last capture run.
    pub fn from_bytes(data: &[u8]) -> Capture {
        let mut cap = Capture::new();
        let mut parser = StreamParser::new(true);
        parser.feed(data, |r| cap.push(r));
        cap.keep_last_sweep();
        cap
    }

    /// Drops everything decoded before the most recent capture run.
    ///
    /// A recorded session can hold more than one sweep; replaying it has to
    /// keep just the last one, or the sweeps all land in one cloud on top of
    /// each other. Cut by arrival order rather than by timestamp, because
    /// micros() wraps every 71 minutes. Returns the number of samples dropped.
    pub fn keep_last_sweep(&mut self) -> usize {
        if self.telem.is_empty() {
            return 0;
        }
        let mut start: Option<(usize, usize)> = None; // (telem index, sample index)
        let mut was = false;
        for (k, t) in self.telem.iter().enumerate() {
            let now = is_capture_state(t.state);
            if now && !was {
                start = Some((k, t.sample_index));
            }
            was = now;
        }
        let Some((tk, si)) = start else { return 0 };
        let si = si.min(self.samples.len());
        if si == 0 && tk == 0 {
            return 0;
        }
        self.samples.drain(..si);
        self.telem.drain(..tk);
        for t in &mut self.telem {
            t.sample_index = t.sample_index.saturating_sub(si);
        }
        si
    }

    /// Keeps every `stride`-th sample (for fits that do not need the density).
    pub fn thinned(&self, stride: usize) -> Capture {
        let stride = stride.max(1);
        Capture {
            samples: self.samples.iter().step_by(stride).copied().collect(),
            telem: self.telem.clone(),
            config: self.config,
            events: Vec::new(),
        }
    }

    /// Seconds between the first and the last capturing frame.
    pub fn sweep_seconds(&self) -> f64 {
        let mut first: Option<u32> = None;
        let mut last: Option<u32> = None;
        for s in &self.samples {
            if s.capturing {
                if first.is_none() {
                    first = Some(s.t_us);
                }
                last = Some(s.t_us);
            }
        }
        match (first, last) {
            (Some(a), Some(b)) => b.wrapping_sub(a) as f64 / 1e6,
            _ => 0.0,
        }
    }
}
