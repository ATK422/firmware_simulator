//! A running board: the machine plus the host side of its debug channels (RTT/defmt), run in short slices of
//! virtual time so the RTT buffer is drained as a probe would drain it.

use rp2040::{Machine, StopReason};
use sim_core::image::Image;
use sim_core::time::Time;

use crate::defmt::{DefmtDecoder, LogLine};
use crate::rtt::RttReader;

/// Callback for decoded log lines: `(virtual time, line)`.
pub type LogHook = Box<dyn FnMut(Time, &LogLine) + Send>;

pub struct Session {
    pub machine: Machine,
    pub rtt: Option<RttReader>,
    defmt: Option<DefmtDecoder>,
    /// Every decoded log line with the virtual time it was read at.
    pub logs: Vec<(Time, LogLine)>,
    /// Raw bytes from RTT channels that are not defmt (channel 0 when the ELF has no `.defmt`, and channels 1+).
    pub rtt_text: Vec<(usize, Vec<u8>)>,
    pub on_log: Option<LogHook>,
    /// How much virtual time runs between RTT polls.
    pub slice: Time,
    /// Keep decoded lines in `logs` (tests); the CLI only prints them.
    pub keep_logs: bool,
}

impl Session {
    /// Wraps a machine that has `image` loaded, finding its RTT control block and defmt table.
    pub fn new(machine: Machine, image: &Image) -> Session {
        let rtt = RttReader::from_image(image);
        let defmt = image.elf.as_deref().and_then(|elf| match DefmtDecoder::from_elf(elf) {
            Ok(d) => d,
            Err(e) => {
                log::warn!("could not read the defmt table: {e}");
                None
            }
        });
        Session {
            machine,
            rtt,
            defmt,
            logs: Vec::new(),
            rtt_text: Vec::new(),
            on_log: None,
            slice: Time::from_us(100),
            keep_logs: true,
        }
    }

    pub fn has_defmt(&self) -> bool {
        self.defmt.is_some()
    }

    /// Drains RTT now.
    pub fn poll_rtt(&mut self) {
        let Some(rtt) = &self.rtt else { return };
        let now = self.machine.now;
        for (ch, bytes) in rtt.poll(&mut self.machine) {
            match (&mut self.defmt, ch) {
                (Some(d), 0) => {
                    for line in d.feed(&bytes) {
                        if let Some(hook) = &mut self.on_log {
                            hook(now, &line);
                        }
                        if self.keep_logs {
                            self.logs.push((now, line));
                        }
                    }
                }
                _ => self.rtt_text.push((ch, bytes)),
            }
        }
    }

    /// Runs until virtual time `end`, polling RTT every [`Session::slice`].
    pub fn run_until(&mut self, end: Time) -> StopReason {
        loop {
            let target = (self.machine.now + self.slice).min(end);
            let reason = self.machine.run_until(target);
            self.poll_rtt();
            match reason {
                StopReason::TimeLimit if target >= end => return StopReason::TimeLimit,
                StopReason::TimeLimit => {}
                other => return other,
            }
        }
    }

    pub fn run_for(&mut self, d: Time) -> StopReason {
        let end = self.machine.now + d;
        self.run_until(end)
    }

    /// Whether any log line so far contains `text`.
    pub fn logged(&self, text: &str) -> bool {
        self.logs.iter().any(|(_, l)| l.message.contains(text))
    }
}
