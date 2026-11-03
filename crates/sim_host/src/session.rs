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
    /// Everything each UART transmitted, as text (lossy UTF-8 per byte).
    pub uart_text: [String; 2],
    /// Callback for complete UART lines: `(virtual time, uart, line)`.
    pub on_uart_line: Option<UartHook>,
    uart_line: [String; 2],
    /// Pseudo terminals attached to the UARTs.
    pub uart_pty: [Option<crate::pty::Pty>; 2],
}

/// Callback for complete UART output lines.
pub type UartHook = Box<dyn FnMut(Time, usize, &str) + Send>;

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
            uart_text: [String::new(), String::new()],
            on_uart_line: None,
            uart_line: [String::new(), String::new()],
            uart_pty: [None, None],
        }
    }

    pub fn has_defmt(&self) -> bool {
        self.defmt.is_some()
    }

    /// Collects what the UARTs transmitted since the last poll.
    pub fn poll_uarts(&mut self) {
        for n in 0..2 {
            if let Some(pty) = &self.uart_pty[n] {
                let input = pty.read();
                if !input.is_empty() {
                    let now = self.machine.now;
                    self.machine.sys.uart_inject(n, &input, now);
                }
            }
            let out = std::mem::take(&mut self.machine.sys.uart[n].output);
            if let Some(pty) = &self.uart_pty[n] {
                let bytes: Vec<u8> = out.iter().map(|(_, b)| *b).collect();
                pty.write(&bytes);
            }
            for (t, b) in out {
                let c = b as char;
                self.uart_text[n].push(c);
                match b {
                    b'\n' => {
                        let line = std::mem::take(&mut self.uart_line[n]);
                        if let Some(hook) = &mut self.on_uart_line {
                            hook(t, n, line.trim_end_matches('\r'));
                        }
                    }
                    _ => self.uart_line[n].push(c),
                }
            }
        }
    }

    /// Flushes partial UART lines to the hook (end of a run).
    pub fn flush_uarts(&mut self) {
        let now = self.machine.now;
        for n in 0..2 {
            let line = std::mem::take(&mut self.uart_line[n]);
            if !line.is_empty()
                && let Some(hook) = &mut self.on_uart_line
            {
                hook(now, n, &line);
            }
        }
    }

    /// Drains RTT now.
    pub fn poll_rtt(&mut self) {
        self.poll_uarts();
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
