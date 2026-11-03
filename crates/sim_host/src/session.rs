//! A running board: the machine plus the host side of its debug channels (RTT/defmt), run in short slices of
//! virtual time so the RTT buffer is drained as a probe would drain it.

use rp2040::{Machine, StopReason};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::{BusRecord, CanBus, CanEndpoint};

use crate::defmt::{DefmtDecoder, LogLine};
use crate::rtt::RttReader;

/// Callback for decoded log lines: `(virtual time, line)`.
pub type LogHook = Box<dyn FnMut(Time, &LogLine) + Send>;

/// Called with every frame completed on a bus.
pub type CanCallback = Box<dyn FnMut(&BusRecord) + Send>;

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
    /// Everything the device sent on its USB CDC ACM port (when the USB host has opened it), as text.
    pub usb_cdc_text: String,
    /// Callback for complete lines from the USB CDC port.
    pub on_usb_cdc_line: Option<LineHook>,
    usb_cdc_line: String,
    /// A pseudo terminal attached to the USB CDC port (host side of the device's serial port).
    pub usb_cdc_pty: Option<crate::pty::Pty>,
    /// The CAN bus the board's CAN controllers sit on, if any.
    pub can_bus: Option<CanBus>,
    /// Host participants on that bus (bridges, mocks).
    pub can_endpoints: Vec<Box<dyn CanEndpoint + Send>>,
    /// Called with every frame that went over the bus.
    pub on_can: Option<CanCallback>,
    /// Host participants on the board's Ethernet cable (Puddles), if it has one.
    pub eth_endpoints: Vec<Box<dyn crate::ethernet::EthEndpoint>>,
    /// Keep virtual time from running ahead of the wall clock.
    pub realtime: bool,
    pacing: Option<(std::time::Instant, Time)>,
    behind_warned: std::time::Instant,
}

/// Callback for complete lines of text: `(virtual time, line)`.
pub type LineHook = Box<dyn FnMut(Time, &str) + Send>;

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
            usb_cdc_text: String::new(),
            on_usb_cdc_line: None,
            usb_cdc_line: String::new(),
            usb_cdc_pty: None,
            can_bus: None,
            can_endpoints: Vec::new(),
            on_can: None,
            eth_endpoints: Vec::new(),
            realtime: false,
            pacing: None,
            behind_warned: std::time::Instant::now(),
        }
    }

    /// A copy of the session that runs on independently: the machine (see [`Machine::fork`]), the RTT position,
    /// the logs and UART text so far, and the CAN bus. Hooks, host endpoints and ptys are not copied.
    pub fn snapshot(&self) -> Session {
        let mut machine = self.machine.fork();
        if let Some(old) = &self.machine.sys.ethernet {
            let o = old.lock().unwrap();
            let new = std::sync::Arc::new(std::sync::Mutex::new(sim_devices::w5500::WireState {
                to_device: o.to_device.clone(),
                from_device: o.from_device.clone(),
                link_up: o.link_up,
                dropped: o.dropped,
            }));
            for a in machine.sys.spi_devices.iter_mut() {
                if let Some(w) = a.dev.as_any_mut().and_then(|d| d.downcast_mut::<sim_devices::w5500::W5500>()) {
                    w.wire = new.clone();
                }
            }
            machine.sys.ethernet = Some(new);
        }
        Session {
            machine,
            rtt: self.rtt.clone(),
            defmt: self.defmt.as_ref().map(DefmtDecoder::fresh),
            logs: self.logs.clone(),
            rtt_text: self.rtt_text.clone(),
            on_log: None,
            slice: self.slice,
            keep_logs: self.keep_logs,
            uart_text: self.uart_text.clone(),
            on_uart_line: None,
            uart_line: self.uart_line.clone(),
            uart_pty: [None, None],
            usb_cdc_text: self.usb_cdc_text.clone(),
            on_usb_cdc_line: None,
            usb_cdc_line: self.usb_cdc_line.clone(),
            usb_cdc_pty: None,
            can_bus: self.can_bus.clone(),
            can_endpoints: Vec::new(),
            on_can: None,
            eth_endpoints: Vec::new(),
            realtime: self.realtime,
            pacing: None,
            behind_warned: std::time::Instant::now(),
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

    /// Collects what the device sent on its USB CDC port and passes pty input to it. Does nothing unless the USB
    /// host has opened the port ([`sim_devices::usb::host::HostController::cdc_open`]).
    pub fn poll_usb_cdc(&mut self) {
        let now = self.machine.now;
        if let Some(pty) = &self.usb_cdc_pty {
            let input = pty.read();
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(&input));
            }
        }
        let Some(host) = self.machine.sys.usb_host_mut() else { return };
        if host.cdc.is_none() {
            return;
        }
        let rx = host.cdc_take_rx();
        if rx.is_empty() {
            return;
        }
        if let Some(pty) = &self.usb_cdc_pty {
            pty.write(&rx);
        }
        for b in rx {
            let c = b as char;
            self.usb_cdc_text.push(c);
            match b {
                b'\n' => {
                    let line = std::mem::take(&mut self.usb_cdc_line);
                    if let Some(hook) = &mut self.on_usb_cdc_line {
                        hook(now, line.trim_end_matches('\r'));
                    }
                }
                _ => self.usb_cdc_line.push(c),
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
        self.poll_usb_cdc();
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

    /// Runs the CAN bus up to the machine's current time and lets the devices react to what arrived.
    pub fn step_can(&mut self) {
        self.step_ethernet();
        let Some(bus) = &mut self.can_bus else {
            self.machine.update_devices();
            return;
        };
        let now = self.machine.now;
        let mut eps: Vec<&mut dyn CanEndpoint> =
            self.can_endpoints.iter_mut().map(|e| &mut **e as &mut dyn CanEndpoint).collect();
        let mut nodes = self.machine.can_nodes();
        let before = bus.record.as_ref().map_or(0, |r| r.len());
        bus.step(&mut nodes, &mut eps, now);
        if let (Some(rec), Some(hook)) = (&bus.record, &mut self.on_can) {
            for r in &rec[before..] {
                hook(r);
            }
        }
        if bus.record.as_ref().is_some_and(|r| r.len() > 100_000) {
            // Keep memory bounded on long runs; callers that need the history read it as they go
            bus.record.as_mut().unwrap().drain(..50_000);
        }
        self.machine.update_devices();
    }

    /// Carries frames between the board's Ethernet cable and the host endpoints.
    pub fn step_ethernet(&mut self) {
        let Some(wire) = &self.machine.sys.ethernet else { return };
        let now = self.machine.now;
        let frames = std::mem::take(&mut wire.lock().unwrap().from_device);
        for (t, f) in &frames {
            for e in self.eth_endpoints.iter_mut() {
                e.receive(f, *t);
            }
        }
        let mut w = wire.lock().unwrap();
        for e in self.eth_endpoints.iter_mut() {
            w.to_device.extend(e.take_tx(now));
        }
        if w.to_device.len() > 1000 {
            w.to_device.clear();
        }
    }

    /// Runs until virtual time `end`, polling RTT every [`Session::slice`].
    pub fn run_until(&mut self, end: Time) -> StopReason {
        loop {
            let target = (self.machine.now + self.slice).min(end);
            let reason = self.machine.run_until(target);
            self.step_can();
            self.poll_rtt();
            if self.realtime {
                self.pace();
            }
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

    /// Sleeps until the wall clock catches up with virtual time; warns when the emulation falls behind.
    fn pace(&mut self) {
        let now = self.machine.now;
        let (wall0, virt0) = *self.pacing.get_or_insert((std::time::Instant::now(), now));
        let virt = std::time::Duration::from_nanos((now - virt0).as_ns());
        let wall = wall0.elapsed();
        if virt > wall {
            std::thread::sleep(virt - wall);
        } else if wall - virt > std::time::Duration::from_millis(50) && self.behind_warned.elapsed().as_secs() >= 5 {
            self.behind_warned = std::time::Instant::now();
            log::warn!("real-time mode: emulation is {:?} behind the wall clock", wall - virt);
        }
    }

    /// Whether any log line so far contains `text`.
    pub fn logged(&self, text: &str) -> bool {
        self.logs.iter().any(|(_, l)| l.message.contains(text))
    }
}
