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
    /// A pseudo terminal attached to the device's second USB CDC port, e.g. titan_dual_stdio_usb's secondary port,
    /// which the micro-ROS USB transport uses (a raw byte stream: no line hook). The host must have opened port 1.
    pub usb_cdc2_pty: Option<crate::pty::Pty>,
    /// A USB/IP server exporting the board's USB device to the real host.
    pub usbip: Option<crate::usbip::UsbIpServer>,
    /// The CAN bus the board's CAN controllers sit on, if any.
    pub can_bus: Option<CanBus>,
    /// Host participants on that bus (bridges, mocks).
    pub can_endpoints: Vec<Box<dyn CanEndpoint + Send>>,
    /// Called with every frame that went over the bus.
    pub on_can: Option<CanCallback>,
    /// Host participants on the board's Ethernet cable (Puddles), if it has one.
    pub eth_endpoints: Vec<Box<dyn crate::ethernet::EthEndpoint>>,
    /// Frames lost on the way between the board's Ethernet cable and the host endpoints (fault injection).
    pub eth_loss: Option<crate::fault::PacketLoss>,
    /// Keep virtual time from running ahead of the wall clock.
    pub realtime: bool,
    pacing: Pacing,
    /// A sampling profiler of the firmware (see [`crate::profile`]); `None` runs the machine directly.
    pub profiler: Option<crate::profile::Profiler>,
    /// When this is set, [`Session::run_until`] returns [`StopReason::Requested`] at the next slice boundary (the
    /// command line's [`crate::signal::STOP`], set by SIGINT or SIGTERM).
    pub stop_flag: Option<&'static std::sync::atomic::AtomicBool>,
    /// Whether [`Session::run_until`] (and [`Session::run_machine_through_lockups`]) return
    /// [`StopReason::Lockup`] as soon as a core locks up (the default, for tests), or keep the board running as
    /// the chip does, until the watchdog resets it, and return it only when nothing on the chip can
    /// ([`Machine::lockup_is_final`]).
    pub stop_on_lockup: bool,
    /// Lockups the board went through without stopping (`stop_on_lockup` off).
    pub lockups: u64,
    /// The current lockup was returned as final (it is returned once; running on, the board stays locked up).
    final_lockup_reported: bool,
    /// The end of the slice a debugger stop interrupted ([`StopReason::is_debug_stop`]): the next run finishes that
    /// slice first, so stopping never changes where slices end (or anything else about the run).
    slice_end: Option<Time>,
    /// Where the last slice [`Session::run_until`] ran ended (the time it ran to; the machine's own time may be a
    /// little past it, as its last instruction may end later).
    boundary: Time,
}

/// A checkpoint's flash, SRAM and ROM contents, shared with the checkpoint before where they are the same: the
/// flash and ROM whole (the flash while [`rp2040::System::flash_writes`] says it may not have changed, or it
/// compares equal), the SRAM in 4 KB pieces.
#[derive(Clone, Debug)]
pub struct SharedMemory {
    flash: std::sync::Arc<Vec<u8>>,
    flash_writes: u64,
    sram: Vec<std::sync::Arc<[u8]>>,
    rom: std::sync::Arc<Vec<u8>>,
}

const SRAM_PIECE: usize = 4096;

impl SharedMemory {
    /// The memories now, sharing what is unchanged with `prev`; and how many bytes were not shared.
    fn keep(prev: Option<&SharedMemory>, flash: &[u8], sram: &[u8], rom: &[u8], writes: u64) -> (SharedMemory, usize) {
        use std::sync::Arc;
        let mut fresh = 0;
        let flash = match prev {
            Some(p) if p.flash_writes == writes && p.flash.len() == flash.len() => p.flash.clone(),
            Some(p) if *p.flash == flash => p.flash.clone(),
            _ => {
                fresh += flash.len();
                Arc::new(flash.to_vec())
            }
        };
        let rom = match prev {
            Some(p) if *p.rom == rom => p.rom.clone(),
            _ => {
                fresh += rom.len();
                Arc::new(rom.to_vec())
            }
        };
        let sram = sram
            .chunks(SRAM_PIECE)
            .enumerate()
            .map(|(i, piece)| match prev.and_then(|p| p.sram.get(i)) {
                Some(old) if **old == *piece => old.clone(),
                _ => {
                    fresh += piece.len();
                    Arc::from(piece)
                }
            })
            .collect();
        (SharedMemory { flash, flash_writes: writes, sram, rom }, fresh)
    }

    /// Copies the memories into `sys`.
    fn restore(&self, sys: &mut rp2040::System) {
        sys.flash.clone_from(&*self.flash);
        sys.rom.clone_from(&*self.rom);
        sys.sram.clear();
        for piece in &self.sram {
            sys.sram.extend_from_slice(piece);
        }
        sys.flash_writes = self.flash_writes;
    }
}

/// What a session collected (see [`Session::keep_collected`]).
#[derive(Clone, Default)]
pub struct Collected {
    logs: Vec<(Time, LogLine)>,
    rtt_text: Vec<(usize, Vec<u8>)>,
    uart_text: [String; 2],
    usb_cdc_text: String,
}

/// A [`Session::checkpoint`].
pub struct SessionCheckpoint {
    machine: rp2040::machine::Snapshot,
    memory: SharedMemory,
    /// Bytes of memory contents this checkpoint did not share with the one before it.
    fresh: usize,
    rtt: Option<RttReader>,
    defmt_partial: Vec<u8>,
    logs: usize,
    rtt_text: usize,
    uart_text: [usize; 2],
    uart_line: [String; 2],
    usb_cdc_text: usize,
    usb_cdc_line: String,
    can_bus: Option<(CanBus, Option<usize>)>,
    #[allow(clippy::type_complexity)]
    eth: Option<(std::collections::VecDeque<Vec<u8>>, Vec<(Time, Vec<u8>)>, bool, u64)>,
    eth_loss: Option<crate::fault::PacketLoss>,
    lockups: u64,
    final_lockup_reported: bool,
    slice_end: Option<Time>,
    boundary: Time,
}

impl SessionCheckpoint {
    /// The memories (shared with the next checkpoint where they do not change).
    pub fn memory(&self) -> &SharedMemory {
        &self.memory
    }

    /// About how much memory the checkpoint adds: what of the memories it does not share with the one before, and
    /// the rest (devices, peripherals).
    pub fn bytes(&self) -> usize {
        self.machine.bytes() + self.fresh + self.defmt_partial.len() + 4096
    }
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
            usb_cdc2_pty: None,
            usbip: None,
            can_bus: None,
            can_endpoints: Vec::new(),
            on_can: None,
            eth_endpoints: Vec::new(),
            eth_loss: None,
            realtime: false,
            pacing: Pacing::default(),
            profiler: None,
            stop_flag: None,
            stop_on_lockup: true,
            lockups: 0,
            final_lockup_reported: false,
            slice_end: None,
            boundary: Time::ZERO,
        }
    }

    /// A copy of the session that runs on independently: the machine (see [`Machine::fork`]), the RTT position,
    /// the logs and UART text so far, and the CAN bus. Hooks, host endpoints and ptys are not copied. The copied
    /// devices still read the board's physical inputs; [`Session::snapshot_with_inputs`] gives the copy its own.
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
            usb_cdc2_pty: None,
            usbip: None,
            can_bus: self.can_bus.clone(),
            can_endpoints: Vec::new(),
            on_can: None,
            eth_endpoints: Vec::new(),
            eth_loss: self.eth_loss.clone(),
            realtime: self.realtime,
            pacing: Pacing::default(),
            profiler: None,
            stop_flag: None,
            stop_on_lockup: self.stop_on_lockup,
            lockups: self.lockups,
            final_lockup_reported: self.final_lockup_reported,
            slice_end: self.slice_end,
            boundary: self.boundary,
        }
    }

    /// [`Session::snapshot`] for a board whose devices read `inputs` (from [`crate::board::Board::apply`]): the
    /// copy gets inputs of its own with the same values, returned with it, and its devices read those, so the
    /// original and the copy can run side by side with different inputs.
    pub fn snapshot_with_inputs(&self, inputs: &crate::board::Inputs) -> (Session, crate::board::Inputs) {
        let mut s = self.snapshot();
        let (copy, map) = inputs.deep_copy();
        crate::board::relink_inputs(&mut s.machine, &map);
        (s, copy)
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

    /// Collects what the device sent on its (first) USB CDC port and passes pty input to it. Does nothing unless
    /// the USB host has opened the port ([`sim_devices::usb::host::HostController::cdc_open`]).
    pub fn poll_usb_cdc(&mut self) {
        let now = self.machine.now;
        if let Some(pty) = &self.usb_cdc2_pty {
            let input = pty.read();
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(1, &input));
            }
            if let Some(host) = self.machine.sys.usb_host_mut() {
                let rx = host.cdc_take_rx(1);
                if !rx.is_empty() {
                    pty.write(&rx);
                }
            }
        }
        if let Some(pty) = &self.usb_cdc_pty {
            let input = pty.read();
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(0, &input));
            }
        }
        let Some(host) = self.machine.sys.usb_host_mut() else { return };
        let rx = host.cdc_take_rx(0);
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
            if self.eth_loss.as_mut().is_some_and(|l| l.drop_from_board()) {
                continue;
            }
            for e in self.eth_endpoints.iter_mut() {
                e.receive(f, *t);
            }
        }
        let mut w = wire.lock().unwrap();
        for e in self.eth_endpoints.iter_mut() {
            for f in e.take_tx(now) {
                if !self.eth_loss.as_mut().is_some_and(|l| l.drop_to_board()) {
                    w.to_device.push_back(f);
                }
            }
        }
        if w.to_device.len() > 1000 {
            w.to_device.clear();
        }
    }

    /// Runs until virtual time `end`, polling RTT every [`Session::slice`]. Returns [`StopReason::Requested`] at
    /// a slice boundary once [`Session::stop_flag`] is set. A debugger stop ([`StopReason::is_debug_stop`]) returns
    /// in the middle of a slice, before the slice's end (RTT, CAN, ...) is done: the next run goes on to the same
    /// slice end and does it there, as a run that never stopped would.
    pub fn run_until(&mut self, end: Time) -> StopReason {
        loop {
            if crate::signal::is_set(self.stop_flag) {
                return StopReason::Requested;
            }
            let target = self.next_boundary(end);
            self.slice_end = None;
            let reason = self.run_machine_through_lockups(target);
            if reason.is_debug_stop() {
                self.slice_end = Some(target);
                return reason;
            }
            if reason == StopReason::TimeLimit {
                self.boundary = target;
            }
            self.step_can();
            self.poll_rtt();
            if let Some(u) = &mut self.usbip {
                u.poll(&mut self.machine);
            }
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

    /// Where the slice the next [`Session::run_until`]`(end)` runs first ends: the end of a slice a debugger stop
    /// interrupted, else a [`Session::slice`] from now (and never past `end`).
    pub fn next_boundary(&self, end: Time) -> Time {
        self.slice_end.unwrap_or(self.machine.now + self.slice).min(end)
    }

    /// Where the last slice run ended (see [`Session::run_until`]): the session's slice boundary.
    pub fn boundary(&self) -> Time {
        let now = self.machine.now;
        // The machine was run (or moved) outside `run_until` since: where it is now is the boundary
        if self.slice_end.is_none() && (now < self.boundary || now >= self.boundary + self.slice) {
            now
        } else {
            self.boundary
        }
    }

    /// Whether the session is between slices (no slice was interrupted by a debugger stop).
    pub fn at_boundary(&self) -> bool {
        self.slice_end.is_none()
    }

    /// A checkpoint of the session for time travel (see [`crate::timetravel`]): the machine without its trace sink
    /// and debugger settings, and the host side's state (RTT position, a defmt frame in progress, how much log,
    /// RTT, UART and USB text there was, the CAN bus without its record, the Ethernet cable). The flash is shared
    /// with `prev` when it has not changed since. Hooks, endpoints and ptys are not part of it.
    pub fn checkpoint(&mut self, prev: Option<&SharedMemory>) -> SessionCheckpoint {
        let sys = &mut self.machine.sys;
        let (flash, sram, rom) =
            (std::mem::take(&mut sys.flash), std::mem::take(&mut sys.sram), std::mem::take(&mut sys.rom));
        let machine = self.machine.snapshot();
        let (memory, fresh) = SharedMemory::keep(prev, &flash, &sram, &rom, self.machine.sys.flash_writes);
        let sys = &mut self.machine.sys;
        (sys.flash, sys.sram, sys.rom) = (flash, sram, rom);
        let can_bus = self.can_bus.as_mut().map(|bus| {
            let record = bus.record.take();
            let len = record.as_ref().map(|r| r.len());
            let copy = bus.clone();
            bus.record = record;
            (copy, len)
        });
        let eth = self.machine.sys.ethernet.as_ref().map(|w| {
            let w = w.lock().unwrap();
            (w.to_device.clone(), w.from_device.clone(), w.link_up, w.dropped)
        });
        SessionCheckpoint {
            machine,
            memory,
            fresh,
            rtt: self.rtt.clone(),
            defmt_partial: self.defmt.as_ref().map(|d| d.partial().to_vec()).unwrap_or_default(),
            logs: self.logs.len(),
            rtt_text: self.rtt_text.len(),
            uart_text: [self.uart_text[0].len(), self.uart_text[1].len()],
            uart_line: self.uart_line.clone(),
            usb_cdc_text: self.usb_cdc_text.len(),
            usb_cdc_line: self.usb_cdc_line.clone(),
            can_bus,
            eth,
            eth_loss: self.eth_loss.clone(),
            lockups: self.lockups,
            final_lockup_reported: self.final_lockup_reported,
            slice_end: self.slice_end,
            boundary: self.boundary,
        }
    }

    /// Puts the session back where `c` was taken: the machine (keeping its trace sink, breakpoints and
    /// watchpoints), the host side, and the collected logs and text cut back to what there was then. Inputs the
    /// devices read are not part of it (see [`crate::timetravel`]).
    pub fn rewind_to(&mut self, c: &SessionCheckpoint) {
        self.rewind_to_with(c, &Collected::default());
    }

    /// Keeps in `c` what the session collected (logs, RTT, UART and USB text) if it has more than `c` has: going
    /// back cuts them, going forward again to a later checkpoint ([`Session::rewind_to_with`]) gives them back.
    pub fn keep_collected(&self, c: &mut Collected) {
        if self.logs.len() > c.logs.len() {
            c.logs.clone_from(&self.logs);
        }
        if self.rtt_text.len() > c.rtt_text.len() {
            c.rtt_text.clone_from(&self.rtt_text);
        }
        for n in 0..2 {
            if self.uart_text[n].len() > c.uart_text[n].len() {
                c.uart_text[n].clone_from(&self.uart_text[n]);
            }
        }
        if self.usb_cdc_text.len() > c.usb_cdc_text.len() {
            c.usb_cdc_text.clone_from(&self.usb_cdc_text);
        }
    }

    /// Cuts `c` to what the session has now (the future changed).
    pub fn cut_collected(&self, c: &mut Collected) {
        c.logs.truncate(self.logs.len());
        c.rtt_text.truncate(self.rtt_text.len());
        for n in 0..2 {
            if c.uart_text[n].is_char_boundary(self.uart_text[n].len().min(c.uart_text[n].len())) {
                c.uart_text[n].truncate(self.uart_text[n].len());
            }
        }
        if c.usb_cdc_text.is_char_boundary(self.usb_cdc_text.len().min(c.usb_cdc_text.len())) {
            c.usb_cdc_text.truncate(self.usb_cdc_text.len());
        }
    }

    /// [`Session::rewind_to`], taking what was collected by then from `collected` when the session has less now
    /// (it was taken back before the checkpoint).
    pub fn rewind_to_with(&mut self, c: &SessionCheckpoint, collected: &Collected) {
        fn cut<T: Clone>(v: &mut Vec<T>, len: usize, longer: &[T]) {
            if v.len() >= len {
                v.truncate(len);
            } else if longer.len() >= len {
                v.clear();
                v.extend_from_slice(&longer[..len]);
            }
        }
        fn cut_text(v: &mut String, len: usize, longer: &str) {
            if v.len() >= len {
                if v.is_char_boundary(len) {
                    v.truncate(len);
                }
            } else if longer.len() >= len && longer.is_char_boundary(len) {
                v.clear();
                v.push_str(&longer[..len]);
            }
        }
        cut(&mut self.logs, c.logs, &collected.logs);
        cut(&mut self.rtt_text, c.rtt_text, &collected.rtt_text);
        for n in 0..2 {
            cut_text(&mut self.uart_text[n], c.uart_text[n], &collected.uart_text[n]);
        }
        cut_text(&mut self.usb_cdc_text, c.usb_cdc_text, &collected.usb_cdc_text);
        self.machine.restore(&c.machine);
        c.memory.restore(&mut self.machine.sys);
        self.rtt = c.rtt.clone();
        if let Some(d) = &self.defmt {
            self.defmt = Some(d.with_partial(&c.defmt_partial));
        }
        self.uart_line = c.uart_line.clone();
        self.usb_cdc_line = c.usb_cdc_line.clone();
        if let (Some(bus), Some((copy, len))) = (&mut self.can_bus, &c.can_bus) {
            let mut record = bus.record.take();
            if let (Some(r), Some(len)) = (&mut record, len) {
                r.truncate(*len);
            }
            *bus = copy.clone();
            bus.record = record;
        }
        if let (Some(w), Some((to, from, link, dropped))) = (&self.machine.sys.ethernet, &c.eth) {
            let mut w = w.lock().unwrap();
            w.to_device = to.clone();
            w.from_device = from.clone();
            w.link_up = *link;
            w.dropped = *dropped;
        }
        self.eth_loss = c.eth_loss.clone();
        self.lockups = c.lockups;
        self.final_lockup_reported = c.final_lockup_reported;
        self.slice_end = c.slice_end;
        self.boundary = c.boundary;
    }

    /// Runs just the machine until `end` (through the profiler, if there is one), without polling anything.
    pub fn run_machine(&mut self, end: Time) -> StopReason {
        match &mut self.profiler {
            None => self.machine.run_until(end),
            Some(p) => p.run_until(&mut self.machine, end),
        }
    }

    /// [`Session::run_machine`], going on past a lockup unless [`Session::stop_on_lockup`] is set: the locked-up
    /// core stays stopped, the other core and the peripherals (the watchdog among them) go on, as on the chip.
    /// Returns [`StopReason::Lockup`] once when the lockup is final ([`Machine::lockup_is_final`]: nothing on the
    /// chip can end it), which is checked at the lockup and at `end`; called again, it runs on with the core locked
    /// up (until something outside, a power cycle, restarts the chip).
    pub fn run_machine_through_lockups(&mut self, end: Time) -> StopReason {
        loop {
            let reason = self.run_machine(end);
            if self.stop_on_lockup {
                return reason;
            }
            match reason {
                StopReason::Lockup { core, pc } => {
                    self.lockups += 1;
                    self.final_lockup_reported = self.machine.lockup_is_final();
                    if self.final_lockup_reported {
                        return reason;
                    }
                    let wd = match self.machine.sys.watchdog.timeout_due(self.machine.now) {
                        Some(t) => format!(" until the watchdog resets the chip at {t}"),
                        None => String::new(),
                    };
                    log::warn!(
                        "[{}] core{core} locked up at {pc:#010x}: it stays stopped{wd}; the rest of the chip runs on",
                        self.machine.now
                    );
                }
                StopReason::TimeLimit => {
                    // The chip may have run out of ways to end a lockup since (the watchdog stopped, the other
                    // core went to sleep in the bootrom)
                    let locked = (0..2).find(|&c| self.machine.core_enabled[c] && self.machine.cores[c].is_locked_up());
                    match locked {
                        None => self.final_lockup_reported = false,
                        Some(c) if !self.final_lockup_reported && self.machine.lockup_is_final() => {
                            self.final_lockup_reported = true;
                            let pc = sim_core::cpu::Cpu::pc(&self.machine.cores[c]);
                            return StopReason::Lockup { core: c as u8, pc };
                        }
                        Some(_) => {}
                    }
                    return reason;
                }
                other => return other,
            }
        }
    }

    /// Starts profiling the firmware with a sample every `interval` (on average).
    pub fn start_profile(&mut self, interval: Time) {
        self.profiler = Some(crate::profile::Profiler::new(&self.machine, interval));
    }

    pub fn run_for(&mut self, d: Time) -> StopReason {
        let end = self.machine.now + d;
        self.run_until(end)
    }

    /// Starts pacing afresh from now (after virtual time was held, e.g. paused from the dashboard), so the run does
    /// not race to catch up with the wall clock.
    pub fn reset_pacing(&mut self) {
        self.pacing.reset();
    }

    /// Sleeps until the wall clock catches up with virtual time; warns when the emulation falls behind.
    fn pace(&mut self) {
        self.pacing.pace(self.machine.now);
    }

    /// Whether any log line so far contains `text`.
    pub fn logged(&self, text: &str) -> bool {
        self.logs.iter().any(|(_, l)| l.message.contains(text))
    }
}

/// Real-time pacing (`--realtime`) of a board or a robot: keeps virtual time from running ahead of the wall clock,
/// counted from where pacing started, across calls (so runs in short pieces keep real time too), and warns when
/// the emulation cannot keep up.
#[derive(Clone, Debug, Default)]
pub(crate) struct Pacing {
    /// The wall clock and virtual time pacing started from.
    start: Option<(std::time::Instant, Time)>,
    /// When the last "behind the wall clock" warning was given.
    warned: Option<std::time::Instant>,
}

impl Pacing {
    /// Virtual time ahead of the wall clock by less than this is not slept away (there is one call per slice).
    const MIN_SLEEP: std::time::Duration = std::time::Duration::from_millis(2);
    /// Falling behind by more than this is warned about...
    const WARN_BEHIND: std::time::Duration = std::time::Duration::from_millis(100);
    /// ...at most this often.
    const WARN_EVERY: std::time::Duration = std::time::Duration::from_secs(5);

    /// Sleeps while virtual time `now` is ahead of the wall clock. While it is behind by more than
    /// [`Pacing::WARN_BEHIND`], warns (at most every [`Pacing::WARN_EVERY`]) and returns how far behind it is.
    pub(crate) fn pace(&mut self, now: Time) -> Option<std::time::Duration> {
        let (wall0, virt0) = *self.start.get_or_insert((std::time::Instant::now(), now));
        let virt = std::time::Duration::from_nanos((now - virt0).as_ns());
        let wall = wall0.elapsed();
        if virt > wall {
            if virt - wall > Self::MIN_SLEEP {
                std::thread::sleep(virt - wall);
            }
            return None;
        }
        let behind = wall - virt;
        if behind > Self::WARN_BEHIND && self.warned.is_none_or(|w| w.elapsed() >= Self::WARN_EVERY) {
            self.warned = Some(std::time::Instant::now());
            log::warn!("real-time mode: emulation is {behind:.2?} behind the wall clock");
            return Some(behind);
        }
        None
    }

    /// Starts afresh from the next call (virtual time was held, e.g. paused from the dashboard).
    pub(crate) fn reset(&mut self) {
        self.start = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn pacing_warns_when_behind_the_wall_clock_at_most_every_five_seconds() {
        let second_ago = Instant::now().checked_sub(Duration::from_secs(1)).unwrap();
        let mut p = Pacing { start: Some((second_ago, Time::ZERO)), warned: None };
        // 100 ms of virtual time in a second of wall time
        let behind = p.pace(Time::from_ms(100)).expect("a warning");
        assert!((Duration::from_millis(890)..Duration::from_millis(1500)).contains(&behind), "{behind:?}");
        assert_eq!(p.pace(Time::from_ms(110)), None, "not again so soon");
        p.warned = Instant::now().checked_sub(Duration::from_secs(6));
        assert!(p.pace(Time::from_ms(120)).is_some(), "again after five seconds");
    }

    #[test]
    fn pacing_sleeps_while_ahead() {
        let mut p = Pacing::default();
        let t0 = Instant::now();
        assert_eq!(p.pace(Time::ZERO), None, "the first call starts the clock");
        assert_eq!(p.pace(Time::from_ms(30)), None);
        assert!(t0.elapsed() >= Duration::from_millis(28), "{:?}", t0.elapsed());
        // A little behind is no reason for a warning
        assert_eq!(p.pace(Time::from_ms(30)), None);
    }
}
