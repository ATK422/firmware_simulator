//! A running board: the machine plus the host side of its debug channels (RTT/defmt), run in short slices of
//! virtual time so the RTT buffer is drained as a probe would drain it.

use std::sync::Arc;

use rp2040::{Machine, StopReason};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::versioned::Versioned;
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
    /// The flash as the last checkpoint of either kind kept it ([`Session::keep_flash`]): later ones share it while
    /// the flash's version is the same.
    kept_flash: Option<Arc<Versioned>>,
    /// What [`Session::collect`] read from the pseudo terminals and has not been delivered yet.
    host_reads: HostReads,
}

/// What a board's host side read from its pseudo terminals while a robot ran it ahead ([`Session::collect`]),
/// kept until the boundary it was read at is delivered. A board rolled back to before a read gives the bytes back
/// ([`Session::unread_from`]): the next reads of that terminal take them first, so nothing typed is lost or taken
/// in out of order.
#[derive(Default)]
struct HostReads {
    /// (boundary, terminal, bytes), for the reads that got something.
    log: std::collections::VecDeque<(Time, usize, Vec<u8>)>,
    /// Per terminal (UART 0 and 1, CDC port 0 and 1): bytes given back, to be read first.
    unread: [Vec<u8>; 4],
}

/// The terminals [`Session::read_terminal`] reads.
const UART0: usize = 0;
const UART1: usize = 1;
const CDC0: usize = 2;
const CDC1: usize = 3;

/// A board's machine at one moment: what both kinds of checkpoint keep of it, a robot's rollback points for running
/// ahead ([`RollbackPoint`], many, short-lived and taken in the hot loop) and time travel's checkpoints
/// ([`SessionCheckpoint`], periodic and kept within a memory budget). The snapshot holds everything but the flash;
/// the flash is kept apart and shared between checkpoints while it is the same ([`Session::keep_flash`]: the
/// flash is a [`Versioned`] array, so that is a comparison of versions), and copied back on a restore only if the
/// live flash's version differs.
struct MachineState {
    snapshot: rp2040::machine::Snapshot,
    /// What the run loop carries from one run to the next: restored with the snapshot, the machine runs on exactly
    /// as it did, statistics included.
    carry: rp2040::RunCarry,
    flash: Arc<Versioned>,
    lockups: u64,
    final_lockup_reported: bool,
}

/// A board as it was at one moment of a robot running its boards ahead ([`Session::rollback_point`]), to roll it
/// back to ([`Session::roll_back`]) when it turns out to have run past where its window ended. Only what running
/// the machine changes: the host side takes in what the board produced later ([`Session::deliver`]). It also keeps
/// the debugger's positions ([`rp2040::Machine::stop_at`]) and the code coverage, so that what was rolled back
/// leaves no trace (only the committed run counts as covered).
pub(crate) struct RollbackPoint {
    machine: MachineState,
    stop_at: [Option<Time>; 2],
    coverage: Option<Box<rp2040::coverage::CoverageMap>>,
}

/// A time-travel checkpoint's SRAM and ROM contents and code coverage, shared with the checkpoint before where they
/// are the same (the SRAM in 4 KB pieces). (The flash is shared through its version: [`MachineState`].)
#[derive(Clone, Debug)]
pub struct SharedMemory {
    sram: Vec<Arc<[u8]>>,
    rom: Arc<Vec<u8>>,
    coverage: Option<Arc<rp2040::coverage::CoverageMap>>,
}

const SRAM_PIECE: usize = 4096;

impl SharedMemory {
    /// The memories and coverage now, sharing what is unchanged with `prev`; and how many bytes were not shared.
    fn keep(
        prev: Option<&SharedMemory>,
        sram: &[u8],
        rom: &[u8],
        coverage: Option<&rp2040::coverage::CoverageMap>,
    ) -> (SharedMemory, usize) {
        let mut fresh = 0;
        let coverage = coverage.map(|now| match prev.and_then(|p| p.coverage.as_ref()) {
            Some(old) if **old == *now => old.clone(),
            _ => {
                fresh += now.bytes();
                Arc::new(now.clone())
            }
        });
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
        (SharedMemory { sram, rom, coverage }, fresh)
    }

    /// Copies the memories and the coverage into `m`.
    fn restore(&self, m: &mut Machine) {
        let sys = &mut m.sys;
        sys.rom.clone_from(&*self.rom);
        sys.sram.clear();
        for piece in &self.sram {
            sys.sram.extend_from_slice(piece);
        }
        if let (Some(live), Some(kept)) = (&mut m.coverage, &self.coverage) {
            (**live).clone_from(kept);
        }
    }
}

/// What a session collected as text (logs, RTT, UART and USB text), kept by time travel so that going forward
/// again to a later checkpoint gives back what going back cut (see [`Session::keep_collected`]).
#[derive(Clone, Default)]
pub struct CollectedText {
    logs: Vec<(Time, LogLine)>,
    rtt_text: Vec<(usize, Vec<u8>)>,
    uart_text: [String; 2],
    usb_cdc_text: String,
}

/// A time-travel checkpoint of a board ([`Session::checkpoint`]).
pub struct SessionCheckpoint {
    machine: MachineState,
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
        self.machine.snapshot.bytes() + self.fresh + self.defmt_partial.len() + 4096
    }
}

/// What [`Session::collect`] took out of a board's machine at the end of a slice, for [`Session::deliver`].
#[derive(Default)]
pub(crate) struct SliceOutput {
    /// The boundary.
    at: Time,
    now: Time,
    /// The UARTs' output, taken twice (as the end of a slice polls them twice).
    uarts: [[Vec<(Time, u8)>; 2]; 2],
    usb_cdc: Vec<u8>,
    /// The second CDC port's output (only taken with a pseudo terminal on it, as polling does).
    usb_cdc2: Vec<u8>,
    rtt: Vec<(usize, Vec<u8>)>,
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
            kept_flash: None,
            host_reads: HostReads::default(),
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
            kept_flash: self.kept_flash.clone(),
            host_reads: HostReads::default(),
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

    /// Reads terminal `k` ([`UART0`], [`UART1`], [`CDC0`], [`CDC1`]): bytes given back by a rollback first, then
    /// what the pseudo terminal holds now.
    fn read_terminal(&mut self, k: usize) -> Vec<u8> {
        let pty = match k {
            UART0 => &self.uart_pty[0],
            UART1 => &self.uart_pty[1],
            CDC0 => &self.usb_cdc_pty,
            _ => &self.usb_cdc2_pty,
        };
        let mut got = std::mem::take(&mut self.host_reads.unread[k]);
        if let Some(p) = pty {
            got.extend(p.read());
        }
        got
    }

    /// Collects what the UARTs transmitted since the last poll.
    pub fn poll_uarts(&mut self) {
        for n in 0..2 {
            if self.uart_pty[n].is_some() {
                let input = self.read_terminal(UART0 + n);
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
            self.uart_bytes(n, out);
        }
    }

    /// Takes in what UART `n` transmitted: its text, and complete lines to the hook.
    fn uart_bytes(&mut self, n: usize, out: Vec<(Time, u8)>) {
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

    /// Collects what the device sent on its (first) USB CDC port and passes pty input to it. Does nothing unless
    /// the USB host has opened the port ([`sim_devices::usb::host::HostController::cdc_open`]).
    pub fn poll_usb_cdc(&mut self) {
        let now = self.machine.now;
        if self.usb_cdc2_pty.is_some() {
            let input = self.read_terminal(CDC1);
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(1, &input));
            }
            if let (Some(host), Some(pty)) = (self.machine.sys.usb_host_mut(), &self.usb_cdc2_pty) {
                let rx = host.cdc_take_rx(1);
                if !rx.is_empty() {
                    pty.write(&rx);
                }
            }
        }
        if self.usb_cdc_pty.is_some() {
            let input = self.read_terminal(CDC0);
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
        self.usb_cdc_bytes(now, rx);
    }

    /// Takes in what the device sent on its USB CDC port by `now`: its text, and complete lines to the hook.
    fn usb_cdc_bytes(&mut self, now: Time, rx: Vec<u8>) {
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
        let polled = rtt.poll(&mut self.machine);
        self.rtt_bytes(now, polled);
    }

    /// Takes in what the RTT up channels held at `now`: defmt lines (to the hook and `logs`) and raw text.
    fn rtt_bytes(&mut self, now: Time, polled: Vec<(usize, Vec<u8>)>) {
        for (ch, bytes) in polled {
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

    /// Whether a robot may run this board ahead of the others ([`crate::runahead`]): what feeds it from outside the
    /// simulation as it runs comes in at the end of a slice (its pseudo terminals: [`Session::collect`] reads them
    /// and keeps what it read for a rerun), and nothing watches it as it runs but what can take in afterwards what
    /// it did (no USB/IP server or Ethernet cable, whose host sides answer the board at once; no profiler or trace
    /// sink unless the robot runs them ahead, see [`crate::runahead`]), so its host side can take in what it
    /// produced later, and a copy rolled back reruns the same.
    pub(crate) fn can_run_ahead(&self) -> bool {
        self.usbip.is_none() && self.machine.sys.ethernet.is_none()
    }

    /// Whether something outside the simulation feeds the board (a pseudo terminal).
    pub(crate) fn has_live_inputs(&self) -> bool {
        self.uart_pty.iter().any(Option::is_some) || self.usb_cdc_pty.is_some() || self.usb_cdc2_pty.is_some()
    }

    /// A read of terminal `k` at the end of the slice at boundary `at`, run ahead: kept in case the board is rolled
    /// back to before it.
    fn host_read(&mut self, at: Time, k: usize) -> Vec<u8> {
        let got = self.read_terminal(k);
        if !got.is_empty() {
            self.host_reads.log.push_back((at, k, got.clone()));
        }
        got
    }

    /// Gives back what was read from the terminals at boundary `at` and after (the board was rolled back to before
    /// its end of the slice there): the next reads take it first, in the order it was read.
    pub(crate) fn unread_from(&mut self, at: Time) {
        let r = &mut self.host_reads;
        let keep = r.log.iter().position(|e| e.0 >= at).unwrap_or(r.log.len());
        let mut back: [Vec<u8>; 4] = Default::default();
        for (_, k, bytes) in r.log.drain(keep..) {
            back[k].extend(bytes);
        }
        for (k, b) in back.iter_mut().enumerate() {
            b.append(&mut r.unread[k]);
            r.unread[k] = std::mem::take(b);
        }
    }

    /// The UARTs' part of [`Session::collect`]: input from their terminals in, output out.
    fn collect_uarts(&mut self, at: Time) -> [Vec<(Time, u8)>; 2] {
        let now = self.machine.now;
        let mut out = [Vec::new(), Vec::new()];
        for (n, o) in out.iter_mut().enumerate() {
            if self.uart_pty[n].is_some() {
                let input = self.host_read(at, UART0 + n);
                if !input.is_empty() {
                    self.machine.sys.uart_inject(n, &input, now);
                }
            }
            *o = std::mem::take(&mut self.machine.sys.uart[n].output);
        }
        out
    }

    /// The machine side of a robot's end of a slice for this board at boundary `at` ([`Session::poll_rtt`] then
    /// [`Session::poll_uarts`], for a board that [`Session::can_run_ahead`]): passes in what its terminals hold
    /// (and what a rollback gave back) and takes what the UARTs, the USB CDC ports and RTT hold out of the machine,
    /// for [`Session::deliver`] to take in later.
    pub(crate) fn collect(&mut self, at: Time) -> SliceOutput {
        let now = self.machine.now;
        let first = self.collect_uarts(at);
        let mut usb_cdc2 = Vec::new();
        if self.usb_cdc2_pty.is_some() {
            let input = self.host_read(at, CDC1);
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(1, &input));
            }
            if let Some(host) = self.machine.sys.usb_host_mut() {
                usb_cdc2 = host.cdc_take_rx(1);
            }
        }
        if self.usb_cdc_pty.is_some() {
            let input = self.host_read(at, CDC0);
            if !input.is_empty() {
                self.machine.sys.with_usb_host(now, |h| h.cdc_write(0, &input));
            }
        }
        let usb_cdc = self.machine.sys.usb_host_mut().map(|h| h.cdc_take_rx(0)).unwrap_or_default();
        let rtt = self.rtt.as_ref().map(|r| r.poll(&mut self.machine)).unwrap_or_default();
        let again = self.collect_uarts(at);
        SliceOutput { at, now, uarts: [first, again], usb_cdc, usb_cdc2, rtt }
    }

    /// UART `n`'s output to its terminal and its host side.
    fn uart_out(&mut self, n: usize, out: Vec<(Time, u8)>) {
        if let Some(pty) = &self.uart_pty[n]
            && !out.is_empty()
        {
            pty.write(&out.iter().map(|(_, b)| *b).collect::<Vec<u8>>());
        }
        self.uart_bytes(n, out);
    }

    /// The host side of [`Session::collect`]: terminals, text, logs and hooks, as [`Session::poll_rtt`] and
    /// [`Session::poll_uarts`] would have taken them in.
    pub(crate) fn deliver(&mut self, c: SliceOutput) {
        let [[a0, a1], again] = c.uarts;
        self.uart_out(0, a0);
        self.uart_out(1, a1);
        if let Some(pty) = &self.usb_cdc2_pty
            && !c.usb_cdc2.is_empty()
        {
            pty.write(&c.usb_cdc2);
        }
        if let Some(pty) = &self.usb_cdc_pty
            && !c.usb_cdc.is_empty()
        {
            pty.write(&c.usb_cdc);
        }
        self.usb_cdc_bytes(c.now, c.usb_cdc);
        self.rtt_bytes(c.now, c.rtt);
        let [b0, b1] = again;
        self.uart_out(0, b0);
        self.uart_out(1, b1);
        // Final now: no rerun reads there again
        while self.host_reads.log.front().is_some_and(|e| e.0 <= c.at) {
            self.host_reads.log.pop_front();
        }
    }

    /// The flash as it is now, shared with the copy the last checkpoint kept while it is the same: while its
    /// version is the same, or (taking that copy's version) while its contents are. Returns it, and whether it is
    /// a new copy.
    fn keep_flash(&mut self) -> (Arc<Versioned>, bool) {
        let live = &mut self.machine.sys.flash;
        if let Some(kept) = &self.kept_flash
            && live.share_version(kept)
        {
            return (kept.clone(), false);
        }
        // The old copy's allocation is reused when no checkpoint holds it any more
        let copy = match self.kept_flash.take().and_then(|k| Arc::try_unwrap(k).ok()) {
            Some(mut v) => {
                v.copy_from(live);
                v
            }
            None => live.clone(),
        };
        let copy = Arc::new(copy);
        self.kept_flash = Some(copy.clone());
        (copy, true)
    }

    /// The machine's state now ([`MachineState`]); with `apart`, the SRAM and ROM are left out of the snapshot too
    /// (the caller keeps them, shared). Returns it, and whether the flash was copied.
    fn save_machine(&mut self, apart: bool) -> (MachineState, bool) {
        let sys = &mut self.machine.sys;
        let flash = std::mem::take(&mut sys.flash);
        let memories = apart.then(|| (std::mem::take(&mut sys.sram), std::mem::take(&mut sys.rom)));
        let snapshot = self.machine.snapshot();
        let sys = &mut self.machine.sys;
        sys.flash = flash;
        if let Some((sram, rom)) = memories {
            (sys.sram, sys.rom) = (sram, rom);
        }
        let (flash, copied) = self.keep_flash();
        let state = MachineState {
            snapshot,
            carry: self.machine.run_carry(),
            flash,
            lockups: self.lockups,
            final_lockup_reported: self.final_lockup_reported,
        };
        (state, copied)
    }

    /// Puts the machine back as in `s` (keeping its trace sink, breakpoints and watchpoints); the flash is copied
    /// only if the live one's version differs. A snapshot taken `apart` leaves the SRAM and ROM to the caller.
    fn restore_machine(&mut self, s: &MachineState) {
        let mut live = std::mem::take(&mut self.machine.sys.flash);
        self.machine.restore(&s.snapshot);
        if live.version() != s.flash.version() {
            live.copy_from(&s.flash);
        }
        self.machine.sys.flash = live;
        self.kept_flash = Some(s.flash.clone());
        self.machine.set_run_carry(&s.carry);
        self.lockups = s.lockups;
        self.final_lockup_reported = s.final_lockup_reported;
    }

    /// The state a robot running its boards ahead keeps to roll this board back to ([`Session::roll_back`]): the
    /// machine and what running it changes in the session ([`RollbackPoint`]).
    pub(crate) fn rollback_point(&mut self) -> RollbackPoint {
        let (machine, _) = self.save_machine(false);
        RollbackPoint { machine, stop_at: self.machine.stop_at, coverage: self.machine.coverage.clone() }
    }

    /// Puts the board back where [`Session::rollback_point`] was taken.
    pub(crate) fn roll_back(&mut self, p: &RollbackPoint) {
        self.restore_machine(&p.machine);
        self.machine.stop_at = p.stop_at;
        if let (Some(live), Some(kept)) = (&mut self.machine.coverage, &p.coverage) {
            live.clone_from(kept);
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
    /// with earlier checkpoints while it is the same (see [`MachineState`]), the SRAM, ROM and code coverage with
    /// `prev` where they are. Hooks, endpoints and ptys are not part of it.
    pub fn checkpoint(&mut self, prev: Option<&SharedMemory>) -> SessionCheckpoint {
        let (machine, flash_copied) = self.save_machine(true);
        let sys = &self.machine.sys;
        let (memory, mut fresh) = SharedMemory::keep(prev, &sys.sram, &sys.rom, self.machine.coverage.as_deref());
        if flash_copied {
            fresh += sys.flash.len();
        }
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
            slice_end: self.slice_end,
            boundary: self.boundary,
        }
    }

    /// Puts the session back where `c` was taken: the machine (keeping its trace sink, breakpoints and
    /// watchpoints), the host side, and the collected logs and text and the code coverage cut back to what there was
    /// then (both follow the timeline: what the run did to get where it is). Inputs the
    /// devices read are not part of it (see [`crate::timetravel`]).
    pub fn rewind_to(&mut self, c: &SessionCheckpoint) {
        self.rewind_to_with(c, &CollectedText::default());
    }

    /// Keeps in `c` what the session collected (logs, RTT, UART and USB text) if it has more than `c` has: going
    /// back cuts them, going forward again to a later checkpoint ([`Session::rewind_to_with`]) gives them back.
    pub fn keep_collected(&self, c: &mut CollectedText) {
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
    pub fn cut_collected(&self, c: &mut CollectedText) {
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
    pub fn rewind_to_with(&mut self, c: &SessionCheckpoint, collected: &CollectedText) {
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
        self.restore_machine(&c.machine);
        c.memory.restore(&mut self.machine);
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

    /// The virtual time the wall clock is at now, counted from where pacing started (from `now`, if it has not
    /// started yet: this starts it).
    pub(crate) fn wall_virtual(&mut self, now: Time) -> Time {
        let (wall0, virt0) = *self.start.get_or_insert((std::time::Instant::now(), now));
        virt0 + Time::from_ns(wall0.elapsed().as_nanos() as u64)
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

    /// What a board run ahead read from a terminal is given back when it is rolled back to before the read: the
    /// next reads take it first, then what was typed since, in order; delivered boundaries are not given back.
    #[test]
    fn terminal_reads_are_given_back_on_a_rollback() {
        use std::io::Write;
        let Ok(pty) = crate::pty::Pty::open() else { return };
        let mut term = std::fs::OpenOptions::new().write(true).open(&pty.path).unwrap();
        let mut s = Session::new(Machine::new(Default::default()), &Image::default());
        s.uart_pty[0] = Some(pty);
        assert!(s.has_live_inputs());
        let typed = |term: &mut std::fs::File, b: &[u8]| {
            term.write_all(b).unwrap();
            std::thread::sleep(Duration::from_millis(30));
        };
        let at = |k: u64| Time::from_us(100 * k);
        typed(&mut term, b"ab");
        assert_eq!(s.host_read(at(1), UART0), b"ab");
        typed(&mut term, b"cd");
        assert_eq!(s.host_read(at(2), UART0), b"cd");
        typed(&mut term, b"ef");
        assert_eq!(s.host_read(at(3), UART0), b"ef");
        // Boundary 1 is final; the board goes back to boundary 2
        s.deliver(SliceOutput { at: at(1), ..Default::default() });
        s.unread_from(at(2));
        typed(&mut term, b"gh");
        assert_eq!(s.host_read(at(2), UART0), b"cdefgh");
        // Back again to 2, then to 1: nothing before 2 is given back, it was delivered
        s.unread_from(at(2));
        s.unread_from(at(1));
        assert_eq!(s.read_terminal(UART0), b"cdefgh");
        assert_eq!(s.read_terminal(UART0), b"");
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

    fn template() -> Option<Session> {
        let repo = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let image = Image::load(&crate::titan_rs_image(&repo, "template")?).unwrap();
        let mut m =
            Machine::new(rp2040::MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
        m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
        m.load(&image).unwrap();
        m.direct_boot().unwrap();
        Some(Session::new(m, &image))
    }

    /// Both kinds of checkpoint keep the flash the same way: one copy shared while its version is the same (or its
    /// contents are), a new one once it was written, and put back on a restore only if it differs.
    #[test]
    fn checkpoints_share_the_flash_while_it_is_the_same() {
        let Some(mut s) = template() else { return };
        s.run_until(Time::from_ms(2));
        let a = s.rollback_point();
        let b = s.checkpoint(None);
        let c = s.rollback_point();
        assert!(Arc::ptr_eq(&a.machine.flash, &b.machine.flash) && Arc::ptr_eq(&a.machine.flash, &c.machine.flash));
        assert!(b.fresh >= s.machine.sys.sram.len(), "the first time-travel checkpoint copies the SRAM");
        let b2 = s.checkpoint(Some(b.memory()));
        assert_eq!(b2.fresh, 0, "nothing changed: everything shared");
        // A write makes a new copy (compared with the last copy kept, which it differs from)
        let old = s.machine.sys.flash[0x1000];
        s.machine.sys.flash[0x1000] = !old;
        let d = s.checkpoint(Some(b.memory()));
        assert!(!Arc::ptr_eq(&a.machine.flash, &d.machine.flash));
        assert_eq!(d.fresh, s.machine.sys.flash.len(), "the flash was copied");
        s.machine.sys.flash[0x1000] = old;
        let e = s.rollback_point();
        assert!(!Arc::ptr_eq(&d.machine.flash, &e.machine.flash), "the last copy is compared, and it differs");
        let f = s.checkpoint(None);
        assert!(Arc::ptr_eq(&e.machine.flash, &f.machine.flash));
        // Going back to before the write puts the old contents back, with their version
        s.machine.sys.flash[0x1000] = !old;
        s.rewind_to(&b);
        assert_eq!((s.machine.sys.flash[0x1000], s.machine.sys.flash.version()), (old, a.machine.flash.version()));
        s.rewind_to(&d);
        assert_eq!(s.machine.sys.flash[0x1000], !old);
        s.roll_back(&a);
        assert_eq!(s.machine.sys.flash[0x1000], old);
    }
}
