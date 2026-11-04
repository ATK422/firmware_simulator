//! GDB remote serial protocol stub.
//!
//! Each RP2040 core is a GDB thread (thread 1 = core 0, thread 2 = core 1). Supported: registers (`g`/`G`/`p`/`P`,
//! with an M-profile target description), memory (`m`/`M`/`X`; writes reach flash, so `load` works), software and
//! hardware breakpoints (`Z0`/`Z1`), watchpoints (`Z2` write, `Z3` read, `Z4` access, any number of any length),
//! `c`/`s`/`vCont`, Ctrl-C, `qRcmd` monitor commands (`reset`, `reset halt`, `halt`, `time`: what Cortex-Debug and
//! OpenOCD-style launch scripts send), no-ack mode, `target extended-remote`, and detach/kill (`k`, `vKill`). While
//! running, the session keeps polling RTT so logs keep flowing.
//!
//! With time travel ([`Travel`]: a target recorded by a [`TimeTravel`]), the stub also runs backwards:
//! `reverse-continue` (`bc`) goes back to the most recent breakpoint or watchpoint hit, `reverse-stepi` (`bs`) one
//! step of the current core (GDB builds `reverse-step`, `reverse-next` and `reverse-finish` on these), and `monitor
//! rewind TIME` goes to a virtual time; `ReverseStep+` and `ReverseContinue+` are advertised. Going forward again
//! re-executes the recorded history (stopping at breakpoints on the way) up to where the run had got, then runs on
//! live. Stepping steps the current core while the other one runs as scheduled, and stops never change the run.
//!
//! The stub debugs a [`Target`]: one board on its own (a [`Session`]), or one board of a robot ([`RobotBoard`]),
//! where running it runs every board and bus of the robot. A connection that closes before sending a packet (a
//! port check) is not a debugger: the stub waits for the next one. GDB going away while the target runs ends the
//! session, as does the run being asked to stop (SIGINT, SIGTERM), which GDB hears as the program exiting.

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;

use cpu_armv6m::reg;
use rp2040::StopReason;
use rp2040::machine::{BkptPolicy, WatchKind, Watchpoint};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;

use crate::robot::Robot;
use crate::session::Session;
use crate::timetravel::{Change, Pos, Ran, Rewindable, TimeTravel, Travelled};

/// What GDB debugs: a board ([`Target::session`]: the registers, memory and breakpoints GDB sees) and how it runs.
pub trait Target {
    /// The board being debugged.
    fn session(&mut self) -> &mut Session;
    /// Runs for `d` of virtual time. Returns why it stopped: [`StopReason::TimeLimit`] when it ran the whole
    /// piece, [`StopReason::Requested`] when the run was asked to stop.
    fn run_for(&mut self, d: sim_core::Time) -> StopReason;
    /// The recorder, the recorded target and the debugged board's index in it, when the target can run backwards.
    fn travel(&mut self) -> Option<(&mut TimeTravel, &mut dyn Rewindable, usize)> {
        None
    }
}

/// A board (`target` a [`Session`], `board` 0) or a robot's board recorded by `tt`: GDB can run it backwards.
pub struct Travel<'a> {
    pub tt: &'a mut TimeTravel,
    pub target: &'a mut dyn Rewindable,
    pub board: usize,
}

impl Target for Travel<'_> {
    fn session(&mut self) -> &mut Session {
        self.target.session(self.board)
    }

    fn run_for(&mut self, d: sim_core::Time) -> StopReason {
        let end = self.target.now() + d;
        match self.tt.run_until(self.target, end) {
            Ran::Reached => StopReason::TimeLimit,
            Ran::Interrupted => StopReason::Requested,
            Ran::Stopped { reason, .. } => reason,
        }
    }

    fn travel(&mut self) -> Option<(&mut TimeTravel, &mut dyn Rewindable, usize)> {
        Some((self.tt, self.target, self.board))
    }
}

/// Live runs under time travel go in pieces of this much virtual time, ending on multiples of it (slice
/// boundaries), so the run is the same wherever GDB stopped or interrupted it.
pub const PIECE: sim_core::Time = sim_core::Time::from_us(500);

/// The next multiple of [`PIECE`] after `now`.
fn piece_end(now: sim_core::Time) -> sim_core::Time {
    let p = PIECE.as_ps();
    sim_core::Time((now.as_ps() / p + 1) * p)
}

/// Applies a packet that changes the target (`G`, `P`, `M`, `X`) to `m` as seen from `core`. `None` for other
/// packets; else whether it worked.
pub fn apply_write(m: &mut rp2040::Machine, core: usize, pkt: &str) -> Option<bool> {
    if let Some(data) = pkt.strip_prefix('G') {
        let bytes = unhex(data)?;
        for (n, chunk) in bytes.chunks(4).enumerate().take(G_REGS) {
            if chunk.len() == 4 {
                m.cores[core].write_reg(n, u32::from_le_bytes(chunk.try_into().unwrap()));
            }
        }
        Some(true)
    } else if let Some(rest) = pkt.strip_prefix('P') {
        let (n, v) = rest.split_once('=')?;
        let n = parse_hex(n)? as usize;
        let v = unhex(v)?;
        if n < reg::COUNT && v.len() == 4 {
            m.cores[core].write_reg(n, u32::from_le_bytes(v.try_into().unwrap()));
            Some(true)
        } else {
            Some(false)
        }
    } else if let Some(rest) = pkt.strip_prefix('M') {
        let (head, data) = rest.split_once(':')?;
        let (addr, _) = head.split_once(',')?;
        let addr = parse_hex(addr)?;
        let bytes = unhex(data)?;
        Some(bytes.iter().enumerate().all(|(i, &b)| m.sys.poke(addr + i as u32, Size::Byte, b as u32)))
    } else if let Some(rest) = pkt.strip_prefix('X') {
        let (head, data) = rest.split_once(':')?;
        let (addr, _) = head.split_once(',')?;
        let addr = parse_hex(addr)?;
        Some(data.chars().enumerate().all(|(i, c)| m.sys.poke(addr + i as u32, Size::Byte, c as u32)))
    } else {
        None
    }
}

/// Whether a packet changes the target's state (registers, memory).
fn is_write(pkt: &str) -> bool {
    matches!(pkt.as_bytes().first(), Some(b'G' | b'P' | b'M' | b'X'))
}

/// Parses a time for `monitor rewind`: seconds, or with a unit (`1.5s`, `200ms`, `350us`, `100ns`).
pub fn parse_time(s: &str) -> Option<sim_core::Time> {
    let s = s.trim();
    let (num, scale) = if let Some(n) = s.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = s.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = s.strip_suffix("ns") {
        (n, 1e-9)
    } else if let Some(n) = s.strip_suffix('s') {
        (n, 1.0)
    } else {
        (s, 1.0)
    };
    let v: f64 = num.trim().parse().ok()?;
    (v >= 0.0 && v.is_finite()).then(|| sim_core::Time::from_secs_f64(v * scale))
}

impl Target for Session {
    fn session(&mut self) -> &mut Session {
        self
    }

    fn run_for(&mut self, d: sim_core::Time) -> StopReason {
        Session::run_for(self, d)
    }
}

/// Board `board` of a robot. Running it runs the whole robot (every board, bus, the water and the vehicles), so the
/// other boards keep pace with it, and it stops when the debugged board stops (a breakpoint, `BKPT`, a final
/// lockup). Another board that stops (a final lockup) ends the run, as it does without GDB. The boards run in turn
/// on the calling thread while GDB is attached.
pub struct RobotBoard<'a> {
    pub robot: &'a mut Robot,
    pub board: usize,
}

impl Target for RobotBoard<'_> {
    fn session(&mut self) -> &mut Session {
        &mut self.robot.boards[self.board]
    }

    fn run_for(&mut self, d: sim_core::Time) -> StopReason {
        let (threads, end) = (self.robot.threads, self.robot.now + d);
        self.robot.threads = 1;
        let stopped = self.robot.run_until(end);
        self.robot.threads = threads;
        match stopped {
            Some((i, reason)) if i == self.board => reason,
            Some((i, reason)) => {
                log::warn!("[{}] {} stopped: {reason:?}; ending the run", self.robot.now, self.robot.names[i]);
                StopReason::Requested
            }
            None if self.robot.now < end => StopReason::Requested,
            None => StopReason::TimeLimit,
        }
    }
}

const TARGET_XML: &str = r#"<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32" regnum="0"/>
    <reg name="r1" bitsize="32"/>
    <reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/>
    <reg name="r4" bitsize="32"/>
    <reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/>
    <reg name="r7" bitsize="32"/>
    <reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/>
    <reg name="r10" bitsize="32"/>
    <reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" regnum="16"/>
  </feature>
  <feature name="org.gnu.gdb.arm.m-system">
    <reg name="msp" bitsize="32" regnum="17" type="data_ptr" group="system"/>
    <reg name="psp" bitsize="32" regnum="18" type="data_ptr" group="system"/>
    <reg name="primask" bitsize="32" regnum="19" group="system"/>
    <reg name="control" bitsize="32" regnum="20" group="system"/>
  </feature>
</target>
"#;

/// Number of registers in the `g` packet (r0-r15, xPSR).
const G_REGS: usize = 17;

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    if !s.len().is_multiple_of(2) {
        return None;
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok()).collect()
}

fn hex_u32_le(v: u32) -> String {
    hex(&v.to_le_bytes())
}

fn parse_hex(s: &str) -> Option<u32> {
    u32::from_str_radix(s, 16).ok()
}

/// Whether a packet resumes execution (continue or step, either way).
fn is_resume(pkt: &str) -> bool {
    pkt == "c"
        || pkt == "s"
        || pkt == "bc"
        || pkt == "bs"
        || pkt.starts_with("vCont;")
        || pkt.starts_with('C')
        || pkt.starts_with('S')
}

/// `Z2`/`Z3`/`Z4` (insert) and `z2`/`z3`/`z4` (remove): whether it inserts, the kind and `addr,len`.
fn watch_packet(pkt: &str) -> Option<(bool, WatchKind, &str)> {
    let insert = match pkt.as_bytes().first()? {
        b'Z' => true,
        b'z' => false,
        _ => return None,
    };
    let kind = match pkt.get(1..3)? {
        "2," => WatchKind::Write,
        "3," => WatchKind::Read,
        "4," => WatchKind::Access,
        _ => return None,
    };
    Some((insert, kind, &pkt[3..]))
}

/// The stop reply field for a watchpoint hit.
fn watch_field(kind: WatchKind, addr: u32) -> String {
    let name = match kind {
        WatchKind::Write => "watch",
        WatchKind::Read => "rwatch",
        WatchKind::Access => "awatch",
    };
    format!("{name}:{addr:x};")
}

/// Packet framing over a byte stream.
pub struct Connection<S: Read + Write> {
    stream: S,
    ack: bool,
    buf: Vec<u8>,
}

/// What came in from GDB.
#[derive(Debug, PartialEq, Eq)]
pub enum Incoming {
    Packet(String),
    Interrupt,
    Closed,
}

impl<S: Read + Write> Connection<S> {
    pub fn new(stream: S) -> Self {
        Connection { stream, ack: true, buf: Vec::new() }
    }

    pub fn send(&mut self, payload: &str) -> io::Result<()> {
        let mut out = String::with_capacity(payload.len() + 4);
        out.push('$');
        for c in payload.chars() {
            // Escape the protocol's special characters
            if matches!(c, '$' | '#' | '}' | '*') {
                out.push('}');
                out.push((c as u8 ^ 0x20) as char);
            } else {
                out.push(c);
            }
        }
        // The checksum is over the packet data as transmitted, escapes included
        let sum = out.bytes().skip(1).fold(0u8, |a, b| a.wrapping_add(b));
        out.push('#');
        out.push_str(&format!("{sum:02x}"));
        self.stream.write_all(out.as_bytes())?;
        self.stream.flush()
    }

    /// Tries to extract one complete message from the buffer.
    fn take_message(&mut self) -> Option<Incoming> {
        loop {
            let first = *self.buf.first()?;
            match first {
                b'+' | b'-' => {
                    self.buf.remove(0);
                }
                0x03 => {
                    self.buf.remove(0);
                    return Some(Incoming::Interrupt);
                }
                b'$' => {
                    let end = self.buf.iter().position(|&b| b == b'#')?;
                    if self.buf.len() < end + 3 {
                        return None;
                    }
                    let raw: Vec<u8> = self.buf[1..end].to_vec();
                    self.buf.drain(..end + 3);
                    // Unescape (binary X packets)
                    let mut data = Vec::with_capacity(raw.len());
                    let mut it = raw.into_iter();
                    while let Some(b) = it.next() {
                        if b == b'}' {
                            if let Some(n) = it.next() {
                                data.push(n ^ 0x20);
                            }
                        } else {
                            data.push(b);
                        }
                    }
                    if self.ack {
                        let _ = self.stream.write_all(b"+");
                    }
                    // Binary payloads are carried as Latin-1 so they survive the String round trip
                    return Some(Incoming::Packet(data.iter().map(|&b| b as char).collect()));
                }
                _ => {
                    self.buf.remove(0);
                }
            }
        }
    }

    /// Blocks until a packet or interrupt arrives.
    pub fn recv(&mut self) -> io::Result<Incoming> {
        loop {
            if let Some(m) = self.take_message() {
                return Ok(m);
            }
            let mut chunk = [0u8; 4096];
            let n = self.stream.read(&mut chunk)?;
            if n == 0 {
                return Ok(Incoming::Closed);
            }
            self.buf.extend_from_slice(&chunk[..n]);
        }
    }
}

impl Connection<TcpStream> {
    /// Returns an interrupt or packet if one is waiting, without blocking.
    pub fn poll(&mut self) -> io::Result<Option<Incoming>> {
        if let Some(m) = self.take_message() {
            return Ok(Some(m));
        }
        self.stream.set_nonblocking(true)?;
        let mut chunk = [0u8; 1024];
        let r = self.stream.read(&mut chunk);
        self.stream.set_nonblocking(false)?;
        match r {
            Ok(0) => Ok(Some(Incoming::Closed)),
            Ok(n) => {
                self.buf.extend_from_slice(&chunk[..n]);
                Ok(self.take_message())
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e),
        }
    }
}

/// How a debugging session ended.
#[derive(Debug, PartialEq, Eq)]
pub enum Ended {
    /// GDB detached; the machine should keep running.
    Detached,
    /// GDB killed the target or closed the connection.
    Killed,
    /// The run was asked to stop while GDB was attached (SIGINT, SIGTERM; for a robot, another board stopped).
    Stopped,
}

/// The protocol state for one connected debugger.
pub struct GdbStub {
    /// Thread (core) selected for register access and stepping (0-based core index).
    current: usize,
    /// The target can run backwards (advertised in `qSupported`).
    reverse: bool,
}

impl Default for GdbStub {
    fn default() -> Self {
        Self::new()
    }
}

impl GdbStub {
    pub fn new() -> Self {
        GdbStub { current: 0, reverse: false }
    }

    fn thread_of(&self, core: usize) -> String {
        format!("{:x}", core + 1)
    }

    fn stop_reply(&self, session: &Session, reason: Option<StopReason>) -> String {
        let (core, extra) = match reason {
            Some(StopReason::Breakpoint { core, .. }) => (core as usize, "swbreak:;".to_string()),
            Some(StopReason::BkptInstruction { core, .. }) => (core as usize, "swbreak:;".to_string()),
            Some(StopReason::Watchpoint { core, addr, kind, .. }) => (core as usize, watch_field(kind, addr)),
            Some(StopReason::Lockup { core, .. } | StopReason::Reached { core, .. }) => (core as usize, String::new()),
            _ => (self.current, String::new()),
        };
        let _ = session;
        format!("T05thread:{};{extra}", self.thread_of(core))
    }

    fn read_reg(&self, session: &Session, n: usize) -> Option<u32> {
        (n < reg::COUNT).then(|| session.machine.cores[self.current].read_reg(n))
    }

    fn threads(&self, session: &Session) -> Vec<usize> {
        let mut t = vec![0];
        if session.machine.core_enabled[1] {
            t.push(1);
        }
        t
    }

    fn monitor(&mut self, session: &mut Session, cmd: &str) -> String {
        let words: Vec<&str> = cmd.split_whitespace().collect();
        let out = match words[..] {
            // OpenOCD's spellings, which Cortex-Debug and many launch scripts send: the target is halted whenever
            // GDB can send a command, so they all reset and stay halted until GDB resumes
            ["reset"] | ["reset", "halt" | "init" | "run"] => {
                let m = &mut session.machine;
                m.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
                self.current = 0;
                "machine reset (watchdog), halted at the reset vector until GDB resumes; \
                 `maintenance flush register-cache` shows the new registers\n"
                    .to_string()
            }
            ["halt"] => "halted\n".to_string(),
            ["time"] => format!("virtual time {}\n", session.machine.now),
            ["help"] | [] => {
                let travel = if self.reverse { ", rewind TIME, checkpoints" } else { "" };
                format!("monitor commands: reset [halt|init|run], halt, time{travel}, help\n")
            }
            _ => format!("unknown monitor command {:?} (monitor help lists them)\n", cmd.trim()),
        };
        hex(out.as_bytes())
    }

    /// Handles one packet that does not resume execution. Returns the reply, or `None` for resume packets.
    pub fn handle(&mut self, session: &mut Session, conn_ack: &mut bool, pkt: &str) -> Option<String> {
        let m = &mut session.machine;
        let reply = if pkt == "?" {
            self.stop_reply(session, None)
        } else if pkt.starts_with("qSupported") {
            let reverse = if self.reverse { ";ReverseStep+;ReverseContinue+" } else { "" };
            format!("PacketSize=4000;qXfer:features:read+;vContSupported+;swbreak+;hwbreak+;QStartNoAckMode+{reverse}")
        } else if pkt == "QStartNoAckMode" {
            *conn_ack = false;
            "OK".into()
        } else if let Some(rest) = pkt.strip_prefix("qXfer:features:read:target.xml:") {
            let (off, len) = rest.split_once(',')?;
            let off = parse_hex(off)? as usize;
            let len = parse_hex(len)? as usize;
            let xml = TARGET_XML.as_bytes();
            if off >= xml.len() {
                "l".into()
            } else {
                let end = (off + len).min(xml.len());
                let prefix = if end == xml.len() { 'l' } else { 'm' };
                format!("{prefix}{}", std::str::from_utf8(&xml[off..end]).unwrap_or(""))
            }
        } else if pkt == "qAttached" {
            "1".into()
        } else if pkt == "!" || pkt == "qSymbol::" {
            // Extended mode (`target extended-remote`) changes nothing here; no symbols are needed
            "OK".into()
        } else if pkt == "qC" {
            format!("QC{}", self.thread_of(self.current))
        } else if pkt == "qfThreadInfo" {
            let ts: Vec<String> = self.threads(session).iter().map(|&c| self.thread_of(c)).collect();
            format!("m{}", ts.join(","))
        } else if pkt == "qsThreadInfo" {
            "l".into()
        } else if let Some(t) = pkt.strip_prefix("qThreadExtraInfo,") {
            let core = parse_hex(t).unwrap_or(1).saturating_sub(1);
            hex(format!("core {core}").as_bytes())
        } else if let Some(t) = pkt.strip_prefix('T') {
            let core = parse_hex(t).unwrap_or(0) as usize;
            if (1..=2).contains(&core) && self.threads(session).contains(&(core - 1)) {
                "OK".into()
            } else {
                "E01".into()
            }
        } else if let Some(t) = pkt.strip_prefix("Hg").or_else(|| pkt.strip_prefix("Hc")) {
            match t {
                "0" | "-1" | "" => {}
                t => {
                    if let Some(n) = parse_hex(t)
                        && (1..=2).contains(&n)
                    {
                        self.current = n as usize - 1;
                    }
                }
            }
            "OK".into()
        } else if pkt == "g" {
            (0..G_REGS).map(|n| hex_u32_le(self.read_reg(session, n).unwrap_or(0))).collect()
        } else if is_write(pkt) {
            match apply_write(m, self.current, pkt)? {
                true => "OK".into(),
                false => "E01".into(),
            }
        } else if let Some(n) = pkt.strip_prefix('p') {
            match self.read_reg(session, parse_hex(n)? as usize) {
                Some(v) => hex_u32_le(v),
                None => "E01".into(),
            }
        } else if let Some(rest) = pkt.strip_prefix('m') {
            let (addr, len) = rest.split_once(',')?;
            let addr = parse_hex(addr)?;
            let len = parse_hex(len)?.min(0x1000);
            let mut out = Vec::with_capacity(len as usize);
            for i in 0..len {
                match m.debug_read(self.current, addr.wrapping_add(i), Size::Byte) {
                    Some(v) => out.push(v as u8),
                    None => break,
                }
            }
            if out.is_empty() && len > 0 { "E01".into() } else { hex(&out) }
        } else if let Some(rest) = pkt.strip_prefix("Z0,").or_else(|| pkt.strip_prefix("Z1,")) {
            let addr = parse_hex(rest.split(',').next()?)?;
            m.breakpoints.insert(addr & !1);
            "OK".into()
        } else if let Some(rest) = pkt.strip_prefix("z0,").or_else(|| pkt.strip_prefix("z1,")) {
            let addr = parse_hex(rest.split(',').next()?)?;
            m.breakpoints.remove(&(addr & !1));
            "OK".into()
        } else if let Some((insert, kind, rest)) = watch_packet(pkt) {
            let mut it = rest.split(',');
            let addr = parse_hex(it.next()?)?;
            let len = parse_hex(it.next()?)?;
            let w = Watchpoint { addr, len, kind };
            if insert {
                m.watchpoints.push(w);
            } else if let Some(i) = m.watchpoints.iter().position(|&x| x == w) {
                m.watchpoints.remove(i);
            }
            "OK".into()
        } else if pkt == "vCont?" {
            "vCont;c;C;s;S".into()
        } else if let Some(hexcmd) = pkt.strip_prefix("qRcmd,") {
            let cmd = String::from_utf8(unhex(hexcmd)?).ok()?;
            let out = self.monitor(session, &cmd);
            // Console output, then OK
            return Some(format!("O{out}\u{0}OK"));
        } else if is_resume(pkt) {
            return None;
        } else {
            String::new()
        };
        Some(reply)
    }

    /// The reply for a stop on the debugged board.
    fn travel_stop(&mut self, session: &Session, reason: StopReason) -> String {
        if let StopReason::Breakpoint { core, .. }
        | StopReason::BkptInstruction { core, .. }
        | StopReason::Watchpoint { core, .. }
        | StopReason::Reached { core, .. }
        | StopReason::Lockup { core, .. } = reason
        {
            self.current = core as usize;
        }
        self.stop_reply(session, Some(reason))
    }

    /// [`GdbStub::resume`] with time travel: either direction, in the recorded history or live.
    fn resume_travel(
        &mut self,
        target: &mut dyn Target,
        pkt: &str,
        step: bool,
        mut poll: impl FnMut() -> io::Result<Option<Incoming>>,
    ) -> Resumed {
        let Some((tt, rw, board)) = target.travel() else { unreachable!("checked by the caller") };
        let core = self.current;
        let begin = |me: &Self| format!("T05replaylog:begin;thread:{};", me.thread_of(core));
        if pkt == "bc" || pkt == "bs" {
            let r = if pkt == "bc" { tt.reverse_continue(rw) } else { tt.reverse_step(rw, board, core) };
            return match r {
                Travelled::Hit(h) => Resumed::Stopped(self.travel_stop(rw.session(board), h.reason)),
                Travelled::Beginning => Resumed::Stopped(begin(self)),
                Travelled::Interrupted => Resumed::Ended,
                Travelled::Arrived | Travelled::Frontier => {
                    Resumed::Stopped(format!("T05thread:{};", self.thread_of(core)))
                }
            };
        }
        let goal =
            step.then(|| Pos::At { board, core, time: rw.session(board).machine.core_time(core) + sim_core::Time(1) });
        let resumed = loop {
            if !tt.at_frontier() {
                let until = piece_end(rw.now());
                match tt.forward(rw, goal, until) {
                    Travelled::Arrived if goal.is_some() => {
                        break Resumed::Stopped(format!("T05thread:{};", self.thread_of(core)));
                    }
                    Travelled::Hit(h) => break Resumed::Stopped(self.travel_stop(rw.session(board), h.reason)),
                    Travelled::Interrupted => break Resumed::Ended,
                    Travelled::Frontier => continue,
                    Travelled::Arrived | Travelled::Beginning => {}
                }
            } else {
                if let Some(Pos::At { time, .. }) = goal {
                    rw.session(board).machine.stop_at[core] = Some(time);
                }
                let end = piece_end(rw.now());
                match tt.run_until(rw, end) {
                    Ran::Reached => {}
                    Ran::Interrupted => break Resumed::Ended,
                    Ran::Stopped { reason: StopReason::Deadlock, .. } => {
                        break Resumed::Stopped(format!("T05thread:{};", self.thread_of(core)));
                    }
                    Ran::Stopped { reason, .. } => break Resumed::Stopped(self.travel_stop(rw.session(board), reason)),
                }
            }
            match poll() {
                Ok(Some(Incoming::Interrupt)) => {
                    break Resumed::Stopped(format!("T02thread:{};", self.thread_of(self.current)));
                }
                Ok(Some(Incoming::Closed)) | Err(_) => break Resumed::Closed,
                Ok(_) => {}
            }
        };
        rw.session(board).machine.stop_at = [None; 2];
        resumed
    }

    /// Monitor commands and changes to the target, with time travel: changes are logged (replays make them again)
    /// and drop the recorded future when made in the past. `None` for packets [`GdbStub::handle`] answers.
    fn travel_packet(&mut self, target: &mut dyn Target, pkt: &str) -> Option<String> {
        let core = self.current;
        let (tt, rw, board) = target.travel()?;
        if is_write(pkt) {
            let ok = apply_write(&mut rw.session(board).machine.fork(), core, pkt)?;
            if ok {
                let p = pkt.to_string();
                let change: Change = Arc::new(move |m| {
                    apply_write(m, core, &p);
                });
                tt.apply(rw, board, change);
            }
            return Some(if ok { "OK" } else { "E01" }.into());
        }
        let cmd = String::from_utf8(unhex(pkt.strip_prefix("qRcmd,")?)?).ok()?;
        let words: Vec<&str> = cmd.split_whitespace().collect();
        let out = match words[..] {
            ["reset"] | ["reset", "halt" | "init" | "run"] => {
                let change: Change = Arc::new(|m| m.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force));
                tt.apply(rw, board, change);
                self.current = 0;
                "machine reset (watchdog), halted at the reset vector until GDB resumes; \
                 `maintenance flush register-cache` shows the new registers\n"
                    .to_string()
            }
            ["rewind", t] => match parse_time(t) {
                Some(t) => {
                    let r = tt.rewind_to(rw, t);
                    let now = rw.now();
                    let note = match r {
                        Travelled::Beginning => " (the start of the recorded history)",
                        Travelled::Frontier => " (as far as the run has got)",
                        _ => "",
                    };
                    format!(
                        "at {now}{note}; `maintenance flush register-cache` (or `flushregs`) shows the registers there\n"
                    )
                }
                None => format!("not a time: {t:?} (1.5s, 200ms, 350us)\n"),
            },
            ["checkpoints"] => {
                let st = tt.stats();
                format!(
                    "{} checkpoints, {:.1} MB, history from {} to {}; {} taken ({:.2} ms each), {} restores, {} replayed in {:.2} s\n",
                    st.checkpoints,
                    st.bytes as f64 / (1 << 20) as f64,
                    st.oldest,
                    st.frontier,
                    st.taken,
                    st.taking_ns as f64 / 1e6 / st.taken.max(1) as f64,
                    st.restores,
                    st.replayed,
                    st.replay_ns as f64 / 1e9
                )
            }
            ["time"] => {
                let pos = match tt.here() {
                    Pos::Boundary(t) => format!("slice boundary {t}"),
                    Pos::At { core, time, .. } => format!("core{core} at {time}"),
                };
                let front = if tt.at_frontier() { "live" } else { "in the recorded history" };
                format!("virtual time {}; {pos}; {front}\n", rw.now())
            }
            _ => return None,
        };
        Some(format!("O{}\u{0}OK", hex(out.as_bytes())))
    }

    /// Runs a resume packet (`c`, `s`, `vCont`) on `target`. `poll` is asked between slices whether GDB sent an
    /// interrupt or went away.
    pub fn resume(
        &mut self,
        target: &mut dyn Target,
        pkt: &str,
        mut poll: impl FnMut() -> io::Result<Option<Incoming>>,
    ) -> Resumed {
        let step = pkt == "s" || pkt.starts_with("vCont;s") || pkt.starts_with("vCont;S") || pkt.starts_with('S');
        if step {
            // vCont;s:N selects the thread
            if let Some(t) = pkt.split(':').nth(1).and_then(|t| parse_hex(t.split(';').next().unwrap_or(t)))
                && (1..=2).contains(&t)
            {
                self.current = t as usize - 1;
            }
        }
        if target.travel().is_some() {
            return self.resume_travel(target, pkt, step, poll);
        }
        if step {
            let core = self.current;
            let session = target.session();
            session.machine.single_step(core);
            session.poll_rtt();
            return Resumed::Stopped(format!("T05thread:{};", self.thread_of(core)));
        }
        loop {
            let piece = target.session().slice.max(sim_core::Time::from_us(500));
            match target.run_for(piece) {
                StopReason::TimeLimit => match poll() {
                    Ok(Some(Incoming::Interrupt)) => {
                        return Resumed::Stopped(format!("T02thread:{};", self.thread_of(self.current)));
                    }
                    Ok(Some(Incoming::Closed)) | Err(_) => return Resumed::Closed,
                    // Nothing, or a packet GDB should not send while the target runs
                    Ok(_) => {}
                },
                StopReason::Requested => return Resumed::Ended,
                StopReason::Deadlock => {
                    // Nothing will ever happen again; report a stop so the user can look around
                    return Resumed::Stopped(format!("T05thread:{};", self.thread_of(self.current)));
                }
                other => {
                    if let StopReason::Breakpoint { core, .. }
                    | StopReason::BkptInstruction { core, .. }
                    | StopReason::Lockup { core, .. } = other
                    {
                        self.current = core as usize;
                    }
                    return Resumed::Stopped(self.stop_reply(target.session(), Some(other)));
                }
            }
        }
    }
}

/// How running the target for GDB ended.
#[derive(Debug, PartialEq, Eq)]
pub enum Resumed {
    /// It stopped (a breakpoint, a step, an interrupt): the stop reply for GDB.
    Stopped(String),
    /// GDB closed the connection while it ran.
    Closed,
    /// The run was asked to stop (SIGINT, SIGTERM, another board of a robot stopped).
    Ended,
}

/// Serves one GDB connection on `listener`, driving `session` until GDB detaches or disconnects.
pub fn serve(listener: &TcpListener, session: &mut Session) -> io::Result<Ended> {
    serve_target(listener, session)
}

/// Waits for a debugger on `listener`: a connection that closes before sending anything (a port check) is let go.
/// Returns the connection and its first message.
fn accept_debugger(listener: &TcpListener) -> io::Result<(Connection<TcpStream>, Incoming)> {
    loop {
        let (stream, peer) = listener.accept()?;
        stream.set_nodelay(true)?;
        let mut conn = Connection::new(stream);
        match conn.recv() {
            Ok(Incoming::Closed) | Err(_) => {
                log::info!("{peer} closed the connection without a packet (a port check?); still waiting for GDB");
            }
            Ok(first) => {
                log::info!("GDB connected from {peer}");
                return Ok((conn, first));
            }
        }
    }
}

/// Serves one GDB connection on `listener`, driving `target` until GDB detaches or disconnects, or the run is
/// asked to stop.
pub fn serve_target(listener: &TcpListener, target: &mut dyn Target) -> io::Result<Ended> {
    let (mut conn, first) = accept_debugger(listener)?;
    let old_policy = target.session().machine.bkpt_policy;
    target.session().machine.bkpt_policy = BkptPolicy::Stop;
    let mut stub = GdbStub::new();
    stub.reverse = target.travel().is_some();
    let mut next = Some(first);
    let result = loop {
        let msg = match next.take() {
            Some(m) => m,
            None => conn.recv()?,
        };
        match msg {
            Incoming::Closed => break Ended::Killed,
            Incoming::Interrupt => conn.send(&format!("T02thread:{};", stub.thread_of(stub.current)))?,
            Incoming::Packet(pkt) => {
                if pkt == "k" {
                    break Ended::Killed;
                }
                if pkt.starts_with("vKill") {
                    // extended-remote's kill
                    conn.send("OK")?;
                    break Ended::Killed;
                }
                if pkt.starts_with('D') {
                    conn.send("OK")?;
                    break Ended::Detached;
                }
                if let Some(reply) = stub.travel_packet(target, &pkt) {
                    for part in reply.split('\u{0}') {
                        conn.send(part)?;
                    }
                    continue;
                }
                let mut ack = conn.ack;
                match stub.handle(target.session(), &mut ack, &pkt) {
                    Some(reply) => {
                        // A NUL separates several packets (monitor output, then the final reply)
                        for part in reply.split('\u{0}') {
                            conn.send(part)?;
                        }
                        conn.ack = ack;
                    }
                    None => {
                        let resumed = {
                            let conn_ref = &mut conn;
                            stub.resume(target, &pkt, || conn_ref.poll())
                        };
                        match resumed {
                            Resumed::Stopped(reply) => conn.send(&reply)?,
                            Resumed::Closed => break Ended::Killed,
                            Resumed::Ended => {
                                // GDB hears that the program exited; it may already be gone
                                let _ = conn.send("W00");
                                break Ended::Stopped;
                            }
                        }
                    }
                }
            }
        }
    };
    let m = &mut target.session().machine;
    m.bkpt_policy = old_policy;
    m.breakpoints.clear();
    m.watchpoints.clear();
    m.stop_at = [None; 2];
    log::info!(
        "GDB {}",
        match result {
            Ended::Detached => "detached",
            Ended::Killed => "disconnected",
            Ended::Stopped => "told the program exited: the run was asked to stop",
        }
    );
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Pipe {
        input: std::io::Cursor<Vec<u8>>,
        output: Vec<u8>,
    }
    impl Read for Pipe {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.input.read(buf)
        }
    }
    impl Write for Pipe {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.output.extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn framing_checksums_and_escapes() {
        let mut c =
            Connection::new(Pipe { input: std::io::Cursor::new(b"+$qC#b4\x03$X0,1:}]#00".to_vec()), output: vec![] });
        assert_eq!(c.recv().unwrap(), Incoming::Packet("qC".into()));
        assert_eq!(c.recv().unwrap(), Incoming::Interrupt);
        assert_eq!(c.recv().unwrap(), Incoming::Packet("X0,1:}".into()));
        assert_eq!(c.recv().unwrap(), Incoming::Closed);
        c.send("OK").unwrap();
        assert!(c.stream.output.ends_with(b"$OK#9a"));
        c.send("a#b").unwrap();
        // The checksum covers the packet data as sent, escapes included: 'a' + '}' + 0x03 + 'b' = 0x143
        assert!(c.stream.output.ends_with(b"$a}\x03b#43"), "{:?}", String::from_utf8_lossy(&c.stream.output));
    }

    #[test]
    fn hex_helpers() {
        assert_eq!(hex(&[0xde, 0xad]), "dead");
        assert_eq!(unhex("dead"), Some(vec![0xde, 0xad]));
        assert_eq!(unhex("abc"), None);
        assert_eq!(hex_u32_le(0x1000_0100), "00010010");
    }
}
