//! GDB remote serial protocol stub.
//!
//! Each RP2040 core is a GDB thread (thread 1 = core 0, thread 2 = core 1). Supported: registers (`g`/`G`/`p`/`P`,
//! with an M-profile target description), memory (`m`/`M`/`X`; writes reach flash, so `load` works), software and
//! hardware breakpoints (`Z0`/`Z1`), `c`/`s`/`vCont`, Ctrl-C, `qRcmd` monitor commands (`reset`, `time`), no-ack
//! mode and detach/kill. While running, the session keeps polling RTT so logs keep flowing.

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};

use cpu_armv6m::reg;
use rp2040::StopReason;
use rp2040::machine::BkptPolicy;
use sim_core::bus::Size;
use sim_core::cpu::Cpu;

use crate::session::Session;

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

/// Whether a packet resumes execution (continue or step).
fn is_resume(pkt: &str) -> bool {
    pkt == "c" || pkt == "s" || pkt.starts_with("vCont;") || pkt.starts_with('C') || pkt.starts_with('S')
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
        let sum = payload.bytes().fold(0u8, |a, b| a.wrapping_add(b));
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
}

/// The protocol state for one connected debugger.
pub struct GdbStub {
    /// Thread (core) selected for register access and stepping (0-based core index).
    current: usize,
}

impl Default for GdbStub {
    fn default() -> Self {
        Self::new()
    }
}

impl GdbStub {
    pub fn new() -> Self {
        GdbStub { current: 0 }
    }

    fn thread_of(&self, core: usize) -> String {
        format!("{:x}", core + 1)
    }

    fn stop_reply(&self, session: &Session, reason: Option<StopReason>) -> String {
        let (core, extra) = match reason {
            Some(StopReason::Breakpoint { core, .. }) => (core as usize, "swbreak:;"),
            Some(StopReason::BkptInstruction { core, .. }) => (core as usize, "swbreak:;"),
            Some(StopReason::Lockup { core, .. }) => (core as usize, ""),
            _ => (self.current, ""),
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
        let out = match cmd.trim() {
            "reset" => {
                let m = &mut session.machine;
                m.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
                "machine reset\n".to_string()
            }
            "time" => format!("virtual time {}\n", session.machine.now),
            "help" => "monitor commands: reset, time\n".to_string(),
            other => format!("unknown monitor command {other:?}\n"),
        };
        hex(out.as_bytes())
    }

    /// Handles one packet that does not resume execution. Returns the reply, or `None` for resume packets.
    pub fn handle(&mut self, session: &mut Session, conn_ack: &mut bool, pkt: &str) -> Option<String> {
        let m = &mut session.machine;
        let reply = if pkt == "?" {
            self.stop_reply(session, None)
        } else if pkt.starts_with("qSupported") {
            "PacketSize=4000;qXfer:features:read+;vContSupported+;swbreak+;hwbreak+;QStartNoAckMode+".into()
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
        } else if let Some(data) = pkt.strip_prefix('G') {
            let bytes = unhex(data)?;
            for (n, chunk) in bytes.chunks(4).enumerate().take(G_REGS) {
                if chunk.len() == 4 {
                    m.cores[self.current].write_reg(n, u32::from_le_bytes(chunk.try_into().unwrap()));
                }
            }
            "OK".into()
        } else if let Some(n) = pkt.strip_prefix('p') {
            match self.read_reg(session, parse_hex(n)? as usize) {
                Some(v) => hex_u32_le(v),
                None => "E01".into(),
            }
        } else if let Some(rest) = pkt.strip_prefix('P') {
            let (n, v) = rest.split_once('=')?;
            let n = parse_hex(n)? as usize;
            let v = unhex(v)?;
            if n < reg::COUNT && v.len() == 4 {
                m.cores[self.current].write_reg(n, u32::from_le_bytes(v.try_into().unwrap()));
                "OK".into()
            } else {
                "E01".into()
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
        } else if let Some(rest) = pkt.strip_prefix('M') {
            let (head, data) = rest.split_once(':')?;
            let (addr, _) = head.split_once(',')?;
            let addr = parse_hex(addr)?;
            let bytes = unhex(data)?;
            let ok = bytes.iter().enumerate().all(|(i, &b)| m.sys.poke(addr + i as u32, Size::Byte, b as u32));
            if ok { "OK".into() } else { "E01".into() }
        } else if let Some(rest) = pkt.strip_prefix('X') {
            let (head, data) = rest.split_once(':')?;
            let (addr, _) = head.split_once(',')?;
            let addr = parse_hex(addr)?;
            let ok = data.chars().enumerate().all(|(i, c)| m.sys.poke(addr + i as u32, Size::Byte, c as u32));
            if ok { "OK".into() } else { "E01".into() }
        } else if let Some(rest) = pkt.strip_prefix("Z0,").or_else(|| pkt.strip_prefix("Z1,")) {
            let addr = parse_hex(rest.split(',').next()?)?;
            m.breakpoints.insert(addr & !1);
            "OK".into()
        } else if let Some(rest) = pkt.strip_prefix("z0,").or_else(|| pkt.strip_prefix("z1,")) {
            let addr = parse_hex(rest.split(',').next()?)?;
            m.breakpoints.remove(&(addr & !1));
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

    /// Runs a resume packet (`c`, `s`, `vCont`) and returns the stop reply. `poll` is asked between slices whether
    /// GDB sent an interrupt.
    pub fn resume(&mut self, session: &mut Session, pkt: &str, mut interrupted: impl FnMut() -> bool) -> String {
        let step = pkt == "s" || pkt.starts_with("vCont;s") || pkt.starts_with("vCont;S") || pkt.starts_with('S');
        if step {
            // vCont;s:N selects the thread
            if let Some(t) = pkt.split(':').nth(1).and_then(|t| parse_hex(t.split(';').next().unwrap_or(t)))
                && (1..=2).contains(&t)
            {
                self.current = t as usize - 1;
            }
            let core = self.current;
            session.machine.single_step(core);
            session.poll_rtt();
            return format!("T05thread:{};", self.thread_of(core));
        }
        loop {
            let reason = session.run_for(session.slice.max(sim_core::Time::from_us(500)));
            match reason {
                StopReason::TimeLimit | StopReason::Requested => {
                    if interrupted() {
                        return format!("T02thread:{};", self.thread_of(self.current));
                    }
                }
                StopReason::Deadlock => {
                    // Nothing will ever happen again; report a stop so the user can look around
                    return format!("T05thread:{};", self.thread_of(self.current));
                }
                other => {
                    if let StopReason::Breakpoint { core, .. }
                    | StopReason::BkptInstruction { core, .. }
                    | StopReason::Lockup { core, .. } = other
                    {
                        self.current = core as usize;
                    }
                    return self.stop_reply(session, Some(other));
                }
            }
        }
    }
}

/// Serves one GDB connection on `listener`, driving `session` until GDB detaches or disconnects.
pub fn serve(listener: &TcpListener, session: &mut Session) -> io::Result<Ended> {
    let (stream, peer) = listener.accept()?;
    stream.set_nodelay(true)?;
    log::info!("GDB connected from {peer}");
    let old_policy = session.machine.bkpt_policy;
    session.machine.bkpt_policy = BkptPolicy::Stop;
    let mut conn = Connection::new(stream);
    let mut stub = GdbStub::new();
    let result = loop {
        match conn.recv()? {
            Incoming::Closed => break Ended::Killed,
            Incoming::Interrupt => conn.send(&format!("T02thread:{};", stub.thread_of(stub.current)))?,
            Incoming::Packet(pkt) => {
                if pkt == "k" {
                    break Ended::Killed;
                }
                if pkt.starts_with('D') {
                    conn.send("OK")?;
                    break Ended::Detached;
                }
                let mut ack = conn.ack;
                match stub.handle(session, &mut ack, &pkt) {
                    Some(reply) => {
                        // A NUL separates several packets (monitor output, then the final reply)
                        for part in reply.split('\u{0}') {
                            conn.send(part)?;
                        }
                        conn.ack = ack;
                    }
                    None => {
                        let reply = {
                            let conn_ref = &mut conn;
                            stub.resume(session, &pkt, || matches!(conn_ref.poll(), Ok(Some(Incoming::Interrupt))))
                        };
                        conn.send(&reply)?;
                    }
                }
            }
        }
    };
    session.machine.bkpt_policy = old_policy;
    session.machine.breakpoints.clear();
    log::info!("GDB {}", if result == Ended::Detached { "detached" } else { "disconnected" });
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
        assert!(
            c.stream.output.ends_with(b"$a}\x03b#"[..].as_ref())
                || String::from_utf8_lossy(&c.stream.output).contains("a}\u{3}b#")
        );
    }

    #[test]
    fn hex_helpers() {
        assert_eq!(hex(&[0xde, 0xad]), "dead");
        assert_eq!(unhex("dead"), Some(vec![0xde, 0xad]));
        assert_eq!(unhex("abc"), None);
        assert_eq!(hex_u32_le(0x1000_0100), "00010010");
    }
}
