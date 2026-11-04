//! M1: the GDB stub breaks in `main`, reads registers and memory, single steps and stops on Ctrl-C, driven over
//! TCP by a minimal RSP client. M11: what GDB sends for Cortex-Debug (`target extended-remote`, OpenOCD-style
//! monitor commands, peripheral reads for the SVD view, both cores), port checks, GDB going away while the target
//! runs, the run being stopped, and debugging one board of a robot.

mod common;

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use sim_host::Session;
use sim_host::gdb::{self, Ended};

struct Client {
    s: TcpStream,
}

impl Client {
    fn connect(port: u16) -> Client {
        let s = TcpStream::connect(("127.0.0.1", port)).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(30))).unwrap();
        Client { s }
    }

    /// Sends a packet without waiting for the reply.
    fn send(&mut self, payload: &str) {
        let sum = payload.bytes().fold(0u8, |a, b| a.wrapping_add(b));
        self.send_raw(format!("${payload}#{sum:02x}").as_bytes());
    }

    /// The text a `monitor` command printed (its `O` packets, hex decoded), up to the final `OK`.
    fn monitor(&mut self, cmd: &str) -> String {
        let hex: String = cmd.bytes().map(|b| format!("{b:02x}")).collect();
        let mut out = String::new();
        let mut reply = self.cmd(&format!("qRcmd,{hex}"));
        while let Some(h) = reply.strip_prefix('O').filter(|_| reply != "OK") {
            let bytes: Vec<u8> =
                (0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap()).collect();
            out.push_str(&String::from_utf8(bytes).unwrap());
            reply = self.reply();
        }
        assert_eq!(reply, "OK");
        out
    }

    fn send_raw(&mut self, data: &[u8]) {
        self.s.write_all(data).unwrap();
    }

    fn cmd(&mut self, payload: &str) -> String {
        self.send(payload);
        self.reply()
    }

    fn reply(&mut self) -> String {
        let mut buf = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            self.s.read_exact(&mut byte).unwrap();
            match byte[0] {
                b'+' if buf.is_empty() => continue,
                b'#' => {
                    let mut sum = [0u8; 2];
                    self.s.read_exact(&mut sum).unwrap();
                    self.send_raw(b"+");
                    let start = buf.iter().position(|&b| b == b'$').unwrap();
                    return String::from_utf8_lossy(&buf[start + 1..]).into_owned();
                }
                b => buf.push(b),
            }
        }
    }
}

fn le_hex_to_u32(s: &str) -> u32 {
    let b: Vec<u8> = (0..8).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect();
    u32::from_le_bytes(b.try_into().unwrap())
}

#[test]
fn break_in_main_step_and_interrupt() {
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    let image = sim_core::Image::load(&elf).unwrap();
    let main = image.symbol("main").expect("main symbol").addr & !1;
    let mut session = common::boot(&elf, None);
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = std::thread::spawn(move || {
        let ended = gdb::serve(&listener, &mut session).unwrap();
        (ended, session)
    });

    let mut c = Client { s: TcpStream::connect(("127.0.0.1", port)).unwrap() };
    assert!(c.cmd("qSupported:swbreak+").contains("qXfer:features:read+"));
    assert!(c.cmd("qXfer:features:read:target.xml:0,fff").contains("org.gnu.gdb.arm.m-profile"));
    assert_eq!(c.cmd("?"), "T05thread:1;");
    let regs = c.cmd("g");
    assert_eq!(regs.len(), 17 * 8);
    let pc = le_hex_to_u32(&regs[15 * 8..16 * 8]);
    assert_eq!(pc, image.symbol("Reset").unwrap().addr & !1, "halted at the reset handler");

    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main, "stopped at main");
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");

    // Memory: the vector table's reset vector
    let vec = c.cmd("m10000104,4");
    assert_eq!(le_hex_to_u32(&vec) & !1, image.symbol("Reset").unwrap().addr & !1);
    // PPB registers come from the core: VTOR
    assert_eq!(le_hex_to_u32(&c.cmd("me000ed08,4")), 0x1000_0100);

    // Single step moves the PC
    let before = le_hex_to_u32(&c.cmd("p0f"));
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    assert_ne!(le_hex_to_u32(&c.cmd("p0f")), before);

    // Monitor command
    let out = c.cmd("qRcmd,74696d65"); // "time"
    assert!(out.starts_with('O'));
    assert_eq!(c.reply(), "OK");

    // Continue, then interrupt
    let sum = "c".bytes().fold(0u8, |a, b| a.wrapping_add(b));
    c.send_raw(format!("$c#{sum:02x}").as_bytes());
    std::thread::sleep(std::time::Duration::from_millis(50));
    c.send_raw(&[0x03]);
    assert_eq!(c.reply(), "T02thread:1;");

    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    let (ended, session) = server.join().unwrap();
    assert_eq!(ended, Ended::Detached);
    assert!(session.machine.now > sim_core::Time::ZERO);
    assert!(session.machine.breakpoints.is_empty());
}

/// Serves `session` on a free port on another thread; returns the port and a receiver for how it ended. A test that
/// breaks the server fails on the receiver's timeout instead of hanging.
fn serve(mut session: Session) -> (u16, std::sync::mpsc::Receiver<(Ended, Session)>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let ended = gdb::serve(&listener, &mut session).unwrap();
        let _ = tx.send((ended, session));
    });
    (port, rx)
}

fn blinky() -> (sim_core::Image, Session) {
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    (sim_core::Image::load(&elf).unwrap(), common::boot(&elf, None))
}

fn ended(rx: &std::sync::mpsc::Receiver<(Ended, Session)>) -> (Ended, Session) {
    rx.recv_timeout(Duration::from_secs(30)).expect("the GDB server did not end")
}

/// The packets GDB sends when Cortex-Debug (`servertype: external`) connects with `target extended-remote` and runs
/// its launch, restart and peripheral (SVD) view commands.
#[test]
fn cortex_debug_session() {
    let (image, session) = blinky();
    let reset = image.symbol("Reset").unwrap().addr & !1;
    let main = image.symbol("main").unwrap().addr & !1;
    let (port, rx) = serve(session);
    let mut c = Client::connect(port);
    assert!(c.cmd("qSupported:multiprocess+;swbreak+;hwbreak+;vContSupported+").contains("QStartNoAckMode+"));
    assert_eq!(c.cmd("vMustReplyEmpty"), "");
    assert_eq!(c.cmd("QStartNoAckMode"), "OK");
    // Extended mode, and the queries GDB makes on connecting
    assert_eq!(c.cmd("!"), "OK");
    assert_eq!(c.cmd("Hg0"), "OK");
    assert_eq!(c.cmd("qTStatus"), "");
    assert_eq!(c.cmd("?"), "T05thread:1;");
    // Core 1 waits in the bootrom to be launched: a thread from the start
    assert_eq!(c.cmd("qfThreadInfo"), "m1,2");
    assert_eq!(c.cmd("qsThreadInfo"), "l");
    assert_eq!(c.cmd("qAttached"), "1");
    assert_eq!(c.cmd("Hc-1"), "OK");
    assert_eq!(c.cmd("qOffsets"), "");
    assert_eq!(c.cmd("qSymbol::"), "OK");

    // Launch: OpenOCD-style monitor commands
    assert_eq!(c.monitor("halt"), "halted\n");
    assert!(c.monitor("help").contains("reset [halt|init|run]"));
    assert!(c.monitor("frobnicate").contains("unknown monitor command \"frobnicate\""));
    // runToEntryPoint: main
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main);
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");
    // xPSR, MSP, PSP, PRIMASK and CONTROL (the registers view)
    assert_eq!(le_hex_to_u32(&c.cmd("p10")) & (1 << 24), 1 << 24, "Thumb bit");
    assert_eq!(c.cmd("p11"), c.cmd("p0d"), "SP is MSP in thread mode");
    for r in ["p12", "p13", "p14"] {
        assert_eq!(c.cmd(r).len(), 8, "{r}");
    }

    // Run a while, interrupt (pause), and read peripheral registers as the SVD view does: whole address blocks
    c.send("vCont;c");
    std::thread::sleep(Duration::from_millis(100));
    c.send_raw(&[0x03]);
    assert_eq!(c.reply(), "T02thread:1;");
    let timer = c.cmd("m40054000,1000"); // TIMER's 4 KB address block
    assert_eq!(timer.len(), 0x1000 * 2);
    let timerawl = le_hex_to_u32(&timer[0x28 * 2..0x2c * 2]);
    assert!(timerawl > 0, "TIMERAWL counts microseconds");
    // The SIO is per core: CPUID reads 0 on core 0
    assert_eq!(le_hex_to_u32(&c.cmd("md0000000,4")), 0);
    // SCB CPUID (PPB): a Cortex-M0+
    assert_eq!(le_hex_to_u32(&c.cmd("me000ed00,4")) >> 4 & 0xfff, 0xc60);

    // Restart: reset and stay halted at the reset vector
    let resets = c.monitor("reset halt");
    assert!(resets.contains("machine reset"), "{resets}");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), reset, "halted at the reset handler");
    assert!(c.monitor("reset").contains("machine reset"));

    // Stop debugging: extended-remote's kill
    assert_eq!(c.cmd("vKill;a410"), "OK");
    let (ended, session) = ended(&rx);
    assert_eq!(ended, Ended::Killed);
    assert_eq!(session.machine.resets, 2);
    assert!(session.machine.breakpoints.is_empty());
}

/// Each core is a thread: thread 2 has core 1's registers and its view of the SIO, and steps core 1.
#[test]
fn both_cores_are_threads() {
    let Some(elf) = common::titan_elf("esc_board") else { return };
    let mut session = common::boot(&elf, None);
    session.run_until(sim_core::Time::from_ms(200));
    assert!(session.machine.core_enabled[1], "the ESC board's control loop runs on core 1");
    let (port, rx) = serve(session);
    let mut c = Client::connect(port);
    assert_eq!(c.cmd("qfThreadInfo"), "m1,2");
    assert_eq!(c.cmd("T2"), "OK");
    let info = c.cmd("qThreadExtraInfo,2");
    assert_eq!(info, "636f72652031", "\"core 1\"");
    assert_eq!(c.cmd("Hg2"), "OK");
    assert_eq!(le_hex_to_u32(&c.cmd("md0000000,4")), 1, "SIO CPUID on core 1");
    let pc1 = le_hex_to_u32(&c.cmd("p0f"));
    assert_eq!(c.cmd("Hg1"), "OK");
    assert_eq!(le_hex_to_u32(&c.cmd("md0000000,4")), 0);
    assert_ne!(le_hex_to_u32(&c.cmd("p0f")), pc1, "core 0's PC");
    // Stepping thread 2 steps core 1
    assert_eq!(c.cmd("vCont;s:2;c"), "T05thread:2;");
    assert_eq!(c.cmd("qC"), "QC2");
    c.send_raw(b"$k#6b");
    assert_eq!(ended(&rx).0, Ended::Killed);
}

/// A connection that closes before sending anything (a port check, a task waiting for the port) is not taken for
/// the debugger: the next connection is.
#[test]
fn port_checks_are_not_debuggers() {
    let (_, session) = blinky();
    let (port, rx) = serve(session);
    for _ in 0..2 {
        drop(TcpStream::connect(("127.0.0.1", port)).unwrap());
    }
    let mut c = Client::connect(port);
    assert_eq!(c.cmd("?"), "T05thread:1;");
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// GDB going away while the target runs ends the session (the run used to go on for ever).
#[test]
fn gdb_closing_while_running_ends_the_session() {
    let (_, session) = blinky();
    let (port, rx) = serve(session);
    let mut c = Client::connect(port);
    assert_eq!(c.cmd("?"), "T05thread:1;");
    c.send("c");
    std::thread::sleep(Duration::from_millis(50));
    drop(c);
    let (ended, session) = ended(&rx);
    assert_eq!(ended, Ended::Killed);
    assert!(session.machine.now > sim_core::Time::ZERO);
}

/// The run being asked to stop (SIGINT, SIGTERM) while the target runs: GDB hears that the program exited.
#[test]
fn stopping_the_run_tells_gdb_the_program_exited() {
    static STOP: AtomicBool = AtomicBool::new(false);
    let (_, mut session) = blinky();
    session.stop_flag = Some(&STOP);
    let (port, rx) = serve(session);
    let mut c = Client::connect(port);
    assert_eq!(c.cmd("?"), "T05thread:1;");
    c.send("c");
    std::thread::sleep(Duration::from_millis(50));
    STOP.store(true, Ordering::SeqCst);
    assert_eq!(c.reply(), "W00");
    assert_eq!(ended(&rx).0, Ended::Stopped);
}

/// One board of a robot: continuing runs every board, a breakpoint on the debugged board stops them all.
#[test]
fn robot_board_runs_the_whole_robot() {
    let (image, a) = blinky();
    let (_, b) = blinky();
    let main = image.symbol("main").unwrap().addr & !1;
    let mut robot = sim_host::robot::Robot::default();
    robot.add("a", a, None);
    robot.add("b", b, None);
    robot.threads = 0;
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let ended = gdb::serve_target(&listener, &mut gdb::RobotBoard { robot: &mut robot, board: 1 }).unwrap();
        let _ = tx.send((ended, robot));
    });
    let mut c = Client::connect(port);
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("c"), "T05thread:1;swbreak:;");
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");
    c.send("c");
    std::thread::sleep(Duration::from_millis(100));
    c.send_raw(&[0x03]);
    assert_eq!(c.reply(), "T02thread:1;");
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    let (ended, robot) = rx.recv_timeout(Duration::from_secs(30)).unwrap();
    assert_eq!(ended, Ended::Detached);
    assert_eq!(robot.threads, 0, "restored");
    let (ta, tb) = (robot.boards[0].machine.now, robot.boards[1].machine.now);
    assert!(robot.now > sim_core::Time::from_ms(1), "{}", robot.now);
    // Both boards ran, in step: neither is more than a slice ahead of the robot
    for t in [ta, tb] {
        assert!(t >= robot.now && t <= robot.now + robot.slice, "{t} vs {}", robot.now);
    }
    assert!(robot.boards[1].machine.breakpoints.is_empty());
}

/// A breakpoint stops the core again after the PC moved away from where it last stopped without running through
/// the breakpoint check: a `monitor reset halt` (Cortex-Debug's restart) or GDB writing the PC. Resuming used to
/// let the next breakpoint hit through, wherever it was, so `runToEntryPoint` after a restart ran past `main`.
#[test]
fn breakpoints_stop_again_after_a_reset_or_a_pc_write() {
    let (image, session) = blinky();
    let reset = image.symbol("Reset").unwrap().addr & !1;
    let main = image.symbol("main").unwrap().addr & !1;
    let (port, rx) = serve(session);
    let mut c = Client::connect(port);
    // Resume, and interrupt if the breakpoint did not stop it (main is reached within microseconds)
    let resume = |c: &mut Client| {
        c.send("c");
        std::thread::sleep(Duration::from_millis(200));
        c.send_raw(&[0x03]);
        let r = c.reply();
        if r.starts_with("T05") {
            // The interrupt arrived after the stop: the stub answers it with another stop reply
            assert_eq!(c.reply(), "T02thread:1;");
        }
        r
    };
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(resume(&mut c), "T05thread:1;swbreak:;");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main);
    // GDB writes the PC (`jump`, `set $pc`): back to the reset handler, which reaches main again
    assert_eq!(c.cmd(&format!("P0f={}", hex_le(reset | 1))), "OK");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), reset);
    assert_eq!(resume(&mut c), "T05thread:1;swbreak:;", "stopped at main after the PC write");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main);
    // Restart
    assert!(c.monitor("reset halt").contains("machine reset"));
    assert_eq!(resume(&mut c), "T05thread:1;swbreak:;", "stopped at main after the reset");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), main);
    // Resuming from the breakpoint itself runs the instruction there: main is not reached again
    assert_eq!(resume(&mut c), "T02thread:1;");
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

fn hex_le(v: u32) -> String {
    v.to_le_bytes().iter().map(|b| format!("{b:02x}")).collect()
}
