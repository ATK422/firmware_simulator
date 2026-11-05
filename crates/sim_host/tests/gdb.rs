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

/// Serves `session` recorded for time travel (as `run --gdb` does): GDB can run it backwards.
fn serve_travel(mut session: Session) -> (u16, std::sync::mpsc::Receiver<(Ended, Session)>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let mut tt = sim_host::timetravel::TimeTravel::new(&mut session, vec![]);
        tt.every = sim_core::Time::from_ms(2);
        let mut target = gdb::Travel { tt: &mut tt, target: &mut session, board: 0 };
        let ended = gdb::serve_target(&listener, &mut target).unwrap();
        let _ = tx.send((ended, session));
    });
    (port, rx)
}

/// The time on core 0's clock as `monitor time` says it (`core0 at T`), in seconds.
fn core_time(c: &mut Client) -> String {
    let out = c.monitor("time");
    out.split("core0 at ").nth(1).map(|t| t.split(';').next().unwrap().to_string()).unwrap_or(out)
}

/// `reverse-continue` (`bc`) and `reverse-stepi` (`bs`): back to the previous breakpoint hit with the registers it
/// had, step by step back over what ran, the start of the history, and forward again through it.
#[test]
fn reverse_continue_and_reverse_step() {
    let (image, session) = blinky();
    let main = image.symbol("main").unwrap().addr & !1;
    let (port, rx) = serve_travel(session);
    let mut c = Client::connect(port);
    let sup = c.cmd("qSupported:swbreak+;hwbreak+");
    assert!(sup.contains("ReverseStep+") && sup.contains("ReverseContinue+"), "{sup}");
    assert!(c.monitor("help").contains("rewind"));

    // Forward: main, then a few steps, remembering each PC and the registers
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    let at_main = c.cmd("g");
    let main_time = core_time(&mut c);
    let mut steps = vec![(c.cmd("g"), core_time(&mut c))];
    for _ in 0..20 {
        assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
        steps.push((c.cmd("g"), core_time(&mut c)));
    }
    // Back one step at a time: the same registers at each
    for (regs, t) in steps.iter().rev().skip(1) {
        assert_eq!(c.cmd("bs"), "T05thread:1;");
        assert_eq!(&c.cmd("g"), regs);
        assert_eq!(&core_time(&mut c), t);
    }
    // Forward again in the recorded history: the steps as they were
    for (regs, _) in steps.iter().skip(1).take(5) {
        assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
        assert_eq!(&c.cmd("g"), regs);
    }
    assert!(c.monitor("time").contains("in the recorded history"));

    // Run on (live past the history), then back to the breakpoint in main
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");
    c.send("c");
    std::thread::sleep(Duration::from_millis(100));
    c.send_raw(&[0x03]);
    assert_eq!(c.reply(), "T02thread:1;");
    assert!(c.monitor("time").contains("live"));
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("bc"), "T05thread:1;swbreak:;");
    assert_eq!(c.cmd("g"), at_main);
    assert_eq!(core_time(&mut c), main_time);
    // Nothing earlier: the start of the history, at the reset vector
    assert_eq!(c.cmd("bc"), "T05replaylog:begin;thread:1;");
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), image.symbol("Reset").unwrap().addr & !1);
    // Forward to main again
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    assert_eq!(c.cmd("g"), at_main);

    // monitor rewind goes to a time
    assert!(c.monitor("rewind 1ms").starts_with("at 0.001"), "rewound");
    assert!(c.monitor("checkpoints").contains("checkpoints"));
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// Watchpoints (`Z2` write, `Z3` read, `Z4` access) stop after the access; going back to one stops before the
/// instruction that made it.
#[test]
fn watchpoints_both_ways() {
    let (image, session) = blinky();
    let main = image.symbol("main").unwrap().addr & !1;
    let (port, rx) = serve_travel(session);
    let mut c = Client::connect(port);
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    assert_eq!(c.cmd(&format!("z0,{main:x},2")), "OK");
    // A word just below the stack pointer: the next call writes it
    let sp = le_hex_to_u32(&c.cmd("p0d"));
    let addr = sp - 4;
    assert_eq!(c.cmd(&format!("Z2,{addr:x},4")), "OK");
    let r = c.cmd("vCont;c");
    assert_eq!(r, format!("T05thread:1;watch:{addr:x};"));
    let written = c.cmd(&format!("m{addr:x},4"));
    let after_pc = le_hex_to_u32(&c.cmd("p0f"));
    // Back: before the write, the old value there
    assert_eq!(c.cmd("bc"), format!("T05thread:1;watch:{addr:x};"));
    let before_pc = le_hex_to_u32(&c.cmd("p0f"));
    assert_ne!(before_pc, after_pc);
    // One step forward makes the write again
    assert_eq!(c.cmd(&format!("z2,{addr:x},4")), "OK");
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    assert_eq!(c.cmd(&format!("m{addr:x},4")), written);
    assert_eq!(le_hex_to_u32(&c.cmd("p0f")), after_pc);
    // Read and access watchpoints are accepted and removed
    for z in ["3", "4"] {
        assert_eq!(c.cmd(&format!("Z{z},{addr:x},4")), "OK");
        assert_eq!(c.cmd(&format!("z{z},{addr:x},4")), "OK");
    }
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// A register written in the past is part of the run from then on: the recorded future is dropped, and going back
/// and forward again over the write makes it again.
#[test]
fn writes_in_the_past_change_the_run_from_there() {
    let (image, session) = blinky();
    let main = image.symbol("main").unwrap().addr & !1;
    let (port, rx) = serve_travel(session);
    let mut c = Client::connect(port);
    assert_eq!(c.cmd(&format!("Z0,{main:x},2")), "OK");
    assert_eq!(c.cmd("vCont;c"), "T05thread:1;swbreak:;");
    for _ in 0..5 {
        c.cmd("vCont;s:1");
    }
    for _ in 0..3 {
        assert_eq!(c.cmd("bs"), "T05thread:1;");
    }
    assert!(c.monitor("time").contains("in the recorded history"));
    assert_eq!(c.cmd("P8=78563412"), "OK");
    assert!(c.monitor("time").contains("live"), "the future after a write is dropped");
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    let regs = c.cmd("g");
    assert_eq!(c.cmd("bs"), "T05thread:1;");
    assert_eq!(c.cmd("bs"), "T05thread:1;");
    assert_ne!(le_hex_to_u32(&c.cmd("p08")), 0x1234_5678, "before the write");
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    assert_eq!(c.cmd("vCont;s:1"), "T05thread:1;");
    assert_eq!(c.cmd("g"), regs, "the write made again");
    c.send_raw(b"$D#44");
    assert_eq!(c.reply(), "OK");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// A GDB for Arm to drive the emulator with, as a user would: `$GDB` if set, else `gdb-multiarch` (Debian, Ubuntu),
/// else `arm-none-eabi-gdb` (Arm's toolchain, Nix, macOS). The tests below skip without one. Checked with
/// gdb-multiarch 15.1 and arm-none-eabi-gdb 14.
fn real_gdb() -> Option<String> {
    let candidates = match std::env::var("GDB") {
        Ok(g) if !g.is_empty() => vec![g],
        _ => vec!["gdb-multiarch".to_string(), "arm-none-eabi-gdb".to_string()],
    };
    let found = candidates
        .into_iter()
        .find(|g| std::process::Command::new(g).arg("--version").output().is_ok_and(|o| o.status.success()));
    if found.is_none() {
        eprintln!("skipping: no gdb-multiarch or arm-none-eabi-gdb (or $GDB)");
    }
    found
}

/// Runs the real GDB in batch mode on `elf` with `commands` as its command file (so `while` loops work); its
/// stdout and stderr.
fn gdb_batch(gdb: &str, elf: &std::path::Path, commands: &[String]) -> String {
    static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let script = std::env::temp_dir().join(format!("sim-gdb-{}-{n}.gdb", std::process::id()));
    let text = ["set pagination off".to_string(), "set confirm off".into()].iter().chain(commands).fold(
        String::new(),
        |mut s, c| {
            s.push_str(c);
            s.push('\n');
            s
        },
    );
    std::fs::write(&script, text).unwrap();
    let out = std::process::Command::new(gdb).args(["-nx", "-batch", "-x"]).arg(&script).arg(elf).output().unwrap();
    let _ = std::fs::remove_file(&script);
    String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr)
}

/// The real GDB: `reverse-stepi`, the MI command VS Code's Native Debug sends for Step Back
/// (`-exec-step-instruction --reverse`), and `reverse-continue`.
#[test]
fn real_gdb_steps_and_continues_backwards() {
    let Some(gdb) = real_gdb() else {
        return;
    };
    let (image, session) = blinky();
    let main = image.symbol("main").unwrap().addr & !1;
    let reset = image.symbol("Reset").unwrap().addr & !1;
    let (port, rx) = serve_travel(session);
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    let commands: Vec<String> = [
        format!("target extended-remote :{port}"),
        "break main".into(),
        "continue".into(),
        "printf \"at main %x\\n\", $pc".into(),
        "stepi".into(),
        "stepi".into(),
        "stepi".into(),
        "printf \"stepped %x\\n\", $pc".into(),
        "reverse-stepi".into(),
        "interpreter-exec mi \"-exec-step-instruction --reverse\"".into(),
        "reverse-stepi".into(),
        "printf \"back %x\\n\", $pc".into(),
        "reverse-continue".into(),
        "printf \"start %x\\n\", $pc".into(),
        "detach".into(),
    ]
    .into();
    let text = gdb_batch(&gdb, &elf, &commands);
    // (GDB breaks after main's prologue)
    let at_main = text.split("at main ").nth(1).and_then(|t| t.split_whitespace().next()).expect(&text).to_string();
    let pc = u32::from_str_radix(&at_main, 16).unwrap();
    assert!((main..main + 16).contains(&pc), "{text}");
    assert!(!text.contains(&format!("stepped {at_main}")), "{text}");
    assert!(text.contains(&format!("back {at_main}")), "three steps back is main again: {text}");
    assert!(text.contains("No more reverse-execution history"), "{text}");
    assert!(text.contains(&format!("start {reset:x}")), "the start of the history: {text}");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// The real GDB stepping 300 instructions forward and then back, interrupts and both cores' work included: every
/// step back lands where the step forward started, with the same registers. Then a watchpoint: `continue` stops
/// after the write with the new value, `reverse-continue` before it with the old one.
#[test]
fn real_gdb_reverse_steps_retrace_the_forward_steps_and_watchpoints_go_both_ways() {
    let Some(gdb) = real_gdb() else {
        return;
    };
    let (_, session) = blinky();
    let (port, rx) = serve_travel(session);
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    let n = 300;
    let commands: Vec<String> = [
        format!("target extended-remote :{port}"),
        "break main".into(),
        "continue".into(),
        "delete".into(),
        "set $i = 0".into(),
        format!("while $i < {n}\nstepi\nprintf \"F %d %x %x %x %x\\n\", $i, $pc, $sp, $r0, $lr\nset $i = $i + 1\nend"),
        format!("set $i = {}", n - 1),
        "while $i > 0\nreverse-stepi\nset $i = $i - 1\nprintf \"R %d %x %x %x %x\\n\", $i, $pc, $sp, $r0, $lr\nend"
            .into(),
        // A word just below the stack pointer: the next call writes it
        "watch -location *(unsigned *)($sp - 4)".into(),
        "continue".into(),
        "reverse-continue".into(),
        "detach".into(),
    ]
    .into();
    let text = gdb_batch(&gdb, &elf, &commands);
    let steps = |k: &str| -> std::collections::BTreeMap<u32, String> {
        text.lines()
            .filter_map(|l| l.strip_prefix(k))
            .map(|l| {
                let (i, regs) = l.split_once(' ').unwrap();
                (i.parse().unwrap(), regs.to_string())
            })
            .collect()
    };
    let (fwd, back) = (steps("F "), steps("R "));
    assert_eq!(fwd.len(), n as usize, "{text}");
    assert_eq!(back.len(), n as usize - 1, "{text}");
    for (i, regs) in &back {
        assert_eq!(Some(regs), fwd.get(i), "step {i}: {text}");
    }
    let pcs: std::collections::BTreeSet<&str> = fwd.values().map(|r| r.split(' ').next().unwrap()).collect();
    assert!(pcs.len() > 50, "the steps went somewhere: {pcs:?}");
    // Forward: "Old value = A / New value = B"; backward the same watchpoint, the values the other way round
    let values: Vec<(&str, &str)> = text
        .split("Old value = ")
        .skip(1)
        .map(|t| {
            let old = t.lines().next().unwrap().trim();
            let new = t.split("New value = ").nth(1).unwrap().lines().next().unwrap().trim();
            (old, new)
        })
        .collect();
    assert_eq!(values.len(), 2, "{text}");
    assert_eq!((values[1].0, values[1].1), (values[0].1, values[0].0), "{text}");
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// A GDB/MI session with the real GDB, as Cortex-Debug drives one with `servertype: external` and the overrides in
/// `.vscode/launch.json`: connect, `monitor halt`, run to main, threads, a peripheral block for the SVD view, pause,
/// Native Debug's Step Back and Reverse Continue, Restart (`monitor reset halt`, flush the register cache), run to
/// main again, detach.
#[test]
fn real_gdb_mi_session_as_cortex_debug_runs_it() {
    let Some(gdb) = real_gdb() else {
        return;
    };
    let (image, session) = blinky();
    let reset = image.symbol("Reset").unwrap().addr & !1;
    let (port, rx) = serve_travel(session);
    let elf = common::repo().join("test_programs/embassy_blinky.elf");
    let mut mi = Mi::start(&gdb, &elf);
    mi.cmd("-gdb-set mi-async on", None);
    let out = mi.cmd(&format!("-target-select extended-remote localhost:{port}"), None);
    assert!(out.iter().any(|l| l.contains("^connected")), "{out:?}");
    // overrideLaunchCommands
    let out = mi.cmd("-interpreter-exec console \"monitor halt\"", None);
    assert!(out.iter().any(|l| l == "@\"halted\\n\""), "{out:?}");
    // runToEntryPoint: main
    mi.cmd("-break-insert -t --function main", None);
    let out = mi.cmd("-exec-continue", Some("*stopped"));
    assert!(
        out.iter().any(|l| l.starts_with("*stopped,reason=\"breakpoint-hit\"") && l.contains("func=\"main\"")),
        "{out:?}"
    );
    let out = mi.cmd("-thread-info", None);
    let threads = out.iter().find(|l| l.contains("^done,threads=")).unwrap();
    assert!(threads.contains("details=\"core 0\"") && threads.contains("details=\"core 1\""), "{threads}");
    // The Cortex Peripherals view: TIMER's registers (TIMERAWL at 0x28 counts microseconds)
    let out = mi.cmd("-data-read-memory-bytes 0x40054000 64", None);
    let mem = out.iter().find(|l| l.contains("contents=")).unwrap();
    assert_eq!(mem.split("contents=\"").nth(1).unwrap().split('"').next().unwrap().len(), 128, "{mem}");
    // Pause
    mi.cmd("-exec-continue", None);
    std::thread::sleep(Duration::from_millis(200));
    let out = mi.cmd("-exec-interrupt", Some("*stopped"));
    assert!(out.iter().any(|l| l.contains("*stopped,reason=\"signal-received\",signal-name=\"SIGINT\"")), "{out:?}");
    // Native Debug's Step Back, then Reverse Continue to the start of the history (no breakpoints are left)
    let out = mi.cmd("-exec-step-instruction --reverse", Some("*stopped"));
    assert!(out.iter().any(|l| l.contains("*stopped,reason=\"end-stepping-range\"")), "{out:?}");
    let out = mi.cmd("-exec-continue --reverse", Some("*stopped"));
    assert!(out.iter().any(|l| l.contains("*stopped,reason=\"no-history\"")), "{out:?}");
    // Restart: overrideRestartCommands, then the reset vector's registers
    let out = mi.cmd("-interpreter-exec console \"monitor reset halt\"", None);
    assert!(out.iter().any(|l| l.contains("halted at the reset vector")), "{out:?}");
    mi.cmd("-interpreter-exec console \"maintenance flush register-cache\"", None);
    let out = mi.cmd("-data-list-register-values x 15", None);
    assert!(out.iter().any(|l| l.contains(&format!("value=\"{reset:#x}\""))), "{out:?}");
    mi.cmd("-break-insert -t --function main", None);
    let out = mi.cmd("-exec-continue", Some("*stopped"));
    assert!(out.iter().any(|l| l.starts_with("*stopped,reason=\"breakpoint-hit\"")), "{out:?}");
    let out = mi.cmd("-target-detach", None);
    assert!(out.iter().any(|l| l.ends_with("^done")), "{out:?}");
    mi.cmd("-gdb-exit", None);
    assert_eq!(ended(&rx).0, Ended::Detached);
}

/// The real GDB in MI mode over its stdin and stdout, one command at a time.
struct Mi {
    child: std::process::Child,
    lines: std::sync::mpsc::Receiver<String>,
    token: u32,
}

impl Mi {
    fn start(gdb: &str, elf: &std::path::Path) -> Mi {
        let mut child = std::process::Command::new(gdb)
            .args(["-nx", "--interpreter=mi2"])
            .arg(elf)
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::null())
            .spawn()
            .unwrap();
        let stdout = child.stdout.take().unwrap();
        let (tx, lines) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            use std::io::BufRead;
            for l in std::io::BufReader::new(stdout).lines().map_while(Result::ok) {
                if tx.send(l).is_err() {
                    break;
                }
            }
        });
        Mi { child, lines, token: 0 }
    }

    /// Sends `command` and returns GDB's output up to its result record (`^done`, `^running`, ...), and with
    /// `until`, on to the first line starting with that (an async record such as `*stopped`).
    fn cmd(&mut self, command: &str, until: Option<&str>) -> Vec<String> {
        self.token += 1;
        let t = self.token.to_string();
        let stdin = self.child.stdin.as_mut().unwrap();
        writeln!(stdin, "{t}{command}").unwrap();
        stdin.flush().unwrap();
        let mut out = Vec::new();
        let (mut done, mut seen) = (false, until.is_none());
        while !(done && seen) {
            let l = self.lines.recv_timeout(Duration::from_secs(30)).unwrap_or_else(|_| panic!("{command}: {out:?}"));
            done |= l.starts_with(&format!("{t}^"));
            assert!(!l.starts_with(&format!("{t}^error")), "{command}: {l}");
            seen |= until.is_some_and(|u| l.starts_with(u));
            out.push(l);
        }
        out
    }
}

impl Drop for Mi {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
