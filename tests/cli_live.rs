//! The command line while a run is still going on: what it writes reaches its files and pipes promptly, so tools
//! can follow a run as it happens.

use std::io::BufRead;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("cli-live-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// A running emulator whose stdout and stderr lines are collected as they arrive.
struct Running {
    child: Child,
    stdout: Arc<Mutex<Vec<String>>>,
    stderr: Arc<Mutex<Vec<String>>>,
}

fn collect(pipe: impl std::io::Read + Send + 'static) -> Arc<Mutex<Vec<String>>> {
    let lines = Arc::new(Mutex::new(Vec::new()));
    let l = lines.clone();
    std::thread::spawn(move || {
        for line in std::io::BufReader::new(pipe).lines() {
            let Ok(line) = line else { return };
            l.lock().unwrap().push(line);
        }
    });
    lines
}

impl Running {
    fn start(args: &[&str]) -> Running {
        let mut child = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
            .args(args)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let stdout = collect(child.stdout.take().unwrap());
        let stderr = collect(child.stderr.take().unwrap());
        Running { child, stdout, stderr }
    }

    fn still_running(&mut self) -> bool {
        self.child.try_wait().unwrap().is_none()
    }

    fn stderr(&self) -> String {
        self.stderr.lock().unwrap().join("\n")
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Waits up to `secs` seconds for `done`.
fn wait_until(secs: u64, mut done: impl FnMut() -> bool) -> bool {
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(secs) {
        if done() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    done()
}

#[test]
fn output_files_and_stdout_can_be_followed_while_the_run_goes_on() {
    let json = temp("live.jsonl");
    let trace = temp("live.trace");
    let elf = repo().join("test_programs/pico/hello_uart.elf");
    // hello_uart prints one line early on and then sleeps: nothing more is written for the rest of the run
    let mut r = Running::start(&[
        "run",
        elf.to_str().unwrap(),
        "--realtime",
        "--time",
        "60s",
        "--log-json",
        json.to_str().unwrap(),
        "--trace",
        "gpio",
        "--trace-file",
        trace.to_str().unwrap(),
    ]);
    let stdout = r.stdout.clone();
    assert!(
        wait_until(20, || stdout.lock().unwrap().iter().any(|l| l.contains("uart0: AB Hello, UART!"))),
        "stdout: {:?}\nstderr: {}",
        stdout.lock().unwrap(),
        r.stderr()
    );
    let read = |p: &PathBuf| std::fs::read_to_string(p).unwrap_or_default();
    assert!(wait_until(5, || read(&json).contains("Hello, UART!")), "the JSON log has {:?}", read(&json));
    assert!(wait_until(5, || read(&trace).contains("GPIO0 = 1")), "the trace has {:?}", read(&trace));
    assert!(r.still_running(), "the lines arrived while the run was going on");
}

/// Sends `sig` (`TERM`, `INT`) to the run, as `kill` does.
fn signal(r: &Running, sig: &str) {
    let ok = Command::new("kill").args([&format!("-{sig}"), &r.child.id().to_string()]).status().unwrap().success();
    assert!(ok, "kill -{sig}");
}

/// Waits up to `secs` seconds for the run to end; its exit code.
fn wait_exit(r: &mut Running, secs: u64) -> Option<i32> {
    assert!(wait_until(secs, || !r.still_running()), "still running {secs} s later:\n{}", r.stderr());
    let status = r.child.wait().unwrap();
    // The reader threads see the pipes close once the process is gone
    std::thread::sleep(Duration::from_millis(100));
    status.code()
}

#[test]
fn sigterm_ends_the_run_cleanly_and_keeps_the_flash() {
    let flash = temp("sigterm.flash");
    let json = temp("sigterm.jsonl");
    let _ = std::fs::remove_file(&flash);
    let elf = repo().join("test_programs/pico/flash_program.elf");
    // flash_program erases and programs a page 256 KiB into the flash early on, then sleeps; the run would go on
    // for ten minutes of real time
    let mut r = Running::start(&[
        "run",
        elf.to_str().unwrap(),
        "--realtime",
        "--time",
        "600s",
        "--flash-file",
        flash.to_str().unwrap(),
        "--log-json",
        json.to_str().unwrap(),
        "--unmodelled",
        "off",
    ]);
    let stdout = r.stdout.clone();
    let done = || stdout.lock().unwrap().iter().any(|l| l.contains("Programming successful!"));
    assert!(wait_until(30, done), "stdout: {:?}\nstderr: {}", stdout.lock().unwrap(), r.stderr());
    assert!(!flash.exists(), "the flash file is only written when the run ends");
    signal(&r, "TERM");
    assert_eq!(wait_exit(&mut r, 20), Some(128 + 15), "a run ended by SIGTERM exits as one:\n{}", r.stderr());
    let err = r.stderr();
    assert!(err.contains("(Interrupted); core0"), "the stop line:\n{err}");
    // The page the program wrote is in the file: the 16 lines of 16 bytes it printed after "Generated random data:"
    let lines = stdout.lock().unwrap().clone();
    let start = lines.iter().position(|l| l.contains("Generated random data:")).unwrap() + 1;
    let data: Vec<u8> = lines[start..start + 16]
        .iter()
        .flat_map(|l| {
            let hex = l.split_once("uart0: ").unwrap().1;
            hex.split_whitespace().map(|b| u8::from_str_radix(b, 16).unwrap()).collect::<Vec<_>>()
        })
        .collect();
    assert_eq!(data.len(), 256);
    let contents = std::fs::read(&flash).unwrap();
    assert_eq!(contents.len(), 2 << 20);
    assert_eq!(&contents[256 * 1024..256 * 1024 + 256], &data[..], "the programmed page");
    let end = std::fs::read_to_string(&json).unwrap();
    let last: serde_json::Value = serde_json::from_str(end.lines().last().unwrap()).unwrap();
    assert_eq!((&last["kind"], &last["reason"]), (&"end".into(), &"Interrupted".into()), "{end}");
}

#[test]
fn sigint_while_the_dashboard_is_held_quits() {
    let elf = repo().join("test_programs/blink.elf");
    let mut r = Running::start(&[
        "run",
        elf.to_str().unwrap(),
        "--time",
        "1ms",
        "--dashboard",
        "127.0.0.1:0",
        "--dashboard-hold",
        "--unmodelled",
        "off",
    ]);
    let stderr = r.stderr.clone();
    let held = || stderr.lock().unwrap().iter().any(|l| l.contains("the dashboard keeps serving"));
    assert!(wait_until(20, held), "{}", r.stderr());
    signal(&r, "INT");
    assert_eq!(wait_exit(&mut r, 10), Some(128 + 2), "{}", r.stderr());
}

#[test]
fn sigterm_ends_a_robot_run_cleanly() {
    let blink = repo().join("test_programs/blink.elf");
    let board = repo().join("boards/mk2/power_board.toml");
    let robot = temp("sigterm_robot.toml");
    std::fs::write(
        &robot,
        format!(
            "[[board]]\nname = \"a\"\nboard = {b:?}\nelf = {e:?}\n\n[[board]]\nname = \"b\"\nboard = {b:?}\nelf = {e:?}\n",
            b = board.to_str().unwrap(),
            e = blink.to_str().unwrap()
        ),
    )
    .unwrap();
    let json = temp("sigterm_robot.jsonl");
    let vcd = temp("sigterm_robot.vcd");
    let mut r = Running::start(&[
        "robot",
        robot.to_str().unwrap(),
        "--realtime",
        "--time",
        "600s",
        "--log-json",
        json.to_str().unwrap(),
        "--vcd",
        vcd.to_str().unwrap(),
        "--vcd-pins",
        "25",
    ]);
    let stderr = r.stderr.clone();
    assert!(wait_until(20, || stderr.lock().unwrap().iter().any(|l| l.contains("2 board(s)"))), "{}", r.stderr());
    std::thread::sleep(Duration::from_millis(300));
    signal(&r, "TERM");
    assert_eq!(wait_exit(&mut r, 20), Some(128 + 15), "{}", r.stderr());
    assert!(r.stderr().contains("(Interrupted);"), "{}", r.stderr());
    // The waveform got its trailer, the JSON log its end record
    let v = sim_host::vcd::parse(&std::fs::read_to_string(&vcd).unwrap()).unwrap();
    assert!(v.signal("a", "gpio25").unwrap().len() > 2);
    let log = std::fs::read_to_string(&json).unwrap();
    let last: serde_json::Value = serde_json::from_str(log.lines().last().unwrap()).unwrap();
    assert_eq!((&last["kind"], &last["reason"]), (&"end".into(), &"Interrupted".into()), "{log}");
}

#[test]
fn a_robot_behind_real_time_says_so() {
    if sim_host::titan_rs_image(&repo(), "esc_board").is_none() {
        return;
    }
    // Talos's nine boards on one thread run at about half of real time
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["robot", repo().join("robots/talos.toml").to_str().unwrap(), "--realtime", "--threads", "1"])
        .args(["--time", "0.5s"])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "{err}");
    assert!(err.contains("real-time mode: emulation is ") && err.contains(" behind the wall clock"), "{err}");
}

/// Talos's Rust boards served to host tools with no vcan: `--cannelloni BUS=ADDR` alone is a hub, and a peer that
/// only knows its address (as titan_bridge's `--interface cannelloni:HOST:PORT`) gets the bus's frames: every
/// board's CANmore heartbeat.
#[test]
fn a_cannelloni_hub_serves_a_peer_that_only_knows_its_address() {
    let robot = repo().join("robots/talos_nereus.toml");
    // A free port for the hub. Between freeing it here and the emulator binding it, another test may take it
    // (macOS hands out ephemeral ports in order): then the emulator says so and exits, and it gets another
    let (mut r, addr) = (0..5)
        .find_map(|_| {
            let hub = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            let addr = hub.local_addr().unwrap();
            drop(hub);
            let mut r = Running::start(&[
                "robot",
                robot.to_str().unwrap(),
                "--cannelloni",
                &format!("internal={addr}"),
                "--dashboard",
                "127.0.0.1:0",
                "--realtime",
                "--time",
                "600s",
            ]);
            let taken = wait_until(5, || !r.still_running() || r.stderr().contains("cannelloni hub over UDP"))
                && r.stderr().contains("Address already in use");
            (!taken).then_some((r, addr))
        })
        .expect("a free port for the hub in 5 tries");
    let peer = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
    let mut heartbeats = std::collections::BTreeSet::new();
    let mut seq = 0u8;
    let t0 = Instant::now();
    while heartbeats.len() < 4 && t0.elapsed() < Duration::from_secs(60) && r.still_running() {
        // A keepalive (a datagram without frames) introduces the peer and keeps it known
        let _ = peer.send_to(&[2, 0, seq, 0, 0], addr);
        seq = seq.wrapping_add(1);
        let mut buf = [0u8; 2048];
        while let Ok(n) = peer.recv(&mut buf) {
            for f in sim_host::cannelloni::decode(&buf[..n]).unwrap().1 {
                let fields = canmore::id::Fields::decode(f.id, f.extended);
                if fields.frame_type == canmore::id::FrameType::Util && fields.noc == canmore::CHAN_HEARTBEAT {
                    heartbeats.insert(fields.client_id);
                }
            }
        }
    }
    assert_eq!(heartbeats.into_iter().collect::<Vec<_>>(), [1, 2, 3, 4], "{}", r.stderr());
    assert!(r.stderr().contains("cannelloni hub over UDP"), "{}", r.stderr());
}

/// `--micro-ros-agent BUS=HOST[:PORT]`: a bad address or bus is refused before anything runs.
#[test]
fn micro_ros_agent_option_errors() {
    let robot = repo().join("robots/talos_nereus.toml");
    for (arg, says) in [("internal=127.0.0.1:99999", "micro-ROS agent address"), ("nobus=127.0.0.1", "nobus")] {
        let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
            .args(["robot", robot.to_str().unwrap(), "--micro-ros-agent", arg, "--dashboard", "127.0.0.1:0"])
            .args(["--time", "1ms"])
            .output()
            .unwrap();
        let err = String::from_utf8_lossy(&out.stderr);
        assert!(!out.status.success() && err.contains(says), "{arg}: {err}");
    }
}

/// A titan_firmware C board's micro-ROS session over UDP (`--micro-ros-agent`), no vcan and no CAN agent: the
/// PoAC's XRCE-DDS packets (its agent pings, GET_INFO) reach a UDP socket playing the agent, one datagram per packet,
/// all from one address (the board's client endpoint). The real agent's whole session is in docs/nereus.md. Needs the C PoAC image
/// (`$TITAN_FIRMWARE_DIR/talos/PoacBoard/build/poac_board_with_bl.uf2`); skipped without it.
#[test]
fn a_c_boards_micro_ros_session_reaches_a_udp_agent() {
    let Ok(tf) = std::env::var("TITAN_FIRMWARE_DIR") else { return };
    let image = PathBuf::from(&tf).join("talos/PoacBoard/build/poac_board_with_bl.uf2");
    if !image.exists() {
        eprintln!("skipped: no {}", image.display());
        return;
    }
    let robot = temp("udp_agent_poac.toml");
    std::fs::write(
        &robot,
        format!(
            "[[bus]]\nname = \"internal\"\nrates = \"internal\"\n\n[[board]]\nname = \"poac\"\nboard = {b:?}\n\
             elf = {e:?}\nrom_boot = true\nbus = \"internal\"\n",
            b = repo().join("boards/talos/poac_board.toml").to_str().unwrap(),
            e = image.to_str().unwrap()
        ),
    )
    .unwrap();
    let agent = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    agent.set_read_timeout(Some(Duration::from_secs(60))).unwrap();
    let r = Running::start(&[
        "robot",
        robot.to_str().unwrap(),
        "--micro-ros-agent",
        &format!("internal={}", agent.local_addr().unwrap()),
        "--realtime",
        "--time",
        "600s",
    ]);
    let mut buf = [0u8; 2048];
    let mut from = Vec::new();
    while from.len() < 2 {
        let (n, addr) = agent.recv_from(&mut buf).unwrap_or_else(|e| panic!("{e}: {}", r.stderr()));
        // XRCE: session 0x80 (no client key), stream 0, then GET_INFO (submessage 2; rmw_uros_ping_agent) or
        // CREATE_CLIENT (0, with the "XRCE" cookie)
        let p = &buf[..n];
        let ok = p.len() >= 8 && p[0] & 0x80 != 0 && (p[4] == 2 || (p[4] == 0 && p.get(8..12) == Some(b"XRCE")));
        assert!(ok, "{p:02x?}");
        from.push(addr);
    }
    assert_eq!(from[0], from[1], "one client endpoint per board");
    assert!(r.stderr().contains("micro-ROS over UDP: client 1 -> agent"), "{}", r.stderr());
}

/// Starts a long run of the blink test program (no firmware build needed), with `pre` run in the child before
/// exec, and waits until it has started running.
fn long_run(extra: &[&str], pre: Option<fn() -> std::io::Result<()>>) -> Running {
    let blink = repo().join("test_programs/blink.elf");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"));
    cmd.args(["run", blink.to_str().unwrap(), "--time", "100000s", "--realtime"]).args(extra);
    if let Some(pre) = pre {
        use std::os::unix::process::CommandExt;
        // SAFETY: the hook only calls async-signal-safe functions (sigprocmask)
        unsafe { cmd.pre_exec(pre) };
    }
    let mut child = cmd.stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap();
    let stdout = collect(child.stdout.take().unwrap());
    let stderr = collect(child.stderr.take().unwrap());
    let mut r = Running { child, stdout, stderr };
    assert!(wait_until(30, || !r.still_running() || r.stderr().contains("INFO")), "{}", r.stderr());
    std::thread::sleep(Duration::from_millis(300));
    r
}

/// Sends one SIGINT and returns the exit code, or `None` if the run was still going 10 s later.
fn one_ctrl_c(r: &mut Running) -> Option<i32> {
    // SAFETY: kill(2) on our own child
    unsafe { libc::kill(r.child.id() as libc::pid_t, libc::SIGINT) };
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(10) {
        if let Some(status) = r.child.try_wait().unwrap() {
            use std::os::unix::process::ExitStatusExt;
            return Some(status.code().unwrap_or(128 + status.signal().unwrap_or(0)));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    None
}

/// A run started with SIGINT and SIGTERM blocked (inherited from whatever started it: some shells, wrappers and
/// dev environments do this) used to ignore every Ctrl-C. The emulator unblocks them, says so, and one Ctrl-C
/// ends the run cleanly.
#[test]
fn ctrl_c_ends_a_run_started_with_sigint_blocked() {
    fn block() -> std::io::Result<()> {
        // SAFETY: sigprocmask on local sets, async-signal-safe
        unsafe {
            let mut set: libc::sigset_t = std::mem::zeroed();
            libc::sigemptyset(&mut set);
            libc::sigaddset(&mut set, libc::SIGINT);
            libc::sigaddset(&mut set, libc::SIGTERM);
            if libc::sigprocmask(libc::SIG_BLOCK, &set, std::ptr::null_mut()) != 0 {
                return Err(std::io::Error::last_os_error());
            }
        }
        Ok(())
    }
    let mut r = long_run(&[], Some(block));
    assert_eq!(one_ctrl_c(&mut r), Some(130), "{}", r.stderr());
    assert!(r.stderr().contains("were blocked by whatever started the emulator"), "{}", r.stderr());
    assert!(r.stderr().contains("(Interrupted)"), "{}", r.stderr());
}

/// Ctrl-C while the run waits for GDB to connect ends it at once (a blocking accept would have been restarted,
/// leaving only a second Ctrl-C to quit).
#[test]
fn ctrl_c_while_waiting_for_gdb_ends_the_run() {
    let mut r = long_run(&["--gdb", "127.0.0.1:0"], None);
    assert!(wait_until(10, || r.stderr().contains("waiting for GDB")), "{}", r.stderr());
    assert_eq!(one_ctrl_c(&mut r), Some(130), "{}", r.stderr());
    assert!(r.stderr().contains("(Interrupted)"), "{}", r.stderr());
}
