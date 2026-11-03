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
    if !sim_host::titan_rs_release_dir(&repo()).join("esc_board").exists() {
        eprintln!("skipping: titan_firmware_rs images not built");
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
