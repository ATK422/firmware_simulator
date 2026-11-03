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
