//! Emulator child processes: spawned so that they die with the server, their console output collected.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Read};
use std::net::SocketAddr;
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::{Arc, Mutex, OnceLock, mpsc};
use std::time::{Duration, Instant};

/// Console lines kept per process.
const KEPT_LINES: usize = 10_000;

/// What a child printed, stdout and stderr in the order the lines arrived.
#[derive(Default)]
pub struct Output {
    pub lines: VecDeque<String>,
    /// Lines printed in all, kept or not.
    pub total: u64,
    /// Where the child's dashboard listens, once it has said so.
    pub dashboard: Option<SocketAddr>,
    /// Streams at their end (2: both).
    closed: u8,
}

impl Output {
    fn push(&mut self, line: String) {
        if self.dashboard.is_none()
            && let Some(rest) = line.split("dashboard on http://").nth(1)
        {
            self.dashboard = rest.split('/').next().and_then(|a| a.parse().ok());
        }
        self.total += 1;
        self.lines.push_back(line);
        if self.lines.len() > KEPT_LINES {
            self.lines.pop_front();
        }
    }
}

/// A running (or finished) emulator.
pub struct Proc {
    pub pid: u32,
    /// The command line, for people.
    pub command: String,
    pub started: Instant,
    child: Mutex<Child>,
    status: Mutex<Option<ExitStatus>>,
    out: Arc<Mutex<Output>>,
}

type SpawnRequest = (Command, mpsc::Sender<std::io::Result<Child>>);

/// Children are spawned from one thread that lives as long as the server, so that on Linux they can be told to
/// die with it (`PR_SET_PDEATHSIG` fires when the thread that spawned the child ends, not the process).
fn spawner() -> &'static Mutex<mpsc::Sender<SpawnRequest>> {
    static SPAWNER: OnceLock<Mutex<mpsc::Sender<SpawnRequest>>> = OnceLock::new();
    SPAWNER.get_or_init(|| {
        let (tx, rx) = mpsc::channel::<SpawnRequest>();
        std::thread::Builder::new()
            .name("mcp-spawner".into())
            .spawn(move || {
                for (mut cmd, reply) in rx {
                    let _ = reply.send(cmd.spawn());
                }
            })
            .expect("spawning the spawner thread");
        Mutex::new(tx)
    })
}

/// Every child still to be killed when the server stops.
fn registry() -> &'static Mutex<Vec<Arc<Proc>>> {
    static PROCS: OnceLock<Mutex<Vec<Arc<Proc>>>> = OnceLock::new();
    PROCS.get_or_init(Mutex::default)
}

/// Kills every child (the server is stopping).
pub fn kill_all() {
    let procs = std::mem::take(&mut *registry().lock().unwrap());
    for p in procs {
        p.kill();
    }
}

fn read_lines(stream: impl Read + Send + 'static, out: Arc<Mutex<Output>>) {
    std::thread::spawn(move || {
        let mut r = BufReader::new(stream);
        let mut buf = Vec::new();
        loop {
            buf.clear();
            match r.read_until(b'\n', &mut buf) {
                Ok(0) | Err(_) => break,
                Ok(_) => {
                    let line = String::from_utf8_lossy(&buf);
                    out.lock().unwrap().push(line.trim_end_matches(['\n', '\r']).to_string());
                }
            }
        }
        out.lock().unwrap().closed += 1;
    });
}

impl Proc {
    /// Starts `exe args...` in `cwd` with its output collected.
    pub fn spawn(exe: &Path, args: &[String], cwd: Option<&Path>) -> Result<Arc<Proc>, String> {
        let mut cmd = Command::new(exe);
        cmd.args(args).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
        if let Some(d) = cwd {
            cmd.current_dir(d);
        }
        #[cfg(target_os = "linux")]
        {
            use std::os::unix::process::CommandExt;
            // SAFETY: prctl is async-signal-safe and touches no memory of the parent.
            unsafe {
                cmd.pre_exec(|| {
                    libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL);
                    Ok(())
                });
            }
        }
        let (tx, rx) = mpsc::channel();
        spawner().lock().unwrap().send((cmd, tx)).map_err(|_| "the spawner thread is gone")?;
        let mut child = rx
            .recv()
            .map_err(|_| "the spawner thread is gone")?
            .map_err(|e| format!("starting {}: {e}", exe.display()))?;
        let out = Arc::new(Mutex::new(Output::default()));
        read_lines(child.stdout.take().unwrap(), out.clone());
        read_lines(child.stderr.take().unwrap(), out.clone());
        let command = std::iter::once("firmware_simulator".to_string())
            .chain(args.iter().map(|a| if a.contains(' ') || a.is_empty() { format!("{a:?}") } else { a.clone() }))
            .collect::<Vec<_>>()
            .join(" ");
        let p = Arc::new(Proc {
            pid: child.id(),
            command,
            started: Instant::now(),
            child: Mutex::new(child),
            status: Mutex::new(None),
            out,
        });
        registry().lock().unwrap().push(p.clone());
        Ok(p)
    }

    /// How it ended, if it has.
    pub fn status(&self) -> Option<ExitStatus> {
        let mut st = self.status.lock().unwrap();
        if st.is_none() {
            *st = self.child.lock().unwrap().try_wait().ok().flatten();
            if st.is_some() {
                self.forget();
            }
        }
        *st
    }

    /// Waits up to `timeout` for it to end.
    pub fn wait_timeout(&self, timeout: Duration) -> Option<ExitStatus> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(s) = self.status() {
                return Some(s);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// Kills it (if it still runs) and reaps it.
    pub fn kill(&self) -> Option<ExitStatus> {
        if let Some(s) = self.status() {
            return Some(s);
        }
        let mut child = self.child.lock().unwrap();
        let _ = child.kill();
        let s = child.wait().ok();
        drop(child);
        *self.status.lock().unwrap() = s;
        self.forget();
        s
    }

    fn forget(&self) {
        registry().lock().unwrap().retain(|p| p.pid != self.pid);
    }

    /// Waits (briefly) for both output streams to end, once the process has.
    pub fn drain(&self) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while self.out.lock().unwrap().closed < 2 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    pub fn dashboard(&self) -> Option<SocketAddr> {
        self.out.lock().unwrap().dashboard
    }

    /// The console lines kept, and how many there were in all.
    pub fn lines(&self) -> (Vec<String>, u64) {
        let o = self.out.lock().unwrap();
        (o.lines.iter().cloned().collect(), o.total)
    }
}

/// An exit status for people and for JSON: `exit 0`, `killed by signal 9`.
pub fn describe(s: Option<ExitStatus>) -> String {
    match s {
        None => "running".into(),
        Some(s) => match s.code() {
            Some(c) => format!("exit {c}"),
            None => {
                use std::os::unix::process::ExitStatusExt;
                format!("killed by signal {}", s.signal().unwrap_or(0))
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collects_output_and_kills() {
        let sh = Path::new("/bin/sh");
        let p = Proc::spawn(
            sh,
            &["-c".into(), "echo one; echo two >&2; echo 'dashboard on http://127.0.0.1:4321/'".into()],
            None,
        )
        .unwrap();
        assert!(p.wait_timeout(Duration::from_secs(10)).unwrap().success());
        p.drain();
        let (lines, total) = p.lines();
        assert_eq!(total, 3);
        assert!(lines.contains(&"one".to_string()) && lines.contains(&"two".to_string()));
        assert_eq!(p.dashboard(), Some("127.0.0.1:4321".parse().unwrap()));
        assert_eq!(describe(p.status()), "exit 0");

        let p = Proc::spawn(sh, &["-c".into(), "sleep 30".into()], None).unwrap();
        assert!(p.status().is_none());
        assert_eq!(describe(p.kill()), "killed by signal 9");
        assert!(!registry().lock().unwrap().iter().any(|q| q.pid == p.pid));
    }

    #[test]
    fn keeps_the_last_lines() {
        let mut o = Output::default();
        for i in 0..KEPT_LINES + 5 {
            o.push(format!("{i}"));
        }
        assert_eq!(o.total as usize, KEPT_LINES + 5);
        assert_eq!(o.lines.front().unwrap(), "5");
    }
}
