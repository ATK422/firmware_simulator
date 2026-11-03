//! Output files other programs read while a run goes on (`--log-json`, `--trace-file`, `--candump-log`). They are
//! written through a buffer, so a busy run makes few system calls, and a background thread flushes them every
//! [`FLUSH_PERIOD`] of wall time, so whatever was written reaches the file within that time even when the run
//! writes nothing more for a while (a tool waiting for one line, `tail -f`). Standard output needs none of this:
//! Rust's is line buffered whatever it is connected to.

use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The longest a written line waits in a buffer before it reaches its file.
pub const FLUSH_PERIOD: Duration = Duration::from_millis(100);

/// Calls `flush(&shared)` every `period` from a background thread, until every other handle to `shared` is gone
/// (then the thread ends).
pub fn flush_periodically<T: Send + Sync + 'static>(shared: &Arc<T>, period: Duration, flush: fn(&T)) {
    let weak = Arc::downgrade(shared);
    let _ = std::thread::Builder::new().name("output-flush".into()).spawn(move || {
        loop {
            std::thread::sleep(period);
            let Some(shared) = weak.upgrade() else { return };
            flush(&shared);
        }
    });
}

/// Flushes a shared writer, unless a panic while it was held left it poisoned.
pub fn flush_mutex<W: Write>(m: &Mutex<W>) {
    if let Ok(mut w) = m.lock() {
        let _ = w.flush();
    }
}

/// A writer several boards (threads) share, each `write_all` under one lock, so lines stay whole: a buffered file
/// kept flushed (see the module docs), or stderr.
#[derive(Clone)]
pub struct SharedWriter(Arc<Mutex<Box<dyn Write + Send>>>);

impl SharedWriter {
    /// `out`, buffered by the caller or not, flushed every [`FLUSH_PERIOD`].
    pub fn new(out: Box<dyn Write + Send>) -> SharedWriter {
        let w = Arc::new(Mutex::new(out));
        flush_periodically(&w, FLUSH_PERIOD, flush_mutex);
        SharedWriter(w)
    }

    /// A new file at `path`.
    pub fn create(path: &std::path::Path) -> std::io::Result<SharedWriter> {
        Ok(SharedWriter::new(Box::new(std::io::BufWriter::new(std::fs::File::create(path)?))))
    }

    /// Standard error (unbuffered: nothing to flush).
    pub fn stderr() -> SharedWriter {
        SharedWriter(Arc::new(Mutex::new(Box::new(std::io::stderr()))))
    }

    /// A file at `path`, or stderr without one.
    pub fn file_or_stderr(path: Option<&std::path::Path>) -> std::io::Result<SharedWriter> {
        match path {
            Some(p) => SharedWriter::create(p),
            None => Ok(SharedWriter::stderr()),
        }
    }
}

impl Write for SharedWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().write(buf)
    }
    /// Under one lock: each trace line is one `write_all`, and stays whole with boards on several threads.
    fn write_all(&mut self, buf: &[u8]) -> std::io::Result<()> {
        self.0.lock().unwrap().write_all(buf)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.0.lock().unwrap().flush()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Polls `path` for up to two seconds until its contents satisfy `done`; the contents.
    pub(crate) fn wait_for_file(path: &std::path::Path, done: impl Fn(&str) -> bool) -> String {
        let t0 = std::time::Instant::now();
        loop {
            let text = std::fs::read_to_string(path).unwrap_or_default();
            if done(&text) || t0.elapsed() > Duration::from_secs(2) {
                return text;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    pub(crate) fn temp_path(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("fwsim-flush-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    #[test]
    fn a_line_reaches_the_file_without_a_flush_or_more_output() {
        let path = temp_path("shared.txt");
        let mut w = SharedWriter::create(&path).unwrap();
        w.write_all(b"one line\n").unwrap();
        // The writer is still open and nothing else is written: the background flush has to deliver it
        let text = wait_for_file(&path, |t| t.contains("one line"));
        assert_eq!(text, "one line\n");
        drop(w);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

    #[test]
    fn the_flush_thread_ends_with_the_writer() {
        let shared = Arc::new(Mutex::new(Vec::<u8>::new()));
        flush_periodically(&shared, Duration::from_millis(1), flush_mutex);
        let weak = Arc::downgrade(&shared);
        drop(shared);
        // The thread holds a handle only while it flushes; once it has let go, nothing keeps the writer alive
        std::thread::sleep(Duration::from_millis(20));
        assert!(weak.upgrade().is_none());
    }
}
