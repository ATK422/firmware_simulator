//! A pseudo terminal for a simulated UART: firmware output appears on the pty, anything written to it reaches the
//! UART's RX. Connect with `screen`, `picocom` or `minicom` on the printed path.

use std::ffi::CStr;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

pub struct Pty {
    master: OwnedFd,
    /// Path of the slave side, e.g. `/dev/pts/5`.
    pub path: String,
}

impl Pty {
    /// Opens a pty pair in raw mode with a non-blocking master.
    pub fn open() -> io::Result<Pty> {
        // SAFETY: plain libc calls on a file descriptor we own; error returns are checked
        unsafe {
            let fd = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            let master = OwnedFd::from_raw_fd(fd);
            if libc::grantpt(fd) != 0 || libc::unlockpt(fd) != 0 {
                return Err(io::Error::last_os_error());
            }
            let path = slave_path(fd)?;
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) == 0 {
                libc::cfmakeraw(&mut tio);
                libc::tcsetattr(fd, libc::TCSANOW, &tio);
            }
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
            Ok(Pty { master, path })
        }
    }

    /// Writes UART output to the terminal. Bytes are dropped while nobody has the pty open.
    pub fn write(&self, data: &[u8]) {
        // SAFETY: writing from a valid buffer to our own fd
        unsafe {
            libc::write(self.master.as_raw_fd(), data.as_ptr().cast(), data.len());
        }
    }

    /// Reads whatever the user typed, without blocking.
    pub fn read(&self) -> Vec<u8> {
        let mut buf = [0u8; 256];
        // SAFETY: reading into a valid buffer from our own non-blocking fd
        let n = unsafe { libc::read(self.master.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len()) };
        if n > 0 { buf[..n as usize].to_vec() } else { Vec::new() }
    }
}

/// The slave side's path, with the reentrant `ptsname_r` (Linux).
#[cfg(target_os = "linux")]
unsafe fn slave_path(fd: libc::c_int) -> io::Result<String> {
    let mut buf = [0 as libc::c_char; 128];
    // SAFETY: the buffer is valid for its length, and ptsname_r NUL-terminates it on success
    if unsafe { libc::ptsname_r(fd, buf.as_mut_ptr(), buf.len()) } != 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: NUL-terminated by ptsname_r
    Ok(unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned())
}

/// The slave side's path, with `ptsname` (macOS and other Unixes have no `ptsname_r`), which returns a static buffer:
/// calls are serialised and the path is copied out before the lock is released.
#[cfg(not(target_os = "linux"))]
unsafe fn slave_path(fd: libc::c_int) -> io::Result<String> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // SAFETY: ptsname returns NULL or a NUL-terminated static buffer, read while the lock is held
    let p = unsafe { libc::ptsname(fd) };
    if p.is_null() {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: non-null, NUL-terminated
    Ok(unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};

    #[test]
    fn round_trip_through_the_slave() {
        let pty = match Pty::open() {
            Ok(p) => p,
            Err(e) => {
                eprintln!("skipping: no pty support here ({e})");
                return;
            }
        };
        let mut slave = std::fs::OpenOptions::new().read(true).write(true).open(&pty.path).unwrap();
        pty.write(b"hello\n");
        let mut buf = [0u8; 6];
        slave.read_exact(&mut buf).unwrap();
        assert_eq!(&buf, b"hello\n");
        slave.write_all(b"ok").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(20));
        assert_eq!(pty.read(), b"ok");
    }
}
