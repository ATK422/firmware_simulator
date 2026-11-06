//! Ending a run cleanly on SIGINT (Ctrl-C) or SIGTERM (`kill`, `timeout`, a test harness): the first signal sets
//! [`STOP`], which a [`crate::Session`] or [`crate::robot::Robot`] given it as its `stop_flag` checks at every slice
//! boundary, so the command line can finish the run as if it had reached its time (save the flash file, write the
//! log, waveform and profile trailers, print the stop line). A second signal ends the process at once, as does a
//! signal once [`exit_on_signal`] was called (nothing left to finish, e.g. while the dashboard is held).

use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

/// Set by the first SIGINT or SIGTERM after [`install`].
pub static STOP: AtomicBool = AtomicBool::new(false);
/// The signal that set [`STOP`].
static SIGNAL: AtomicI32 = AtomicI32::new(0);
/// The next signal ends the process.
static EXIT_NOW: AtomicBool = AtomicBool::new(false);

extern "C" fn handler(sig: libc::c_int) {
    // Only async-signal-safe calls in here: atomics, write(2) and _exit(2)
    if STOP.swap(true, Ordering::SeqCst) || EXIT_NOW.load(Ordering::SeqCst) {
        unsafe { libc::_exit(128 + sig) };
    }
    SIGNAL.store(sig, Ordering::SeqCst);
    let msg = b"\ninterrupted: ending the run cleanly (signal again to quit at once)\n";
    unsafe { libc::write(2, msg.as_ptr().cast(), msg.len()) };
}

/// Catches SIGINT and SIGTERM from now on (see the module docs).
pub fn install() -> std::io::Result<()> {
    for sig in [libc::SIGINT, libc::SIGTERM] {
        // SAFETY: a zeroed sigaction is valid (no flags, empty mask once sigemptyset has run), and the handler only
        // makes async-signal-safe calls
        let r = unsafe {
            let mut sa: libc::sigaction = std::mem::zeroed();
            sa.sa_sigaction = handler as extern "C" fn(libc::c_int) as libc::sighandler_t;
            libc::sigemptyset(&mut sa.sa_mask);
            sa.sa_flags = libc::SA_RESTART;
            libc::sigaction(sig, &sa, std::ptr::null_mut())
        };
        if r != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    // A process inherits its signal mask from whatever started it. If that blocked SIGINT (some shells, wrappers
    // and dev environments do), a handler never runs and Ctrl-C does nothing at all, the first time or the fifth.
    // So unblock them here, on the main thread, before any thread starts (threads inherit the mask).
    // SAFETY: plain sigset operations on local values
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        let mut old: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGINT);
        libc::sigaddset(&mut set, libc::SIGTERM);
        if libc::pthread_sigmask(libc::SIG_UNBLOCK, &set, &mut old) == 0
            && (libc::sigismember(&old, libc::SIGINT) == 1 || libc::sigismember(&old, libc::SIGTERM) == 1)
        {
            log::warn!(
                "SIGINT/SIGTERM were blocked by whatever started the emulator (so Ctrl-C did nothing); unblocked them"
            );
        }
    }
    Ok(())
}

/// Whether a signal asked the run to stop.
pub fn requested() -> bool {
    STOP.load(Ordering::SeqCst)
}

/// The signal that asked the run to stop, if one did.
pub fn received() -> Option<i32> {
    requested().then(|| SIGNAL.load(Ordering::SeqCst))
}

/// From now on a signal ends the process at once: there is nothing left to finish.
pub fn exit_on_signal() {
    EXIT_NOW.store(true, Ordering::SeqCst);
}

/// Whether `flag` (a session's or robot's `stop_flag`) is set.
pub(crate) fn is_set(flag: Option<&AtomicBool>) -> bool {
    flag.is_some_and(|f| f.load(Ordering::Relaxed))
}
