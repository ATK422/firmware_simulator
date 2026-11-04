//! Robot boards with their USB port plugged into a host (`usb_cdc`, `usb_cdc2`, `robot --usb-cdc BOARD=SINK`), and
//! robot files pointing at images anywhere: UF2 files, absolute paths, `~` and environment variables.

mod common;

use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use sim_core::time::Time;
use sim_host::robot::Robot;
use sim_host::usb::CdcSink;

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("robot-usb-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn q(p: &std::path::Path) -> String {
    format!("{:?}", p.to_str().unwrap())
}

/// A robot of `c_dual_usb` (titan_dual_stdio_usb: text on CDC port 0, upper-case echo on port 1), switched on 5 ms
/// late, next to a blinking power board, with `usb` the extra keys of the USB board.
fn dual_usb_robot(name: &str, usb: &str) -> Robot {
    let path = scratch(name);
    let repo = common::repo();
    std::fs::write(
        &path,
        format!(
            "[[board]]\nname = \"blinky\"\nboard = {pb}\nelf = {blink}\n\n\
             [[board]]\nname = \"c_ivc\"\nboard = {ivc}\nelf = {dual}\npower_on_ms = 5\n{usb}\n",
            pb = q(&repo.join("boards/mk2/power_board.toml")),
            blink = q(&repo.join("test_programs/blink.elf")),
            ivc = q(&repo.join("boards/talos/ivc_board.toml")),
            dual = q(&repo.join("test_programs/c_dual_usb/c_dual_usb.elf")),
        ),
    )
    .unwrap();
    Robot::load(&path).unwrap()
}

/// What a run of the dual-USB robot shows: port 0's lines with their times, and when each upper-cased byte came
/// back on port 1 (sent at 300 ms through the host).
/// Port 0's lines, the echoes on port 1 and port 0's text.
type Exchange = (Vec<(Time, String)>, Vec<(Time, Vec<u8>)>, String);

fn exchange(threads: usize) -> Exchange {
    let mut r = dual_usb_robot(&format!("dual{threads}.toml"), "usb_cdc = \"stdout\"");
    r.set_usb_cdc(1, 1, CdcSink::Pty(None)).unwrap();
    assert_eq!(
        r.usb[1],
        sim_host::robot::UsbPorts { plugged: true, cdc: Some(CdcSink::Stdout), cdc2: Some(CdcSink::Pty(None)) }
    );
    assert!(!r.usb[0].plugged && r.boards[0].machine.sys.usb_host.is_none(), "only the board asked for");
    r.threads = threads;
    if threads > 1 {
        // Each board on its own thread, whatever they cost
        r.fix_threads(vec![0, 1]);
    }
    let lines = Arc::new(Mutex::new(Vec::new()));
    let l = lines.clone();
    r.boards[1].on_usb_cdc_line = Some(Box::new(move |t, line| l.lock().unwrap().push((t, line.to_string()))));
    r.run_until(Time::from_ms(300));
    let s = &mut r.boards[1];
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(1, b"ping"));
    let mut echoed = Vec::new();
    while r.now < Time::from_ms(400) {
        let t = r.now + Time::from_ms(1);
        r.run_until(t);
        let rx = r.boards[1].machine.sys.usb_host_mut().unwrap().cdc_take_rx(1);
        if !rx.is_empty() {
            echoed.push((r.now, rx));
        }
    }
    if threads > 1 {
        assert_eq!(r.thread_of_boards(), [0, 1], "the boards ran on two threads");
    }
    let lines = lines.lock().unwrap().clone();
    (lines, echoed, r.boards[1].usb_cdc_text.clone())
}

#[test]
fn robot_board_usb_cdc_ports() {
    let (lines, echoed, text) = exchange(1);
    // Port 0: the text log, as lines (the CLI prints them as `[t] c_ivc usb: ...`)
    assert_eq!(lines.iter().map(|l| l.1.as_str()).collect::<Vec<_>>(), ["titan dual stdio up"], "{text:?}");
    assert!(lines[0].0 > Time::from_ms(5), "enumerated after its late power-on");
    // Port 1: the bytes sent in come back upper case, and stay off port 0
    let all: Vec<u8> = echoed.iter().flat_map(|e| e.1.clone()).collect();
    assert_eq!(all, b"PING");
    assert!(!text.contains("PING"));
    // The same run, to the byte and the time, with the boards on threads
    assert_eq!(exchange(2), (lines, echoed, text));
}

#[test]
fn robot_board_usb_port_on_a_pty() {
    let link = scratch("c_ivc-ros");
    // c_dual_usb waits for the host to open port 0 (DTR) before it echoes on port 1
    let keys = format!("usb_cdc = \"stdout\"\nusb_cdc2 = \"pty:{}\"", link.display());
    let mut r = dual_usb_robot("pty.toml", &keys);
    assert_eq!(r.usb[1].cdc2, Some(CdcSink::Pty(Some(link.clone()))));
    let opened = r.open_usb_ptys().unwrap();
    assert_eq!(opened.len(), 1);
    assert!(opened[0].starts_with("c_ivc: USB CDC port 1 on /dev/") && opened[0].contains("linked from"), "{opened:?}");
    // Opening again opens nothing new
    assert!(r.open_usb_ptys().unwrap().is_empty());
    r.run_until(Time::from_ms(300));
    // A snapshot keeps the USB host and its ports, but not the terminal; it can have its own
    let mut copy = r.snapshot();
    assert!(copy.boards[1].usb_cdc2_pty.is_none() && copy.boards[1].machine.sys.usb_host.is_some());
    assert_eq!(copy.usb, r.usb);

    let mut term = std::fs::OpenOptions::new().read(true).write(true).open(&link).expect("the linked terminal");
    term.write_all(b"ros").unwrap();
    term.flush().unwrap();
    set_nonblocking(&term);
    let mut got = Vec::new();
    for _ in 0..200 {
        let t = r.now + Time::from_ms(1);
        r.run_until(t);
        let mut buf = [0u8; 64];
        if let Ok(n) = term.read(&mut buf) {
            got.extend_from_slice(&buf[..n]);
        }
        if got.len() >= 3 {
            break;
        }
    }
    assert_eq!(got, b"ROS");

    // The copy's own terminal replaces the link (the original's terminal stays open)
    let opened = copy.open_usb_ptys().unwrap();
    assert_eq!(opened.len(), 1);
    assert!(std::fs::read_link(&link).unwrap().to_str().unwrap() == copy.boards[1].usb_cdc2_pty.as_ref().unwrap().path);
}

#[test]
fn usb_cdc_options_are_checked() {
    let mut r = dual_usb_robot("bad.toml", "");
    assert!(r.set_usb_cdc(1, 1, CdcSink::Stdout).unwrap_err().contains("raw bytes"));
    assert!(r.set_usb_cdc(1, 2, CdcSink::Stdout).unwrap_err().contains("no CDC port 2"));
    assert!(!r.usb[1].plugged);
    let path = scratch("badkey.toml");
    std::fs::write(
        &path,
        format!(
            "[[board]]\nname = \"a\"\nboard = {}\nelf = {}\nusb_cdc = \"file\"\n",
            q(&common::repo().join("boards/talos/ivc_board.toml")),
            q(&common::repo().join("test_programs/blink.elf"))
        ),
    )
    .unwrap();
    let err = Robot::load(&path).err().unwrap();
    assert!(err.contains("expected stdout, pty or pty:PATH"), "{err}");
}

/// A C build's `_with_bl.uf2` (bootloader and application), named by an absolute path built from an environment
/// variable, boots in a robot: through the bootloader into the application, direct and through the bootrom.
#[test]
fn robot_file_takes_a_uf2_from_an_environment_variable_path() {
    if common::built("bl_app_v0/bl_app_with_bl.uf2").is_none() {
        return;
    }
    for rom_boot in [false, true] {
        let path = scratch(&format!("uf2-{rom_boot}.toml"));
        std::fs::write(
            &path,
            format!(
                "[[board]]\nname = \"led\"\nboard = \"${{CARGO_MANIFEST_DIR}}/../../boards/talos/led_board.toml\"\n\
                 elf = \"${{CARGO_MANIFEST_DIR}}/../../build/c/bl_app_v0/bl_app_with_bl.uf2\"\nrom_boot = {rom_boot}\n"
            ),
        )
        .unwrap();
        let mut r = Robot::load(&path).unwrap();
        assert_eq!(r.images[0], "bl_app_with_bl.uf2");
        r.run_until(Time::from_ms(1000));
        let log = &r.boards[0].uart_text[0];
        assert!(log.contains("titan_bootloader") && log.contains("Bootloader test app 1.0"), "{rom_boot}: {log}");
    }
    // An unset variable says which
    let path = scratch("unset.toml");
    std::fs::write(
        &path,
        format!(
            "[[board]]\nname = \"x\"\nboard = {}\nelf = \"${{TITAN_UNSET_FOR_TEST}}/IVCBoard2.uf2\"\n",
            q(&common::repo().join("boards/talos/led_board.toml"))
        ),
    )
    .unwrap();
    let err = Robot::load(&path).err().unwrap();
    assert!(err.contains("TITAN_UNSET_FOR_TEST is not set"), "{err}");
}

fn set_nonblocking(f: &std::fs::File) {
    use std::os::fd::AsRawFd;
    // SAFETY: fcntl on a file descriptor we own
    unsafe {
        let flags = libc::fcntl(f.as_raw_fd(), libc::F_GETFL);
        libc::fcntl(f.as_raw_fd(), libc::F_SETFL, flags | libc::O_NONBLOCK);
    }
}

/// What a board with a terminal on its second CDC port did when sent a line through the terminal every 20 ms (put
/// there between runs, so every way of running reads it at the same boundary): what came back on the terminal,
/// port 0's lines, and the board's state.
fn pty_exchange(threads: usize, stress: Option<u64>) -> (Vec<u8>, Vec<(Time, String)>, String, u64) {
    use sim_core::cpu::Cpu;
    let link = scratch(&format!("c_ivc-ros-{threads}-{stress:?}"));
    let keys = format!("usb_cdc = \"stdout\"\nusb_cdc2 = \"pty:{}\"", link.display());
    let mut r = dual_usb_robot(&format!("pty-{threads}-{stress:?}.toml"), &keys);
    r.threads = threads;
    r.run_ahead_stress = stress;
    if threads > 1 {
        r.fix_threads(vec![0, 1]);
    }
    r.open_usb_ptys().unwrap();
    let lines = Arc::new(Mutex::new(Vec::new()));
    let l = lines.clone();
    r.boards[1].on_usb_cdc_line = Some(Box::new(move |t, line| l.lock().unwrap().push((t, line.to_string()))));
    r.run_until(Time::from_ms(300));
    let mut term = std::fs::OpenOptions::new().read(true).write(true).open(&link).unwrap();
    set_nonblocking(&term);
    let mut got = Vec::new();
    let mut read = |term: &mut std::fs::File| {
        let mut buf = [0u8; 256];
        while let Ok(n) = term.read(&mut buf) {
            if n == 0 {
                break;
            }
            got.extend_from_slice(&buf[..n]);
        }
    };
    for k in 0..30u64 {
        if k % 10 == 0 {
            term.write_all(format!("ros{k};").as_bytes()).unwrap();
            term.flush().unwrap();
            // The terminal passes it on to the emulator's side
            std::thread::sleep(std::time::Duration::from_millis(30));
        }
        let t = r.now + Time::from_ms(2);
        assert!(r.run_until(t).is_none());
        read(&mut term);
    }
    std::thread::sleep(std::time::Duration::from_millis(30));
    read(&mut term);
    if threads > 1 {
        assert!(r.run_ahead_stats.windows > 0, "{:?}", r.run_ahead_stats);
    }
    let m = &r.boards[1].machine;
    let state = format!("{} {} {:?}", m.now, m.cores[0].instructions(), m.cores[0].cycles());
    let lines = lines.lock().unwrap().clone();
    let mut h = std::collections::hash_map::DefaultHasher::new();
    std::hash::Hash::hash(&m.sys.sram, &mut h);
    (got, lines, state, std::hash::Hasher::finish(&h))
}

/// A board fed from a terminal runs ahead of the others: what it reads is taken in at the boundary it was read at
/// (and given back for the rerun if the board is rolled back), what it writes reaches the terminal once final. The
/// run is the one taking turns gives.
#[test]
fn a_board_fed_from_a_terminal_runs_ahead_as_in_turns() {
    let want = pty_exchange(1, None);
    assert_eq!(want.0, b"ROS0;ROS10;ROS20;", "{:?}", String::from_utf8_lossy(&want.0));
    assert_eq!(pty_exchange(2, None), want);
    assert_eq!(pty_exchange(2, Some(5)), want);
}
