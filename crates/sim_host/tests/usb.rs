//! USB: the real bootrom's BOOTSEL mode on the host model. `picotool load -x` over PICOBOOT and a UF2 copy to the
//! mass storage drive both flash an image into the (blank) QSPI flash model through the bootrom's own flash code,
//! and the reboot runs it from flash through boot2.

mod common;

use rp2040::{Machine, MachineConfig};
use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::usb::host::HostController;
use sim_host::Session;
use sim_host::usb::Picoboot;

/// A board with a blank flash, powered on with BOOTSEL held and its USB port plugged into a host.
fn bootsel_board() -> Machine {
    let mut m = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..Default::default() });
    m.sys.unmodelled.log_mode = rp2040::unmodelled::LogMode::Off;
    m.sys.io_qspi.bootsel_pressed = true;
    m.rom_boot().unwrap();
    m.sys.attach_usb_host(HostController::new(), Time::ZERO);
    m
}

#[test]
fn picoboot_load_flashes_and_boots_embassy_blinky() {
    let image = Image::load(&common::repo().join("test_programs/embassy_blinky.elf")).unwrap();
    let mut m = bootsel_board();
    let mut pb = Picoboot::open(&mut m, Time::from_secs(1)).expect("BOOTSEL device with PICOBOOT");
    let report = pb.load(&mut m, &image, 2 << 20, Some(100)).expect("load");
    eprintln!("loaded {:?} in {}", report.ranges, report.took);
    assert!(report.bytes > 4096);
    // Let go of BOOTSEL (the reboot would otherwise come back to USB boot)
    m.sys.io_qspi.bootsel_pressed = false;
    let mut s = Session::new(m, &image);
    s.run_for(Time::from_ms(1500));
    assert!(s.logged("blinky up, clk_sys 125000000 Hz"), "{:?}", s.logs);
    assert!(s.logged("toggle 4"), "{:?}", s.logs);
}

/// Runs until the session's CDC text has `text` after position `from` (or `timeout` passes).
fn cdc_wait(s: &mut Session, from: usize, text: &str, timeout: Time) -> bool {
    let end = s.machine.now + timeout;
    while s.machine.now < end {
        s.run_for(Time::from_ms(1));
        if s.usb_cdc_text[from..].contains(text) {
            return true;
        }
    }
    false
}

#[test]
fn tinyusb_cdc_stdio_prints_echoes_and_reboots_to_bootsel() {
    let mut s = common::boot(&common::repo().join("test_programs/pico/hello_usb.elf"), None);
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    let ok = cdc_wait(&mut s, 0, "Hello, USB!\r\n", Time::from_secs(2));
    assert!(ok, "{:?}", s.usb_cdc_text);
    let dev = s.machine.sys.usb_host.as_ref().unwrap().configured().unwrap().clone();
    assert_eq!((dev.vendor_id(), dev.product_id()), (0x2E8A, 0x000A), "pico-sdk stdio_usb IDs");
    assert_eq!(dev.product.as_deref(), Some("Pico"));
    // Input: each character comes back on a line of its own
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(0, b"xy"));
    let from = s.usb_cdc_text.len();
    assert!(cdc_wait(&mut s, from, "echo y (2)\r\n", Time::from_secs(1)), "{:?}", s.usb_cdc_text);
    assert!(s.usb_cdc_text[from..].contains("echo x (1)\r\n"), "{:?}", s.usb_cdc_text);
    // 'b' calls reset_usb_boot: the device leaves, and comes back as the bootrom's RP2 Boot
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(0, b"b"));
    let pb = Picoboot::open(&mut s, Time::from_secs(2)).expect("back in BOOTSEL");
    assert!(s.machine.usb_boot_requested);
    assert!(s.machine.boots_from_rom());
    let _ = pb;
}

#[test]
fn uf2_copied_to_the_bootsel_drive_flashes_and_boots() {
    use sim_host::usb::MscDrive;
    let image = Image::load(&common::repo().join("test_programs/embassy_blinky.elf")).unwrap();
    let mut m = bootsel_board();
    let mut drive = MscDrive::open(&mut m, Time::from_secs(1)).expect("BOOTSEL drive");
    // The button only matters at boot
    m.sys.io_qspi.bootsel_pressed = false;
    assert_eq!(drive.block_size, 512);
    assert!(drive.blocks * 512 >= 64 << 20, "a 128 MB FAT16 volume: {} blocks", drive.blocks);
    // An MBR with one FAT partition starting at sector 1, whose boot sector names the volume
    let mbr = drive.read_blocks(&mut m, 0, 1).unwrap();
    assert_eq!(&mbr[510..512], &[0x55, 0xAA]);
    assert_eq!(u32::from_le_bytes(mbr[0x1C6..0x1CA].try_into().unwrap()), 1, "{:02x?}", &mbr[0x1BE..0x1CE]);
    let boot = drive.read_blocks(&mut m, 1, 1).unwrap();
    assert_eq!(&boot[43..54], b"RPI-RP2    ", "volume label");
    assert_eq!(&boot[54..62], b"FAT16   ");
    let uf2 = image.to_uf2();
    drive.copy_uf2(&mut m, &uf2, 0x1000).expect("copy");
    // The bootrom reboots 1 s after the last block into the new image
    let mut s = Session::new(m, &image);
    s.run_for(Time::from_ms(2500));
    assert!(s.logged("blinky up"), "{:?}", s.logs);
    assert!(s.machine.resets >= 1);
}

#[test]
fn titan_dual_stdio_usb_two_cdc_ports() {
    let mut s = common::boot(&common::repo().join("test_programs/c_dual_usb/c_dual_usb.elf"), None);
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    host.cdc_open(1, 115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    assert!(cdc_wait(&mut s, 0, "titan dual stdio up\r\n", Time::from_secs(2)), "{:?}", s.usb_cdc_text);
    let dev = s.machine.sys.usb_host.as_ref().unwrap().configured().unwrap().clone();
    let cdc_ifs = dev.interfaces.iter().filter(|i| i.class == 2).count();
    assert_eq!(cdc_ifs, 2, "{:?}", dev.interfaces);
    // The secondary port (the micro-ROS transport's) echoes in upper case
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(1, b"ping"));
    let mut got = Vec::new();
    for _ in 0..200 {
        s.run_for(Time::from_ms(1));
        got.extend(s.machine.sys.usb_host_mut().unwrap().cdc_take_rx(1));
        if got.len() >= 4 {
            break;
        }
    }
    assert_eq!(got, b"PING");
}

#[test]
fn picotool_style_forced_reload_through_the_reset_interface() {
    // A pico-sdk stdio_usb firmware is running; `picotool load -f` reboots it into BOOTSEL through its reset
    // interface, loads the new image and runs it
    let mut s = common::boot(&common::repo().join("test_programs/pico/hello_usb.elf"), None);
    s.machine.sys.attach_usb_host(HostController::new(), Time::ZERO);
    sim_host::usb::reboot_via_reset_interface(&mut s, true, Time::from_secs(1)).expect("reset interface");
    let mut pb = Picoboot::open(&mut s, Time::from_secs(2)).expect("BOOTSEL after the reset request");
    assert!(s.machine.usb_boot_requested);
    let image = Image::load(&common::repo().join("test_programs/embassy_blinky.elf")).unwrap();
    pb.load(&mut s, &image, 2 << 20, Some(10)).expect("load");
    let mut s2 = Session::new(s.machine.fork(), &image);
    s2.run_for(Time::from_ms(300));
    assert!(s2.logged("blinky up"), "{:?}", s2.logs);
}

#[test]
fn magic_1200_baud_reboots_into_bootsel() {
    let mut s = common::boot(&common::repo().join("test_programs/pico/hello_usb.elf"), None);
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    assert!(cdc_wait(&mut s, 0, "Hello, USB!", Time::from_secs(2)));
    // What the Arduino IDE and `picotool` fall-backs do: set 1200 baud
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_set_baud(0, 1200));
    Picoboot::open(&mut s, Time::from_secs(2)).expect("BOOTSEL after 1200 baud");
    assert!(s.machine.usb_boot_requested);
}

#[test]
fn picoboot_rejects_bad_commands_and_recovers() {
    use sim_host::usb::Error;
    let mut m = bootsel_board();
    let mut pb = Picoboot::open(&mut m, Time::from_secs(1)).unwrap();
    pb.exclusive_access(&mut m, 1).unwrap();
    pb.exit_xip(&mut m).unwrap();
    // Flash writes must be page aligned: the bootrom halts the data endpoint and reports BAD_ALIGNMENT
    assert_eq!(pb.write(&mut m, 0x1000_0010, &[0; 16]), Err(Error::Picoboot { cmd: 0x05, status: 5 }));
    // Commands without a data phase are acknowledged whatever their task returns (the bootrom's `_atc_ack` does
    // not look at the result): a misaligned erase "succeeds" and erases nothing
    pb.write(&mut m, 0x1000_0000, &[0x11; 256]).unwrap();
    pb.flash_erase(&mut m, 0x1000_0100, 4096).unwrap();
    assert_eq!(m.sys.flash[0], 0x11, "nothing erased");
    assert_eq!(pb.read(&mut m, 0x3000_0000, 64), Err(Error::Picoboot { cmd: 0x84, status: 4 }), "bad address");
    // After the interface reset everything works again: RAM write/read, ROM read, flash program
    pb.write(&mut m, 0x2000_1000, b"PICOBOOT RAM").unwrap();
    assert_eq!(&m.sys.sram[0x1000..0x100C], b"PICOBOOT RAM");
    assert_eq!(pb.read(&mut m, 0x2000_1000, 12).unwrap(), b"PICOBOOT RAM");
    assert_eq!(pb.read(&mut m, 0x0000_0010, 4).unwrap(), m.sys.rom[0x10..0x14], "bootrom magic and version");
    pb.flash_erase(&mut m, 0x1001_0000, 4096).unwrap();
    pb.write(&mut m, 0x1001_0000, &[0xA5; 256]).unwrap();
    assert!(m.sys.flash[0x1_0000..0x1_0100].iter().all(|&b| b == 0xA5));
    assert_eq!(
        pb.read(&mut m, 0x1001_00FC, 8),
        Err(Error::Picoboot { cmd: 0x84, status: 5 }),
        "flash reads page aligned"
    );
    assert_eq!(pb.read(&mut m, 0x1001_0000, 4).unwrap(), [0xA5; 4]);
    let st = pb.status(&mut m).unwrap();
    assert_eq!((st.status, st.cmd, st.in_progress), (0, 0x84, false));
}

#[test]
fn picoboot_load_titan_template_and_its_watchdog_reboots_go_through_the_bootrom() {
    let Some(elf) = common::titan_elf("template") else { return };
    let image = Image::load(&elf).unwrap();
    let mut m = bootsel_board();
    let mut pb = Picoboot::open(&mut m, Time::from_secs(1)).unwrap();
    m.sys.io_qspi.bootsel_pressed = false;
    let report = pb.load(&mut m, &image, 2 << 20, Some(10)).expect("load");
    assert!(report.bytes > 32 * 1024, "{report:?}");
    let resets = m.resets;
    let mut s = Session::new(m, &image);
    s.run_for(Time::from_ms(1500));
    // The template panics without its CAN controller and resets through the watchdog: the bootrom boots it from
    // flash again (through boot2), and it reports the panic
    assert!(s.logged("mk2/power_board v1.0"), "{:?}", s.logs);
    assert!(s.logged("Reset by panic after"), "{:?}", s.logs);
    assert!(s.machine.resets >= resets + 2);
}

#[test]
fn picoboot_loads_a_no_flash_binary_into_sram_and_runs_it() {
    let image = Image::load(&common::repo().join("test_programs/pico/hello_usb_ram.elf")).unwrap();
    let mut m = bootsel_board();
    let mut pb = Picoboot::open(&mut m, Time::from_secs(1)).unwrap();
    let report = pb.load(&mut m, &image, 2 << 20, Some(10)).expect("load into SRAM");
    assert_eq!(report.ranges[0].0, 0x2000_0000);
    assert!(m.sys.flash.iter().all(|&b| b == 0xFF), "flash untouched");
    // The bootrom's reboot enters the image through the watchdog scratch vector; BOOTSEL stays held throughout
    m.sys.usb_host_mut().unwrap().cdc_open(0, 115_200);
    let mut s = Session::new(m, &image);
    assert!(cdc_wait(&mut s, 0, "Hello, USB!\r\n", Time::from_secs(2)), "{:?}", s.usb_cdc_text);
    assert!(s.machine.sys.io_qspi.bootsel_pressed);
}

/// A minimal USB/IP client (what vhci-hcd sends), driving the session while it waits for answers.
struct UsbIpClient {
    sock: std::net::TcpStream,
    seq: u32,
    rx: Vec<u8>,
}

impl UsbIpClient {
    fn read_exact(&mut self, s: &mut Session, n: usize) -> Vec<u8> {
        use std::io::Read;
        let mut buf = [0u8; 4096];
        let wall = std::time::Instant::now();
        while wall.elapsed() < std::time::Duration::from_secs(10) {
            if self.rx.len() >= n {
                return self.rx.drain(..n).collect();
            }
            match self.sock.read(&mut buf) {
                Ok(0) => panic!("server closed the connection"),
                Ok(k) => self.rx.extend_from_slice(&buf[..k]),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    s.run_for(Time::from_us(500));
                }
                Err(e) => panic!("{e}"),
            }
        }
        panic!("no answer from the USB/IP server ({} of {n} bytes, at {})", self.rx.len(), s.machine.now);
    }

    fn send(&mut self, b: &[u8]) {
        use std::io::Write;
        self.sock.write_all(b).unwrap();
    }

    /// CMD_SUBMIT; returns the sequence number.
    fn submit(&mut self, ep: u32, dir_in: bool, setup: [u8; 8], len: u32, out: &[u8]) -> u32 {
        self.seq += 1;
        let mut m = Vec::new();
        for w in [1, self.seq, 0x0001_0002, dir_in as u32, ep, 0, len, 0, 0xFFFF_FFFF, 0] {
            m.extend(u32::to_be_bytes(w));
        }
        m.extend(setup);
        m.extend(out);
        self.send(&m);
        self.seq
    }

    /// Reads one RET_SUBMIT: (seqnum, status, data).
    fn ret(&mut self, s: &mut Session) -> (u32, i32, Vec<u8>) {
        let h = self.read_exact(s, 48);
        let w = |i: usize| u32::from_be_bytes(h[i..i + 4].try_into().unwrap());
        assert_eq!(w(0), 3, "RET_SUBMIT");
        // (Only IN requests are submitted here, so the actual length is data that follows)
        let data = self.read_exact(s, w(24) as usize);
        (w(4), w(20) as i32, data)
    }
}

#[test]
fn usbip_exports_the_bootsel_device() {
    let mut m = bootsel_board();
    sim_host::usb::wait_for_device(&mut m, Time::from_secs(1), |_| true).unwrap();
    let mut s = Session::new(m, &Image::default());
    let server = sim_host::usbip::UsbIpServer::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    s.usbip = Some(server);
    let sock = std::net::TcpStream::connect(addr).unwrap();
    sock.set_nonblocking(true).unwrap();
    // As usbip does (otherwise Nagle holds back small requests behind unacknowledged ones)
    sock.set_nodelay(true).unwrap();
    let mut c = UsbIpClient { sock, seq: 0, rx: Vec::new() };
    // OP_REQ_DEVLIST: one device, 2e8a:0003, two interfaces (mass storage, vendor)
    c.send(&[0x01, 0x11, 0x80, 0x05, 0, 0, 0, 0]);
    let r = c.read_exact(&mut s, 12 + 312 + 8);
    assert_eq!(&r[..4], &[0x01, 0x11, 0x00, 0x05]);
    assert_eq!(u32::from_be_bytes(r[8..12].try_into().unwrap()), 1);
    let dev = &r[12..324];
    assert_eq!(&dev[256..259], b"1-1");
    assert_eq!(&dev[300..304], &[0x2E, 0x8A, 0x00, 0x03]);
    assert_eq!(dev[311], 2, "bNumInterfaces");
    assert_eq!(&r[324..332], &[0x08, 0x06, 0x50, 0, 0xFF, 0, 0, 0]);
    // OP_REQ_IMPORT 1-1 (on a fresh connection, as usbip does)
    drop(c);
    s.run_for(Time::from_ms(1));
    let sock = std::net::TcpStream::connect(addr).unwrap();
    sock.set_nonblocking(true).unwrap();
    // As usbip does (otherwise Nagle holds back small requests behind unacknowledged ones)
    sock.set_nodelay(true).unwrap();
    let mut c = UsbIpClient { sock, seq: 0, rx: Vec::new() };
    let mut imp = vec![0x01, 0x11, 0x80, 0x03, 0, 0, 0, 0];
    let mut busid = b"1-1".to_vec();
    busid.resize(32, 0);
    imp.extend(busid);
    c.send(&imp);
    let r = c.read_exact(&mut s, 8 + 312);
    assert_eq!(&r[..8], &[0x01, 0x11, 0x00, 0x03, 0, 0, 0, 0]);
    // GET_DESCRIPTOR(device) through CMD_SUBMIT
    let seq = c.submit(0, true, [0x80, 6, 0, 1, 0, 0, 18, 0], 18, &[]);
    let (rs, status, data) = c.ret(&mut s);
    assert_eq!((rs, status, data.len()), (seq, 0, 18));
    assert_eq!(&data[8..12], &[0x8A, 0x2E, 0x03, 0x00]);
    // PICOBOOT_IF_CMD_STATUS on interface 1
    let seq = c.submit(0, true, [0xC1, 0x42, 0, 0, 1, 0, 16, 0], 16, &[]);
    let (rs, status, data) = c.ret(&mut s);
    assert_eq!((rs, status, data.len()), (seq, 0, 16));
    // A bulk IN that would wait forever, unlinked
    let victim = c.submit(4, true, [0; 8], 64, &[]);
    s.run_for(Time::from_ms(5));
    c.seq += 1;
    let mut m = Vec::new();
    for w in [2, c.seq, 0x0001_0002, 0, 0, victim] {
        m.extend(u32::to_be_bytes(w));
    }
    m.resize(48, 0);
    c.send(&m);
    let h = c.read_exact(&mut s, 48);
    assert_eq!(u32::from_be_bytes(h[0..4].try_into().unwrap()), 4, "RET_UNLINK");
    assert_eq!(i32::from_be_bytes(h[20..24].try_into().unwrap()), -104, "ECONNRESET: it was still pending");
    assert!(s.usbip.as_ref().unwrap().attached());
}

#[test]
fn tinyusb_survives_suspend_and_resume() {
    let mut s = common::boot(&common::repo().join("test_programs/pico/hello_usb.elf"), None);
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    assert!(cdc_wait(&mut s, 0, "Hello, USB!\r\n", Time::from_secs(2)));
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.suspend(now));
    let sink = sim_core::trace::VecSink::new(&[sim_core::trace::Category::Device]);
    s.machine.set_trace(Box::new(sink.clone()));
    s.run_for(Time::from_ms(50));
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| {
        h.resume(now);
        h.cdc_write(0, b"z");
    });
    let from = s.usb_cdc_text.len();
    assert!(cdc_wait(&mut s, from, "echo z (1)\r\n", Time::from_secs(1)), "{:?}", s.usb_cdc_text);
    let events: Vec<String> = sink.take().iter().map(|(_, e)| e.to_string()).collect();
    assert!(events.iter().any(|e| e.contains("device suspended")), "{events:?}");
    assert!(events.iter().any(|e| e.contains("device resumed")), "{events:?}");
}

#[test]
fn cable_pulled_and_plugged_back_in() {
    let mut s = common::boot(&common::repo().join("test_programs/pico/hello_usb.elf"), None);
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    assert!(cdc_wait(&mut s, 0, "Hello, USB!\r\n", Time::from_secs(2)));
    let old = s.machine.sys.detach_usb_host().unwrap();
    assert_eq!(old.enumerations, 1);
    s.run_for(Time::from_ms(100));
    // A new host (another port): enumerates the device again; the firmware's stdio carries on
    let mut host = HostController::new();
    host.cdc_open(0, 115_200);
    let now = s.machine.now;
    s.machine.sys.attach_usb_host(host, now);
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(0, b"k"));
    let from = s.usb_cdc_text.len();
    assert!(cdc_wait(&mut s, from, "echo k (1)\r\n", Time::from_secs(2)), "{:?}", s.usb_cdc_text);
}

/// CMD_UNLINK for a request that the device already answered (its RET_SUBMIT not yet sent: the transfer finished
/// in the same slice the unlink arrived in) must not throw the answer away: usbip-host then sends the RET_SUBMIT and
/// a RET_UNLINK with status 0, and the client keeps the data (USB/IP protocol, "USBIP_RET_UNLINK").
#[test]
fn usbip_unlink_of_a_finished_request_keeps_its_answer() {
    let mut m = bootsel_board();
    sim_host::usb::wait_for_device(&mut m, Time::from_secs(1), |_| true).unwrap();
    let mut s = Session::new(m, &Image::default());
    let server = sim_host::usbip::UsbIpServer::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    s.usbip = Some(server);
    let sock = std::net::TcpStream::connect(addr).unwrap();
    sock.set_nonblocking(true).unwrap();
    sock.set_nodelay(true).unwrap();
    let mut c = UsbIpClient { sock, seq: 0, rx: Vec::new() };
    let mut imp = vec![0x01, 0x11, 0x80, 0x03, 0, 0, 0, 0];
    let mut busid = b"1-1".to_vec();
    busid.resize(32, 0);
    imp.extend(busid);
    c.send(&imp);
    c.read_exact(&mut s, 8 + 312);
    // The server takes the request, then the machine runs (without the server looking) until the device answered
    let poll = |s: &mut Session| {
        std::thread::sleep(std::time::Duration::from_millis(50));
        s.usbip.as_mut().unwrap().poll(&mut s.machine);
    };
    let seq = c.submit(0, true, [0x80, 6, 0, 1, 0, 0, 18, 0], 18, &[]);
    poll(&mut s);
    s.machine.run_for(Time::from_ms(20));
    // The client gives up on it just as the answer comes in
    c.seq += 1;
    let unlink_seq = c.seq;
    let mut msg = Vec::new();
    for w in [2, unlink_seq, 0x0001_0002, 0, 0, seq] {
        msg.extend(u32::to_be_bytes(w));
    }
    msg.resize(48, 0);
    c.send(&msg);
    poll(&mut s);
    let h = c.read_exact(&mut s, 48);
    let w = |i: usize| u32::from_be_bytes(h[i..i + 4].try_into().unwrap());
    assert_eq!((w(0), w(4)), (3, seq), "the finished request's RET_SUBMIT first");
    assert_eq!((w(20), w(24)), (0, 18));
    let data = c.read_exact(&mut s, 18);
    assert_eq!(&data[8..12], &[0x8A, 0x2E, 0x03, 0x00]);
    let h = c.read_exact(&mut s, 48);
    let w = |i: usize| u32::from_be_bytes(h[i..i + 4].try_into().unwrap());
    assert_eq!((w(0), w(4), w(20)), (4, unlink_seq, 0), "RET_UNLINK, status 0: nothing was unlinked");
}
