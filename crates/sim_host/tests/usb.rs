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
