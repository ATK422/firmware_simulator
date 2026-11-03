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
    host.cdc_open(115_200);
    s.machine.sys.attach_usb_host(host, Time::ZERO);
    let ok = cdc_wait(&mut s, 0, "Hello, USB!\r\n", Time::from_secs(2));
    assert!(ok, "{:?}", s.usb_cdc_text);
    let dev = s.machine.sys.usb_host.as_ref().unwrap().configured().unwrap().clone();
    assert_eq!((dev.vendor_id(), dev.product_id()), (0x2E8A, 0x000A), "pico-sdk stdio_usb IDs");
    assert_eq!(dev.product.as_deref(), Some("Pico"));
    // Input: each character comes back on a line of its own
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(b"xy"));
    let from = s.usb_cdc_text.len();
    assert!(cdc_wait(&mut s, from, "echo y (2)\r\n", Time::from_secs(1)), "{:?}", s.usb_cdc_text);
    assert!(s.usb_cdc_text[from..].contains("echo x (1)\r\n"), "{:?}", s.usb_cdc_text);
    // 'b' calls reset_usb_boot: the device leaves, and comes back as the bootrom's RP2 Boot
    let now = s.machine.now;
    s.machine.sys.with_usb_host(now, |h| h.cdc_write(b"b"));
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
