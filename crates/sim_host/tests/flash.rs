//! M6: the real flash path. Boots go through the bootrom and boot2 against the QSPI flash model; the C Titan
//! bootloader reflashes its application over CAN; flash contents and crash records survive resets.

mod common;

use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::CanBus;
use sim_host::board::Board;
use sim_host::upload::{CanmoreLink, upload_ota};
use titan_link::{HostToDevice, ivc2};

fn board(name: &str) -> Board {
    Board::load(&common::repo().join("boards").join(format!("{name}.toml"))).unwrap()
}

#[test]
fn upload_tool_reflashes_through_the_c_bootloader() {
    let (Some(with_bl), Some(ota)) =
        (common::built("bl_app_v0/bl_app_with_bl.uf2"), common::built("bl_app_v1/bl_app_ota.uf2"))
    else {
        return;
    };
    let b = board("talos/led_board");
    let mut s = common::boot_rom(&with_bl);
    b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    let link = CanmoreLink::new(6);
    s.can_endpoints.push(Box::new(link.clone()));

    // Power-on: bootrom, boot2, the bootloader, its boot delay, then application 1.0
    s.run_until(Time::from_ms(1000));
    s.poll_uarts();
    let log = s.uart_text[0].clone();
    assert!(log.contains("titan_bootloader"), "{log}");
    assert!(log.contains("Bootloader test app 1.0"), "{log}");

    // Reset the board (watchdog) and catch it in the boot delay; upload 1.1
    s.machine.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
    let image = Image::load(&ota).unwrap();
    let report = upload_ota(&mut s, &link, &image, Time::from_secs(6)).expect("upload");
    assert_eq!(report.app_base, 0x1000_4000);
    assert_eq!(report.flash_size, 2 << 20);
    assert_eq!(report.flash_id, u64::from_le_bytes(rp2040::periph::qspi_flash::UNIQUE_ID), "the chip's unique ID");
    assert!(report.pages_written > 50, "{report:?}");
    eprintln!("{report:?}");

    // The bootloader reboots into the new application
    s.uart_text[0].clear();
    let t = s.machine.now;
    s.run_until(t + Time::from_ms(1500));
    s.poll_uarts();
    assert!(s.uart_text[0].contains("Bootloader test app 1.1"), "{}", s.uart_text[0]);
    // And the flash holds exactly the uploaded pages
    for (addr, data) in sim_host::upload::pages(&image) {
        let o = (addr - 0x1000_0000) as usize;
        assert_eq!(&s.machine.sys.flash[o..o + 256], &data[..], "{addr:#x}");
    }
}

/// Forces a panic on core 0 by calling the firmware's own panic path (`__defmt_default_panic`, which calls
/// `core::panicking::panic` with a real message and location).
fn force_panic(s: &mut sim_host::Session, image: &Image) {
    let f = image.symbol("__defmt_default_panic").expect("panic symbol").addr;
    s.machine.inject_call(0, f, &[]);
}

#[test]
fn panic_leaves_a_crash_record_across_watchdog_resets() {
    let Some(elf) = common::titan_elf("power_board") else { return };
    let image = Image::load(&elf).unwrap();
    let b = board("mk2/power_board");
    let mut s = common::boot_rom(&elf);
    b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    // ROS connects only later: until then the watchdog timeout is the 3 s setup timeout
    let bridge = sim_host::mock_bridge::MockBridge::new(1, true, Time::from_ms(2500));
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    let counts =
        |w: u8, p: u8| format!("Crash counts since power on: CrashCounts {{ watchdog_timeouts: {w}, panics: {p} }}");

    s.run_until(Time::from_ms(1500));
    assert!(s.logged(&counts(0, 0)), "{:?}", s.logs);
    let resets = s.machine.resets;

    // A panic: the handler logs it, marks scratch3, waits and forces a watchdog reset
    force_panic(&mut s, &image);
    s.run_until(Time::from_ms(4000));
    assert!(s.logged("PANIC: panicked at"), "{:?}", s.logs);
    assert_eq!(s.machine.resets, resets + 1, "one watchdog reset");
    assert_eq!(s.machine.sys.watchdog.read(s.machine.now, 0x08), Some(2), "REASON = FORCE");
    assert!(s.logged("Reset by panic after 13"), "uptime at the panic: {:?}", s.logs);
    assert!(s.logged("Fault FAULT_WATCHDOG_RESET raised"), "{:?}", s.logs);
    assert!(s.logged(&counts(0, 1)), "{:?}", s.logs);
    // ROS sees FAULT_WATCHDOG_RESET (bit 0) in the firmware status of the new boot
    let st = bridge.statuses();
    assert!(!st.is_empty(), "the bridge connected");
    assert!(st.iter().all(|(_, f)| f.faults & 1 == 1), "{st:?}");

    // A panic while ROS is connected (250 ms active watchdog timeout) is recorded as a panic too: the handler
    // reloads the watchdog before its 100 ms log drain and then forces the reset. (Before titan_firmware_rs
    // b2afb53 it busy-waited 300 ms, the timeout fired first and the boot reported a watchdog timeout.)
    s.logs.clear();
    force_panic(&mut s, &image);
    s.run_until(Time::from_ms(6000));
    assert!(s.logged("PANIC: panicked at"), "{:?}", s.logs);
    assert_eq!(s.machine.resets, resets + 2);
    assert_eq!(s.machine.sys.watchdog.read(s.machine.now, 0x08), Some(2), "REASON = FORCE");
    assert!(s.logged("Reset by panic after"), "{:?}", s.logs);
    assert!(!s.logged("Watchdog timeout after"), "{:?}", s.logs);
    assert!(s.logged(&counts(0, 2)), "{:?}", s.logs);

    // A power cycle clears the record (the scratch registers lose power) and REASON
    s.logs.clear();
    s.machine.power_cycle();
    let t = s.machine.now;
    s.run_until(t + Time::from_ms(500));
    assert!(s.logged(&counts(0, 0)), "{:?}", s.logs);
    assert!(!s.logged("Reset by"));
    assert_eq!(s.machine.sys.watchdog.read(s.machine.now, 0x08), Some(0));
}

#[test]
fn ivc2_flash_recording_survives_a_watchdog_reset() {
    let Some(elf) = common::titan_elf("ivc_board2") else { return };
    let b = board("talos/ivc_board");
    let mut s = common::boot_rom(&elf);
    b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(8, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    s.run_until(Time::from_ms(600));
    assert!(s.logged("ROS connected"), "{:?}", s.logs);

    // Record: erase the top 512 KiB a sector at a time, then fill chunk n (4096 bytes) with n + 1
    bridge.send(HostToDevice::Ivc2(ivc2::HostToDevice::StorageConfigure(1)));
    let mut t = Time::from_ms(600);
    while !s.logged("DONE WRITING") {
        assert!(t < Time::from_secs(20), "{:?}", s.logs);
        t += Time::from_ms(500);
        s.run_until(t);
    }
    assert!(s.logged("Erasing and writing flash storage"));
    assert!(!s.logs.iter().any(|(_, l)| l.message.contains("failed")), "{:?}", s.logs);
    let area = &s.machine.sys.flash[0x18_0000..0x20_0000];
    for (n, chunk) in area.chunks(4096).enumerate() {
        assert!(chunk.iter().all(|&b| b == n as u8 + 1), "chunk {n}");
    }
    let chip = &s.machine.sys.ssi.chip;
    assert_eq!((chip.erases, chip.programs), (128, 2048), "128 sector erases, 2048 page programs");
    eprintln!("recorded by {}", s.machine.now);

    // Reset the board; the recording is still there and the firmware dumps it
    s.machine.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
    s.logs.clear();
    let t = s.machine.now;
    s.run_until(t + Time::from_ms(600));
    assert!(s.logged("ROS connected"), "{:?}", s.logs);
    bridge.send(HostToDevice::Ivc2(ivc2::HostToDevice::StorageConfigure(2)));
    s.run_until(t + Time::from_ms(1300));
    assert!(s.logged("START"), "{:?}", s.logs);
    let line = |off: u32, b: u8| format!("{off:#07x}: [{}]", vec![format!("{b:02x}"); 32].join(", "));
    assert!(s.logged(&line(0, 1)), "{} in {:?}", line(0, 1), s.logs);
    assert!(s.logged(&line(0x1000, 2)), "{:?}", s.logs);
    assert!(s.logged(&line(0x1fe0, 2)), "{:?}", s.logs);
}

#[test]
fn camera_cage_surface_pressure_survives_a_watchdog_reset_in_ram() {
    use titan_link::DeviceToHost;
    use titan_link::camera_cage::DeviceToHost as Cage;
    let Some(elf) = common::titan_elf("camera_cage_bb") else { return };
    let b = board("mk2/camera_cage_bb");
    let mut s = common::boot_rom(&elf);
    let inputs = b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(4, b.can_rates().unwrap().data.is_some(), Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    let depths = |bridge: &sim_host::mock_bridge::MockBridge, after: Time| -> Vec<f32> {
        let st = bridge.state();
        st.received
            .iter()
            .filter(|(t, _)| *t > after)
            .filter_map(|(_, m)| match m {
                DeviceToHost::CameraCage(Cage::Depth(d)) => Some(*d),
                _ => None,
            })
            .collect()
    };

    // Zeroed at the surface, then dive to 2.5 m
    s.run_until(Time::from_ms(3000));
    assert!(s.logged("Depth zeroed, surface pressure"), "{:?}", s.logs);
    let surface = inputs.get("depth.pressure_pa");
    inputs.set("depth.pressure_pa", surface + 997.0 * 9.80665 * 2.5);
    s.run_until(Time::from_ms(3500));

    // A watchdog reset underwater: the surface pressure in `.uninit` survives, so there is no zeroing and the depth
    // is right straight away
    s.logs.clear();
    s.machine.watchdog_reset(rp2040::periph::timer::WatchdogReset::Force);
    let t = s.machine.now;
    s.run_until(t + Time::from_ms(1000));
    assert!(s.logged("Depth surface pressure found"), "{:?}", s.logs);
    assert!(!s.logged("Depth zeroed"));
    let d = depths(&bridge, t);
    assert!(!d.is_empty() && d.iter().all(|d| (d + 2.5).abs() < 0.01), "{d:?}");

    // A power cycle loses SRAM: the board zeroes again, at 2.5 m, so it now reads 0 there
    s.logs.clear();
    s.machine.power_cycle();
    let t = s.machine.now;
    s.run_until(t + Time::from_ms(3000));
    assert!(s.logged("Depth zeroed, surface pressure"), "{:?}", s.logs);
    assert!(!s.logged("Depth surface pressure found"));
    let d = depths(&bridge, t + Time::from_ms(2500));
    assert!(!d.is_empty() && d.iter().all(|d| d.abs() < 0.01), "{d:?}");
}
