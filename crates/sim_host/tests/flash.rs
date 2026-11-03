//! M6: the real flash path. Boots go through the bootrom and boot2 against the QSPI flash model; the C Titan
//! bootloader reflashes its application over CAN; flash contents and crash records survive resets.

mod common;

use sim_core::image::Image;
use sim_core::time::Time;
use sim_devices::can::CanBus;
use sim_host::board::Board;
use sim_host::upload::{CanmoreLink, upload_ota};

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
