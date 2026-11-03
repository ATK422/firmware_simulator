//! USB: the real bootrom's USB boot mode (BOOTSEL) enumerating on the host model.

use rp2040::{Machine, MachineConfig};
use sim_core::time::Time;
use sim_devices::usb::host::{Enumeration, HostController};

fn bootsel_machine() -> Machine {
    let mut m = Machine::new(MachineConfig { bootrom: Some(rp2040::BOOTROM_B1.to_vec()), ..MachineConfig::default() });
    m.sys.io_qspi.bootsel_pressed = true;
    m.rom_boot().unwrap();
    let now = m.now;
    m.sys.attach_usb_host(HostController::new(), now);
    m
}

#[test]
fn bootrom_enumerates_as_rp2_boot() {
    let mut m = bootsel_machine();
    let sink =
        sim_core::trace::VecSink::new(&[sim_core::trace::Category::Device, sim_core::trace::Category::Unmodelled]);
    m.set_trace(Box::new(sink.clone()));
    m.run_until(Time::from_ms(400));
    for (t, e) in sink.take() {
        eprintln!("[{t}] {e}");
    }
    let host = m.sys.usb_host.as_ref().unwrap();
    assert_eq!(host.enumeration, Enumeration::Configured, "{:?}", host.enumeration);
    let dev = host.configured().unwrap();
    assert_eq!((dev.vendor_id(), dev.product_id()), (0x2E8A, 0x0003));
    assert_eq!(dev.product.as_deref(), Some("RP2 Boot"));
    assert!(dev.serial.as_deref().is_some_and(|s| s.starts_with("E0C912")), "{:?}", dev.serial);
}
