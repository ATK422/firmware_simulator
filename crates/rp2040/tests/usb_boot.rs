//! USB: the real bootrom's USB boot mode (BOOTSEL) enumerating on the host model, and the controller's bus and
//! interrupt wiring.

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

#[test]
fn dpram_takes_any_width_and_usbctrl_irq_reaches_the_nvic() {
    use sim_core::bus::Size;
    use sim_core::trace::NullSink;
    let mut m = Machine::new(MachineConfig::default());
    let t = Time::ZERO;
    let sys = &mut m.sys;
    let w = |sys: &mut rp2040::System, a: u32, size: Size, v: u32| sys.write(&mut NullSink, 0, t, a, size, v).unwrap();
    // Out of reset
    w(sys, 0x4000_C000 + 0x3000, Size::Word, 1 << 24);
    w(sys, 0x5010_0101, Size::Byte, 0xAB);
    w(sys, 0x5010_0102, Size::Half, 0x1234);
    assert_eq!(sys.read(&mut NullSink, 0, t, 0x5010_0100, Size::Word), Ok(0x1234_AB00));
    assert_eq!(sys.peek(0x5010_0101, Size::Byte), Some(0xAB));
    // INTF forces BUFF_STATUS; enabling it in INTE through the SET alias raises USBCTRL_IRQ (5) on both cores
    w(sys, 0x5011_0094, Size::Word, 1 << 4);
    assert_eq!(sys.irq_lines(0) & (1 << 5), 0);
    w(sys, 0x5011_2090, Size::Word, 1 << 4);
    assert_ne!(sys.irq_lines(0) & (1 << 5), 0);
    assert_ne!(sys.irq_lines(1) & (1 << 5), 0);
    assert_eq!(sys.read(&mut NullSink, 0, t, 0x5011_0098, Size::Word), Ok(1 << 4), "INTS");
    // RESETS.USBCTRL resets the registers; the DPRAM keeps its contents
    w(sys, 0x4000_C000 + 0x2000, Size::Word, 1 << 24);
    assert_eq!(sys.irq_lines(0) & (1 << 5), 0);
    assert_eq!(sys.read(&mut NullSink, 0, t, 0x5011_0090, Size::Word), Ok(0));
    assert_eq!(sys.read(&mut NullSink, 0, t, 0x5010_0100, Size::Word), Ok(0x1234_AB00));
}
