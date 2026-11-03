//! Register names, offsets and reset values generated from the pico-sdk's `rp2040.svd`.

/// A peripheral as described by the SVD.
#[derive(Debug)]
pub struct SvdPeripheral {
    pub name: &'static str,
    pub base: u32,
    pub registers: &'static [SvdRegister],
}

/// A register as described by the SVD.
#[derive(Debug)]
pub struct SvdRegister {
    pub name: &'static str,
    pub offset: u32,
    pub reset: u32,
}

include!(concat!(env!("OUT_DIR"), "/svd_gen.rs"));

/// Size of the address window each peripheral occupies (the next base, capped at 64 KiB).
fn window(i: usize) -> u32 {
    let base = PERIPHERALS[i].base;
    PERIPHERALS.get(i + 1).map(|p| (p.base - base).min(0x1_0000)).unwrap_or(0x1_0000)
}

/// Finds the peripheral containing `addr` (ignoring the RP2040 atomic alias bits for APB/AHB peripherals).
pub fn peripheral_at(addr: u32) -> Option<&'static SvdPeripheral> {
    let idx = PERIPHERALS.partition_point(|p| p.base <= addr);
    let i = idx.checked_sub(1)?;
    let p = &PERIPHERALS[i];
    (addr - p.base < window(i)).then_some(p)
}

/// The register at `addr`, if the SVD describes one there. Alias bits (`+0x1000/+0x2000/+0x3000`) are stripped
/// for peripherals in the APB/AHB ranges.
pub fn register_at(addr: u32) -> Option<(&'static SvdPeripheral, &'static SvdRegister)> {
    let p = peripheral_at(addr)?;
    let mut offset = addr - p.base;
    if has_atomic_aliases(p.base) {
        offset &= 0xFFF;
    }
    let reg = p.registers.iter().find(|r| r.offset == offset & !3)?;
    Some((p, reg))
}

/// Peripherals on the APB and AHB-lite busses have XOR/SET/CLR aliases; SIO and the PPB do not.
pub fn has_atomic_aliases(base: u32) -> bool {
    (0x4000_0000..0x6000_0000).contains(&base)
}

/// Human readable name for an address, e.g. `TIMER.ALARM0` or `SPI0+0x0f0`.
pub fn describe(addr: u32) -> String {
    match register_at(addr) {
        Some((p, r)) => format!("{}.{}", p.name, r.name),
        None => match peripheral_at(addr) {
            Some(p) => format!("{}+{:#05x}", p.name, addr - p.base),
            None => format!("{addr:#010x}"),
        },
    }
}

/// Reset value of the register at `addr` from the SVD, or 0.
pub fn reset_value(addr: u32) -> u32 {
    register_at(addr).map(|(_, r)| r.reset).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookups() {
        assert_eq!(describe(0x4005_4010), "TIMER.ALARM0");
        assert_eq!(describe(0x4005_6010), "TIMER.ALARM0", "SET alias");
        assert_eq!(describe(0xD000_0010), "SIO.GPIO_OUT");
        assert_eq!(describe(0x4003_8000), "UART1.UARTDR", "derived peripheral");
        assert_eq!(reset_value(0x4000_C000), 0x01FF_FFFF, "RESETS.RESET");
        assert!(peripheral_at(0x3000_0000).is_none());
        assert_eq!(offsets::timer::ALARM0, 0x10);
        assert_eq!(offsets::sio::GPIO_OUT_SET, 0x14);
    }
}
