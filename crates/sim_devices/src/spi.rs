//! SPI device interface.
//!
//! The SoC's SPI controller exchanges one frame at a time with the devices whose chip select is asserted. Chip
//! selects are ordinary GPIOs (the UWRT boards drive them from SIO), so the machine tells a device when its CS pin
//! changes.

use crate::DeviceCtx;
use crate::can::CanNode;

pub trait SpiDevice: Send {
    fn name(&self) -> String;
    /// The device's chip select changed (`true` = asserted).
    fn select(&mut self, ctx: &mut dyn DeviceCtx, selected: bool);
    /// Exchanges one frame of `bits` bits while selected; returns the device's MISO frame.
    fn transfer(&mut self, ctx: &mut dyn DeviceCtx, mosi: u16, bits: u8) -> u16;
    /// Lets the device update its outputs after time passed or something reached it from outside (a CAN frame).
    fn update(&mut self, ctx: &mut dyn DeviceCtx) {
        let _ = ctx;
    }
    /// The device's CAN side, if it is a CAN controller.
    fn can_node(&mut self) -> Option<&mut dyn CanNode> {
        None
    }
    fn clone_box(&self) -> Box<dyn SpiDevice>;
    /// For finding a device of a particular type again (e.g. to re-link a snapshot's copy).
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
    /// Points the shared input values the device reads at their counterparts in `map` (a snapshot's own inputs,
    /// see [`sim_core::analog::Relink`]). Devices that read inputs implement it; the default reads none.
    fn relink_inputs(&mut self, _map: &sim_core::analog::Relink) {}
}

sim_core::clone_box!(SpiDevice);
