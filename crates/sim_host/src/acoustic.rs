//! Boards in a shared acoustic medium: the pin device that feeds a board's projector.

use sim_acoustics::medium::SharedMedium;
use sim_devices::DeviceCtx;
use sim_devices::pin::PinDevice;

/// Watches a projector's drive pin and records its level changes in the medium.
#[derive(Clone)]
pub struct ProjectorPin {
    medium: SharedMedium,
    id: usize,
    pin: u8,
}

impl ProjectorPin {
    pub fn new(medium: SharedMedium, id: usize, pin: u8) -> Self {
        ProjectorPin { medium, id, pin }
    }
}

impl PinDevice for ProjectorPin {
    fn name(&self) -> String {
        format!("projector (GPIO {})", self.pin)
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.pin
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, _pin: u8, level: bool) {
        self.medium.lock().unwrap().edge(self.id, ctx.now(), level);
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

/// The hydrophones attached to a machine's ADC inputs.
pub fn hydrophones(m: &mut rp2040::Machine) -> impl Iterator<Item = &mut sim_acoustics::Hydrophone> {
    m.sys
        .analog_inputs
        .iter_mut()
        .flatten()
        .filter_map(|h| h.as_any_mut().and_then(|a| a.downcast_mut::<sim_acoustics::Hydrophone>()))
}

/// Points a machine's hydrophones and projector pins that use medium `old` at `new` (a copy of it).
pub fn relink(m: &mut rp2040::Machine, old: &SharedMedium, new: &SharedMedium) {
    for h in m.sys.analog_inputs.iter_mut().flatten() {
        if let Some(h) = h.as_any_mut().and_then(|a| a.downcast_mut::<sim_acoustics::Hydrophone>())
            && h.medium().is_some_and(|m| std::sync::Arc::ptr_eq(m, old))
        {
            h.relink(new.clone());
        }
    }
    for d in m.sys.pin_devices.iter_mut() {
        if let Some(p) = d.as_any_mut().and_then(|a| a.downcast_mut::<ProjectorPin>())
            && std::sync::Arc::ptr_eq(&p.medium, old)
        {
            p.medium = new.clone();
        }
    }
}
