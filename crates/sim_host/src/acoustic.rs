//! Boards in a shared acoustic medium: the pin device that feeds a board's projector.

use sim_acoustics::Hydrophone;
use sim_acoustics::medium::SharedMedium;
use sim_core::analog::AnalogSource;
use sim_devices::DeviceCtx;
use sim_devices::fault::FaultyAnalog;
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

/// The hydrophone an ADC input's source is, if it is one, looking through a fault injector wrapped around it (an
/// ADC fault on the hydrophone's input).
fn hydrophone_in(src: &mut dyn AnalogSource) -> Option<&mut Hydrophone> {
    let any = src.as_any_mut()?;
    if any.is::<Hydrophone>() {
        return any.downcast_mut::<Hydrophone>();
    }
    hydrophone_in(&mut *any.downcast_mut::<FaultyAnalog>()?.inner)
}

/// The hydrophones attached to a machine's ADC inputs.
pub fn hydrophones(m: &mut rp2040::Machine) -> impl Iterator<Item = &mut Hydrophone> {
    m.sys.analog_inputs.iter_mut().flatten().filter_map(|h| hydrophone_in(&mut **h))
}

/// Points a machine's hydrophones and projector pins that use medium `old` at `new` (a copy of it).
pub fn relink(m: &mut rp2040::Machine, old: &SharedMedium, new: &SharedMedium) {
    for h in hydrophones(m) {
        if h.medium().is_some_and(|m| std::sync::Arc::ptr_eq(m, old)) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use sim_acoustics::chain::ChainSpec;
    use sim_acoustics::medium::Medium;
    use sim_acoustics::transducer::Transducer;
    use sim_acoustics::water::Water;

    #[test]
    fn hydrophones_are_found_behind_a_fault_injector() {
        let medium = Medium::shared(std::sync::Arc::new(sim_acoustics::Environment::open_water(Water::default())));
        let env = medium.lock().unwrap().env.clone();
        let h = Hydrophone::new(env, [0.0; 3], Transducer::hydrophone(-190.0), ChainSpec::default())
            .in_medium(medium.clone(), 0);
        let mut m = rp2040::Machine::new(Default::default());
        m.sys.attach_analog(0, Box::new(FaultyAnalog::new(Box::new(h), 1)));
        assert_eq!(hydrophones(&mut m).count(), 1);
        // A snapshot's copy of the machine is moved to the copy of the medium
        let copy = std::sync::Arc::new(std::sync::Mutex::new(medium.lock().unwrap().clone()));
        relink(&mut m, &medium, &copy);
        assert!(std::sync::Arc::ptr_eq(hydrophones(&mut m).next().unwrap().medium().unwrap(), &copy));
    }
}
