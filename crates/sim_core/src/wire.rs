//! IRQ lines and digital pin levels.

/// Logic level seen on, or driven onto, a pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Level {
    Low,
    High,
}

impl Level {
    pub fn from_bool(high: bool) -> Level {
        if high { Level::High } else { Level::Low }
    }
    pub fn is_high(self) -> bool {
        self == Level::High
    }
}

/// What is connected to a pin from outside the chip.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum External {
    /// Nothing; the pin floats (reads its pad pull, or low if none).
    #[default]
    Floating,
    /// Driven to a level by an external device.
    Driven(Level),
    /// A weak external pull, overridden by anything driving the pin.
    Pull(Level),
}

/// Resolves the level on a wire from the chip's output (if enabled), the pad pulls and the outside world.
///
/// Strong drivers win over pulls. If the chip and an external device drive opposite levels, the chip wins (the
/// conflict is the caller's to report).
pub fn resolve(chip_drive: Option<Level>, pull_up: bool, pull_down: bool, external: External) -> Level {
    if let Some(l) = chip_drive {
        return l;
    }
    match external {
        External::Driven(l) => l,
        External::Pull(l) if !pull_up && !pull_down => l,
        _ => {
            if pull_up && !pull_down {
                Level::High
            } else if pull_down && !pull_up {
                Level::Low
            } else if let External::Pull(l) = external {
                l
            } else if pull_up && pull_down {
                // Bus keeper: hold the previous value; without history, high
                Level::High
            } else {
                Level::Low
            }
        }
    }
}

/// A set of up to 32 level-sensitive interrupt lines.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct IrqLines(pub u32);

impl IrqLines {
    pub fn set(&mut self, n: u32, asserted: bool) {
        if asserted {
            self.0 |= 1 << n;
        } else {
            self.0 &= !(1 << n);
        }
    }
    pub fn get(&self, n: u32) -> bool {
        self.0 & (1 << n) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolution() {
        assert_eq!(resolve(Some(Level::Low), true, false, External::Driven(Level::High)), Level::Low);
        assert_eq!(resolve(None, false, false, External::Driven(Level::High)), Level::High);
        assert_eq!(resolve(None, true, false, External::Floating), Level::High);
        assert_eq!(resolve(None, false, true, External::Floating), Level::Low);
        assert_eq!(resolve(None, false, false, External::Floating), Level::Low);
        assert_eq!(resolve(None, false, false, External::Pull(Level::High)), Level::High);
        // Pad pull-down beats nothing but loses to a driver
        assert_eq!(resolve(None, false, true, External::Pull(Level::High)), Level::Low);
    }

    #[test]
    fn irq_lines() {
        let mut l = IrqLines::default();
        l.set(3, true);
        l.set(25, true);
        assert!(l.get(3) && l.get(25) && !l.get(4));
        l.set(3, false);
        assert_eq!(l.0, 1 << 25);
    }
}
