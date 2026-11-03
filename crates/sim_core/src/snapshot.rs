//! Snapshot support.
//!
//! Machines are plain data, so a snapshot of a running machine is a deep copy of it and restoring is replacing the
//! machine with the copy. That is what makes `sim_test` setups fast: boot once, snapshot, run each test from the
//! snapshot. Parts held as trait objects (external devices) provide a `clone_box` method; [`clone_box!`] then
//! implements `Clone` for the boxed trait object.

/// Implements `Clone` for `Box<dyn Trait>` given that `Trait` has a `fn clone_box(&self) -> Box<dyn Trait>` method.
#[macro_export]
macro_rules! clone_box {
    ($trait:path) => {
        impl Clone for Box<dyn $trait> {
            fn clone(&self) -> Self {
                self.clone_box()
            }
        }
    };
}

#[cfg(test)]
mod tests {
    trait Shape {
        fn area(&self) -> u32;
        fn clone_box(&self) -> Box<dyn Shape>;
    }
    crate::clone_box!(Shape);

    #[derive(Clone)]
    struct Square(u32);
    impl Shape for Square {
        fn area(&self) -> u32 {
            self.0 * self.0
        }
        fn clone_box(&self) -> Box<dyn Shape> {
            Box::new(self.clone())
        }
    }

    #[test]
    fn boxed_trait_objects_clone() {
        let a: Box<dyn Shape> = Box::new(Square(3));
        let b = a.clone();
        assert_eq!(b.area(), 9);
    }
}
