//! Memory that tells whether it changed.

use std::ops::{Deref, DerefMut};
use std::sync::atomic::{AtomicU64, Ordering};

/// Versions are unique across all arrays: equal versions mean equal contents.
static NEXT_VERSION: AtomicU64 = AtomicU64::new(1);

fn next_version() -> u64 {
    NEXT_VERSION.fetch_add(1, Ordering::Relaxed)
}

/// A byte array with a version that changes whenever it is borrowed mutably (written, perhaps): whoever keeps a
/// copy can tell cheaply whether the array still has the copy's contents, without comparing them. Versions are
/// unique across all arrays, and a clone keeps its original's. Reads go to the array as they would to a `Vec`.
/// For memories that are large and rarely written, like a board's flash.
#[derive(Clone, Default)]
pub struct Versioned {
    data: Vec<u8>,
    version: u64,
}

impl Versioned {
    /// The contents' version: two arrays with the same version have the same contents.
    pub fn version(&self) -> u64 {
        self.version
    }

    /// Replaces the contents with a copy of `other`'s (reusing the allocation), version included.
    pub fn copy_from(&mut self, other: &Versioned) {
        self.data.clone_from(&other.data);
        self.version = other.version;
    }
}

impl PartialEq for Versioned {
    fn eq(&self, other: &Self) -> bool {
        self.version == other.version || self.data == other.data
    }
}

impl Eq for Versioned {}

impl std::hash::Hash for Versioned {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.data.hash(state);
    }
}

impl From<Vec<u8>> for Versioned {
    fn from(data: Vec<u8>) -> Self {
        Versioned { data, version: next_version() }
    }
}

impl Deref for Versioned {
    type Target = [u8];
    #[inline(always)]
    fn deref(&self) -> &[u8] {
        &self.data
    }
}

impl DerefMut for Versioned {
    #[inline]
    fn deref_mut(&mut self) -> &mut [u8] {
        self.version = next_version();
        &mut self.data
    }
}

impl AsRef<[u8]> for Versioned {
    fn as_ref(&self) -> &[u8] {
        &self.data
    }
}

impl std::fmt::Debug for Versioned {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Versioned({} bytes, version {})", self.data.len(), self.version)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_change_the_version_and_reads_do_not() {
        let mut a = Versioned::from(vec![1, 2, 3]);
        let v = a.version();
        assert_eq!((a[1], a.len(), a.version()), (2, 3, v));
        let copy = a.clone();
        assert_eq!(copy.version(), v, "a clone has the same contents");
        a[1] = 9;
        assert_ne!(a.version(), v);
        assert_eq!(&a[..], [1, 9, 3]);
        assert_eq!(&copy[..], [1, 2, 3]);
        let mut b = Versioned::default();
        b.copy_from(&a);
        assert_eq!((b.version(), &b[..]), (a.version(), &[1, 9, 3][..]));
        // Arrays made apart never share a version, even with the same contents
        let (c, d) = (Versioned::from(vec![1]), Versioned::from(vec![1]));
        assert_ne!(c.version(), d.version());
        assert_eq!(c, d, "equal contents compare equal");
    }
}
