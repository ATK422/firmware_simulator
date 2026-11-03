//! Positions, boundaries and the image method.
//!
//! Coordinates are metres; `z` is depth below the surface (positive down).

use serde::Deserialize;

pub type Vec3 = [f64; 3];

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

pub fn angle_between(a: Vec3, b: Vec3) -> f64 {
    let n = norm(a) * norm(b);
    if n == 0.0 { 0.0 } else { (dot(a, b) / n).clamp(-1.0, 1.0).acos() }
}

/// What bounds the water.
#[derive(Clone, Debug, PartialEq)]
pub enum Boundaries {
    /// Free field: no surface, bottom or walls.
    Open,
    /// A rectangular tank or pool from (0, 0, 0) to `size`, with the surface at z = 0.
    Tank { size: Vec3 },
    /// Horizontally unbounded water of `depth` metres.
    Lake { depth: f64 },
}

/// Reflection coefficients (pressure amplitude). The surface coefficient is applied with a sign flip (pressure
/// release).
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct Reflections {
    pub surface: f64,
    pub bottom: f64,
    pub walls: f64,
}

/// One path from a source to a receiver: through the image at `image`, with amplitude factor `coefficient` from the
/// reflections on the way (sign included) and `order` reflections.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Image {
    pub position: Vec3,
    pub coefficient: f64,
    pub order: u32,
}

/// Image positions along one axis bounded by walls at 0 and `len` with coefficients `b0` and `b1`: for
/// `(1 - 2q) x + 2 n len` the path meets the wall at 0 `|n - q|` times and the wall at `len` `|n|` times.
fn axis_images(x: f64, len: Option<f64>, b0: f64, b1: f64, max_order: u32) -> Vec<(f64, f64, u32)> {
    let Some(len) = len else { return vec![(x, 1.0, 0)] };
    let mut out = Vec::new();
    let m = max_order as i64;
    for n in -m..=m {
        for q in 0..=1i64 {
            let hits0 = (n - q).unsigned_abs() as u32;
            let hits1 = n.unsigned_abs() as u32;
            let order = hits0 + hits1;
            if order > max_order {
                continue;
            }
            let pos = (1 - 2 * q) as f64 * x + 2.0 * n as f64 * len;
            out.push((pos, b0.powi(hits0 as i32) * b1.powi(hits1 as i32), order));
        }
    }
    out
}

impl Boundaries {
    /// Every image of a source at `src` with at most `max_order` reflections (the source itself included).
    pub fn images(&self, src: Vec3, refl: Reflections, max_order: u32) -> Vec<Image> {
        let (lx, ly, lz) = match self {
            Boundaries::Open => return vec![Image { position: src, coefficient: 1.0, order: 0 }],
            Boundaries::Tank { size } => (Some(size[0]), Some(size[1]), Some(size[2])),
            Boundaries::Lake { depth } => (None, None, Some(*depth)),
        };
        let xs = axis_images(src[0], lx, refl.walls, refl.walls, max_order);
        let ys = axis_images(src[1], ly, refl.walls, refl.walls, max_order);
        let zs = axis_images(src[2], lz, -refl.surface, refl.bottom, max_order);
        let mut out = Vec::new();
        for &(x, cx, ox) in &xs {
            for &(y, cy, oy) in &ys {
                for &(z, cz, oz) in &zs {
                    let order = ox + oy + oz;
                    if order <= max_order {
                        out.push(Image { position: [x, y, z], coefficient: cx * cy * cz, order });
                    }
                }
            }
        }
        out
    }

    /// Whether `p` is in the water.
    pub fn contains(&self, p: Vec3) -> bool {
        match self {
            Boundaries::Open => true,
            Boundaries::Tank { size } => (0..3).all(|i| p[i] >= 0.0 && p[i] <= size[i]),
            Boundaries::Lake { depth } => p[2] >= 0.0 && p[2] <= *depth,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const R: Reflections = Reflections { surface: 1.0, bottom: 0.5, walls: 0.25 };

    #[test]
    fn open_water_is_the_direct_path() {
        assert_eq!(Boundaries::Open.images([1.0, 2.0, 3.0], R, 3).len(), 1);
    }

    #[test]
    fn lake_first_order_images() {
        let imgs = Boundaries::Lake { depth: 10.0 }.images([0.0, 0.0, 2.0], R, 1);
        // Direct, surface image at z = -2 (sign flipped), bottom image at z = 18
        assert_eq!(imgs.len(), 3);
        let surface = imgs.iter().find(|i| i.position[2] == -2.0).unwrap();
        assert_eq!((surface.coefficient, surface.order), (-1.0, 1));
        let bottom = imgs.iter().find(|i| i.position[2] == 18.0).unwrap();
        assert_eq!((bottom.coefficient, bottom.order), (0.5, 1));
        // Second order adds surface-bottom (z = -18 ... ) and bottom-surface (z = 22) paths
        let imgs = Boundaries::Lake { depth: 10.0 }.images([0.0, 0.0, 2.0], R, 2);
        assert_eq!(imgs.len(), 5);
        assert!(imgs.iter().any(|i| i.position[2] == 22.0 && i.coefficient == -0.5));
    }

    #[test]
    fn tank_image_count_and_walls() {
        let tank = Boundaries::Tank { size: [25.0, 12.0, 3.0] };
        let imgs = tank.images([20.0, 6.0, 2.0], R, 1);
        assert_eq!(imgs.len(), 7, "direct + one image per face");
        let wall = imgs.iter().find(|i| i.position[0] == 30.0).unwrap();
        assert_eq!(wall.coefficient, 0.25);
        // Order 2 and 3: 25 and 63 images (the octahedral numbers of the 3D lattice)
        assert_eq!(tank.images([20.0, 6.0, 2.0], R, 2).len(), 25);
        assert_eq!(tank.images([20.0, 6.0, 2.0], R, 3).len(), 63);
        assert!(tank.contains([1.0, 1.0, 1.0]) && !tank.contains([1.0, 1.0, 4.0]));
    }
}
