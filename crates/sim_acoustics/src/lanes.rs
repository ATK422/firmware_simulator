//! A hydrophone's path sum computed [`LANES`] paths at a time in SIMD lanes, bit for bit as one path at a time.
//!
//! A hydrophone's output at a grid step is a sum over its paths (`Hydrophone::raw`): each path's delay and gain
//! interpolated across the block, the origin's pressure read at the step's time less the delay, times the gain,
//! added to the sum path by path. Floating-point addition is not associative, so a step's terms must be added in
//! that order; but each term is computed from its own path alone. So the paths of an origin are kept side by side,
//! [`LANES`] to a lane set (`Hydrophone::raw_lanes`), and each step computes their terms a lane set at a time, every
//! lane doing the operations `raw` does for its path, in the same order, on the same operands (the kernels here),
//! and then adds the terms to the step's sum one after the other, in `raw`'s order. Written as loops over
//! fixed-size arrays, in functions taking references (so that LLVM knows their output does not overlap their
//! input), LLVM turns them into SIMD instructions on the target's baseline: SSE2 on x86-64, NEON on aarch64, two
//! `f64`s at a time. The reads of a projector's samples (a gather) are done lane by lane
//! (`WaveReader::pressure_lanes`). Nothing here asks for a fused multiply-add (Rust never contracts `a * b + c`
//! into one without `mul_add`), and nothing changes the floating-point environment: subnormal numbers are computed
//! as IEEE 754 says on both (no flush to zero), as the scalar code does.
//!
//! The one operation without a baseline SIMD instruction on x86-64 is `floor` (SSE4.1 has `roundpd`; without it
//! `f64::floor` is a call into the runtime per value, which was the costliest part of the sum one path at a time):
//! [`floor_small`] computes it exactly with an addition and a subtraction for the times it is used on, and
//! [`whole`] makes the sample index of the result without a saturating conversion.
//!
//! `Hydrophone::lanes` switches this off (the sum one path at a time stays the reference); the tests compare the
//! two bit for bit (`f64::to_bits`) over random paths, sources and projector waveforms, moving and fixed ends, block
//! edges, gaps, and subnormal and zero values.

/// Paths computed together.
pub const LANES: usize = 8;

/// [`floor_small`] is exact below this (2^52: from there on every `f64` is a whole number).
pub const FLOOR_MAX: f64 = 4_503_599_627_370_496.0;

/// `x.floor()`, exactly, for `0 <= x < FLOOR_MAX` (`-0.0` gives `+0.0`, which `pressure`'s `floor() as u64 as
/// f64` also does). Adding 2^52 rounds `x` to a whole number (the `f64`s from 2^52 to 2^53 are one apart; to the
/// nearest, ties to even, the default rounding that Rust code always runs with), subtracting it again is exact,
/// and a result above `x` was rounded up, by exactly one. Branch-free, so it vectorizes on SSE2.
#[inline(always)]
pub fn floor_small(x: f64) -> f64 {
    let r = (x + FLOOR_MAX) - FLOOR_MAX;
    if r > x { r - 1.0 } else { r }
}

/// A whole number `0 <= x < FLOOR_MAX` as a `u64`. Added to 2^52, it is the low bits of the sum (exactly); this
/// is an addition and a subtraction of integers, where `x as u64` is a conversion that saturates, several
/// instructions on x86-64.
#[inline(always)]
pub fn whole(x: f64) -> u64 {
    (x + FLOOR_MAX).to_bits() - FLOOR_MAX.to_bits()
}

// The kernels, on whole lanes. Each is a function of its own, taking references, so that LLVM knows what it
// writes does not overlap what it reads (it then vectorizes them where it can't through iterators over vectors).

/// The times a step at `t` (s), at `f` through its block, reads its paths at: `t - (d0 + dd * f)`, as
/// `Hydrophone::raw` computes them.
#[inline(always)]
pub fn times(at: &mut [f64; LANES], d0: &[f64; LANES], dd: &[f64; LANES], t: f64, f: f64) {
    for j in 0..LANES {
        at[j] = t - (d0[j] + dd[j] * f);
    }
}

/// The paths' terms: their gains `g0 + dg * f` times the pressures `p`, as `Hydrophone::raw` computes them.
#[inline(always)]
pub fn terms(term: &mut [f64; LANES], g0: &[f64; LANES], dg: &[f64; LANES], p: &[f64; LANES], f: f64) {
    for j in 0..LANES {
        term[j] = (g0[j] + dg[j] * f) * p[j];
    }
}

/// For a projector's waveform at `rate` read at times `t` (`Wave::pressure`): the samples before them `k` (as
/// `f64`s) and how far past them they are, `f`. Returns whether every time is one this computes exactly
/// (`0 <= t`, `t * rate < FLOOR_MAX`: false for NaN too), and whether all of `k` are at least `first` and less
/// than `last`.
#[inline(always)]
pub fn split(
    t: &[f64; LANES],
    rate: f64,
    k: &mut [f64; LANES],
    f: &mut [f64; LANES],
    first: f64,
    last: f64,
) -> (bool, bool) {
    let (mut ok, mut there) = (true, true);
    for j in 0..LANES {
        let x = t[j] * rate;
        ok &= t[j] >= 0.0 && x < FLOOR_MAX;
        k[j] = floor_small(x);
        f[j] = x - k[j];
        there &= k[j] >= first && k[j] < last;
    }
    (ok, there)
}

/// Linear interpolation between `a` and `b` at `f` (into `f`): `a + (b - a) * f`, as `Wave::pressure` does it.
#[inline(always)]
pub fn interpolate(f: &mut [f64; LANES], a: &[f64; LANES], b: &[f64; LANES]) {
    for j in 0..LANES {
        f[j] = a[j] + (b[j] - a[j]) * f[j];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_small_is_floor() {
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        let check = |v: f64| {
            if (0.0..FLOOR_MAX).contains(&v) {
                assert_eq!(floor_small(v).to_bits(), ((v.floor() as u64) as f64).to_bits(), "{v:e}");
                assert_eq!(floor_small(v), v.floor(), "{v:e}");
                assert_eq!(whole(floor_small(v)), v.floor() as u64, "{v:e}");
            }
        };
        for v in [0.0, -0.0, f64::MIN_POSITIVE, 5e-324, 0.5, 1.0, 1.5, 2.5, 3.5, 0.999_999_999_999_999_9] {
            check(v);
        }
        for k in [1u64, 2, 3, 1_000_000, 123_456_789_012, (1 << 51) - 1, 1 << 51, (1 << 52) - 2, (1 << 52) - 1] {
            let k = k as f64;
            for v in [k, k.next_down(), k.next_up(), k - 0.5, k + 0.5, k + 0.25, k + 0.75] {
                check(v);
            }
        }
        for _ in 0..1_000_000 {
            let r = next();
            // Any bit pattern of a positive double, and times in seconds times the grid rate
            check(f64::from_bits(r >> 1));
            check((r >> 11) as f64 / (1u64 << 53) as f64 * 1e4 * 1e6);
            check((r % 100_000_000_000) as f64 + (next() >> 11) as f64 / (1u64 << 53) as f64);
        }
    }
}
