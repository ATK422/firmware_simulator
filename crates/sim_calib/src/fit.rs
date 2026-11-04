//! Deterministic least-squares fitting: Levenberg–Marquardt with a finite-difference Jacobian, and Nelder–Mead for
//! residuals that are not smooth enough for derivatives.
//!
//! Both work on named [`Param`]s with bounds and an optional log scale (for corner frequencies, drag coefficients
//! and other positive quantities spanning decades), minimising half the sum of squared residuals. The residual
//! function is pure (same parameters, same residuals), so a fit is a pure function of its inputs: no random
//! restarts, no wall clock. Jacobian columns are evaluated on threads (each a separate call of the same pure
//! function), which changes the wall time and nothing else.

/// One parameter of a model being fitted.
#[derive(Clone, Debug, PartialEq)]
pub struct Param {
    pub name: String,
    /// Starting value (and, after a fit, the fitted one).
    pub value: f64,
    pub lo: f64,
    pub hi: f64,
    /// Work on `ln(value)` (the value must stay positive): steps are relative.
    pub log: bool,
    /// Finite-difference step and Nelder–Mead's initial simplex size, in the working scale (a relative step for a
    /// log parameter).
    pub step: f64,
}

impl Param {
    pub fn new(name: &str, value: f64, step: f64) -> Param {
        Param { name: name.into(), value, lo: f64::NEG_INFINITY, hi: f64::INFINITY, log: false, step }
    }

    /// A positive parameter fitted on a log scale, `rel` the relative finite-difference step.
    pub fn positive(name: &str, value: f64, rel: f64) -> Param {
        Param { name: name.into(), value, lo: 0.0, hi: f64::INFINITY, log: true, step: rel }
    }

    pub fn bounded(mut self, lo: f64, hi: f64) -> Param {
        self.lo = lo;
        self.hi = hi;
        self
    }

    fn to_u(&self, v: f64) -> f64 {
        if self.log { v.max(1e-300).ln() } else { v }
    }

    fn of_u(&self, u: f64) -> f64 {
        let v = if self.log { u.exp() } else { u };
        v.clamp(if self.log { self.lo.max(1e-300) } else { self.lo }, self.hi)
    }
}

/// Which optimiser.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Method {
    #[default]
    LevenbergMarquardt,
    NelderMead,
}

impl std::str::FromStr for Method {
    type Err = String;
    fn from_str(s: &str) -> Result<Method, String> {
        match s {
            "lm" | "levenberg-marquardt" => Ok(Method::LevenbergMarquardt),
            "nm" | "nelder-mead" => Ok(Method::NelderMead),
            _ => Err(format!("{s:?}: expected lm or nelder-mead")),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Options {
    pub method: Method,
    pub max_iter: usize,
    /// Stop when an iteration lowers the cost by less than this fraction.
    pub rel_tol: f64,
    /// Evaluate Jacobian columns on threads.
    pub threads: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options { method: Method::LevenbergMarquardt, max_iter: 100, rel_tol: 1e-9, threads: true }
    }
}

/// The result of a fit.
#[derive(Clone, Debug)]
pub struct Fit {
    /// The parameters at their fitted values.
    pub params: Vec<Param>,
    /// Half the sum of squared residuals at the start and at the end.
    pub initial_cost: f64,
    pub cost: f64,
    pub residuals: Vec<f64>,
    pub iterations: usize,
    pub evaluations: usize,
    /// One standard error per parameter (in its own units, from the Jacobian at the solution and the residual
    /// variance; `NaN` when it cannot be told, e.g. a parameter the data do not constrain).
    pub std_err: Vec<f64>,
    /// Correlation between the parameters' estimates (rows and columns in parameter order).
    pub correlation: Vec<Vec<f64>>,
}

impl Fit {
    pub fn value(&self, name: &str) -> Option<f64> {
        self.params.iter().find(|p| p.name == name).map(|p| p.value)
    }

    /// Pairs of parameters whose estimates are almost perfectly correlated: the data cannot tell them apart.
    pub fn degenerate_pairs(&self, limit: f64) -> Vec<(String, String, f64)> {
        let mut out = Vec::new();
        for i in 0..self.params.len() {
            for j in i + 1..self.params.len() {
                let c = self.correlation[i][j];
                if c.abs() > limit {
                    out.push((self.params[i].name.clone(), self.params[j].name.clone(), c));
                }
            }
        }
        out
    }

    /// Root mean square of the residuals.
    pub fn rms(&self) -> f64 {
        rms(&self.residuals)
    }
}

pub fn rms(r: &[f64]) -> f64 {
    if r.is_empty() { 0.0 } else { (r.iter().map(|x| x * x).sum::<f64>() / r.len() as f64).sqrt() }
}

fn cost_of(r: &[f64]) -> f64 {
    let c = 0.5 * r.iter().map(|x| x * x).sum::<f64>();
    if c.is_finite() { c } else { f64::INFINITY }
}

/// Fits `params` to minimise the residuals `f` returns for their values (in the same order).
pub fn fit<F>(params: &[Param], f: F, opts: &Options) -> Fit
where
    F: Fn(&[f64]) -> Vec<f64> + Sync,
{
    let values = |u: &[f64]| -> Vec<f64> { params.iter().zip(u).map(|(p, &u)| p.of_u(u)).collect() };
    let u0: Vec<f64> = params.iter().map(|p| p.to_u(p.value)).collect();
    let mut evals = 0usize;
    let r0 = f(&values(&u0));
    evals += 1;
    let initial_cost = cost_of(&r0);
    let (u, r, iterations) = if params.is_empty() {
        (u0, r0, 0)
    } else {
        match opts.method {
            Method::LevenbergMarquardt => lm(params, &f, &values, u0, r0, opts, &mut evals),
            Method::NelderMead => nelder_mead(params, &f, &values, u0, r0, opts, &mut evals),
        }
    };
    let v = values(&u);
    let (std_err, correlation) = uncertainty(params, &f, &values, &u, &r, opts.threads, &mut evals);
    let mut out = params.to_vec();
    for (p, &x) in out.iter_mut().zip(&v) {
        p.value = x;
    }
    Fit {
        params: out,
        initial_cost,
        cost: cost_of(&r),
        residuals: r,
        iterations,
        evaluations: evals,
        std_err,
        correlation,
    }
}

/// The Jacobian of the residuals with respect to the working-scale parameters, by forward differences.
fn jacobian<F, V>(
    params: &[Param],
    f: &F,
    values: &V,
    u: &[f64],
    r: &[f64],
    threads: bool,
    evals: &mut usize,
) -> Vec<Vec<f64>>
where
    F: Fn(&[f64]) -> Vec<f64> + Sync,
    V: Fn(&[f64]) -> Vec<f64> + Sync,
{
    let column = |k: usize| -> Vec<f64> {
        let mut h = params[k].step;
        // Step towards the inside of the bounds
        let mut uk = u.to_vec();
        uk[k] += h;
        if params[k].of_u(uk[k]) == params[k].of_u(u[k]) {
            h = -h;
            uk[k] = u[k] + h;
        }
        let rk = f(&values(&uk));
        let actual = uk[k] - u[k];
        r.iter().zip(&rk).map(|(a, b)| if b.is_finite() { (b - a) / actual } else { 0.0 }).collect()
    };
    *evals += params.len();
    if threads && params.len() > 1 {
        let n = std::thread::available_parallelism().map_or(1, |n| n.get()).min(params.len());
        let mut cols: Vec<Option<Vec<f64>>> = vec![None; params.len()];
        std::thread::scope(|s| {
            let chunks: Vec<Vec<usize>> = (0..n).map(|t| (t..params.len()).step_by(n).collect()).collect();
            let handles: Vec<_> = chunks
                .into_iter()
                .map(|ks| {
                    let column = &column;
                    s.spawn(move || ks.into_iter().map(|k| (k, column(k))).collect::<Vec<_>>())
                })
                .collect();
            for h in handles {
                for (k, c) in h.join().unwrap() {
                    cols[k] = Some(c);
                }
            }
        });
        cols.into_iter().map(Option::unwrap).collect()
    } else {
        (0..params.len()).map(column).collect()
    }
}

/// Solves `a x = b` (Gaussian elimination with partial pivoting); `None` if singular.
pub fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-300 || !a[p][c].is_finite() {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        let (top, rest) = a.split_at_mut(c + 1);
        let pivot = &top[c];
        for (k, row) in rest.iter_mut().enumerate() {
            let m = row[c] / pivot[c];
            for (x, p) in row[c..].iter_mut().zip(&pivot[c..]) {
                *x -= m * p;
            }
            b[c + 1 + k] -= m * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let s: f64 = (i + 1..n).map(|j| a[i][j] * x[j]).sum();
        x[i] = (b[i] - s) / a[i][i];
    }
    Some(x)
}

/// `JᵀJ` and `Jᵀr` of a column-major Jacobian.
fn normal(j: &[Vec<f64>], r: &[f64]) -> (Vec<Vec<f64>>, Vec<f64>) {
    let n = j.len();
    let mut jtj = vec![vec![0.0; n]; n];
    let mut jtr = vec![0.0; n];
    for a in 0..n {
        for b in a..n {
            let s: f64 = j[a].iter().zip(&j[b]).map(|(x, y)| x * y).sum();
            jtj[a][b] = s;
            jtj[b][a] = s;
        }
        jtr[a] = j[a].iter().zip(r).map(|(x, y)| x * y).sum();
    }
    (jtj, jtr)
}

fn lm<F, V>(
    params: &[Param],
    f: &F,
    values: &V,
    mut u: Vec<f64>,
    mut r: Vec<f64>,
    opts: &Options,
    evals: &mut usize,
) -> (Vec<f64>, Vec<f64>, usize)
where
    F: Fn(&[f64]) -> Vec<f64> + Sync,
    V: Fn(&[f64]) -> Vec<f64> + Sync,
{
    let n = params.len();
    let mut cost = cost_of(&r);
    let mut lambda = 1e-3;
    let mut it = 0;
    while it < opts.max_iter {
        it += 1;
        let j = jacobian(params, f, values, &u, &r, opts.threads, evals);
        let (jtj, jtr) = normal(&j, &r);
        let mut improved = false;
        for _ in 0..12 {
            let mut a = jtj.clone();
            for k in 0..n {
                a[k][k] += lambda * jtj[k][k].max(1e-12);
            }
            let Some(delta) = solve(a, jtr.iter().map(|x| -x).collect()) else {
                lambda *= 10.0;
                continue;
            };
            // Keep the step inside the bounds (projected)
            let un: Vec<f64> = u.iter().zip(&delta).zip(params).map(|((&x, &d), p)| p.to_u(p.of_u(x + d))).collect();
            let rn = f(&values(&un));
            *evals += 1;
            let cn = cost_of(&rn);
            if cn < cost {
                let gain = (cost - cn) / cost.max(1e-300);
                u = un;
                r = rn;
                cost = cn;
                lambda = (lambda / 3.0).max(1e-12);
                improved = true;
                if gain < opts.rel_tol {
                    return (u, r, it);
                }
                break;
            }
            lambda *= 4.0;
        }
        if !improved || cost == 0.0 {
            break;
        }
    }
    (u, r, it)
}

fn nelder_mead<F, V>(
    params: &[Param],
    f: &F,
    values: &V,
    u0: Vec<f64>,
    r0: Vec<f64>,
    opts: &Options,
    evals: &mut usize,
) -> (Vec<f64>, Vec<f64>, usize)
where
    F: Fn(&[f64]) -> Vec<f64> + Sync,
    V: Fn(&[f64]) -> Vec<f64> + Sync,
{
    let n = params.len();
    let mut eval = |u: &[f64]| -> (f64, Vec<f64>) {
        *evals += 1;
        let r = f(&values(u));
        (cost_of(&r), r)
    };
    let mut simplex: Vec<(Vec<f64>, f64, Vec<f64>)> = vec![(u0.clone(), cost_of(&r0), r0)];
    for k in 0..n {
        let mut u = u0.clone();
        u[k] += params[k].step * 5.0;
        let (c, r) = eval(&u);
        simplex.push((u, c, r));
    }
    let max_iter = opts.max_iter * (n + 1) * 4;
    let mut it = 0;
    while it < max_iter {
        it += 1;
        simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (best, worst) = (simplex[0].1, simplex[n].1);
        if (worst - best).abs() <= opts.rel_tol * best.abs().max(1e-300) {
            break;
        }
        let centroid: Vec<f64> = (0..n).map(|k| simplex[..n].iter().map(|s| s.0[k]).sum::<f64>() / n as f64).collect();
        let along =
            |t: f64| -> Vec<f64> { (0..n).map(|k| centroid[k] + t * (simplex[n].0[k] - centroid[k])).collect() };
        let ur = along(-1.0);
        let (cr, rr) = eval(&ur);
        if cr < simplex[0].1 {
            let ue = along(-2.0);
            let (ce, re) = eval(&ue);
            simplex[n] = if ce < cr { (ue, ce, re) } else { (ur, cr, rr) };
        } else if cr < simplex[n - 1].1 {
            simplex[n] = (ur, cr, rr);
        } else {
            let uc = if cr < simplex[n].1 { along(-0.5) } else { along(0.5) };
            let (cc, rc) = eval(&uc);
            if cc < simplex[n].1.min(cr) {
                simplex[n] = (uc, cc, rc);
            } else {
                let b = simplex[0].0.clone();
                for s in simplex.iter_mut().skip(1) {
                    let us: Vec<f64> = (0..n).map(|k| b[k] + 0.5 * (s.0[k] - b[k])).collect();
                    let (c, r) = eval(&us);
                    *s = (us, c, r);
                }
            }
        }
    }
    simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
    let (u, _, r) = simplex.swap_remove(0);
    (u, r, it)
}

/// Standard errors and correlations at the solution, from `(JᵀJ)⁻¹ s²`.
fn uncertainty<F, V>(
    params: &[Param],
    f: &F,
    values: &V,
    u: &[f64],
    r: &[f64],
    threads: bool,
    evals: &mut usize,
) -> (Vec<f64>, Vec<Vec<f64>>)
where
    F: Fn(&[f64]) -> Vec<f64> + Sync,
    V: Fn(&[f64]) -> Vec<f64> + Sync,
{
    let n = params.len();
    let nan = (vec![f64::NAN; n], vec![vec![f64::NAN; n]; n]);
    if n == 0 || r.len() <= n {
        return nan;
    }
    let j = jacobian(params, f, values, u, r, threads, evals);
    let (jtj, _) = normal(&j, r);
    let s2 = 2.0 * cost_of(r) / (r.len() - n) as f64;
    let mut cov = vec![vec![0.0; n]; n];
    for k in 0..n {
        let mut e = vec![0.0; n];
        e[k] = 1.0;
        let Some(col) = solve(jtj.clone(), e) else { return nan };
        for i in 0..n {
            cov[i][k] = col[i] * s2;
        }
    }
    let v = values(u);
    // From the working scale to the parameter's own units (d value / d ln value = value)
    let scale: Vec<f64> = params.iter().zip(&v).map(|(p, &x)| if p.log { x } else { 1.0 }).collect();
    let se: Vec<f64> = (0..n).map(|k| cov[k][k].max(0.0).sqrt() * scale[k]).collect();
    let corr =
        (0..n).map(|i| (0..n).map(|k| cov[i][k] / (cov[i][i] * cov[k][k]).sqrt().max(1e-300)).collect()).collect();
    (se, corr)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// y = a exp(-b x) + c, sampled without noise.
    fn decay(p: &[f64]) -> Vec<f64> {
        (0..40).map(|i| i as f64 * 0.1).map(|x| p[0] * (-p[1] * x).exp() + p[2]).collect()
    }

    #[test]
    fn both_methods_recover_a_nonlinear_model() {
        let truth = [2.5, 1.3, 0.4];
        let data = decay(&truth);
        let params = [
            Param::new("a", 1.0, 1e-4),
            Param::positive("b", 0.5, 1e-4).bounded(0.01, 10.0),
            Param::new("c", 0.0, 1e-4),
        ];
        let resid = |p: &[f64]| -> Vec<f64> { decay(p).iter().zip(&data).map(|(m, d)| m - d).collect() };
        for method in [Method::LevenbergMarquardt, Method::NelderMead] {
            let opts = Options { method, max_iter: 400, rel_tol: 1e-14, ..Default::default() };
            let f = fit(&params, resid, &opts);
            for (p, t) in f.params.iter().zip(truth) {
                assert!((p.value - t).abs() < 1e-3, "{method:?} {}: {} vs {t}", p.name, p.value);
            }
            assert!(f.cost < f.initial_cost * 1e-6, "{method:?}");
        }
    }

    #[test]
    fn fits_are_deterministic_and_report_uncertainty() {
        // A straight line with deterministic "noise": the standard errors match the textbook formula
        let x: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|x| 3.0 * x + 1.0 + 0.5 * ((x * 1.7).sin())).collect();
        let resid = |p: &[f64]| -> Vec<f64> { x.iter().zip(&y).map(|(x, y)| p[0] * x + p[1] - y).collect() };
        let params = [Param::new("m", 0.0, 1e-3), Param::new("q", 0.0, 1e-3)];
        let a = fit(&params, resid, &Options::default());
        let b = fit(&params, resid, &Options { threads: false, ..Default::default() });
        assert_eq!(a.params, b.params);
        assert_eq!(a.residuals, b.residuals);
        let n = x.len() as f64;
        let mx = x.iter().sum::<f64>() / n;
        let sxx: f64 = x.iter().map(|v| (v - mx).powi(2)).sum();
        let s2 = 2.0 * a.cost / (n - 2.0);
        assert!((a.std_err[0] - (s2 / sxx).sqrt()).abs() < 1e-6 * a.std_err[0].max(1e-9) + 1e-9);
        // Slope and intercept of a line far from the origin are anticorrelated
        assert!(a.correlation[0][1] < -0.5);
    }

    #[test]
    fn degenerate_parameters_are_flagged_and_bounds_hold() {
        // Only the sum of a and b matters
        let resid = |p: &[f64]| -> Vec<f64> { (0..10).map(|i| p[0] + p[1] - 3.0 + 0.0 * i as f64).collect() };
        let params = [Param::new("a", 0.0, 1e-3), Param::new("b", 0.0, 1e-3).bounded(-1.0, 1.0)];
        let f = fit(&params, resid, &Options::default());
        assert!((f.params[0].value + f.params[1].value - 3.0).abs() < 1e-6);
        assert!(f.params[1].value <= 1.0);
        // The normal matrix is singular: no standard errors
        assert!(f.std_err.iter().all(|s| s.is_nan()) || !f.degenerate_pairs(0.95).is_empty());
    }

    #[test]
    fn solve_handles_pivoting() {
        let x = solve(vec![vec![0.0, 1.0], vec![2.0, 1.0]], vec![1.0, 4.0]).unwrap();
        assert!((x[0] - 1.5).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12);
        assert!(solve(vec![vec![1.0, 2.0], vec![2.0, 4.0]], vec![1.0, 2.0]).is_none());
    }
}
