//! Numeric CSV tables (the calibration inputs), and small writers for the outputs: Markdown tables and SVG plots.

use std::fmt::Write as _;
use std::path::Path;

/// A CSV file of numbers with a header row: columns by name, rows of values (an empty cell is `NaN`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Table {
    pub columns: Vec<String>,
    pub rows: Vec<Vec<f64>>,
}

impl Table {
    /// Parses CSV text: the first non-comment line is the header; `#` lines and blank lines are skipped; values
    /// are separated by commas (or tabs or semicolons, as spreadsheets export them).
    pub fn parse(text: &str) -> Result<Table, String> {
        let mut columns: Option<Vec<String>> = None;
        let mut rows = Vec::new();
        for (i, raw) in text.lines().enumerate() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let sep = if line.contains(',') {
                ','
            } else if line.contains('\t') {
                '\t'
            } else {
                ';'
            };
            let cells: Vec<&str> = line.split(sep).map(str::trim).collect();
            match &columns {
                None => columns = Some(cells.iter().map(|c| c.trim_matches('"').to_string()).collect()),
                Some(cols) => {
                    if cells.len() != cols.len() {
                        return Err(format!("line {}: {} values for {} columns", i + 1, cells.len(), cols.len()));
                    }
                    let row = cells
                        .iter()
                        .map(|c| {
                            if c.is_empty() || c.eq_ignore_ascii_case("nan") {
                                Ok(f64::NAN)
                            } else {
                                c.parse::<f64>().map_err(|_| format!("line {}: {c:?} is not a number", i + 1))
                            }
                        })
                        .collect::<Result<Vec<f64>, String>>()?;
                    rows.push(row);
                }
            }
        }
        Ok(Table { columns: columns.ok_or("no header line")?, rows })
    }

    pub fn load(path: &Path) -> Result<Table, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn index(&self, name: &str) -> Option<usize> {
        self.columns.iter().position(|c| c == name)
    }

    /// A column's values, if the table has it.
    pub fn col(&self, name: &str) -> Option<Vec<f64>> {
        let i = self.index(name)?;
        Some(self.rows.iter().map(|r| r[i]).collect())
    }

    /// The first of these columns the table has.
    pub fn any_col(&self, names: &[&str]) -> Option<(String, Vec<f64>)> {
        names.iter().find_map(|n| self.col(n).map(|c| (n.to_string(), c)))
    }

    /// The table as CSV text, values with `digits` decimals.
    pub fn to_csv(&self, digits: usize) -> String {
        let mut o = self.columns.join(",") + "\n";
        for r in &self.rows {
            let cells: Vec<String> =
                r.iter().map(|v| if v.is_nan() { String::new() } else { format!("{v:.digits$}") }).collect();
            o += &cells.join(",");
            o.push('\n');
        }
        o
    }

    /// Concatenates tables with the same columns.
    pub fn concat(tables: Vec<Table>) -> Result<Table, String> {
        let mut it = tables.into_iter();
        let mut out = it.next().ok_or("no tables")?;
        for t in it {
            if t.columns != out.columns {
                return Err(format!("columns differ: {:?} and {:?}", out.columns, t.columns));
            }
            out.rows.extend(t.rows);
        }
        Ok(out)
    }
}

/// A Markdown table.
pub fn markdown(header: &[&str], rows: &[Vec<String>]) -> String {
    let mut o = format!("| {} |\n|{}\n", header.join(" | "), "---|".repeat(header.len()));
    for r in rows {
        let _ = writeln!(o, "| {} |", r.join(" | "));
    }
    o
}

/// One line of a plot.
pub struct Series {
    pub name: String,
    pub points: Vec<(f64, f64)>,
    /// Draw dots instead of a line (measurements).
    pub dots: bool,
}

impl Series {
    pub fn line(name: &str, points: Vec<(f64, f64)>) -> Series {
        Series { name: name.into(), points, dots: false }
    }

    pub fn dots(name: &str, points: Vec<(f64, f64)>) -> Series {
        Series { name: name.into(), points, dots: true }
    }
}

const COLOURS: [&str; 6] = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"];

fn nice_ticks(lo: f64, hi: f64) -> Vec<f64> {
    let span = (hi - lo).max(1e-12);
    let raw = span / 5.0;
    let mag = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| span / s <= 6.0).unwrap_or(10.0 * mag);
    let mut t = (lo / step).ceil() * step;
    let mut out = Vec::new();
    while t <= hi + step * 1e-9 {
        out.push(if t.abs() < step * 1e-9 { 0.0 } else { t });
        t += step;
    }
    out
}

/// A plot as an SVG document: the series, axes with ticks, a legend. `log_x` puts x on a log scale.
pub fn svg_plot(title: &str, x_label: &str, y_label: &str, series: &[Series], log_x: bool) -> String {
    let (w, h, ml, mr, mt, mb) = (640.0, 400.0, 70.0, 20.0, 40.0, 50.0);
    let tx = |x: f64| if log_x { x.max(1e-12).log10() } else { x };
    let pts: Vec<(f64, f64)> =
        series.iter().flat_map(|s| s.points.iter().copied()).filter(|(x, y)| x.is_finite() && y.is_finite()).collect();
    let (mut x0, mut x1, mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y) in &pts {
        x0 = x0.min(tx(x));
        x1 = x1.max(tx(x));
        y0 = y0.min(y);
        y1 = y1.max(y);
    }
    if pts.is_empty() {
        (x0, x1, y0, y1) = (0.0, 1.0, 0.0, 1.0);
    }
    if x1 <= x0 {
        x1 = x0 + 1.0;
    }
    if y1 <= y0 {
        y1 = y0 + 1.0;
    }
    let pad = (y1 - y0) * 0.05;
    let (y0, y1) = (y0 - pad, y1 + pad);
    let px = |x: f64| ml + (tx(x) - x0) / (x1 - x0) * (w - ml - mr);
    let py = |y: f64| h - mb - (y - y0) / (y1 - y0) * (h - mt - mb);
    let mut o = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\" \
         font-family=\"sans-serif\" font-size=\"12\">\n<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n\
         <text x=\"{}\" y=\"20\" text-anchor=\"middle\" font-size=\"14\">{}</text>\n",
        w / 2.0,
        escape(title)
    );
    let _ = writeln!(
        o,
        "<rect x=\"{ml}\" y=\"{mt}\" width=\"{}\" height=\"{}\" fill=\"none\" stroke=\"#444\"/>",
        w - ml - mr,
        h - mt - mb
    );
    let xticks: Vec<f64> = if log_x {
        let mut v = Vec::new();
        let mut d = x0.floor();
        while d <= x1 {
            for m in [1.0, 2.0, 5.0] {
                let l = d + f64::log10(m);
                if l >= x0 && l <= x1 {
                    v.push(10f64.powf(l));
                }
            }
            d += 1.0;
        }
        v
    } else {
        nice_ticks(x0, x1)
    };
    for t in xticks {
        let x = px(t);
        let _ = writeln!(
            o,
            "<line x1=\"{x:.1}\" y1=\"{mt}\" x2=\"{x:.1}\" y2=\"{}\" stroke=\"#ddd\"/><text x=\"{x:.1}\" y=\"{}\" \
             text-anchor=\"middle\">{}</text>",
            h - mb,
            h - mb + 15.0,
            fmt_tick(t)
        );
    }
    for t in nice_ticks(y0, y1) {
        let y = py(t);
        let _ = writeln!(
            o,
            "<line x1=\"{ml}\" y1=\"{y:.1}\" x2=\"{}\" y2=\"{y:.1}\" stroke=\"#ddd\"/><text x=\"{}\" y=\"{:.1}\" \
             text-anchor=\"end\">{}</text>",
            w - mr,
            ml - 5.0,
            y + 4.0,
            fmt_tick(t)
        );
    }
    let _ = writeln!(
        o,
        "<text x=\"{}\" y=\"{}\" text-anchor=\"middle\">{}</text>\n<text x=\"15\" y=\"{}\" text-anchor=\"middle\" \
         transform=\"rotate(-90 15 {})\">{}</text>",
        (ml + w - mr) / 2.0,
        h - 12.0,
        escape(x_label),
        (mt + h - mb) / 2.0,
        (mt + h - mb) / 2.0,
        escape(y_label)
    );
    for (k, s) in series.iter().enumerate() {
        let c = COLOURS[k % COLOURS.len()];
        let p: Vec<(f64, f64)> = s.points.iter().copied().filter(|(x, y)| x.is_finite() && y.is_finite()).collect();
        if s.dots {
            for (x, y) in p {
                let _ = writeln!(o, "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"2.5\" fill=\"{c}\"/>", px(x), py(y));
            }
        } else if !p.is_empty() {
            let path: Vec<String> = p.iter().map(|&(x, y)| format!("{:.1},{:.1}", px(x), py(y))).collect();
            let _ = writeln!(
                o,
                "<polyline fill=\"none\" stroke=\"{c}\" stroke-width=\"1.5\" points=\"{}\"/>",
                path.join(" ")
            );
        }
        let ly = mt + 15.0 + 16.0 * k as f64;
        let _ = writeln!(
            o,
            "<rect x=\"{}\" y=\"{}\" width=\"12\" height=\"3\" fill=\"{c}\"/><text x=\"{}\" y=\"{}\">{}</text>",
            ml + 10.0,
            ly - 4.0,
            ml + 27.0,
            ly,
            escape(&s.name)
        );
    }
    o += "</svg>\n";
    o
}

fn fmt_tick(t: f64) -> String {
    if t.abs() >= 1e4 || (t != 0.0 && t.abs() < 1e-2) {
        format!("{t:.0e}")
    } else if t.fract() == 0.0 {
        format!("{t:.0}")
    } else {
        format!("{t:.2}").trim_end_matches('0').to_string()
    }
}

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csv_parses_with_comments_blanks_and_separators() {
        let t = Table::parse("# a comment\ntime_s,rpm,thrust_n\n0,100,1.5\n\n0.1,,2\n").unwrap();
        assert_eq!(t.columns, ["time_s", "rpm", "thrust_n"]);
        assert_eq!(t.col("time_s").unwrap(), [0.0, 0.1]);
        assert!(t.col("rpm").unwrap()[1].is_nan());
        assert!(t.col("nope").is_none());
        assert_eq!(t.any_col(&["x", "thrust_n"]).unwrap().0, "thrust_n");
        let t2 = Table::parse("a;b\n1;2\n").unwrap();
        assert_eq!(t2.rows, [[1.0, 2.0]]);
        assert!(Table::parse("a,b\n1\n").unwrap_err().contains("line 2"));
        assert!(Table::parse("a,b\n1,x\n").unwrap_err().contains("not a number"));
        let round = Table::parse(&t.to_csv(3)).unwrap();
        assert_eq!(round.columns, t.columns);
        assert_eq!(round.rows[0], t.rows[0]);
    }

    #[test]
    fn plots_are_well_formed_svg() {
        let s = svg_plot(
            "T <1>",
            "rpm",
            "N",
            &[
                Series::dots("measured", vec![(1.0, 2.0), (10.0, 3.0)]),
                Series::line("fit", vec![(1.0, 2.1), (1e4, 5.0)]),
            ],
            true,
        );
        assert!(s.starts_with("<svg") && s.trim_end().ends_with("</svg>"));
        assert!(s.contains("T &lt;1&gt;") && s.contains("<polyline") && s.contains("<circle"));
        assert_eq!(nice_ticks(0.0, 10.0), [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
        assert!(markdown(&["a", "b"], &[vec!["1".into(), "2".into()]]).contains("| 1 | 2 |"));
    }
}
