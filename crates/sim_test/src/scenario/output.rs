//! What a run of scenarios says: lines for a person, JUnit XML for CI, JSON for scripts.

use std::fmt::Write as _;

use super::run::{Outcome, Status};

/// One scenario's result as text: a line per step (✓, ✗ or · for not reached) with its virtual time and its
/// evidence, then PASS or FAIL.
pub fn human(o: &Outcome) -> String {
    let mut s = String::new();
    let doc = if o.document > 0 { format!(" #{}", o.document + 1) } else { String::new() };
    let _ = writeln!(s, "● {}  ({}{doc})", o.title(), o.file.display());
    if let Some(why) = &o.skipped {
        let _ = writeln!(s, "  SKIPPED: {why}");
        return s;
    }
    for r in &o.steps {
        let mark = match r.status {
            Status::Pass => "✓",
            Status::Fail => "✗",
            Status::NotReached => "·",
        };
        let t = r.time.map_or_else(|| "-".to_string(), |t| t.to_string());
        let _ = writeln!(s, "  {mark} {t:>14}  {}", r.description);
        // A passing step shows what decided it; a failing one everything it has
        let n = if r.status == Status::Pass && r.kind != "pause" { 1 } else { usize::MAX };
        for e in r.evidence.iter().filter(|e| !e.is_empty()).take(n) {
            let e = e.trim_end().replace('\n', &format!("\n  {:>16}      ", ""));
            let _ = writeln!(s, "  {:>16}    {e}", "");
        }
    }
    if let Some(e) = &o.error {
        let _ = writeln!(s, "  ERROR: {e}");
    }
    let failed = o.steps.iter().filter(|r| r.status != Status::Pass).count();
    let verdict = if o.passed() { "PASS".to_string() } else { format!("FAIL ({failed} of {} steps)", o.steps.len()) };
    let _ = writeln!(s, "  {verdict}: {} of virtual time in {:.2?} wall", o.end, o.wall);
    s
}

/// The line under all the scenarios: how many passed, failed and were skipped.
pub fn summary(all: &[Outcome]) -> String {
    let passed = all.iter().filter(|o| o.skipped.is_none() && o.passed()).count();
    let skipped = all.iter().filter(|o| o.skipped.is_some()).count();
    let failed: Vec<String> = all.iter().filter(|o| o.skipped.is_none() && !o.passed()).map(Outcome::title).collect();
    let mut s = format!("{} scenario(s): {passed} passed, {} failed, {skipped} skipped", all.len(), failed.len());
    if !failed.is_empty() {
        s += &format!(" ({})", failed.join("; "));
    }
    s
}

fn xml(s: &str) -> String {
    s.chars()
        .filter(|c| !c.is_control() || *c == '\n' || *c == '\t')
        .map(|c| match c {
            '&' => "&amp;".into(),
            '<' => "&lt;".into(),
            '>' => "&gt;".into(),
            '"' => "&quot;".into(),
            '\'' => "&apos;".into(),
            c => c.to_string(),
        })
        .collect()
}

/// JUnit XML: a test suite per file, a test case per scenario (or case), a `<failure>` naming the failed steps and
/// the whole step list as its output.
pub fn junit(all: &[Outcome]) -> String {
    let mut files: Vec<&std::path::Path> = Vec::new();
    for o in all {
        if !files.contains(&o.file.as_path()) {
            files.push(&o.file);
        }
    }
    let count = |os: &[&Outcome]| {
        let fail = os.iter().filter(|o| o.skipped.is_none() && o.error.is_none() && !o.passed()).count();
        let err = os.iter().filter(|o| o.error.is_some()).count();
        let skip = os.iter().filter(|o| o.skipped.is_some()).count();
        let wall: f64 = os.iter().map(|o| o.wall.as_secs_f64()).sum();
        (fail, err, skip, wall)
    };
    let refs: Vec<&Outcome> = all.iter().collect();
    let (f, e, sk, w) = count(&refs);
    let mut s = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    let _ = writeln!(
        s,
        "<testsuites name=\"firmware_simulator scenarios\" tests=\"{}\" failures=\"{f}\" errors=\"{e}\" skipped=\"{sk}\" time=\"{w:.3}\">",
        all.len()
    );
    for file in files {
        let os: Vec<&Outcome> = all.iter().filter(|o| o.file == file).collect();
        let (f, e, sk, w) = count(&os);
        let suite = file.display().to_string();
        let _ = writeln!(
            s,
            "  <testsuite name=\"{}\" tests=\"{}\" failures=\"{f}\" errors=\"{e}\" skipped=\"{sk}\" time=\"{w:.3}\">",
            xml(&suite),
            os.len()
        );
        let class = file.with_extension("").to_string_lossy().replace(['/', '\\'], ".");
        for o in os {
            let _ = write!(
                s,
                "    <testcase classname=\"{}\" name=\"{}\" time=\"{:.3}\">",
                xml(&class),
                xml(&o.title()),
                o.wall.as_secs_f64()
            );
            if let Some(why) = &o.skipped {
                let _ = write!(s, "<skipped message=\"{}\"/>", xml(why));
            } else if let Some(err) = &o.error {
                let _ = write!(s, "\n      <error message=\"{}\"/>", xml(err));
            } else if !o.passed() {
                let failed: Vec<String> = o
                    .steps
                    .iter()
                    .filter(|r| r.status != Status::Pass)
                    .map(|r| format!("step {}: {}", r.number, r.description))
                    .collect();
                let _ = write!(s, "\n      <failure message=\"{}\"/>", xml(&failed.join("; ")));
            }
            if o.skipped.is_none() {
                let _ = write!(s, "\n      <system-out>{}</system-out>\n    ", xml(&human(o)));
            }
            s.push_str("</testcase>\n");
        }
        s.push_str("  </testsuite>\n");
    }
    s.push_str("</testsuites>\n");
    s
}

/// One outcome as JSON. Everything but `wall_s` is the same on every run of the same files and images.
pub fn json_one(o: &Outcome) -> serde_json::Value {
    serde_json::json!({
        "name": o.name,
        "case": o.case,
        "file": o.file.display().to_string(),
        "document": o.document,
        "passed": o.skipped.is_none() && o.passed(),
        "skipped": o.skipped,
        "error": o.error,
        "end_s": o.end.as_secs_f64(),
        "wall_s": o.wall.as_secs_f64(),
        "images": o.images.iter().map(|(b, p)| serde_json::json!({"board": b, "image": p.display().to_string()})).collect::<Vec<_>>(),
        "steps": o.steps,
    })
}

/// Every outcome as JSON, with the totals.
pub fn json(all: &[Outcome]) -> serde_json::Value {
    let passed = all.iter().filter(|o| o.skipped.is_none() && o.passed()).count();
    let skipped = all.iter().filter(|o| o.skipped.is_some()).count();
    serde_json::json!({
        "passed": passed,
        "failed": all.len() - passed - skipped,
        "skipped": skipped,
        "scenarios": all.iter().map(json_one).collect::<Vec<_>>(),
    })
}
