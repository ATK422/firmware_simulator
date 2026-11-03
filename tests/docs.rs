//! Keeps the guide (`docs/*.md`) and the README honest about the command line and about themselves: every
//! `firmware_simulator` command they show uses options its subcommand has, every other `--option` they mention is
//! one of the emulator's or of a known outside tool, and every relative link points at a file (and heading) that
//! exists.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// The guide's pages and the README, with their text.
fn pages() -> Vec<(PathBuf, String)> {
    let mut paths = vec![repo().join("README.md")];
    let mut docs: Vec<PathBuf> = std::fs::read_dir(repo().join("docs"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "md"))
        .collect();
    docs.sort();
    paths.extend(docs);
    paths.into_iter().map(|p| (p.clone(), std::fs::read_to_string(&p).unwrap())).collect()
}

/// `--long` options in a text.
fn options(text: &str) -> BTreeSet<String> {
    let b = text.as_bytes();
    let mut out = BTreeSet::new();
    let mut i = 0;
    while let Some(at) = text[i..].find("--") {
        let start = i + at;
        let mut end = start + 2;
        while end < b.len() && (b[end].is_ascii_lowercase() || b[end].is_ascii_digit() || b[end] == b'-') {
            end += 1;
        }
        let before_ok = start == 0 || !(b[start - 1].is_ascii_alphanumeric() || b[start - 1] == b'-');
        if before_ok && end > start + 2 && b[start + 2].is_ascii_lowercase() {
            out.insert(text[start..end].trim_end_matches('-').to_string());
        }
        i = end.max(start + 2);
    }
    out
}

/// The options `firmware_simulator SUBCOMMAND --help` lists.
fn help_options(sub: &[&str]) -> BTreeSet<String> {
    let out = Command::new(env!("CARGO_BIN_EXE_firmware_simulator")).args(sub).arg("--help").output().unwrap();
    assert!(out.status.success(), "{sub:?} --help");
    options(&String::from_utf8_lossy(&out.stdout))
}

/// Options of other programs the pages mention, with whose they are.
const OUTSIDE: &[(&str, &str)] = &[
    ("--release", "cargo"),
    ("--workspace", "cargo"),
    ("--all", "cargo fmt"),
    ("--all-targets", "cargo clippy"),
    ("--example", "cargo"),
    ("--test", "cargo"),
    ("--nocapture", "the test harness"),
    ("--init", "git submodule"),
    ("--recursive", "git submodule"),
    ("--interface", "titan_bridge"),
    ("--client-id", "titan_bridge"),
    ("--check", "scripts/gen_boards.py, cargo fmt"),
    ("--down", "scripts/vcan_up.sh"),
    ("--tool", "valgrind"),
    ("--repeat", "the bench example"),
];

/// A `firmware_simulator` command shown on a page: its subcommand and the text of its arguments (continuation
/// lines included).
fn commands(text: &str) -> Vec<(String, String)> {
    let lines: Vec<&str> = text.lines().collect();
    let mut out = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        for (at, _) in line.match_indices("firmware_simulator ") {
            let rest = &line[at + "firmware_simulator ".len()..];
            let sub = rest.split_whitespace().next().unwrap_or("");
            let sub = sub.trim_matches(|c: char| !c.is_ascii_alphanumeric() && c != '-');
            if !["run", "robot", "can-diff", "bridges", "can-tunnel", "picoboot"].contains(&sub) {
                continue;
            }
            // Up to the end of the code span or the line, with `\` continuations
            let mut args = rest.split('`').next().unwrap().to_string();
            let mut j = i;
            while args.trim_end().ends_with('\\') && j + 1 < lines.len() {
                j += 1;
                args.push(' ');
                args.push_str(lines[j]);
            }
            // A trailing comment is not part of the command
            let args = args.split(" #").next().unwrap().to_string();
            out.push((sub.to_string(), args));
        }
    }
    out
}

#[test]
fn commands_use_options_their_subcommand_has() {
    let run = help_options(&["run"]);
    let mut picoboot = help_options(&["picoboot", "load"]);
    picoboot.extend(run.iter().cloned());
    let by_sub = [
        ("run", run),
        ("robot", help_options(&["robot"])),
        ("can-diff", help_options(&["can-diff"])),
        ("bridges", help_options(&["bridges"])),
        ("can-tunnel", help_options(&["can-tunnel"])),
        ("picoboot", picoboot),
    ];
    let mut checked = 0;
    let mut wrong = Vec::new();
    for (path, text) in pages() {
        for (sub, args) in commands(&text) {
            let known = &by_sub.iter().find(|(s, _)| *s == sub).unwrap().1;
            for opt in options(&args) {
                checked += 1;
                if !known.contains(&opt) {
                    wrong.push(format!("{}: `firmware_simulator {sub}` has no {opt}: {args}", path.display()));
                }
            }
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
    assert!(checked > 50, "only {checked} options checked: is the command pattern still right?");
}

#[test]
fn every_option_mentioned_exists() {
    let mut known = BTreeSet::new();
    for sub in [&["run"][..], &["robot"], &["can-diff"], &["bridges"], &["can-tunnel"], &["picoboot", "load"]] {
        known.extend(help_options(sub));
    }
    known.extend(OUTSIDE.iter().map(|(o, _)| o.to_string()));
    let mut unknown = Vec::new();
    for (path, text) in pages() {
        for opt in options(&text) {
            if !known.contains(&opt) {
                unknown.push(format!("{}: {opt}", path.display()));
            }
        }
    }
    assert!(unknown.is_empty(), "options that are neither the emulator's nor in OUTSIDE:\n{}", unknown.join("\n"));
}

/// GitHub's anchor for a heading.
fn slug(heading: &str) -> String {
    heading
        .trim()
        .to_lowercase()
        .chars()
        .filter(|c| c.is_alphanumeric() || *c == ' ' || *c == '-' || *c == '_')
        .map(|c| if c == ' ' { '-' } else { c })
        .collect()
}

fn anchors(text: &str) -> BTreeSet<String> {
    let mut in_code = false;
    let mut out = BTreeSet::new();
    for line in text.lines() {
        if line.starts_with("```") {
            in_code = !in_code;
        } else if !in_code && line.starts_with('#') {
            out.insert(slug(line.trim_start_matches('#')));
        }
    }
    out
}

#[test]
fn relative_links_resolve() {
    let mut broken = Vec::new();
    for (path, text) in pages() {
        let dir = path.parent().unwrap();
        for (at, _) in text.match_indices("](") {
            let rest = &text[at + 2..];
            let target = &rest[..rest.find(')').unwrap()];
            if target.starts_with("http") {
                continue;
            }
            let (file, anchor) = match target.split_once('#') {
                Some((f, a)) => (f, Some(a)),
                None => (target, None),
            };
            let file_path: PathBuf = if file.is_empty() { path.clone() } else { dir.join(file) };
            if !file_path.exists() {
                broken.push(format!("{}: {target} (no such file)", path.display()));
                continue;
            }
            if let Some(a) = anchor
                && file_path.extension().is_some_and(|x| x == "md")
            {
                let t = std::fs::read_to_string(&file_path).unwrap();
                if !anchors(&t).contains(a) {
                    broken.push(format!("{}: {target} (no such heading)", path.display()));
                }
            }
        }
    }
    assert!(broken.is_empty(), "{}", broken.join("\n"));
    assert!(Path::new(&repo().join("docs/README.md")).exists());
}
