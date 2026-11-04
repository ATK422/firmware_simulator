//! Host bridges: RTT and defmt (M1), the GDB remote protocol (M1), SocketCAN and UDP (M3, M7), UART ptys; and the
//! host side of CAN testing: candump logs and replay ([`canlog`]), comparing logs ([`candiff`]), tunnels over UDP
//! ([`cannelloni`]) and titan_bridge command lines for a robot ([`bridges`]).
//!
//! Bridges see the target through [`TargetMemory`] (debug-probe style memory access) so they work with any machine.

pub mod acoustic;

/// Where the titan_firmware_rs release images are: `$TITAN_RS_RELEASE_DIR` if set (a worktree, another checkout of
/// titan_firmware_rs, CI), else `repo/external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release`.
pub fn titan_rs_release_dir(repo: &std::path::Path) -> std::path::PathBuf {
    match std::env::var_os("TITAN_RS_RELEASE_DIR") {
        Some(d) => d.into(),
        None => repo.join(RS_RELEASE_SUFFIX),
    }
}

/// The titan_firmware_rs release image `name` ([`titan_rs_release_dir`]), for a test: `None` when it has not been
/// built, which the test takes as a reason to skip (saying so), unless [`titan_rs_required`], when it panics.
pub fn titan_rs_image(repo: &std::path::Path, name: &str) -> Option<std::path::PathBuf> {
    let p = titan_rs_release_dir(repo).join(name);
    if p.exists() {
        return Some(p);
    }
    let how =
        "cd external/titan_firmware_rs/firmware && cargo build --workspace --release, or set TITAN_RS_RELEASE_DIR";
    if titan_rs_required() {
        panic!("{} not built ({how}); SIM_REQUIRE_TITAN_RS is set, so this fails instead of skipping", p.display());
    }
    eprintln!("skipping: {} not built ({how})", p.display());
    None
}

/// Whether a test fails instead of skipping when a titan_firmware_rs image is missing: `SIM_REQUIRE_TITAN_RS` is set
/// to anything but empty or `0`. titan_firmware_rs's CI sets it, so missing images cannot pass as skipped tests.
pub fn titan_rs_required() -> bool {
    required(std::env::var_os("SIM_REQUIRE_TITAN_RS").as_deref())
}

fn required(v: Option<&std::ffi::OsStr>) -> bool {
    v.is_some_and(|v| !v.is_empty() && v != "0")
}

#[cfg(test)]
mod required_tests {
    #[test]
    fn sim_require_titan_rs_values() {
        let r = |v: Option<&str>| super::required(v.map(std::ffi::OsStr::new));
        assert!(!r(None));
        assert!(!r(Some("")));
        assert!(!r(Some("0")));
        assert!(r(Some("1")));
        assert!(r(Some("yes")));
    }

    #[test]
    fn a_built_image_is_found() {
        let dir = std::env::temp_dir().join(format!("titan-rs-image-{}", std::process::id()));
        let release = dir.join(super::RS_RELEASE_SUFFIX);
        std::fs::create_dir_all(&release).unwrap();
        std::fs::write(release.join("template"), b"").unwrap();
        if std::env::var_os("TITAN_RS_RELEASE_DIR").is_none() {
            assert_eq!(super::titan_rs_image(&dir, "template"), Some(release.join("template")));
            if !super::titan_rs_required() {
                assert_eq!(super::titan_rs_image(&dir, "nope"), None);
            }
        }
        std::fs::remove_dir_all(&dir).unwrap();
    }
}

const RS_RELEASE_SUFFIX: &str = "external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release";

/// An image path from a robot file, with the titan_firmware_rs release directory replaced by
/// `$TITAN_RS_RELEASE_DIR` when that is set.
pub fn resolve_image(path: &std::path::Path) -> std::path::PathBuf {
    let s = path.to_string_lossy();
    match (std::env::var("TITAN_RS_RELEASE_DIR"), s.find(RS_RELEASE_SUFFIX)) {
        (Ok(dir), Some(i)) => std::path::Path::new(&dir).join(s[i + RS_RELEASE_SUFFIX.len()..].trim_start_matches('/')),
        _ => path.to_path_buf(),
    }
}
/// Expands a path from a robot file: a leading `~` (or `~/...`) is `$HOME`, and `$NAME` or `${NAME}` is the value
/// of the environment variable `NAME` (letters, digits and `_`), e.g. `${TITAN_FIRMWARE_DIR}/talos/...`. `$$` is a
/// literal `$`. An unset variable is an error that names it.
pub fn expand_path(path: &str) -> Result<std::path::PathBuf, String> {
    expand_path_with(path, |name| std::env::var(name).ok())
}

/// [`expand_path`] with the variables (and `HOME`) looked up by `var`.
pub fn expand_path_with(path: &str, var: impl Fn(&str) -> Option<String>) -> Result<std::path::PathBuf, String> {
    let lookup = |name: &str| {
        var(name).ok_or_else(|| match name {
            "TITAN_FIRMWARE_DIR" => format!(
                "{path}: TITAN_FIRMWARE_DIR is not set (point it at your titan_firmware checkout, e.g. \
                 `export TITAN_FIRMWARE_DIR=~/titan_firmware`)"
            ),
            _ => format!("{path}: environment variable {name} is not set"),
        })
    };
    let mut out = String::new();
    let mut rest = path;
    if let Some(after) = rest.strip_prefix('~')
        && (after.is_empty() || after.starts_with('/'))
    {
        out.push_str(&lookup("HOME")?);
        rest = after;
    }
    while let Some(i) = rest.find('$') {
        out.push_str(&rest[..i]);
        let after = &rest[i + 1..];
        let is_name = |c: char| c.is_ascii_alphanumeric() || c == '_';
        if let Some(a) = after.strip_prefix('$') {
            out.push('$');
            rest = a;
        } else if let Some(braced) = after.strip_prefix('{') {
            let end = braced.find('}').ok_or_else(|| format!("{path}: unclosed ${{"))?;
            let name = &braced[..end];
            if name.is_empty() || !name.chars().all(is_name) {
                return Err(format!("{path}: bad variable name {name:?}"));
            }
            out.push_str(&lookup(name)?);
            rest = &braced[end + 1..];
        } else {
            let end = after.find(|c: char| !is_name(c)).unwrap_or(after.len());
            if end == 0 {
                return Err(format!("{path}: `$` without a variable name (write `$$` for a `$`)"));
            }
            out.push_str(&lookup(&after[..end])?);
            rest = &after[end..];
        }
    }
    out.push_str(rest);
    Ok(out.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vars(name: &str) -> Option<String> {
        match name {
            "HOME" => Some("/home/me".into()),
            "TITAN_FIRMWARE_DIR" => Some("/src/titan_firmware".into()),
            "A_1" => Some("x".into()),
            _ => None,
        }
    }

    #[test]
    fn expands_home_and_variables() {
        let e = |p: &str| expand_path_with(p, vars).map(|p| p.to_string_lossy().into_owned());
        assert_eq!(e("~").unwrap(), "/home/me");
        assert_eq!(e("~/build/a.uf2").unwrap(), "/home/me/build/a.uf2");
        assert_eq!(e("~other/a").unwrap(), "~other/a", "only the user's own home");
        assert_eq!(
            e("${TITAN_FIRMWARE_DIR}/talos/IVCBoard2/build/IVCBoard2_with_bl.uf2").unwrap(),
            "/src/titan_firmware/talos/IVCBoard2/build/IVCBoard2_with_bl.uf2"
        );
        assert_eq!(e("$A_1/b-$A_1.elf").unwrap(), "x/b-x.elf");
        assert_eq!(e("a$$b").unwrap(), "a$b");
        assert_eq!(e("../relative/path").unwrap(), "../relative/path");
    }

    #[test]
    fn unset_and_malformed_variables_are_errors() {
        let e = |p: &str| expand_path_with(p, |_| None).unwrap_err();
        assert!(
            e("${TITAN_FIRMWARE_DIR}/x").contains("TITAN_FIRMWARE_DIR is not set"),
            "{}",
            e("${TITAN_FIRMWARE_DIR}")
        );
        assert!(e("$NOPE/x").contains("NOPE is not set"));
        assert!(e("~/x").contains("HOME is not set"));
        assert!(expand_path_with("${A", vars).unwrap_err().contains("unclosed"));
        assert!(expand_path_with("${A-B}", vars).unwrap_err().contains("bad variable name"));
        assert!(expand_path_with("a$/b", vars).unwrap_err().contains("without a variable name"));
    }
}

pub mod board;
pub mod bridges;
pub mod candiff;
pub mod canlog;
pub mod cannelloni;
pub mod dashboard;
pub mod defmt;
pub mod demangle;
pub mod ethernet;
pub mod fault;
pub mod flush;
pub mod gdb;
pub mod jsonlog;
pub mod mock_bridge;
pub mod parallel;
pub mod profile;
pub mod pty;
pub mod robot;
pub mod rtt;
pub mod session;
pub mod signal;
pub mod socketcan;
pub mod target;
pub mod timetravel;
pub mod unwind;
pub mod upload;
pub mod usb;
pub mod usbip;
pub mod vcd;
pub mod vehicle;

pub use session::Session;
pub use target::TargetMemory;
