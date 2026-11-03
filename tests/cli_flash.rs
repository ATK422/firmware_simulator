//! `run` sizes the flash from the board file unless `--flash-size` says otherwise. The flash file written at the
//! end of a run has the flash's size, which makes the choice visible.

use std::process::Command;

fn flash_file_len(extra: &[&str]) -> u64 {
    let dir = std::env::temp_dir().join(format!("fwsim-cli-flash-{}-{}", std::process::id(), extra.len()));
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join("flash.bin");
    let root = env!("CARGO_MANIFEST_DIR");
    let status = Command::new(env!("CARGO_BIN_EXE_firmware_simulator"))
        .args(["run", &format!("{root}/test_programs/blink.elf"), "--time", "1ms", "--unmodelled", "off"])
        .args(["--uart0", "none", "--uart1", "none"])
        .arg("--flash-file")
        .arg(&file)
        .args(extra)
        .status()
        .unwrap();
    assert!(status.success());
    let len = std::fs::metadata(&file).unwrap().len();
    std::fs::remove_dir_all(&dir).unwrap();
    len
}

#[test]
fn flash_size_comes_from_the_board_file_unless_given() {
    let root = env!("CARGO_MANIFEST_DIR");
    let board = format!("{root}/boards/mk2/puddles_backplane.toml");
    assert_eq!(flash_file_len(&[]), 2 << 20, "no board: 2 MiB");
    assert_eq!(flash_file_len(&["--board", &board]), 16 << 20, "the board file says 16 MiB");
    assert_eq!(flash_file_len(&["--board", &board, "--flash-size", "4194304"]), 4 << 20, "--flash-size wins");
}
