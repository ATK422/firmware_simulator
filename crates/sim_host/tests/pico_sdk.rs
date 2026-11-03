//! M2: pico-sdk programs (`test_programs/pico`, equivalents of pico-examples' hello_uart, hello_multicore and
//! flash/program built against titan_firmware's pico-sdk) and C titan_firmware programs print over UART0.

mod common;

use sim_core::time::Time;

fn run(path: &std::path::Path, t: Time) -> sim_host::Session {
    let mut s = common::boot(path, None);
    s.run_until(t);
    s.poll_uarts();
    s
}

fn pico(name: &str) -> std::path::PathBuf {
    common::repo().join("test_programs/pico").join(format!("{name}.elf"))
}

#[test]
fn hello_uart() {
    let s = run(&pico("hello_uart"), Time::from_ms(20));
    // uart_putc only converts LF to CRLF when PICO_UART_DEFAULT_CRLF is set (off by default)
    assert_eq!(s.uart_text[0], "AB Hello, UART!\n");
}

#[test]
fn hello_multicore_launches_core1_through_the_bootrom() {
    let s = run(&pico("hello_multicore"), Time::from_ms(50));
    let out = &s.uart_text[0];
    for line in [
        "Hello, multicore!",
        "It's all gone well on core 0!",
        "Its all gone well on core 1!",
        "core 1 divider: -142 rem -6",
        // Both cores incremented under a hardware spinlock
        "shared counter: 2000",
        "interp: 00001023",
    ] {
        assert!(out.contains(line), "missing {line:?} in {out:?}");
    }
    assert!(s.machine.core_enabled[1]);
}

#[test]
fn flash_program_erases_programs_and_verifies() {
    let s = run(&pico("flash_program"), Time::from_ms(400));
    let out = &s.uart_text[0];
    assert!(out.contains("Programming successful!"), "{out}");
    // The page really is in the flash array, 256 KiB in
    let page = &s.machine.sys.flash[256 * 1024..256 * 1024 + 256];
    assert!(page.iter().any(|&b| b != 0xFF));
    assert!(s.machine.sys.flash[256 * 1024 + 256..256 * 1024 + 4096].iter().all(|&b| b == 0xFF));
}

#[test]
fn flash_erase_takes_sector_erase_time() {
    // Line timestamps: "Done." follows "Erasing target region..." by one sector erase (45 ms), give or take the UART
    // time of the lines (the FIFO keeps transmitting during the erase)
    let mut s = common::boot(&pico("flash_program"), None);
    let times = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let t2 = times.clone();
    s.on_uart_line = Some(Box::new(move |t, _, line| t2.lock().unwrap().push((t, line.to_string()))));
    s.run_until(Time::from_ms(400));
    let times = times.lock().unwrap();
    let at = |text: &str| times.iter().find(|(_, l)| l.contains(text)).unwrap().0;
    let erase = at("Done. Read back") - at("Erasing target region");
    assert!((42_000..48_000).contains(&erase.as_us()), "erase took {erase:?}");
}

#[test]
fn c_basic_template_boot_log() {
    let Some(elf) = common::c_elf("basic_template/template_project.elf") else { return };
    let s = run(&elf, Time::from_ms(100));
    let out = &s.uart_text[0];
    assert!(out.contains("[main]: Template Project 1.0"), "{out}");
    assert!(out.contains("[safety]: Clean Boot: Power-on Reset"), "{out}");
}

#[test]
fn c_power_board_bootloader_boot_log() {
    let Some(elf) =
        common::c_elf("PowerBoard/titan_lib/titan_bootloader/src/titan_bootloader-build/titan_bootloader.elf")
    else {
        return;
    };
    let s = run(&elf, Time::from_ms(100));
    let out = &s.uart_text[0];
    assert!(out.contains("titan_bootloader 1.0-talos/mk2/power_board"), "{out}");
}
