//! Emulator test firmware: blinks GPIO 25 at 2 Hz (toggling every 250 ms) using embassy-time, counts toggles over
//! defmt, and does a little floating point and integer division so the ROM float routines and the SIO divider get
//! exercised through embassy-rp's `intrinsics`.
//!
//! Core 1 is started with `embassy_rp::multicore::spawn_core1` (the bootrom launch protocol over the SIO FIFO) and
//! toggles GPIO 24 every 100 ms by busy-waiting on the timer, sharing a counter with core 0 under the spinlock-based
//! critical section.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use core::cell::Cell;

use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static SHARED: Mutex<CriticalSectionRawMutex, Cell<u32>> = Mutex::new(Cell::new(0));

fn bump() -> u32 {
    SHARED.lock(|c| {
        c.set(c.get() + 1);
        c.get()
    })
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic: {}", defmt::Display2Format(info));
    loop {
        cortex_m_rt_wfi();
    }
}

fn cortex_m_rt_wfi() {
    unsafe { core::arch::asm!("wfi") };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut core1_led = Output::new(p.PIN_24, Level::Low);
    spawn_core1(p.CORE1, unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) }, move || {
        defmt::info!("core 1 up");
        loop {
            let until = Instant::now() + Duration::from_millis(100);
            while Instant::now() < until {}
            core1_led.toggle();
            bump();
        }
    });
    defmt::info!("blinky up, clk_sys {} Hz", embassy_rp::clocks::clk_sys_freq());
    let mut n: u32 = 0;
    loop {
        Timer::after_millis(250).await;
        led.toggle();
        n += 1;
        bump();
        let ratio = core::hint::black_box(n as f32) / 3.0;
        let quotient = core::hint::black_box(n * 1000) / core::hint::black_box(7u32);
        if n % 4 == 0 {
            defmt::info!("toggle {} at {} ms ({} {})", n, Instant::now().as_millis(), ratio, quotient);
            defmt::info!("shared counter {}", SHARED.lock(|c| c.get()));
        }
    }
}
