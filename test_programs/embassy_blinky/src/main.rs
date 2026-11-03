//! Emulator test firmware: blinks GPIO 25 at 2 Hz (toggling every 250 ms) using embassy-time, counts toggles over
//! defmt, and does a little floating point and integer division so the ROM float routines and the SIO divider get
//! exercised through embassy-rp's `intrinsics`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Instant, Timer};

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
    defmt::info!("blinky up, clk_sys {} Hz", embassy_rp::clocks::clk_sys_freq());
    let mut n: u32 = 0;
    loop {
        Timer::after_millis(250).await;
        led.toggle();
        n += 1;
        let ratio = core::hint::black_box(n as f32) / 3.0;
        let quotient = core::hint::black_box(n * 1000) / core::hint::black_box(7u32);
        if n % 4 == 0 {
            defmt::info!("toggle {} at {} ms ({} {})", n, Instant::now().as_millis(), ratio, quotient);
        }
    }
}
