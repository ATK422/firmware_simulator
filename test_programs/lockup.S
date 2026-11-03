@ Counts boots in WATCHDOG.SCRATCH0, arms the watchdog for 10 ms (on its first two boots only) and locks up: a UDF
@ faults into HardFault, whose handler faults again (a fault in HardFault locks the core up, PC 0xfffffffe on a
@ real M0+).
@
@ As in watchdog.S, the 10 us watchdog tick from the 6.5 MHz ROSC and LOAD 2000 (counted down twice per tick,
@ RP2040-E1) make the watchdog fire 10 ms after it is armed: on a chip, the reset ends the lockup and the program
@ boots again. The third boot leaves the watchdog off, so nothing ends that lockup.

    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .section .vectors, "a"
    .word 0x20042000
    .word reset
    .word hardfault             @ NMI
    .word hardfault             @ HardFault

    .text
    .thumb_func
    .global reset
reset:
    ldr r0, =0x40058000         @ WATCHDOG
    ldr r1, [r0, #0x0c]         @ SCRATCH0: boot count
    adds r1, #1
    str r1, [r0, #0x0c]
    cmp r1, #3
    bge lock                    @ third boot on: no watchdog
    ldr r1, =0x241              @ TICK: ENABLE | CYCLES 65
    str r1, [r0, #0x2c]
    ldr r1, =2000
    str r1, [r0, #0x04]         @ LOAD
    ldr r1, =0x40000000         @ CTRL.ENABLE
    ldr r2, =0x4005a000         @ CTRL, atomic set alias
    str r1, [r2]
lock:
    udf #0                      @ -> HardFault

    .thumb_func
hardfault:
    udf #1                      @ a fault in HardFault: lockup
