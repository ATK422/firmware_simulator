@ Counts boots in WATCHDOG.SCRATCH0, arms the watchdog for 10 ms and sleeps in WFI until it bites.
@
@ The tick runs from clk_ref (the 6.5 MHz ROSC after reset) divided by 65: 10 us ticks. LOAD 2000 counts down
@ twice per tick (RP2040-E1), so the watchdog fires 1000 ticks = 10 ms after it is armed.

    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .section .vectors, "a"
    .word 0x20042000
    .word reset

    .text
    .thumb_func
    .global reset
reset:
    ldr r0, =0x40058000         @ WATCHDOG
    ldr r1, [r0, #0x0c]         @ SCRATCH0: boot count
    adds r1, #1
    str r1, [r0, #0x0c]
    ldr r1, [r0, #0x08]         @ REASON of this boot
    str r1, [r0, #0x10]         @ -> SCRATCH1
    ldr r1, =0x241              @ TICK: ENABLE | CYCLES 65
    str r1, [r0, #0x2c]
    ldr r1, =2000
    str r1, [r0, #0x04]         @ LOAD
    ldr r1, =0x40000000         @ CTRL.ENABLE
    ldr r2, =0x4005a000         @ CTRL, atomic set alias
    str r1, [r2]
sleep:
    wfi
    b sleep
