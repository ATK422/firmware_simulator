@ Counts boots in WATCHDOG.SCRATCH0 and in a word of SRAM, then asks for a system reset through AIRCR.SYSRESETREQ,
@ ten times. On the RP2040 that resets only the requesting core: the watchdog, its scratch registers, the
@ peripherals and SRAM carry on, and the core boots again from the vector table.

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
    ldr r2, =0x20030000         @ the same count in SRAM
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    cmp r1, #10
    bge done
    ldr r0, =0xe000ed0c         @ AIRCR
    ldr r1, =0x05fa0004         @ VECTKEY | SYSRESETREQ
    dsb
    str r1, [r0]
    dsb
hang:
    b hang
done:
    wfi
    b done
