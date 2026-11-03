@ M0 acceptance program: toggles GPIO 25 through SIO forever.
@
@ Each half period is a 1000-iteration delay loop (SUBS 1 cycle + taken BNE 2 cycles) plus the loop overhead:
@ 3005 cycles. The program does not touch the clocks, so it runs from the ROSC as the bootrom leaves it.

    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .section .vectors, "a"
    .word 0x20042000            @ initial SP: top of SRAM
    .word reset                 @ reset handler

    .text
    .thumb_func
    .global reset
reset:
    ldr r0, =0x4000f000         @ RESETS.RESET, atomic clear alias
    ldr r1, =0x120              @ IO_BANK0 | PADS_BANK0
    str r1, [r0]
    ldr r0, =0x4000c008         @ RESETS.RESET_DONE
wait_reset:
    ldr r2, [r0]
    ands r2, r1
    cmp r2, r1
    bne wait_reset
    ldr r0, =0x400140cc         @ IO_BANK0.GPIO25_CTRL
    movs r1, #5                 @ FUNCSEL = SIO
    str r1, [r0]
    ldr r0, =0xd0000000         @ SIO
    ldr r1, =(1 << 25)
    str r1, [r0, #0x24]         @ GPIO_OE_SET
loop:
    str r1, [r0, #0x1c]         @ GPIO_OUT_XOR
    ldr r2, =1000
delay:
    subs r2, #1
    bne delay
    b loop
