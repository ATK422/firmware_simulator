@ Profiler test program: a known distribution of time over functions, with call frame information.
@
@ reset calls `outer` forever; `outer` calls `hot` (a 3000-iteration delay loop, 9000 cycles), `warm` (1000
@ iterations in a function with a stack frame, 3000 cycles) and `nocfi` (500 iterations, 1500 cycles, no call frame
@ information: the profiler unwinds it through LR). SysTick fires every 20000 cycles and its handler spends 600
@ cycles in a loop, so about 3% of the time is in the handler, on top of whatever it interrupted. Core 1 stays in the
@ bootrom waiting for a launch, sleeping.

    .syntax unified
    .cpu cortex-m0plus
    .thumb
    .cfi_sections .debug_frame

    .section .vectors, "a"
    .word 0x20042000            @ initial SP: top of SRAM
    .word reset                 @ 1: reset
    .rept 13
    .word 0                     @ 2-14: unused
    .endr
    .word systick               @ 15: SysTick

    .text

    .thumb_func
    .global reset
    .type reset, %function
reset:
    .cfi_startproc
    .cfi_undefined lr
    ldr r0, =0xe000e010         @ SYST_CSR
    ldr r1, =19999
    str r1, [r0, #4]            @ SYST_RVR: every 20000 cycles
    movs r1, #0
    str r1, [r0, #8]            @ SYST_CVR
    movs r1, #7                 @ ENABLE | TICKINT | CLKSOURCE (processor clock)
    str r1, [r0]
1:
    bl outer
    b 1b
    .cfi_endproc
    .size reset, .-reset

    .thumb_func
    .global outer
    .type outer, %function
outer:
    .cfi_startproc
    push {r4, lr}
    .cfi_def_cfa_offset 8
    .cfi_offset r4, -8
    .cfi_offset lr, -4
    ldr r0, =3000
    bl hot
    ldr r0, =1000
    bl warm
    bl nocfi
    pop {r4, pc}
    .cfi_endproc
    .size outer, .-outer

    .thumb_func
    .global hot
    .type hot, %function
hot:
    .cfi_startproc
1:
    subs r0, #1
    bne 1b
    bx lr
    .cfi_endproc
    .size hot, .-hot

    .thumb_func
    .global warm
    .type warm, %function
warm:
    .cfi_startproc
    push {r7, lr}
    .cfi_def_cfa_offset 8
    .cfi_offset r7, -8
    .cfi_offset lr, -4
    sub sp, #16
    .cfi_def_cfa_offset 24
1:
    subs r0, #1
    bne 1b
    add sp, #16
    .cfi_def_cfa_offset 8
    pop {r7, pc}
    .cfi_endproc
    .size warm, .-warm

    @ No .cfi directives: no entry in .debug_frame
    .thumb_func
    .global nocfi
    .type nocfi, %function
nocfi:
    ldr r0, =500
1:
    subs r0, #1
    bne 1b
    bx lr
    .size nocfi, .-nocfi

    .thumb_func
    .global systick
    .type systick, %function
systick:
    .cfi_startproc
    push {lr}
    .cfi_def_cfa_offset 4
    .cfi_offset lr, -4
    ldr r0, =200
1:
    subs r0, #1
    bne 1b
    pop {pc}
    .cfi_endproc
    .size systick, .-systick
