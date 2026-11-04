@ Branches for the coverage tests: assembled with -g, so the ELF's line table maps each instruction to a line of
@ this file. `reset` takes the `beq`, calls `helper` three times and sleeps; `not_taken` and `unused` never run.
@ (Line numbers matter: crates/sim_host/tests/coverage.rs expects them.)

    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .section .vectors, "a"
    .word 0x20042000
    .word reset

    .text
    .thumb_func
    .global reset
    .type reset, %function
reset:
    movs r0, #3
    cmp r0, #3
    beq taken
not_taken:
    movs r1, #1
    b done
taken:
    movs r4, #3
again:
    bl helper
    subs r4, #1
    bne again
done:
    wfi
    b done
    .size reset, .-reset

    .thumb_func
    .global helper
    .type helper, %function
helper:
    adds r1, #1
    bx lr
    .size helper, .-helper

    .thumb_func
    .global unused
    .type unused, %function
unused:
    movs r2, #9
    bx lr
    .size unused, .-unused
