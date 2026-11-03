/* Minimal layout for the hand-written test programs: vector table at the start of flash (no boot2). */
MEMORY {
    FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 2M
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 264K
}
SECTIONS {
    .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) } > FLASH
    .bss (NOLOAD) : { *(.bss*) } > RAM
}
