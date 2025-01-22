EXTERN(entry);
ENTRY(entry);

MEMORY {
    RAM   : ORIGIN = 0x20020000,                    LENGTH = 136K
    SLOT  : ORIGIN = 0x10000000 + 512K * SLOT_N,    LENGTH = 512K
}

SECTIONS {
    .header ORIGIN(SLOT) : {
        KEEP(*(.header));
    } > SLOT

    .text : ALIGN(4) {
        . = ALIGN(4);
        *(.text .text.*);
        . = ALIGN(4);
    } > SLOT

    .rodata : ALIGN(4) {
        . = ALIGN(4);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > SLOT

    .data : ALIGN(4) {
        . = ALIGN(4);
        KEEP(*(.data .data.*));
        . = ALIGN(4);
    } > RAM AT>SLOT

    .bss (NOLOAD) : ALIGN(4) {
        . = ALIGN(4);
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
    } > RAM

    .uninit (NOLOAD) : ALIGN(4) {
        . = ALIGN(4);
        *(.uninit .uninit.*);
        . = ALIGN(4);
    } > RAM

    /DISCARD/ : {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}