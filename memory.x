MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 256K - 0x100
    RESERVED: ORIGIN = 0x10040000, LENGTH = 256K
    PROGRAM_SLOT_1: ORIGIN = 0x10080000, LENGTH = 512K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
    PROGRAM_RAM : ORIGIN = 0x20020000, LENGTH = 136K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

SECTIONS {
    .prog1 ORIGIN(PROGRAM_SLOT_1) : {
        KEEP(*(.prog1));
    } > PROGRAM_SLOT_1
} INSERT AFTER .text;