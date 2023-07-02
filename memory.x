MEMORY {
    BOOT2 : org = 0x10000000, len = 0x00000100 /* 0x10000000 to 0x10000100 */
    FLASH : org = 0x10000100, len = 0x001FFF00 /* 0x10000100 to 0x10200000 */
    STACK : org = 0x20000000, len = 0x00004000 /* 0x20000000 to 0x00004000 */
    RAM   : org = 0x20004000, len = 0x0003E000 /* 0x20004000 to 0x20042000 */
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    .boot2 ORIGIN(BOOT2) : {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

/* The stack is placed in the stack area. */
_stack_start = ORIGIN(STACK) + LENGTH(STACK);

/* The heap is placed in the region normally shared by the heap and stack. */
_heap_start = __sheap;
_heap_end = ORIGIN(RAM) + LENGTH(RAM);
