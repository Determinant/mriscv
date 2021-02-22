INCLUDE memory.x
SECTIONS
{
    .  = ORIGIN(FLASH); /* load from the beginning of the flash */
    .text : {
        KEEP(*(.text))
    } > FLASH

    .data : {
        *(.data)
    } > RAM
    /DISCARD/ : {*(*)}
}
