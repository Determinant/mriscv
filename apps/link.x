INCLUDE memory.x
SECTIONS
{
    .  = ORIGIN(FLASH); /* load from the beginning of the flash */
    .text : {
        KEEP(*(.text))
    } > FLASH
    .rodata : {
        *(.rodata)
    } > FLASH
    .data : {
        *(.data)
    } > RAM
    .note.gnu.build-id : {
        *(.note.gnu.build-id)
    } > FLASH
    /DISCARD/ : {*(*)}
}
