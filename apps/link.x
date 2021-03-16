INCLUDE memory.x
SECTIONS
{
    .  = ORIGIN(FLASH); /* load from the beginning of the flash */
    .text : {
        KEEP(*(.text))
        *(.text.startup)
    } > FLASH
    .rodata : {
        *(.rodata)
        *(.rodata.*)
    } > FLASH
    .data : {
        *(.data)
        *(.sdata)
        *(.sbss)
    } > RAM
    .note.gnu.build-id : {
        *(.note.gnu.build-id)
    } > FLASH
    /DISCARD/ : {*(*)}
}
