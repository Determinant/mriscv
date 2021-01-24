MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 64K
    RAM : ORIGIN = 0x00200000, LENGTH = 128K
}

SECTIONS
{
    .  = ORIGIN(FLASH);
    .text :
    {
        KEEP(*(.text))
    } > FLASH

    .data :
    {
        *(.data)
    } > RAM
    /DISCARD/ : {*(*)}
}
