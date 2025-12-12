/* Linker script for nRF52810 - 192KB Flash, 24KB RAM */
/* Reduced stack size for minimal firmware */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000, LENGTH = 0x30000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x6000
}

/* Reduce stack size to 1KB for minimal firmware */
__STACK_SIZE = 0x400;

SECTIONS
{
}

SECTIONS
{
  . = ALIGN(4);
  .mem_section_dummy_ram :
  {
  }
  .log_dynamic_data :
  {
    PROVIDE(__start_log_dynamic_data = .);
    KEEP(*(.log_dynamic_data))
    PROVIDE(__stop_log_dynamic_data = .);
  } > RAM

} INSERT AFTER .data;

SECTIONS
{
  .mem_section_dummy_rom :
  {
  }
    .log_const_data :
  {
    PROVIDE(__start_log_const_data = .);
    KEEP(*(.log_const_data))
    PROVIDE(__stop_log_const_data = .);
  } > FLASH


} INSERT AFTER .text

INCLUDE "nrf5x_common.ld"
