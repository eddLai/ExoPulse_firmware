/* Linker script for nRF52840 Dongle with Open Bootloader */
/* Application starts at 0x1000 (after MBR), bootloader at 0xE0000 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  /* Application area: 0x1000 to 0xDFFFF (approx 895KB) */
  FLASH (rx) : ORIGIN = 0x1000, LENGTH = 0xDF000
  /* RAM starts at 0x20000008 - first 8 bytes reserved by MBR for interrupt forwarding */
  RAM (rwx) :  ORIGIN = 0x20000008, LENGTH = 0x3FFF8
}

/* Stack size */
__STACK_SIZE = 0x1000;

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
