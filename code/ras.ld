/* IMPORTANT(Ryan): We know we have 1GB of RAM that is shared with the GPU.
 Not sure as to the specifics of this sharing.
 So, make conservative use of RAM to ensure smooth sailing.
*/
MEMORY
{
  EXCEPTION_TABLE (rwx) : ORIGIN = 0x00000000, LENGTH = 32K
  INIT_RAM (rwx) : ORIGIN = 0x00008000, LENGTH = 32K
  RAM (rwx) : ORIGIN = 0x000010000, LENGTH = 128M
}

_svc_stack = ORIGIN(RAM) + LENGTH(RAM);
svc_stack_size = 0x2000;
_irq_stack = _svc_stack - svc_stack_size;

_init_ram_addr = 0x00008000;
_exception_table_addr = 0x00000000;

ENTRY(_start)

SECTIONS
{
  /* IMPORTANT(Ryan): We leave off EXCEPTION_TABLE, as will explicitly copy over */
  .init : {
    KEEP(*(.init*))
  } > INIT_RAM

  .text : {
    KEEP(*(.text*))
  } > RAM

  .data : {
    KEEP(*(.data*))
  } > RAM
}
