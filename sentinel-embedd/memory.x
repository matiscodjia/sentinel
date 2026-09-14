/* Memory layout for STM32F411CEU6 (Black Pill) */
MEMORY
{
  /* Flash memory: 512KB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K

  /* RAM: 128KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* The location of the stack can be overridden using the `_stack_start` symbol. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
