MEMORY
{
  /* STM32G0B1RET6: 512K flash, 144K non-parity protected RAM*/
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 144K
}
