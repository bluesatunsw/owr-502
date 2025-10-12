MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* I don't know how much memory is on the specific chips we have.
     This is an educated guess only. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K /* could be 1 Meg */
  RAM : ORIGIN = 0x20000000, LENGTH = 192K
}
