# CAN-FD Bootloader

## Memory Layout

### Logical

The user program is split into the following logical regions:
- `NFO`: 32 bytes of information about the user program used by the bootloader
- `AUX`: data passed to the user program by the bootloader (at the bottom of SRAM1)
- `CCM`: data to be loaded into the top of CCM SRAM, this should include the vector table and any high performance code, default location for the `.vector_table` section
- `RAM`: data to be loaded into SRAM1 after `AUX`, this should be any global variables; default location for `.data`, `.bss` and `.uninit` sections
- `ERO`: read only data (eg. program code) stored in external flash
- `IRO`: read only data (eg. program code) stored in internal flash

If external flash exists the default location for the `.text` and `.rodata` sections are `ERO`, otherwise it is `IRO`.

### Physical

If external flash exists then the `NFO`, `CCM`, `RAM` and `ERO` regions are stored in external flash in that order with the `CCM` and `RAM` sections loaded into memory by the bootloader. Otherwise all sections are stored in internal flash in the same order, with the `ERO` region replaced with `IRO`.
