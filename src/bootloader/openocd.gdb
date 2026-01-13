target extended-remote :3333

source PyCortexMDebug/scripts/gdb.py
svd_load STM32G474.svd

# print demangled symbols
set print asm-demangle on
set disassemble-next-line on
show disassemble-next-line

# Force GDB to use HW breakpoints for next
mem 0x00000000 0x00004000 ro

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

monitor arm semihosting enable

# # send captured ITM to the file itm.fifo
# # (the microcontroller SWO pin must be connected to the programmer SWO pin)
# # 8000000 must match the core clock frequency
# monitor tpiu config internal itm.txt uart off 8000000

# # OR: make the microcontroller SWO pin output compatible with UART (8N1)
# # 8000000 must match the core clock frequency
# # 2000000 is the frequency of the SWO pin
# monitor tpiu config external uart off 8000000 2000000

# # enable ITM port 0
# monitor itm port 0 on

load

# start the process but immediately halt the processor
stepi
