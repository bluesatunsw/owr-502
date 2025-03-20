#!/bin/sh
sudo dfu-util -a 0 -d 0483:df11 -s 0x08000000 -D build/STMicroelectronics.stm32.GenF0/*.ino.bin
