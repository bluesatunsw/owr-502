## Overview

The code in this directory describes firmware for the Mellow FLY-DP5 board,
which controls stepper motors (part of the end effector of the robot actuator)
in response to CAN frames as sent by the Jetson central robot computer.
A specification for the data sent in these CAN frames can be found [elsewhere in
this repository](https://github.com/bluesatunsw/owr-502/blob/B-G431-ESC1/docs/main.pdf).
(It is the same data standard as for the other motor controllers.)

## Development

We are currently using the Arduino framework for the firmware.

Have the following installed for development:
- `dfu-util`: `sudo apt install dfu-util` or similar (see the [utility's website](https://dfu-util.sourceforge.net/))
- `arduino-cli`: follow installation instructions from the [Arduino website](https://arduino.github.io/arduino-cli/1.2/)
- the STM32 libraries for Arduino: see [Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32)

### Build the firmware

```
arduino-cli compile -b STMicroelectronics:stm32:GenF0 -e --board-options "pnum=GENERIC_F072RBTX"
```
You may wish to specify an output directory with `--output-dir <dir>`. See `make.sh`.

### Flash the firmware

The buttons on the board, when oriented with the USB port on the left edge of
the board, are BOOT (left) and RESET (right) as illustrated below. (This image
is taken from the [Mellow FLY-DP5
documentation](https://mellow.klipper.cn/en/docs/ProductDoc/MainBoard/fly-d/fly-dp5/flash/bl).)

![](https://mellow.klipper.cn/en/assets/images/boot-7342bc51eaea0da5cf73805e93469d73.webp)

Steps:

1. Plug the board into the device you're flashing the firmware from using USB-C.
2. Unplug everything from the board's PCIE slots.
3. Put the board into DFU mode: hold down BOOT, then hold down RESET for
   a few seconds, then release RESET for a few seconds, then release BOOT.
   Getting this right can take a few tries. If you were successful, there
   should be a device named `STMicroelectronics STM Device in DFU Mode` present
   when you run `lsusb`.
4. Run `sudo dfu-util -a 0 -d 0483:df11 -s 0x08000000 -D example.ino.bin`.
   Obviously change the filename. You may need to replace `0483:df11` with the
   actual ID of the board as listed in `lsusb`. See `flash.sh`.
5. Either press RESET or power cycle the board to get the firmware running.
