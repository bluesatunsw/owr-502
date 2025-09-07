# Can Converter

USB <-> CANFD Device, based on the upstream CandleLightFD project (STM32G4 variant)

A small USB-C device that allows for the reading of can signals from the rover.


The device uses 5V power from the USB-C port to power, with a buck converter for the 3.3V power on the board. The CAN signals are processed by the TCAN1462V transceiver. The transceiver is connected to the STM32 with data signals RXD, TXD and a standby input. The STM32 also interfaces with two LEDs to display the functionality of the device and actions it's currently performing. CAN signals are processed by the STM32 and sent to the USB-C port to be used for debugging purposes.


# Can Bus Pinout


| Pin | Signal | Colour |
| --- | ------ | ------ |
|  1  | `CANL` | Green  |
|  2  | `CANH` | Yellow |


# Debug Pinout


| Pin | Signal  |
| --- | ------- |
|  1  | `SWDIO` |
|  2  | `SWCLK` |
|  3  | `GND`   |
|  4  | `VDD`   |
|  5  | `SWO`   |
|  6  | `NRST`  |
