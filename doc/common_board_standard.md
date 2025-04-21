# Common Board Standard

Certain elements will be common to the majority of boards designed for the rover. These generally relate to power, debugging, and basic connectivity. Unless otherwise specified, it can be assumed that the rest of this page applies to all custom boards.

## Power
The unregulated power provided to boards is referred to as bus power; nominally 24 V, but with significant noise and transient spikes. Components exposed to bus power should be rated to 50 V. A male XT30 connector is used for power input.

Most components operate off either 12 V, 5 V or 3V3. 3V3 is the preferred voltage for all signals. To generate these rails, bus power is first regulated down to 12 V using a buck converter (MP2459GJ-Z). This allows smaller, lower-noise and more efficient regulators to be used for regulation to lower voltages required by the system.

Since the majority of current is drawn from the 3V3 (also referred to as VDD) rail, another buck converter (AP62200TWU) is used to derive this from the 12 V rail. A charge pump (ME2135A50M6G) is then used to generate a low-current 5 V rail from the 3V3 rail. If required, a VDDA rail can be derived from the 3V3 rail as well using a pi filter composed of a ferrite (BLM18PG121SN1D) and a pair of 1u capacitors.

**[TODO] Provide typical component values for regulators**

## System Core
A single STM32G474RBT6 is used as the MCU for each board. To provide accurate timing (desirable for CAN), a 24 MHz external crystal (XL7EL89CKI-111YLC-24M) should be used, loaded with a pair of 10 pF capacitors.

A 128 kbit I2C EEPROM (ZD24C128A-XGMB) is used to store persistent configuration/calibration data.

The STM32CubeMX tool should be used to assign pin functions in the electrical design phase, as this will check for conflicts.

## Protection
All inputs and outputs must be protected from ESD, inrush current, and reverse polarity using a unidirectional ESD diode. Typically, this will be one of the following components:
- SMF30A (single 30 V) for bus power;
- ESD9X5VU (single 5 V) for logic power;
- RCLAMP0502B (dual 5 V) for USB/CAN data lines; and
- TPAZ1143-04F (quad 3V3) for general logic.

These should be placed between the connector and any downstream components.

## CAN
Boards should have a single CAN-FD transceiver (SIT1057QTK/3) connected to the internal CAN controller of the MCU. A 2-pin JST-PA header is used to connect to the CAN bus.

| Pin | Signal | Colour |
| --- | ------ | ------ |
|  1  | `CANL` | Green  |
|  2  | `CANH` | Yellow |

Weak split termination using two 4k7 resistors and a 1n capacitor should be implemented near the transceiver.

The distance between the MCU, transceiver and header should be kept short (around 100 mm) to avoid transmission line effects. This way, controlled impedance routing is not required.

## Debugging
Trace asynchronous SW debugging should always be enabled; this allows for flashing of software as well as the use of GDB and print statements. A 6-pin 1.27 mm pitch IDC header should be provided to allow for the attachment of an STLink.

| Pin | Signal |
| --- | ------ |
|  1  | `VCC`  |
|  2  | `SWDIO`|
|  3  | `NRST` |
|  4  | `SWCLK`|
|  5  | `GND`  |
|  6  | `SWO`  |

To aid in debugging and inspection, there should be at least two WS2812B ARGB LEDs (XL-2121RGBC-WS2812B). For boards which have multiple distinct output "channels" there should be an additional LED near each channel to show the status of that channel. The control signal can be created using a half-duplex UART externally pulled to 5 V using a 4k7 resistor.

## Housing
A board will be housed in a case composed of two parts which will be screwed together using M2.5 countersunk Torx machine screws. The lower half will be manufactured from 8 mm aluminium plate so that it can provide rigidity and serve as a heat spreader. A custom pattern should be milled so that the heat spreader almost makes contact with the tops of hot components, with a thermal pad used to fill the remaining space. The upper half will be 3D printed.

As reflow soldering using a hotplate becomes difficult with double-sided boards, the majority of components should be placed on the back side of the PCB. The front should be reserved for components which must be accessible, such as connectors, buttons, and LEDs. These should be simple to solder manually with an iron.
