# Main Power Hub (MPH)

A device which monitors and distributes battery power onto four subcircuits (intended to be either computing power hubs or subsystem power hubs). It complies with the common device specification 2025-04.

Power input from the battery is provided via a male XT90 connector and each of the subcircuits has a female XT60 power output connector.

## Switching
Power to each of the subcircuits can be toggled by the MCU. Triggering the estop, a normally closed contact wired to the device, removes power from all subcircuits bypassing the control of the MCU. The MCU is always powered. As implied by the connector name, each channel is able to output 60A peak and 40A continuous.

Switching on ramps slowly (soft pullup resistor + gate capacitance), switching off is instantaneous.

Power MOSFET: C2902884
https://www.lcsc.com/datasheet/C2902884.pdf

Gate drive voltage is provided by a charge pump circuit that sums the battery voltage + the 12V rail.
The gate driver is a simple BJT pullup configuration.

## Monitoring
The total input current of the rover and output current of each channel is monitored.
