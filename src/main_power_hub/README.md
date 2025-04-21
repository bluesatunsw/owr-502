# Main Power Hub (MPH)

A device which monitors and distributes battery power onto four subcircuits (intended to be either computing power hubs or subsystem power hubs). It complies with the common board standard.

Power input from the battery is provided via a male XT90 connector and each of the subcircuits has a female XT60 power output connector.

## Switching
Power to each of the subcircuits can be toggled by the MCU. Triggering the estop, a normally closed contact wired to the board, removes power from all subcircuits bypassing the control of the MCU. The MCU is always powered. As implied by their names, each channel is able to output 60A peak and 40A continuous.

Unlike other power hub devices, the switching of channel is instant (it lacks a soft start).

## Monitoring
The bus voltage, total input current and output current of each channel is monitored.

**[TODO] Explain how this will be implemented**
