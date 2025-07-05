# System Power Hub (SPH)

Further distributes power from the MPH into eight channels. It complies with the common device specification 2025-04.

Power input for the MPH is provided via a male XT60 connector and each of the channel has a female XT30 power output connector.

## Switching
Power to each of the channels can be toggled by the MCU. When a channel is enabled, including when the device initially powers up, a soft start slowly reduces the output impedance to reduce inrush current. As implied by the connector name, each channel is able to output 30A peak and 20A continuous.

## Monitoring
The output current of each channel is monitored.

**[TODO] Explain how this will be implemented**
