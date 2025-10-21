# USB Power Hub

A power hub that uses battery power to operate the kinects using both a 12v rail and a 5v rail. There will also be space space on the 5v rail to power more USB devices to plug into the USB hub in the future for more communication.

USB data hub will output usb A female to communicate with USB B devices.

## Notes for creator
Notes for making the board:
- USB B into the usb data hub FROM the computer
- USB A out of the usb data hub INTO the computer
- USB output pins should ONLY cary data from the computer / hub and should ONLY have 5v rail power coming from the battery / buck converter

## TO DO
- Should decide on what part is being used as an output port at some point

- Find proper footprints for all the USB connectors




## Mega converter link
This for the setup for the VBus to 12V and Vbus to 5V
https://webench.ti.com/appinfo/webench/scripts/SDP.cgi?ID=56A75480E1E04682

This for the 12V to 1.17V and 3.3V
https://www.lcsc.com/product-detail/C327676.html

## Ampage power consumption

3v3:
- 100ma MAX per 5gb hub
- 75ma MAX for the 20g hub
- TOTAL: 500ma

## POWER PROTECTION

- 1A per USB port max
- 1.5 per 12V Kinect port MAX