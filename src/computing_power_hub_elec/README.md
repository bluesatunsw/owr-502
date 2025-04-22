# Computing Power Hub (CPH)
Provides and monitors a regulated voltage to be used by computing and communication devices. It complies with the common device specification 2025-04.

## 18V Rail
To power the Jetson and wireless access point, an 18V 6A rail is created. The regulated output voltage slowly ramps up to reduce inrush current. This is connected to an onboard passive POE injector as well as a 2-pin JST VH header.

## ARGB
There are four three pin ARGB ports. This is powered from a 5V 4A rail derived from the 18V rail.
