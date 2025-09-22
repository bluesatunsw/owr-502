# candleLight_gsusb
The actual source code (tracked with a link to upstream project) is at: https://github.com/Avnzx/bluesat_canconverter_fw.

This implements the interface of the mainline linux gs_usb kernel module and
works out-of-the-box with linux distros packaging this module, e.g. Ubuntu.

Currently, the firmware sends back an echo frame to the host when the frame is written to the CAN peripheral, and not when the frame is actually sent successfully on the bus. This affects timestamps, one-shot mode, and other edge cases.

## Known issues

Be aware that there is a bug in the gs_usb module in linux<4.5 that can crash the kernel on device removal.

Here is a fixed version that should also work for older kernels:
  https://github.com/HubertD/socketcan_gs_usb

The Firmware also implements WCID USB descriptors and thus can be used on recent Windows versions without installing a driver.

## Building

Building requires arm-none-eabi-gcc toolchain.

```shell
sudo apt-get install gcc-arm-none-eabi

mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi-8-2019-q3-update.cmake

# compile a single target , e.g.
make bluesat_canconverter_fw

# Flash board (will tell app to go into DFU mode)
make flash-bluesat_canconverter_fw

#
# to list possible targets :
make help

```

## Download Binaries
Prebuilt binaries can be downloaded by clicking [![CI](https://github.com/candle-usb/candleLight_fw/actions/workflows/ci.yml/badge.svg)](https://github.com/candle-usb/candleLight_fw/actions). On the workflow overview page, select the latest workflow that ran on master branch. The firmware artifacts can downloaded by clicking them at the bottom of the page.

## Flashing

Flashing candleLight on linux: (source: [https://cantact.io/cantact/users-guide.html](https://cantact.io/cantact/users-guide.html))
- Flashing requires the dfu-util tool. On Ubuntu, this can be installed with `sudo apt install dfu-util`.
- compile as above, or download the current binary release: gsusb_cantact_8b2b2b4.bin
- If dfu-util fails due to permission issues on Linux, you may need additional udev rules. Consult your distro's documentation and see `70-candle-usb.rules` provided here.

### recommended simple method
- If compiling with cmake, `make flash-<targetname_fw>`, e.g. `make flash-canable_fw`, to invoke dfu-util.

### method for reflashing a specific device by serial
- when multiple devices are connected, dfu-util may be unable to choose which one to flash.
- Obtain device's serial # by looking at `dfu-util -l`
- adapt the following command accordingly :
 `dfu-util -D CORRECT_FIRMWARE.bin -S "serial_number_here", -a 0 -s 0x08000000:leave`
- note, the `:leave` suffix above may not be supported by older builds of dfu-util and is simply a convenient way to reboot into the normal firmware.

### fail-safe method (or if flashing a blank device)
- Disconnect the USB connector from the CANtact, short the BOOT pins, then reconnect the USB connector. The device should enumerate as "STM32 BOOTLOADER".

- invoke dfu-util manually with: `sudo dfu-util --dfuse-address -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D CORRECT_FIRMWARE.bin` where CORRECT_FIRMWARE is the name of the desired .bin.
- Disconnect the USB connector, un-short the BOOT pins, and reconnect.



## Associating persistent device names
With udev on linux, it is possible to assign a device name to a certain serial number (see udev manpages and [systemd.link](https://www.freedesktop.org/software/systemd/man/systemd.link.html)).
This can be useful when multiple devices are connected at the same time.

An example configuration :

```
 $ cat /etc/systemd/network/60-persistent-candev.link
[Match]
Property=ID_MODEL=cannette_gs_usb ID_SERIAL_SHORT="003800254250431420363230"

[Link]
# from systemd.link manpage:
# Note that specifying a name that the kernel might use for another interface (for example "eth0") is dangerous because the name assignment done by udev will race with the assignment done by the kernel, and only one
#   interface may use the name. Depending on the order of operations, either udev or the kernel will win, making the naming unpredictable. It is best to use some different prefix

Name=cannette99
```

( The serial number can be found with the `lsusb` utility). After reloading systemd units and resetting this board :

```
 $ ip a
....
59: cannette99: <NOARP,ECHO> mtu 16 qdisc noop state DOWN group default qlen 10
    link/can
 $
```
