# Main power module firmware

See `main.rs`. This firmware is to support current sensing, channel control and status LEDs.
There is no documentation for this board yet.

`openocd -f openocd/stm32g4x.cfg` and `cargo run --feature rev_a1` if you have an ST-LINK connected.
These should be run in seperate terminals with semihosting printing to the openocd terminal.