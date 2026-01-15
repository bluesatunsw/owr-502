# Stepper drivebase firmware

See also `main.rs` (and maybe `Cargo.toml`).

## Running and debugging the firmware

`openocd -f openocd/stm32g4x.cfg` and `cargo run --release if you have an
ST-LINK connected. It's pretty much necessary to run in release mode due to
space optimisations being necessary to fit onto the device.

## A note on DSDL

`dsdl/` directory has been removed since we don't currently plan to use any
non-publically regulated types. If we ever do need them then just create the
directory again and add the types like in `canadensis_mvp`.


## Firmware organisation

main.rs has most of the stuff.
HAL sort of not really goes in stm32g4xx/

## Message IDs
30xy, where 30 means drivebase, x is the topic, and y is the channel
