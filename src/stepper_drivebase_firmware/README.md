# Stepper drivebase firmware

See also `main.rs` (and maybe `Cargo.toml`).

## Running and debugging the firmware

`openocd -f openocd/stm32g4x.cfg` and `cargo run --release --feature <rev>`
(where `<rev>` is `rev_a2`, `rev_b2`, `we_act_dev`, etc.) if you have an
ST-LINK connected. It's pretty much necessary to run in release mode due to
space optimisations being necessary to fit onto the device.

## A note on DSDL

`dsdl/` directory has been removed since we don't currently plan to use any
non-publically regulated types. If we ever do need them then just create the
directory again and add the types like in `canadensis_mvp`.

## Firmware organisation

The main principle is that `main.rs`, the firmware business logic, remains 90%
unchanged regardless of which hardware is targeted (dev board, rover board (any
revision), etc.). It calls an `init()` function expected to be defined in
`boards.rs` exactly once, which returns structs that abstract away operations
whose implementation is hardware-specific (e.g. communicating with chips over
SPI, configuring CAN, blinking LEDs, etc.). The abstract interface of these
structs is defined by a set of traits in `boards.rs`, and `boards.rs` patches
in the correct hardware-specific implementations from the `boards/stm32*`
modules depending on what feature (indicating the target hardware) the firmware
is being compiled with.

TL;DR: main business logic in `main.rs`, abstract interface definition in
`boards.rs`, hardware-specific implementations of abstract interface in
`boards/stm32*/*` modules.

## Adding a new board

I've needed to do this a few more times than expected and there are a couple
gotchas so I'll write this up.

### If the chip on the board matches an existing supported STM32 series (e.g. STM32G4xx)

If this is the case, the only expected differences are:

- Amount of flash and RAM available on the chip
- Availability of certain peripherals on-chip
- Which pins are used for which peripherals; which peripherals are actually
  wired up to external hardware

Steps:

1. Add a new feature to the `Cargo.toml` named after the board identifier.
2. Create an appropriate linker script (`memory_*.x`) in the `linker/` directory with the
   appropriate memory sections of the chip being used on the board, or reuse an
   existing linker script if appropriate.
3. Edit `build.rs` to use the new linker script when the feature is selected.
4. Configure the new feature in `Cargo.toml` such that it enables the correct
   chip identifier feature for the HAL when selected (see existing supported
   chips for reference).
5. Edit `main.rs` if necessary (look for existing `cfg_if`s).
6. Edit `boards.rs` to patch in the new feature when selected.
7. Edit the existing `boards/stm32*/*` modules (add `cfg_if`s, mostly) to do
   whatever is specific to the new board (e.g. use different pins or
   peripherals).

### If the chip is a different series

Additional differences potentially include:

- Target CPU architecture
- Peripheral availability, operation and pinout (more so than for the same series)

Steps are as above, but a new `boards/stm32*/*` top-level module will need to
be created, patched in inside `boards.rs` and new implementations for
everything created. Additionally, if the new board uses a different CPU than
that main one supported (Cortex-M4F) you will need to edit `.cargo/config.toml`
to add the new target and remember to build with `--target <target>` (e.g.
`--target thumbv6m-none-eabi` for Cortex-M0+) for that hardware.
