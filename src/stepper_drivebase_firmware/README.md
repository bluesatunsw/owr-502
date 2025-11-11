# Stepper drivebase firmware

See `main.rs` (and maybe `Cargo.toml`). This firmware is pretty underspecified at the moment so there
isn't much to write here. If you really desperately want to read some words I
wrote, check out `canadensis_mvp/README.md`.

`openocd -f openocd/stm32g4x.cfg` and `cargo run --feature rev_a2` if you have an ST-LINK connected.

`dsdl/` directory has been removed since we don't currently plan to use any
non-publically regulated types. If we ever do need them then just create the
directory again and add the types like in `canadensis_mvp`.
