## Development

To build this package use `colcon build --packages-up-to swerve_drive_controller` in the root workspace (owr-502).

If you would like a static analyzer use clangd. Generate compile commands with `colcon build --packages-select swerve_drive_controller --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` (this will NOT install dependencies, run this after running the normal build command that will (`--packages-up-to`))

You may run the autoformatter with `ament_uncrustify --reformat`