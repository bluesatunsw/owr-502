#![no_std]
#![feature(unsafe_cell_access)]
#![feature(iter_array_chunks)]

pub mod argb;
pub mod can;
pub mod clock;
pub mod debug;

#[cfg(feature = "stepper-board")]
pub mod stepper_bus;
#[cfg(feature = "stepper-board")]
pub mod tmc_registers;
#[cfg(feature = "desperation")]
pub mod foolcan;
