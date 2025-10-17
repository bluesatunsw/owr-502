WORK IN PROGRESS!!

This file is not quite accurate now... see boards.rs

Each module contains:
    - a `struct CyphalClock` that implements the `canadensis::core::time::Clock`
      trait and the following methods:
        - `new_singleton(<hardware-specific>...)`: returns a `CyphalClock`, once
        - `start(&mut self)`: starts the `CyphalClock`
    - a `struct GeneralClock` that implements a `now()` method that just
      returns the same timestamp as the CyphalClock does
        - panics if a `CyphalClock` was not initialised and started
        - this exists because ownership of the `CyphalClock` moves to the
          canadensis tranceiver, so we need an additional means of getting the
          timestamp
    - a `struct HwCanDriver` that implements the
      `canadensis_can::CanTransmitter` and `canadensis_can::CanReceiver` traits and the following methods:
        - `new_singleton(<hardware-specific>...)`: returns a `HwCanDriver`, once
