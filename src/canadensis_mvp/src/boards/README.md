WORK IN PROGRESS!!

Each module contains:
    - a `struct CyphalClock` that implements `canadensis::core::time::Clock`
      with `Instant` type `Microseconds48`
        - methods: `new_singleton`, `start`
        - (hmm, different function signatures depending on hardware... use
          `embedded_hal`?)
    - a `struct GeneralClock` that implements a `now()` method that just
      returns the same timestamp as the CyphalClock does
        - panics if a struct CyphalClock was not initialised and started
        - since the CyphalClock gets owned by the canadensis tranceiver, a
          different means of getting the timestamp (for the main processing
          loop) is necessary; this struct provides that means
    - a `struct CyphalCanTransmitter` that implements
      `canadensis_can::CanTransmitter`
    - a `struct CyphalCanReceiver` that implements
      `canadensis_can::CanReceiver`
