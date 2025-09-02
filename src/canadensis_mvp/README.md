# A rough outline to working with canadensis, the Cyphal driver written in Rust

This is an incomplete draft. Please let Jonah know if you think this is missing
useful or important things.

## Misc. notes

[There are examples!](https://github.com/samcrow/canadensis/blob/master/canadensis/examples/basic_node.rs)
- But be careful looking at the master branch, as it incorporates changes that
  aren't found in the latest released version; treat it as experimental.

CAN FD support is enabled by a feature flag; see [crate features](https://docs.rs/crate/canadensis_can/0.3.1/features).
In the source, CAN FD-specific code is annotated with `#[cfg(feature = "can-fd")]`.

## Getting online

So you want your embedded system to present itself as one or more Cyphal nodes
on one or more CAN interfaces. You can create a fully-featured Cyphal node as
follows:

1. Create a node ID using `canadensis_can::CanNodeId::try_from()`.
2. Create a struct that implements the `canadensis::core::time::Clock` trait
   for your given hardware platform.
    - This just returns the current time with the `now()` method.
        - The time value it returns must implement the
          `canadensis::core::time::Instant` trait.
        - Provided types that implement the `Instant` trait include
          `Microseconds32`, `Microseconds48` and `Microseconds64`.
3. If you are on a platform where a driver has been written (i.e. STM32s with bxCAN),
   skip this step. Otherwise, create a struct that implements the
   `canadensis_can::driver::ReceiveDriver` and `::TransmitDriver` traits.
    - This encompasses `try_reserve()`, `transmit()`, `flush()` for
      [TX](https://docs.rs/canadensis_can/0.3.1/canadensis_can/driver/trait.TransmitDriver.html)
      and `receive()`, `apply_filters()` and `apply_accept_all()` for
      [RX](https://docs.rs/canadensis_can/0.3.1/canadensis_can/driver/trait.ReceiveDriver.html).
4. Create a `canadensis::node::CoreNode`. You need to create or define the following
   to pass as parameters to that struct's `new()` method:
    - `clock`: as created in step 2
    - `node_id`: as created in step 1
    - `transmitter`: created using `canadensis_can::CanTransmitter::new(canadensis_can::Mtu::Can8)`
    - `receiver`: created using `canadensis_can::CanReceiver::new(node_id, canadensis_can::Mtu::Can8)`
    - `driver`: created using the driver in step 3.
    Additionally, pass in the following as type and const parameters:
    - `C`: the `Clock` type as in step 2
    - `T`, `U`: the `transmitter` and `receiver` types from step 4
    - `TR`: a struct that implements `canadensis::requester::TransferIdTracker`
        - Should be possible to just use `canadensis::requester::TransferIdFixedMap<canadensis_can::CanTransport, C>`
          where C is some constant power of two 
            - Pick C such that any node will never be:
                1. sending more than C message transfers with a given subject-ID
                   concurrently, and
                2. handling more than C service transfers with a given subject-ID
                   between any given destination node concurrently
        - Alternatively, implement your own transfer ID tracker data structure! :D
    - `D`: the driver type as in step 3
    - `P`: the maximum number of topics that can be published; must be a power of two
    - `R`: the maximum number of services for which requests can be sent; must be a power of two
5. Set up a `canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse` struct.
6. Wrap the `CoreNode` in a `canadensis::node::BasicNode` with the `NodeInfo` struct.
7. Run the node event loop indefinitely. You're probably running `node.receive()`, `node.run_per_second_tasks()` and
   `node.flush()` at minimum; see [the docs](https://docs.rs/canadensis/0.3.3/canadensis/node/struct.BasicNode.html)
   for further functionality. To actually send messages, call
   `node.start_publishing()` on setup and then publish with the resulting
   publish token. Example code for Linux follows.

```
let start_time = std::time::Instant::now();
    let mut prev_seconds = 0;
    loop {
        match node.receive(&mut EmptyHandler) {
            Ok(_) => {}
            Err(Error::Driver(e)) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) => panic!("{:?}", e),
        }

        let seconds = std::time::Instant::now()
            .duration_since(start_time)
            .as_secs();
        if seconds != prev_seconds {
            prev_seconds = seconds;
            node.run_per_second_tasks().unwrap();
            node.flush().unwrap();
        }
    }
```

## Using custom data types

The standard platform-independent way you do this is you have a `.dsdl` file
and then run some kind of compiler on it in your language to get a language
module that lets you deserialise the object (into your language's types) from
a payload and serialise the object from your language's types into a payload.

If you are just transmitting/receiving a straight `canadensis_data_types` type,
you are chilling, because they all implement `Serialize` and `Deserialize`, and
you can use them directly.

If you have a custom `.dsdl` file, look at `canadensis_macro`, which gives you `types_from_dsdl!()`.
You may need to [look at the source](https://docs.rs/canadensis_macro/latest/src/canadensis_macro/lib.rs.html#18-23).

See the [tests](https://github.com/samcrow/canadensis/blob/master/canadensis_macro/tests/dsdl.rs)
for hints on usage.

### Crude syntax guide

For `canadensis_macro` v0.3.2.

```
types_from_dsdl! {
    <type_declaration>*
    <generate_statement>
}

<type_declaration> : <package> | <make_external> | <inline_type>
<package> : package($CARGO_MANIFEST_DIR, "relative/path/to/dsdl")
    // replaces $CARGO_MANIFEST_DIR with that actual environment variable
    // and concatenates all comma-separated arguments
<make_external> : make_external(cyphal.package.name.here, rust::module::here)
    // possible bug with ., :: segments?
    // inserts rust module into cyphal package slot (???)
    // the BTreeMap gets passed to generate_code
<inline_type> :
    type "type.name.x.y" { r#"inline
dsdl
goes
here
    "# }
<generate_statement> : generate()
```

[`make_external`](https://github.com/samcrow/canadensis/tree/master/canadensis_codegen_rust)

The version on `master` on the GitHub passes extra arguments to `generate()`,
but this isn't currently part of a published version of `canadensis_macro`.

## Using Rust DSDL codegen

See the [`canadensis_codegen_rust`
documentation](https://github.com/samcrow/canadensis/tree/master/canadensis_codegen_rust).
For a summary, read on.

Calling `types_from_dsdl!` creates a Rust module with the same name as the
directory under the dsdl directory it was called with (i.e. the DSDL
namespace). For instance, if you have `/path/to/dsdl/owr502` and all your
`*.dsdl` files are under `dsdl/owr502`, then calling `types_from_dsdl!` on
`/path/to/dsdl` will create a module with name "owr502", corresponding to the
"owr502" DSDL namespace.

Suppose we have a DSDL file called `TurboEncabulator_Capacitance.1.2.dsdl` in
the `owr502` namespace. The generated Rust module name is
`turbo_encabulator_capacitance_1_2` (submodule of `owr502`) and the struct it
contains is `TurboEncabulator_Capacitance`.

To use the generated type(s), you will have something like

```
use crate::owr502::turbo_encabulator_capacitance_1_2::TurboEncabulator_Capacitance;
```

at the top of your program, after the `types_from_dsdl!` call.

For services, append either `Request` or `Response` to the base type name to
get the corresponding struct type.

Fixed port-ID values specified as part of the DSDL file name (e.g.
"1337.CoolType.4.2.dsdl") can be accessed directly as SubjectId or ServiceId
objects via `cooltype_4_2::SUBJECT` or `cooltype_4_2::SERVICE` depending on
whether the DSDL describes a subject or service.

Use `cargo expand --bin <canadensis_mvp>` (`cargo install cargo-expand`) if
you need to confirm what `canadensis_macro` generates.
