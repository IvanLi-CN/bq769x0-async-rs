# bq769x0-async-rs

A driver for the BQ769x0 family of battery management system (BMS) ICs, built upon the [embedded-hal](https://crates.io/crates/embedded-hal) traits. This crate provides both `async` and `sync` APIs, making it suitable for a wide range of embedded applications.

## Features

-   **Datasheet-Driven Development**: Implemented based on the official BQ769x0 datasheet ([`docs/bq76920.pdf`](docs/bq76920.pdf)), ensuring accurate register mappings and behavior.
-   **`no-std` Compatible**: Designed for embedded environments, with no reliance on the standard library.
-   **Asynchronous/Synchronous Support**: Offers flexible `async` and `sync` APIs through the `maybe-async-cfg` crate, allowing you to choose the concurrency model that best fits your project.
-   **`embedded-hal` Compliant**: Seamlessly integrates with the `embedded-hal` ecosystem, ensuring compatibility with various microcontrollers and hardware abstraction layers.
-   **`bitflags` Integration**: Utilizes the `bitflags` crate for managing register fields and and data types, enhancing readability, maintainability, and type safety when working with hardware registers and status flags.

## Usage

To add the `bq769x0-async-rs` driver to your project, run one of the following commands:

```shell
# For synchronous operation (default)
cargo add bq769x0-async-rs

# For asynchronous operation
cargo add bq769x0-async-rs --features async
```

## Examples

You can find example projects demonstrating the use of this driver in the [`examples/`](examples/) directory. These examples showcase how to initialize the driver, read battery data, and configure protection features on various embedded platforms.

## License

`bq769x0-async-rs` is distributed under the terms of both the MIT License and the Apache License (Version 2.0).

See [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT) for details.
