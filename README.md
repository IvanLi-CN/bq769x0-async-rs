# Rust BQ769x0 Battery Management System Driver

[![Build Status](https://github.com/IvanLi-CN/bq769x0-async-rs/workflows/Build/badge.svg)](https://github.com/IvanLi-CN/bq769x0-async-rs/actions?query=workflow%3ABuild)
[![Crates.io](https://img.shields.io/crates/v/bq769x0-async-rs.svg)](https://crates.io/crates/bq769x0-async-rs)
[![Docs](https://docs.rs/bq769x0-async-rs/badge.svg)](https://docs.rs/bq769x0-async-rs)
[![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.75+-blue.svg)](https://github.com/rust-lang/rust/blob/master/RELEASES.md)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/IvanLi-CN/bq769x0-async-rs#license)
![no_std](https://img.shields.io/badge/no__std-yes-blue)

This is a platform agnostic Rust driver for the Texas Instruments BQ76920, BQ76930, and BQ76940
battery management system (BMS) analog front-end (AFE) integrated circuits, based on the
[`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

This driver allows you to:

- **Battery Monitoring:**
  - Read individual cell voltages (up to 5/10/15 cells depending on device)
  - Read pack voltage and current
  - Monitor internal and external temperatures
  - Get battery pack status and fault conditions

- **Protection Features:**
  - Configure overvoltage and undervoltage protection thresholds
  - Set overcurrent and short circuit protection limits
  - Configure overtemperature and undertemperature protection
  - Enable/disable protection functions

- **Cell Balancing:**
  - Control individual cell balancing switches
  - Implement custom balancing algorithms
  - Monitor balancing status

- **Power Management:**
  - Control charge and discharge MOSFETs
  - Configure low-power modes
  - Manage device wake-up and shutdown

- **Communication:**
  - Support both async and sync operation modes
  - Optional CRC error checking for reliable communication
  - I²C interface with configurable addressing

## The Devices

The BQ769x0 family consists of highly integrated battery monitor and protection ICs for Li-Ion,
Li-Iron Phosphate, and other battery chemistries. These devices provide comprehensive monitoring
and protection for multi-cell battery packs.

Here is a comparison of the supported devices:

| Device  | Max Cells | Typical Pack Voltage | Cell Voltage Range | Current Sensing | Temperature Sensors |
|---------|-----------|---------------------|-------------------|-----------------|-------------------|
| BQ76920 | 5         | 18V                 | 0.3V to 5.0V      | ±100A          | 2 (internal + 1 external) |
| BQ76930 | 10        | 36V                 | 0.3V to 5.0V      | ±100A          | 2 (internal + 1 external) |
| BQ76940 | 15        | 48V                 | 0.3V to 5.0V      | ±100A          | 2 (internal + 1 external) |

All devices feature:

- Integrated analog front-end (AFE) with high-accuracy ADC
- Programmable protection thresholds
- Cell balancing capability
- I²C communication interface
- Low quiescent current for extended battery life

**Datasheets:**

- [BQ769x0 Family Datasheet](https://www.ti.com/lit/ds/symlink/bq76940.pdf)

## Usage

To use this driver, import this crate and an `embedded_hal` implementation, then instantiate
the appropriate device. The following example shows basic usage with the BQ76940:

```rust
use embedded_hal::i2c::I2c;
use bq769x0_async_rs::{Bq769x0, BatteryConfig, ProtectionConfig, crc::Enabled};

// Number of cells in your battery pack
const NUM_CELLS: usize = 5;

// Create the BMS driver instance
// Parameters: I2C, device address, sense resistor (mΩ), optional delay
let mut bms = Bq769x0::<_, Enabled, NUM_CELLS>::new(
    i2c,
    0x08,           // I2C address
    3,              // Sense resistor value in milliohms
    None            // Optional delay provider
);

// Configure battery protection parameters
let battery_config = BatteryConfig {
    overvoltage_trip: 3650,     // 3.65V per cell
    undervoltage_trip: 2500,    // 2.5V per cell
    protection_config: ProtectionConfig {
        ocd_limit: 10_000,      // 10A overcurrent limit
        ..Default::default()
    },
    rsense: 3,                  // Sense resistor in milliohms
    ..Default::default()
};

// Apply configuration
bms.set_config(&battery_config).await?;

// Read cell voltages
let cell_voltages = bms.read_cell_voltages().await?;
println!("Cell voltages: {:?} mV", cell_voltages);

// Read pack voltage and current
let pack_voltage = bms.read_pack_voltage().await?;
let current = bms.read_current().await?;
println!("Pack: {} mV, Current: {} mA", pack_voltage, current);

// Read temperatures
let internal_temp = bms.read_internal_temperature().await?;
let external_temp = bms.read_external_temperature().await?;
println!("Temperatures: Internal {}°C, External {}°C", internal_temp, external_temp);

// Control MOSFETs
bms.enable_charging().await?;
bms.enable_discharging().await?;
```

For complete examples including hardware setup and advanced features, see the [`examples/`](examples/) directory.

## Features

This crate supports the following optional features:

- `async`: Enable async/await support (requires `embedded-hal-async`)
- `defmt`: Enable defmt logging support for debugging
- `binrw`: Enable binary serialization support
- `std`: Enable standard library support (for testing and development)

Add features to your `Cargo.toml`:

```toml
[dependencies]
bq769x0-async-rs = { version = "0.0.1", features = ["async", "defmt"] }
```

Or install using cargo:

```shell
# For synchronous operation (default)
cargo add bq769x0-async-rs

# For asynchronous operation with defmt logging
cargo add bq769x0-async-rs --features async,defmt
```

## Hardware Setup

### I²C Connection

Connect your BQ769x0 device to your microcontroller via I²C:

- **SDA**: I²C data line (requires pull-up resistor, typically 4.7kΩ)
- **SCL**: I²C clock line (requires pull-up resistor, typically 4.7kΩ)
- **VDD**: Power supply (typically 3.3V or 5V)
- **VSS**: Ground

### Sense Resistor

A current sense resistor must be connected in series with the battery pack. The resistance value
should be chosen based on your maximum expected current and desired accuracy. Common values
range from 1mΩ to 10mΩ.

### I²C Address

The BQ769x0 devices support multiple I²C addresses through the ADDR pin:

- ADDR = GND: 0x08
- ADDR = VDD: 0x18

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the GitHub project](https://github.com/IvanLi-CN/bq769x0-async-rs/issues).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
