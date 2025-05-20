extern crate defmt; // Make defmt available for derive macros

use defmt::Format;

/// Represents potential errors when interacting with the BQ769x0 chip.
#[derive(Debug, Format)] // Add defmt::Format derive
pub enum Error<E> {
    /// An error occurred during I2C communication.
    I2c(E),
    /// Invalid data received from the chip.
    InvalidData,
    // Add other specific error types as needed later, e.g.:
    // UnsupportedFeature,
    /// CRC validation failed.
    CrcError,
}