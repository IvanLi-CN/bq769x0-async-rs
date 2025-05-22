#[cfg(feature = "defmt")]
use defmt::Format;

/// Represents potential errors when interacting with the BQ769x0 chip.
#[derive(Debug)] // 无条件派生 Debug
#[cfg_attr(feature = "defmt", derive(Format))] // 条件性地应用 Format 派生
pub enum Error<E> {
    /// An error occurred during I2C communication.
    I2c(E),
    /// Invalid data received from the chip.
    InvalidData,
    // Add other specific error types as needed later, e.g.:
    // UnsupportedFeature,
    /// CRC validation failed.
    Crc,
}
