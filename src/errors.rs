#[cfg(feature = "defmt")]
use defmt::Format;

/// Represents potential errors when interacting with the BQ769x0 chip.
#[derive(Debug, PartialEq)] // 无条件派生 Debug 和 PartialEq
#[cfg_attr(feature = "defmt", derive(Format))] // 条件性地应用 Format 派生
pub enum Error<E: PartialEq> {
    /// An error occurred during I2C communication.
    I2c(E),
    /// Invalid data received from the chip.
    InvalidData,
    // Add other specific error types as needed later, e.g.:
    // UnsupportedFeature,
    /// CRC validation failed.
    Crc,
}
