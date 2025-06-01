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
    /// CRC validation failed.
    Crc,
    /// Configuration verification failed after writing to the chip.
    ConfigVerificationFailed {
        /// The register जेट्टा (jeṭṭā) verification failed.
        register: crate::registers::Register,
        /// The expected value of the register.
        expected: u8,
        /// The actual value read back from the register.
        actual: u8,
    },
    /// An error occurred during temperature conversion.
    TemperatureConversion(&'static str),
    // Add other specific error types as needed later, e.g.:
    // UnsupportedFeature,
}
