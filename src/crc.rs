/// Represents the CRC mode for the BQ769x0 driver.
pub trait CrcMode {}

/// CRC is disabled.
pub struct Disabled;
impl CrcMode for Disabled {}

/// CRC is enabled.
pub struct Enabled;
impl CrcMode for Enabled {}

/// Calculates the CRC-8 checksum for the given data according to BQ769x0 specifications.
/// Polynomial: x^8 + x^2 + x + 1 (0x07)
/// Initial value: 0x00
/// No input reflection, no output xor.
pub fn calculate_crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}
