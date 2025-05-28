#![allow(unused_imports)]
#![allow(dead_code)]

use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::{self, *},
    Bq769x0, RegisterAccess,
};
use core::ops::Deref;
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use embedded_hal_mock::eh1::MockError;
use heapless::Vec;
use std::cell::RefCell;
use std::rc::Rc;

pub const BQ76920_ADDR: u8 = 0x08;
pub const BQ76930_ADDR: u8 = 0x08; // Default address, can be 0x18
pub const BQ76940_ADDR: u8 = 0x08; // Default address, can be 0x18

// Helper function to create a driver with expected ADC calibration values
pub fn create_driver_with_adc_calibration(
    expectations: &[I2cTransaction],
) -> (Bq769x0<MockI2c, Disabled, 5>, MockI2c) {
    let i2c_mock = MockI2c::new(expectations);
    let driver = Bq769x0::new_without_crc(i2c_mock.clone(), BQ76920_ADDR);
    (driver, i2c_mock)
}

/// A mock I2C device that can be programmed with expected transactions.
#[derive(Clone)] // Derive Clone for MockI2c
pub struct MockI2c {
    mock: Rc<RefCell<I2cMock>>,
}

impl MockI2c {
    /// Creates a new `MockI2c` with the given expected transactions.
    pub fn new(transactions: &[I2cTransaction]) -> Self {
        Self {
            mock: Rc::new(RefCell::new(I2cMock::new(transactions))),
        }
    }

    /// Consumes the mock and verifies that all expected transactions occurred.
    pub fn done(self) {
        // Access the inner mock through RefCell
        self.mock.borrow_mut().done();
    }
}

// Implement ErrorType for MockI2c
impl ErrorType for MockI2c {
    type Error = embedded_hal::i2c::ErrorKind;
}

impl I2c for MockI2c {
    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.mock
            .borrow_mut()
            .write(address, bytes)
            .map_err(|_| embedded_hal::i2c::ErrorKind::Other)
    }

    fn read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.mock
            .borrow_mut()
            .read(address, bytes)
            .map_err(|_| embedded_hal::i2c::ErrorKind::Other)
    }

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.mock
            .borrow_mut()
            .write_read(address, bytes, buffer)
            .map_err(|_| embedded_hal::i2c::ErrorKind::Other)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.mock
            .borrow_mut()
            .transaction(address, operations)
            .map_err(|_| embedded_hal::i2c::ErrorKind::Other)
    }
}

/// Helper function to create a Bq769x0 instance for testing.
pub fn create_bq769x0_driver_disabled_crc<const N: usize>(
    transactions: &[I2cTransaction],
    address: u8,
) -> (Bq769x0<MockI2c, Disabled, N>, MockI2c) {
    let i2c_mock_instance = MockI2c::new(transactions);
    let driver = Bq769x0::new_without_crc(i2c_mock_instance.clone(), address);
    (driver, i2c_mock_instance) // Return the cloned mock for verification
}

/// Helper function to create a Bq769x0 instance for testing with CRC enabled.
pub fn create_bq769x0_driver_enabled_crc<const N: usize>(
    transactions: &[I2cTransaction],
    address: u8,
) -> (Bq769x0<MockI2c, Enabled, N>, MockI2c) {
    let i2c_mock_instance = MockI2c::new(transactions);
    let driver = Bq769x0::new(i2c_mock_instance.clone(), address);
    (driver, i2c_mock_instance) // Return the cloned mock for verification
}

/// Helper for common init sequence
pub fn expect_init_sequence<I2C, M, E, const N: usize>(
    driver: &mut Bq769x0<I2C, M, N>,
) -> Result<(), Error<E>>
where
    I2C: I2c<Error = E>,
    M: CrcMode,
    Bq769x0<I2C, M, N>: RegisterAccess<E>,
    E: PartialEq, // Add PartialEq constraint for E
{
    // Expected writes during init based on common BQ769x0 initialization
    // These are examples and should be refined based on actual init logic in lib.rs
    // and bq76920.pdf recommended settings.

    // Clear all status flags (SYS_STAT)
    driver.write_register(Register::SysStat, 0b11111111)?;
    // Set CC_CFG to 0x19 for optimal performance
    driver.write_register(Register::CcCfg, 0x19)?;
    // Enable ADC and set TEMP_SEL to external thermistor (example)
    driver.write_register(
        Register::SysCtrl1,
        (registers::SysCtrl1Flags::ADC_EN | registers::SysCtrl1Flags::TEMP_SEL).bits(),
    )?;
    // Enable CC continuous readings (example)
    driver.write_register(Register::SysCtrl2, registers::SysCtrl2Flags::CC_EN.bits())?;

    Ok(())
}
