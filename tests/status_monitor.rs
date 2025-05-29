use approx::assert_relative_eq;
use bq769x0_async_rs::units::ElectricPotential;
use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::{CoulombCounter, TemperatureSensorReadings},
    errors::Error,
    registers::{Register, SysCtrl1Flags, SysCtrl2Flags, SysStatFlags},
    Bq769x0, RegisterAccess,
};
use core::ops::Deref;
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use embedded_hal_mock::eh1::MockError;
use heapless::Vec;
use std::cell::RefCell;
use std::rc::Rc;
use uom::si::{
    electric_current::milliampere,
    electric_potential::{millivolt, volt},
    temperature_interval::degree_celsius,
};

pub const BQ76920_ADDR: u8 = 0x08;

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

// Helper function to create a driver with expected ADC calibration values
pub fn create_driver_with_adc_calibration(
    expectations: &[I2cTransaction],
) -> (Bq769x0<MockI2c, Disabled, 5>, MockI2c) {
    let i2c_mock = MockI2c::new(expectations);
    let driver = Bq769x0::new_without_crc(i2c_mock.clone(), BQ76920_ADDR);
    (driver, i2c_mock)
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

#[test]
fn test_read_cell_voltages_bq76920() {
    let expectations = [
        // Vc1Hi/Lo, Vc2Hi/Lo, Vc3Hi/Lo, Vc4Hi/Lo, Vc5Hi/Lo
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::Vc1Hi as u8],
            vec![
                0x0C, 0x00, // Cell 1: 3072 raw (12.00V)
                0x0C, 0x00, // Cell 2: 3072 raw
                0x0C, 0x00, // Cell 3: 3072 raw
                0x0C, 0x00, // Cell 4: 3072 raw
                0x0C, 0x00, // Cell 5: 3072 raw
            ],
        ),
        // ADC calibration reads
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x00]), // adc_gain_raw = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x00]), // adc_offset_signed = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x00]), // adc_gain_raw = 0
    ];
    // ADCGAIN = 365 uV/LSB, ADCOFFSET = 0 mV
    let (mut driver, i2c_mock) = create_driver_with_adc_calibration(&expectations);
    let result = driver.read_cell_voltages();
    assert!(result.is_ok());
    let voltages = result.unwrap();
    for i in 0..5 {
        assert_relative_eq!(
            voltages.voltages[i].get::<millivolt>(),
            3072.0 * 0.365,
            epsilon = 0.01
        );
    }
    i2c_mock.done();
}
