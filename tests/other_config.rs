#![allow(unused_imports)]
#![allow(dead_code)]

use approx::assert_relative_eq;
use bq769x0_async_rs::{
    Bq769x0, RegisterAccess,
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::{self, *},
};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::MockError;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use heapless::Vec;
use std::cell::RefCell;
use std::rc::Rc;

pub const BQ76920_ADDR: u8 = 0x08;
pub const BQ76930_ADDR: u8 = 0x08; // Default address, can be 0x18
pub const BQ76940_ADDR: u8 = 0x08; // Default address, can be 0x18

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
    let driver = Bq769x0::new_without_crc(i2c_mock_instance.clone(), address, 10, None); // Default 10mΩ sense resistor
    (driver, i2c_mock_instance) // Return the cloned mock for verification
}

// Helper function to create a driver with expected ADC calibration values
fn create_driver_with_adc_calibration(
    expectations: &[I2cTransaction],
    adc_gain_raw: u8,
    adc_offset_signed: i8,
) -> (Bq769x0<MockI2c, Disabled, 5>, MockI2c) {
    let mut full_expectations = Vec::<I2cTransaction, 10>::new();
    // Expectations for read_adc_calibration
    let _ = full_expectations.push(I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::ADCGAIN1 as u8],
        vec![adc_gain_raw & 0b00001100],
    ));
    let _ = full_expectations.push(I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::ADCOFFSET as u8],
        vec![adc_offset_signed as u8],
    ));
    let _ = full_expectations.push(I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::ADCGAIN2 as u8],
        vec![(adc_gain_raw << 5) & 0b11100000],
    ));

    for exp in expectations {
        let _ = full_expectations.push(exp.clone());
    }

    let i2c_mock = MockI2c::new(&full_expectations);
    let driver = Bq769x0::new_without_crc(i2c_mock.clone(), BQ76920_ADDR, 10, None); // Default 10mΩ sense resistor
    (driver, i2c_mock) // Return a new mock for verification
}

#[test]
fn test_enter_ship_mode() {
    let expectations = [
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x00]), // SYS_CTRL1 = 0x00
        I2cTransaction::write(BQ76920_ADDR, vec![0x05, 0x00]), // SYS_CTRL2 = 0x00
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x01]), // SYS_CTRL1 = 0x01 (SHUT_B)
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x02]), // SYS_CTRL1 = 0x02 (SHUT_A)
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.enter_ship_mode();
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_is_alert_overridden() {
    let expectations = [
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysStat as u8],
            vec![registers::SysStatFlags::OVRD_ALERT.bits()],
        ), // OVRD_ALERT bit set
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.is_alert_overridden();
    assert_eq!(result, Ok(true));
    i2c_mock.done();

    let expectations_not_overridden = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysStat as u8], vec![0x00]), // OVRD_ALERT bit not set
    ];
    let (mut driver_not_overridden, i2c_mock_not_overridden) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations_not_overridden, BQ76920_ADDR);
    let result_not_overridden = driver_not_overridden.is_alert_overridden();
    assert_eq!(result_not_overridden, Ok(false));
    i2c_mock_not_overridden.done();
}

#[test]
fn test_enable_charging() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysCtrl2 as u8], vec![0x00]), // Read current SYS_CTRL2
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![
                Register::SysCtrl2 as u8,
                registers::SysCtrl2Flags::CHG_ON.bits(),
            ],
        ), // Write SYS_CTRL2 with CHG_ON
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.enable_charging();
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_disable_charging() {
    let expectations = [
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysCtrl2 as u8],
            vec![registers::SysCtrl2Flags::CHG_ON.bits()],
        ), // Read current SYS_CTRL2 (CHG_ON is set)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::SysCtrl2 as u8, 0x00]), // Write SYS_CTRL2 with CHG_ON cleared
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.disable_charging();
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_enable_discharging() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysCtrl2 as u8], vec![0x00]), // Read current SYS_CTRL2
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![
                Register::SysCtrl2 as u8,
                registers::SysCtrl2Flags::DSG_ON.bits(),
            ],
        ), // Write SYS_CTRL2 with DSG_ON
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.enable_discharging();
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_disable_discharging() {
    let expectations = [
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysCtrl2 as u8],
            vec![registers::SysCtrl2Flags::DSG_ON.bits()],
        ), // Read current SYS_CTRL2 (DSG_ON is set)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::SysCtrl2 as u8, 0x00]), // Write SYS_CTRL2 with DSG_ON cleared
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.disable_discharging();
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_set_cell_balancing_bq76920() {
    let expectations = [
        I2cTransaction::write(BQ76920_ADDR, vec![Register::CELLBAL1 as u8, 0b00001011]), // Set cells 1, 2, 4
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.set_cell_balancing(0b00001011); // Cells 1, 2, 4
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_set_cell_balancing_bq76930() {
    let expectations = [
        I2cTransaction::write(BQ76930_ADDR, vec![Register::CELLBAL1 as u8, 0b00001011]), // Set cells 1, 2, 4
        I2cTransaction::write(BQ76930_ADDR, vec![Register::CELLBAL2 as u8, 0x04]), // Set cell 8
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<10>(&expectations, BQ76930_ADDR);
    let result = driver.set_cell_balancing(0b00000000_10001011); // Cells 1, 2, 4, 8
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_set_cell_balancing_bq76940() {
    let expectations = [
        I2cTransaction::write(BQ76940_ADDR, vec![Register::CELLBAL1 as u8, 0b00001011]), // Set cells 1, 2, 4
        I2cTransaction::write(BQ76940_ADDR, vec![Register::CELLBAL2 as u8, 0b00000101]), // Set cell 6, 8
        I2cTransaction::write(BQ76940_ADDR, vec![Register::CELLBAL3 as u8, 0b00000001]), // Set cell 11
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<15>(&expectations, BQ76940_ADDR);
    let mask_value: u16 = 0b00000100_10101011; // Cells 1, 2, 4, 8, 11 (for BQ76940)
    // CELLBAL1: 0b00001011 (CB1, CB2, CB4)
    // CELLBAL2: 0b00000101 (CB6, CB8)
    // CELLBAL3: 0b00000001 (CB11)

    let result = driver.set_cell_balancing(mask_value);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_cc_cfg_read_write() {
    let expectations = [
        I2cTransaction::write(BQ76920_ADDR, vec![Register::CcCfg as u8, 0x19]), // Write 0x19 to CC_CFG
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::CcCfg as u8], vec![0x19]), // Read 0x19 from CC_CFG
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let write_result = driver.write_register(Register::CcCfg, 0x19);
    assert_eq!(write_result, Ok(()));
    let read_result = driver.read_register(Register::CcCfg);
    assert_eq!(read_result, Ok(0x19));
    i2c_mock.done();
}
