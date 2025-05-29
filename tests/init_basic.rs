#![allow(unused_imports)]
#![allow(dead_code)]

use approx::assert_relative_eq;
use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::{self, *},
    Bq769x0, RegisterAccess,
};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use embedded_hal_mock::eh1::MockError;
use heapless::Vec;
use std::cell::RefCell;
use std::rc::Rc;

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

#[test]
fn test_new_without_crc() {
    let expectations = [];
    let (_driver, i2c_mock) = create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    // No explicit assertions needed here, as `done()` will verify expectations.
    // The main goal is to ensure the driver can be instantiated.
    i2c_mock.done();
}

#[test]
fn test_new_with_crc() {
    let expectations = [];
    let (_driver, i2c_mock) = create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    i2c_mock.done();
}

#[test]
fn test_read_register_disabled_crc() {
    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::SysStat as u8],
        vec![0x01],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_register(Register::SysStat);
    assert_eq!(result, Ok(0x01));
    i2c_mock.done();
}

#[test]
fn test_write_register_disabled_crc() {
    let expectations = [I2cTransaction::write(
        BQ76920_ADDR,
        vec![Register::SysStat as u8, 0x01],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.write_register(Register::SysStat, 0x01);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_read_registers_disabled_crc() {
    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::Vc1Hi as u8],
        vec![0x12, 0x34, 0x56, 0x78],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_registers(Register::Vc1Hi, 4);
    assert_eq!(
        result,
        Ok(Vec::from_slice(&[0x12, 0x34, 0x56, 0x78]).unwrap())
    );
    i2c_mock.done();
}

#[test]
fn test_write_registers_disabled_crc() {
    let expectations = [I2cTransaction::write(
        BQ76920_ADDR,
        vec![Register::CELLBAL1 as u8, 0x01, 0x02],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.write_registers(Register::CELLBAL1, &[0x01, 0x02]);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_read_register_enabled_crc_success() {
    let expected_data = 0xAA;
    let slave_addr_read = (BQ76920_ADDR << 1) | 1;
    let calculated_crc = bq769x0_async_rs::crc::calculate_crc(&[slave_addr_read, expected_data]);

    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::SysStat as u8],
        vec![expected_data, calculated_crc],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_register(Register::SysStat);
    assert_eq!(result, Ok(expected_data));
    i2c_mock.done();
}

#[test]
fn test_read_register_enabled_crc_failure() {
    let expected_data = 0xAA;
    let incorrect_crc = 0x00; // Intentionally incorrect CRC

    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![Register::SysStat as u8],
        vec![expected_data, incorrect_crc],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_register(Register::SysStat);
    assert_eq!(result, Err(Error::Crc));
    i2c_mock.done();
}

#[test]
fn test_write_register_enabled_crc_success() {
    let reg_addr = Register::SysStat as u8;
    let value = 0x01;
    let slave_addr_write = BQ76920_ADDR << 1;
    let calculated_crc = bq769x0_async_rs::crc::calculate_crc(&[slave_addr_write, reg_addr, value]);

    let expectations = [I2cTransaction::write(
        BQ76920_ADDR,
        vec![reg_addr, value, calculated_crc],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.write_register(Register::SysStat, value);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_read_registers_enabled_crc_success() {
    let reg_addr = Register::Vc1Hi as u8;
    let data_bytes = [0x12, 0x34, 0x56, 0x78];
    let slave_addr_read = (BQ76920_ADDR << 1) | 1;

    let crc1 = bq769x0_async_rs::crc::calculate_crc(&[slave_addr_read, data_bytes[0]]);
    let crc2 = bq769x0_async_rs::crc::calculate_crc(&[data_bytes[1]]);
    let crc3 = bq769x0_async_rs::crc::calculate_crc(&[data_bytes[2]]);
    let crc4 = bq769x0_async_rs::crc::calculate_crc(&[data_bytes[3]]);

    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![reg_addr],
        vec![
            data_bytes[0],
            crc1,
            data_bytes[1],
            crc2,
            data_bytes[2],
            crc3,
            data_bytes[3],
            crc4,
        ],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_registers(Register::Vc1Hi, 4);
    assert_eq!(result, Ok(Vec::from_slice(&data_bytes).unwrap()));
    i2c_mock.done();
}

#[test]
fn test_read_registers_enabled_crc_failure() {
    let reg_addr = Register::Vc1Hi as u8;
    let data_bytes = [0x12, 0x34, 0x56, 0x78];
    let slave_addr_read = (BQ76920_ADDR << 1) | 1;

    let crc1 = bq769x0_async_rs::crc::calculate_crc(&[slave_addr_read, data_bytes[0]]);
    let incorrect_crc2 = 0x00; // Intentionally incorrect CRC
    let crc3 = bq769x0_async_rs::crc::calculate_crc(&[data_bytes[2]]);
    let crc4 = bq769x0_async_rs::crc::calculate_crc(&[data_bytes[3]]);

    let expectations = [I2cTransaction::write_read(
        BQ76920_ADDR,
        vec![reg_addr],
        vec![
            data_bytes[0],
            crc1,
            data_bytes[1],
            incorrect_crc2, // This will cause CRC error
            data_bytes[2],
            crc3,
            data_bytes[3],
            crc4,
        ],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_registers(Register::Vc1Hi, 4);
    assert_eq!(result, Err(Error::Crc));
    i2c_mock.done();
}

#[test]
fn test_write_registers_enabled_crc_success() {
    let reg_addr = Register::CELLBAL1 as u8;
    let values = [0x01, 0x02, 0x03];
    let slave_addr_write = BQ76920_ADDR << 1;

    let crc_first = bq769x0_async_rs::crc::calculate_crc(&[slave_addr_write, reg_addr, values[0]]);
    let crc_second = bq769x0_async_rs::crc::calculate_crc(&[values[1]]);
    let crc_third = bq769x0_async_rs::crc::calculate_crc(&[values[2]]);

    let expectations = [I2cTransaction::write(
        BQ76920_ADDR,
        vec![
            reg_addr, values[0], crc_first, values[1], crc_second, values[2], crc_third,
        ],
    )];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.write_registers(Register::CELLBAL1, &values);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_init_sequence() {
    let expectations = [
        // Clear all status flags (SYS_STAT)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::SysStat as u8, 0b11111111]),
        // Set CC_CFG to 0x19 for optimal performance
        I2cTransaction::write(BQ76920_ADDR, vec![Register::CcCfg as u8, 0x19]),
        // Enable ADC and set TEMP_SEL to external thermistor (example)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![
                Register::SysCtrl1 as u8,
                (registers::SysCtrl1Flags::ADC_EN | registers::SysCtrl1Flags::TEMP_SEL).bits(),
            ],
        ),
        // Enable CC continuous readings (example)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![
                Register::SysCtrl2 as u8,
                registers::SysCtrl2Flags::CC_EN.bits(),
            ],
        ),
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = expect_init_sequence(&mut driver);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}
