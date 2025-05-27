#![allow(unused_imports)]
#![allow(dead_code)]

#[path = "common.rs"]
mod common;

use approx::assert_relative_eq;
use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::*,
    Bq769x0, RegisterAccess,
};
use common::{create_bq769x0_driver_disabled_crc, expect_init_sequence, BQ76920_ADDR};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use heapless::Vec;

#[test]
fn test_new_without_crc() {
    let expectations = [];
    let (_driver, i2c_mock) =
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    // No explicit assertions needed here, as `done()` will verify expectations.
    // The main goal is to ensure the driver can be instantiated.
    i2c_mock.done();
}

#[test]
fn test_new_with_crc() {
    let expectations = [];
    let (_driver, i2c_mock) =
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_enabled_crc::<5>(&expectations, BQ76920_ADDR);
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
                SYS_CTRL1_ADC_EN | SYS_CTRL1_TEMP_SEL,
            ],
        ),
        // Enable CC continuous readings (example)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::SysCtrl2 as u8, SYS_CTRL2_CC_EN],
        ),
    ];
    let (mut driver, i2c_mock) =
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = expect_init_sequence(&mut driver);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}
