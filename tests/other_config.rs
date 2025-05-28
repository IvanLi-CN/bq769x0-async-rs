#![allow(unused_imports)]
#![allow(dead_code)]

#[path = "common.rs"]
mod common;

use approx::assert_relative_eq;
use bq769x0_async_rs::units::ElectricCurrent;
use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::{self, *},
    Bq769x0, RegisterAccess,
};
use common::{create_bq769x0_driver_disabled_crc, BQ76920_ADDR, BQ76930_ADDR, BQ76940_ADDR};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use heapless::Vec;
use uom::si::electric_current::milliampere;
use uom::si::electric_potential::millivolt;

#[test]
fn test_enter_ship_mode() {
    let expectations = [
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x00]), // SYS_CTRL1 = 0x00
        I2cTransaction::write(BQ76920_ADDR, vec![0x05, 0x00]), // SYS_CTRL2 = 0x00
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x01]), // SYS_CTRL1 = 0x01 (SHUT_B)
        I2cTransaction::write(BQ76920_ADDR, vec![0x04, 0x02]), // SYS_CTRL1 = 0x02 (SHUT_A)
    ];
    let (mut driver, i2c_mock) =
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
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
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.is_alert_overridden();
    assert_eq!(result, Ok(true));
    i2c_mock.done();

    let expectations_not_overridden = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysStat as u8], vec![0x00]), // OVRD_ALERT bit not set
    ];
    let (mut driver_not_overridden, i2c_mock_not_overridden) =
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations_not_overridden, BQ76920_ADDR);
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
