#![allow(unused_imports)]
#![allow(dead_code)]

#[path = "common.rs"]
mod common;

use approx::assert_relative_eq;
use bq769x0_async_rs::units::{ElectricCurrent, ElectricPotential, ElectricalResistance};
use bq769x0_async_rs::{
    crc::{CrcMode, Disabled, Enabled},
    data_types::*,
    errors::Error,
    registers::*,
    Bq769x0, RegisterAccess,
};
use common::{create_bq769x0_driver_disabled_crc, BQ76920_ADDR};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use heapless::Vec;
use uom::si::{
    electric_current::milliampere, electric_potential::millivolt, electrical_resistance::milliohm,
};

// Helper function to create a driver with expected ADC calibration values
fn create_driver_with_adc_calibration(
    expectations: &[I2cTransaction],
    adc_gain_raw: u8,
    adc_offset_signed: i8,
) -> (Bq769x0<common::MockI2c, Disabled, 5>, common::MockI2c) {
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

    let i2c_mock = common::MockI2c::new(&full_expectations);
    let driver = Bq769x0::new_without_crc(i2c_mock, BQ76920_ADDR);
    (driver, common::MockI2c::new(&full_expectations)) // Return a new mock for verification
}

#[test]
fn test_set_config_basic() {
    let config = BatteryConfig {
        load_present: false,
        adc_enable: true,
        temp_sensor_selection: TempSensor::Internal,
        shutdown_a: false,
        shutdown_b: false,
        delay_disable: false,
        cc_enable: true,
        cc_oneshot: false,
        discharge_on: false,
        charge_on: false,
        overvoltage_trip: ElectricPotential::new::<millivolt>(4200.0),
        undervoltage_trip: ElectricPotential::new::<millivolt>(2800.0),
        protection_config: ProtectionConfig {
            rsns_enable: true,
            scd_delay: ScdDelay::Delay70us,
            scd_limit: ElectricCurrent::new::<milliampere>(60000.0), // 60A
            ocd_delay: OcdDelay::Delay10ms,
            ocd_limit: ElectricCurrent::new::<milliampere>(20000.0), // 20A
            uv_delay: UvOvDelay::Delay1s,
            ov_delay: UvOvDelay::Delay1s,
        },
        rsense: ElectricalResistance::new::<milliohm>(10.0), // 10mOhm
    };

    let expectations = [
        // SYS_CTRL1 write
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::SysCtrl1 as u8, SYS_CTRL1_ADC_EN],
        ),
        // SYS_CTRL2 write
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::SysCtrl2 as u8, SYS_CTRL2_CC_EN],
        ),
        // ADC calibration reads for OV/UV trip calculation
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x00]), // ADCGAIN1 (raw 0)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x00]), // ADCOFFSET (raw 0)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x00]), // ADCGAIN2 (raw 0)
        // OV_TRIP write (4200mV, gain 365uV/LSB, offset 0mV -> raw 11506.8 -> 0x2D0A -> 0xD0)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::OvTrip as u8, 0xFF]), // OV_TRIP (4200mV -> 0xFF)
        // UV_TRIP write (2800mV, gain 365uV/LSB, offset 0mV -> raw 7671.2 -> 0x1E07 -> 0xE0)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::UvTrip as u8, 0xFF]), // UV_TRIP (2800mV -> 0xFF)
        // PROTECT1 write (RSNS=1, SCD_DELAY=70us, SCD_THRESH=closest to 60A*10mOhm=600mV)
        // SCD_THRESH for RSNS=1: [44, 67, 89, 111, 133, 155, 178, 200] mV
        // 600mV is far, so it will pick 200mV (bits 7)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![
                Register::PROTECT1 as u8,
                PROTECT1_RSNS | (0b00 << 3) | 0b111,
            ],
        ), // SCD_DELAY_70US (0b00), SCD_THRESH (0b111 for 200mV)
        // PROTECT2 write (OCD_DELAY=10ms, OCD_THRESH=closest to 20A*10mOhm=200mV)
        // OCD_THRESH for RSNS=1: [17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100] mV
        // 200mV is far, so it will pick 100mV (bits 15)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::PROTECT2 as u8, (0b000 << 4) | 0b1111],
        ), // OCD_DELAY_10MS (0b000), OCD_THRESH (0b1111 for 100mV)
        // PROTECT3 write (UV_DELAY=1s, OV_DELAY=1s)
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::PROTECT3 as u8, (0b00 << 6) | (0b00 << 4)],
        ),
        // Clear all status flags (SYS_STAT)
        I2cTransaction::write(BQ76920_ADDR, vec![Register::SysStat as u8, 0b11111111]),
        // Set CC_CFG to 0x19 for optimal performance
        I2cTransaction::write(BQ76920_ADDR, vec![Register::CcCfg as u8, 0x19]),
    ];

    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.set_config(&config);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

// TODO: Add tests for configure_protect1, configure_protect2, configure_protect3, OV_TRIP, UV_TRIP
