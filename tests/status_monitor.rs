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
use common::{create_bq769x0_driver_disabled_crc, BQ76920_ADDR};
use embedded_hal::i2c::{ErrorType, I2c, Operation};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use heapless::Vec;
use uom::si::electric_current::milliampere;
use uom::si::electric_potential::millivolt;
use uom::si::thermodynamic_temperature::kelvin;

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
    let (mut driver, i2c_mock) = common::create_driver_with_adc_calibration(&expectations);
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

#[test]
fn test_read_pack_voltage() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::BatHi as u8], vec![0x0C, 0x00]), // Raw 3072
        // ADC calibration reads
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x00]), // adc_gain_raw = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x00]), // adc_offset_signed = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x00]), // adc_gain_raw = 0
    ];
    // ADCGAIN = 365 uV/LSB, ADCOFFSET = 0 mV
    let (mut driver, i2c_mock) = common::create_driver_with_adc_calibration(&expectations);
    let result = driver.read_pack_voltage();
    assert!(result.is_ok());
    let pack_voltage = result.unwrap();
    // V(BAT) = 4 * GAIN * ADC(cell) + (#Cells x OFFSET)
    // GAIN = 365 uV/LSB = 0.365 mV/LSB
    // OFFSET = 0 mV
    // N = 5 (for BQ76920)
    // pack_voltage_mv = 4 * 0.365 * 3072 + 5 * 0 = 1.46 * 3072 = 4485.12 mV
    assert_relative_eq!(pack_voltage.get::<millivolt>(), 4485.12, epsilon = 0.01);
    i2c_mock.done();
}

#[test]
fn test_read_temperatures_die_temp() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysCtrl1 as u8], vec![0x00]), // TEMP_SEL = 0 (Die Temp)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::Ts1Hi as u8], vec![0x0C, 0x45]), // Raw 3141 (1.2V) for 25C
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_temperatures();
    assert!(result.is_ok());
    let temperatures = result.unwrap();
    // V_TSX = 3141 * 382 uV/LSB = 1200002 uV = 1200.002 mV
    assert_relative_eq!(
        temperatures.ts1.get::<millivolt>(),
        1199.862, // Expected V_TSX for ADC = 3141
        epsilon = 0.01
    );
    assert_eq!(temperatures.is_thermistor, false);
    i2c_mock.done();
}

#[test]
fn test_read_temperatures_external_thermistor() {
    let expectations = [
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysCtrl1 as u8],
            vec![SYS_CTRL1_TEMP_SEL],
        ), // TEMP_SEL = 1 (External Thermistor)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::Ts1Hi as u8], vec![0x04, 0xB0]), // Raw 1200
        // ADC calibration reads for external thermistor calculation (not directly used for V_TSX, but for completeness)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x00]), // adc_gain_raw = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x00]), // adc_offset_signed = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x00]), // adc_gain_raw = 0
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_temperatures();
    assert!(result.is_ok());
    let temperatures = result.unwrap();
    // V_TSX = 1200 * 382 uV/LSB = 458.4 mV
    assert_relative_eq!(
        temperatures.ts1.get::<millivolt>(),
        1200.0 * 365.0 / 1000.0,
        epsilon = 0.01
    ); // V_TSX = 1200 * (365 uV/LSB) = 438000 uV = 438.0 mV
    assert_eq!(temperatures.is_thermistor, true);
    i2c_mock.done();
}

#[test]
fn test_read_current() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::CcHi as u8], vec![0x01, 0x00]), // Raw CC = 256 (signed)
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_current();
    assert_eq!(result, Ok(CoulombCounter { raw_cc: 256 }));
    i2c_mock.done();
}

#[test]
fn test_read_status() {
    let expectations = [
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysStat as u8],
            vec![0b00110101],
        ), // Example status byte (device_xready = true)
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_status();
    assert!(result.is_ok());
    let status = result.unwrap();
    assert_eq!(status.cc_ready, false);
    assert_eq!(status.device_xready, true);
    assert_eq!(status.ovrd_alert, true); // Corrected assertion
    assert_eq!(status.uv, false);
    assert_eq!(status.ov, true);
    assert_eq!(status.scd, false);
    assert_eq!(status.ocd, true);
    i2c_mock.done();
}

#[test]
fn test_clear_status_flags() {
    let expectations = [
        I2cTransaction::write(
            BQ76920_ADDR,
            vec![Register::SysStat as u8, SYS_STAT_UV | SYS_STAT_OV],
        ), // Clear UV and OV flags
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.clear_status_flags(SYS_STAT_UV | SYS_STAT_OV);
    assert_eq!(result, Ok(()));
    i2c_mock.done();
}

#[test]
fn test_read_all_measurements() {
    let expectations = [
        // read_cell_voltages:
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::Vc1Hi as u8],
            vec![
                0x0C, 0x00, // Cell 1: 3072 raw
                0x0C, 0x00, // Cell 2: 3072 raw
                0x0C, 0x00, // Cell 3: 3072 raw
                0x0C, 0x00, // Cell 4: 3072 raw
                0x0C, 0x00, // Cell 5: 3072 raw
            ],
        ),
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x00]), // adc_gain_raw = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x00]), // adc_offset_signed = 0
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x00]), // adc_gain_raw = 0
        // read_temperatures:
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::SysCtrl1 as u8], vec![0x00]), // TEMP_SEL = 0 (Die Temp)
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::Ts1Hi as u8], vec![0x0C, 0x45]), // TS1_HI/LO (3141)
        // read_current:
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::CcHi as u8], vec![0x01, 0x00]), // CC_HI/LO (256)
        // read_status:
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysStat as u8],
            vec![0b00110101],
        ), // Example status byte (device_xready = true)
        // read_mos_status:
        I2cTransaction::write_read(
            BQ76920_ADDR,
            vec![Register::SysCtrl2 as u8],
            vec![0b00000011],
        ), // Charge and Discharge ON
    ];
    let (mut driver, i2c_mock) =
        common::create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);
    let result = driver.read_all_measurements();
    assert!(result.is_ok());
    let measurements = result.unwrap();
    assert_relative_eq!(
        measurements.cell_voltages.voltages[0].get::<millivolt>(),
        3072.0 * 0.365,
        epsilon = 0.01
    );
    // Pack Voltage calculation based on Datasheet section 8.3.1.1.6: V(BAT) = 4 * GAIN * ADC(cell) + (#Cells x OFFSET)
    // Die Temp: V_TSX = 3141 * 382 uV/LSB = 1,199,862 uV ≈ 1200 mV
    // Die Temp calculation based on Datasheet section 8.1.2.4: TEMP_DIE = 25° - ((V_TSX - V_25) ÷ 0.0042)
    // V_TSX = ADC(TS) * 382 μV/LSB
    // V_25 = 1.200 V
    // For ADC = 3141:
    // V_TSX = 3141 * 382 μV = 1,199,862 μV = 1199.862 mV
    // TEMP_DIE = 25 - ((1199.862 - 1200) / 0.0042) ≈ 25 - (-0.138 / 0.0042) ≈ 25 + 32.857 ≈ 57.857 °C
    assert_relative_eq!(
        measurements.temperatures.ts1.get::<millivolt>(), // Asserting voltage in millivolts
        1199.862,                                         // Expected V_TSX for ADC = 3141
        epsilon = 0.01                                    // Adjust epsilon as needed for precision
    );
    // Current: CC Reading = 256 * 8.44 uV/LSB = 2160.64 uV = 2.16064 mV
    // Assuming RSENSE = 10 mOhm (from protection_config.rs)
    // Current = 2.16064 mV / 10 mOhm = 0.216064 A = 216.064 mA
    assert_relative_eq!(
        measurements.current.get::<milliampere>(),
        216.064,
        epsilon = 0.00001
    );
    assert_eq!(measurements.system_status.device_xready, true);
    assert_eq!(measurements.mos_status.charge_on, true);
    assert_eq!(measurements.mos_status.discharge_on, true);
    i2c_mock.done();
}

#[test]
fn test_read_adc_gain_offset_registers() {
    let expectations = [
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN1 as u8], vec![0x04]), // Example raw gain
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCOFFSET as u8], vec![0x10]), // Example raw offset
        I2cTransaction::write_read(BQ76920_ADDR, vec![Register::ADCGAIN2 as u8], vec![0x20]), // Example raw gain
    ];
    let (mut driver, i2c_mock) =
        create_bq769x0_driver_disabled_crc::<5>(&expectations, BQ76920_ADDR);

    let gain1_result = driver.read_register(Register::ADCGAIN1);
    assert_eq!(gain1_result, Ok(0x04));

    let offset_result = driver.read_register(Register::ADCOFFSET);
    assert_eq!(offset_result, Ok(0x10));

    let gain2_result = driver.read_register(Register::ADCGAIN2);
    assert_eq!(gain2_result, Ok(0x20));

    i2c_mock.done();
}
