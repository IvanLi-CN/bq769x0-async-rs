use crate::registers::{SysCtrl1Flags, SysCtrl2Flags, SysStatFlags};

#[cfg(feature = "binrw")]
use binrw::{BinRead, BinWrite};

// Helper functions for Option<u16> serialization/deserialization with binrw
#[cfg(feature = "binrw")]
mod binrw_option_helpers {
    use binrw::{BinRead, BinResult, BinWrite};

    #[binrw::writer(writer, endian)]
    pub fn write_optional_u16(opt: &Option<u16>) -> BinResult<()> {
        let is_some: u8 = if opt.is_some() { 1 } else { 0 };
        is_some.write_options(writer, endian, ())?;
        if let Some(value) = opt {
            value.write_options(writer, endian, ())?;
        }
        Ok(())
    }

    #[binrw::parser(reader, endian)]
    pub fn parse_optional_u16() -> BinResult<Option<u16>> {
        let is_some = u8::read_options(reader, endian, ())?;
        if is_some == 1 {
            Ok(Some(u16::read_options(reader, endian, ())?))
        } else {
            Ok(None)
        }
    }

    #[binrw::writer(writer, endian)]
    pub fn write_optional_i16(opt: &Option<i16>) -> BinResult<()> {
        let is_some: u8 = if opt.is_some() { 1 } else { 0 };
        is_some.write_options(writer, endian, ())?;
        if let Some(value) = opt {
            value.write_options(writer, endian, ())?;
        }
        Ok(())
    }

    #[binrw::parser(reader, endian)]
    pub fn parse_optional_i16() -> BinResult<Option<i16>> {
        let is_some = u8::read_options(reader, endian, ())?;
        if is_some == 1 {
            Ok(Some(i16::read_options(reader, endian, ())?))
        } else {
            Ok(None)
        }
    }
}

/// Represents the measured cell voltages.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct CellVoltages<const N: usize> {
    /// Voltage of cell 1 to 15, in millivolts (mV).
    /// The number of valid cells depends on the chip model (BQ76920: 5, BQ76930: 10, BQ76940: 15).
    pub voltages: [i32; N], // Converted cell voltage in mV
}

impl<const N: usize> Default for CellVoltages<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> CellVoltages<N> {
    pub fn new() -> Self {
        Self {
            voltages: [0i32; N], // Initialize with 0 mV
        }
    }
}

/// Represents the measured temperatures.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct TemperatureSensorReadings {
    /// Voltage from TS1 sensor (if external thermistor) or Die Temperature (if internal).
    pub ts1: u16, // Raw 14-bit ADC value (LSB: 382 µV)

    #[cfg_attr(feature = "binrw", br(parse_with = binrw_option_helpers::parse_optional_u16))]
    #[cfg_attr(feature = "binrw", bw(write_with = binrw_option_helpers::write_optional_u16))]
    pub ts2: Option<u16>, // Raw 14-bit ADC value (LSB: 382 µV)

    #[cfg_attr(feature = "binrw", br(parse_with = binrw_option_helpers::parse_optional_u16))]
    #[cfg_attr(feature = "binrw", bw(write_with = binrw_option_helpers::write_optional_u16))]
    pub ts3: Option<u16>, // Raw 14-bit ADC value (LSB: 382 µV)

    /// Indicates if the temperature readings are Die Temp (false) or Thermistor resistance (true).
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub is_thermistor: bool,
}

impl Default for TemperatureSensorReadings {
    fn default() -> Self {
        Self::new()
    }
}

impl TemperatureSensorReadings {
    /// Creates a new instance of `TemperatureSensorReadings` with default values.
    pub fn new() -> Self {
        Self {
            ts1: 0u16,
            ts2: None,
            ts3: None,
            is_thermistor: false,
        }
    }

    /// Converts the raw sensor voltage readings into temperature data.
    /// Requires NTC thermistor parameters if the readings are in thermistor mode.
    pub fn into_temperature_data(
        &self,
        ntc_params: Option<&NtcParameters>,
    ) -> Result<TemperatureData, &'static str> {
        if self.is_thermistor {
            if ntc_params.is_none() {
                return Err("NTC parameters are required for thermistor readings");
            }
            Err("Lookup table implementation is pending")
        } else {
            let v_25_uv = 1_200_000; // Voltage at 25°C for internal sensor, unit: µV
            let lsb_uv = 382; // ADC LSB for temperature sensors, unit: µV
            let divisor_uv_per_ccc = 42; // Divisor for internal sensor temperature calculation, unit: µV / (0.01 °C)

            let v_ts1_uv = self.ts1 as i32 * lsb_uv;
            let temp_diff_ts1_uv = v_ts1_uv - v_25_uv;
            let ts1_temp_cc = 2500 - (temp_diff_ts1_uv / divisor_uv_per_ccc); // Temperature, unit: 0.01 °C

            let ts2_temp_cc = self.ts2.map(|raw_adc| {
                let v_ts2_uv = raw_adc as i32 * lsb_uv;
                let temp_diff_ts2_uv = v_ts2_uv - v_25_uv;
                2500 - (temp_diff_ts2_uv / divisor_uv_per_ccc)
            });

            let ts3_temp_cc = self.ts3.map(|raw_adc| {
                let v_ts3_uv = raw_adc as i32 * lsb_uv;
                let temp_diff_ts3_uv = v_ts3_uv - v_25_uv;
                2500 - (temp_diff_ts3_uv / divisor_uv_per_ccc)
            });

            Ok(TemperatureData {
                ts1: ts1_temp_cc as i16,
                ts2: ts2_temp_cc.map(|t| t as i16),
                ts3: ts3_temp_cc.map(|t| t as i16),
            })
        }
    }
}

// Define a struct to hold NTC thermistor parameters
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct NtcParameters {
    pub b_value: f32,            // B-value of the NTC thermistor, unit: K (Kelvin)
    pub ref_temp_k: i16,         // Reference temperature for NTC, unit: K (Kelvin)
    pub ref_resistance_ohm: u32, // Reference NTC resistance at reference temperature, unit: Ω (Ohm)
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct TemperatureData {
    pub ts1: i16, // Temperature from TS1, unit: 0.01 °C
    #[cfg_attr(feature = "binrw", br(parse_with = binrw_option_helpers::parse_optional_i16))]
    #[cfg_attr(feature = "binrw", bw(write_with = binrw_option_helpers::write_optional_i16))]
    pub ts2: Option<i16>, // Temperature from TS2, unit: 0.01 °C
    #[cfg_attr(feature = "binrw", br(parse_with = binrw_option_helpers::parse_optional_i16))]
    #[cfg_attr(feature = "binrw", bw(write_with = binrw_option_helpers::write_optional_i16))]
    pub ts3: Option<i16>, // Temperature from TS3, unit: 0.01 °C
}

/// Represents the measured pack current from the Coulomb Counter.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct CoulombCounter {
    pub raw_cc: i16, // Raw Coulomb Counter ADC value (LSB: 8.44 µV). Current (mA) = raw_cc * 8.44 / Rsense (mΩ)
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
#[cfg_attr(feature = "binrw", br(repr = u8))]
#[cfg_attr(feature = "binrw", bw(repr = u8))]
pub enum TempSensor {
    Internal,
    External,
}

/// Represents the system status flags.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::new), bw(map = |&s: &Self| s.0.bits()))]
pub struct SystemStatus(pub SysStatFlags);

impl SystemStatus {
    pub fn new(status_byte: u8) -> Self {
        SystemStatus(SysStatFlags::from_bits_truncate(status_byte))
    }
}

/// Represents the charge/discharge MOS status.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::new), bw(map = |&s: &Self| s.0.bits()))]
pub struct MosStatus(pub SysCtrl2Flags);

impl MosStatus {
    pub fn new(sys_ctrl2_byte: u8) -> Self {
        MosStatus(SysCtrl2Flags::from_bits_truncate(sys_ctrl2_byte))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
#[cfg_attr(feature = "binrw", br(repr = u8))]
#[cfg_attr(feature = "binrw", bw(repr = u8))]
pub enum ScdDelay {
    Delay70us,
    Delay100us,
    Delay200us,
    Delay400us,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
#[cfg_attr(feature = "binrw", br(repr = u8))]
#[cfg_attr(feature = "binrw", bw(repr = u8))]
pub enum OcdDelay {
    Delay10ms,
    Delay20ms,
    Delay40ms,
    Delay80ms,
    Delay160ms,
    Delay320ms,
    Delay640ms,
    Delay1280ms,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
#[cfg_attr(feature = "binrw", br(repr = u8))]
#[cfg_attr(feature = "binrw", bw(repr = u8))]
pub enum UvOvDelay {
    Delay1s,
    Delay2s,
    Delay4s,
    Delay8s,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct ProtectionConfig {
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub rsns_enable: bool,
    pub scd_delay: ScdDelay,
    pub scd_limit: i32, // Short circuit discharge current limit, unit: mA
    pub ocd_delay: OcdDelay,
    pub ocd_limit: i32, // Overcurrent discharge current limit, unit: mA
    pub uv_delay: UvOvDelay,
    pub ov_delay: UvOvDelay,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct BatteryConfig {
    pub sys_ctrl1_flags: SysCtrl1Flags,
    pub sys_ctrl2_flags: SysCtrl2Flags,
    pub temp_sensor_selection: TempSensor,
    pub overvoltage_trip: u32,  // Overvoltage trip threshold, unit: mV
    pub undervoltage_trip: u32, // Undervoltage trip threshold, unit: mV
    pub protection_config: ProtectionConfig,
    pub rsense: u32, // Sense resistor value, unit: mΩ (milliOhms)
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            sys_ctrl1_flags: SysCtrl1Flags::ADC_EN,
            sys_ctrl2_flags: SysCtrl2Flags::CC_EN,
            temp_sensor_selection: TempSensor::Internal,
            overvoltage_trip: 4200u32,  // mV
            undervoltage_trip: 2800u32, // mV
            protection_config: ProtectionConfig {
                rsns_enable: true,
                scd_delay: ScdDelay::Delay70us,
                scd_limit: 60000i32, // mA
                ocd_delay: OcdDelay::Delay10ms,
                ocd_limit: 20000i32, // mA
                uv_delay: UvOvDelay::Delay1s,
                ov_delay: UvOvDelay::Delay1s,
            },
            rsense: 10u32, // mΩ
        }
    }
}

/// Represents the BQ76920 measurements.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct Bq76920Measurements<const N: usize> {
    pub cell_voltages: CellVoltages<N>,
    pub temperatures: TemperatureSensorReadings,
    pub current: i32, // Measured current, unit: mA
    pub system_status: SystemStatus,
    pub mos_status: MosStatus,
}

impl<const N: usize> Default for Bq76920Measurements<N> {
    fn default() -> Self {
        Self {
            cell_voltages: CellVoltages::new(),
            temperatures: TemperatureSensorReadings::new(),
            current: 0i32,
            system_status: SystemStatus::new(0),
            mos_status: MosStatus::new(0),
        }
    }
}
