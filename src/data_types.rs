use crate::registers::{SysCtrl1Flags, SysCtrl2Flags, SysStatFlags};
use crate::units::{ElectricCurrent, ElectricPotential, ElectricalResistance, TemperatureInterval};
use uom::si::electric_potential::volt;
use uom::si::temperature_interval::degree_celsius;
use uom::si::{
    electric_current::milliampere, electric_potential::millivolt, electrical_resistance::milliohm,
};

#[cfg(feature = "binrw")]
use binrw::{BinRead, BinWrite};

/// Represents the measured cell voltages.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct CellVoltages<const N: usize> {
    /// Voltage of cell 1 to 15.
    /// The number of valid cells depends on the chip model (BQ76920: 5, BQ76930: 10, BQ76940: 15).
    #[cfg_attr(feature = "binrw", br(map = |x: [f32; N]| x.map(|v| ElectricPotential::new::<millivolt>(v))))]
    #[cfg_attr(feature = "binrw", bw(map = |x: &[ElectricPotential; N]| x.map(|v| v.get::<millivolt>() )))]
    pub voltages: [ElectricPotential; N],
}

impl<const N: usize> Default for CellVoltages<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> CellVoltages<N> {
    pub fn new() -> Self {
        Self {
            voltages: [ElectricPotential::new::<millivolt>(0.0); N],
        }
    }
}

/// Represents the measured temperatures.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct TemperatureSensorReadings {
    /// Voltage from TS1 sensor (if external thermistor) or Die Temperature (if internal).
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricPotential::new::<millivolt>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<millivolt>()))]
    pub ts1: ElectricPotential,
    /// Voltage from TS2 sensor (BQ76930/40 only) or Die Temperature.
    #[cfg_attr(feature = "binrw", br(map = |x: f32| if x.is_nan() { None } else { Some(ElectricPotential::new::<millivolt>(x)) }))]
    #[cfg_attr(feature = "binrw", bw(map = |x: &Option<ElectricPotential>| x.map_or(f32::NAN, |v| v.get::<millivolt>())))]
    pub ts2: Option<ElectricPotential>,
    /// Voltage from TS3 sensor (BQ76940 only) or Die Temperature.
    #[cfg_attr(feature = "binrw", br(map = |x: f32| if x.is_nan() { None } else { Some(ElectricPotential::new::<millivolt>(x)) }))]
    #[cfg_attr(feature = "binrw", bw(map = |x: &Option<ElectricPotential>| x.map_or(f32::NAN, |v| v.get::<millivolt>())))]
    pub ts3: Option<ElectricPotential>,
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
            ts1: ElectricPotential::new::<uom::si::electric_potential::volt>(0.0),
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
            // External thermistor: calculate resistance, then temperature
            // External thermistor: calculate resistance, then temperature
            if ntc_params.is_none() {
                return Err("NTC parameters are required for thermistor readings");
            }

            // Convert resistance to temperature using B-value formula (without ln)
            // This part needs to be replaced with the lookup table logic
            // For now, I will keep the structure but comment out the ln-based calculation
            // and return a placeholder or error until the lookup table is implemented.

            // Placeholder for lookup table implementation
            Err("Lookup table implementation is pending")
        } else {
            // Internal die temperature: calculate temperature
            let ts1_temp_celsius = 25.0 - (self.ts1.get::<volt>() - 1.2) / 0.0042;
            let ts2_temp_celsius = self.ts2.map(|v| 25.0 - (v.get::<volt>() - 1.2) / 0.0042);
            let ts3_temp_celsius = self.ts3.map(|v| 25.0 - (v.get::<volt>() - 1.2) / 0.0042);

            Ok(TemperatureData {
                ts1: TemperatureInterval::new::<degree_celsius>(ts1_temp_celsius),
                ts2: ts2_temp_celsius.map(TemperatureInterval::new::<degree_celsius>),
                ts3: ts3_temp_celsius.map(TemperatureInterval::new::<degree_celsius>),
            })
        }
    }
}

// Define a struct to hold NTC thermistor parameters
#[derive(Debug, PartialEq)] // Added Debug and PartialEq
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct NtcParameters {
    pub b_value: f32, // B value
    #[cfg_attr(feature = "binrw", br(map = |x: f32| TemperatureInterval::new::<uom::si::temperature_interval::kelvin>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<uom::si::temperature_interval::kelvin>()))]
    pub ref_temp_k: TemperatureInterval, // Reference temperature in Kelvin
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricalResistance::new::<uom::si::electrical_resistance::ohm>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<uom::si::electrical_resistance::ohm>()))]
    pub ref_resistance_ohm: ElectricalResistance, // Reference resistance in Ohm
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct TemperatureData {
    #[cfg_attr(feature = "binrw", br(map = |x: f32| TemperatureInterval::new::<degree_celsius>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<degree_celsius>()))]
    pub ts1: TemperatureInterval,
    #[cfg_attr(feature = "binrw", br(map = |x: f32| if x.is_nan() { None } else { Some(TemperatureInterval::new::<degree_celsius>(x)) }))]
    #[cfg_attr(feature = "binrw", bw(map = |x: &Option<TemperatureInterval>| x.map_or(f32::NAN, |v| v.get::<degree_celsius>())))]
    pub ts2: Option<TemperatureInterval>,
    #[cfg_attr(feature = "binrw", br(map = |x: f32| if x.is_nan() { None } else { Some(TemperatureInterval::new::<degree_celsius>(x)) }))]
    #[cfg_attr(feature = "binrw", bw(map = |x: &Option<TemperatureInterval>| x.map_or(f32::NAN, |v| v.get::<degree_celsius>())))]
    pub ts3: Option<TemperatureInterval>,
}

/// Represents the measured pack current from the Coulomb Counter.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct CoulombCounter {
    /// Raw Coulomb Counter value. Needs conversion based on CC_CFG and Rsense.
    pub raw_cc: i16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(repr = u8), bw(repr = u8))]
pub enum TempSensor {
    Internal,
    External,
}

/// Represents the system status flags.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s| s.bits()))]
pub struct SystemStatus(pub SysStatFlags);

impl SystemStatus {
    pub fn new(status_byte: u8) -> Self {
        SystemStatus(SysStatFlags::from_bits_truncate(status_byte))
    }
}

/// Represents the charge/discharge MOS status.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s| s.bits()))]
pub struct MosStatus(pub SysCtrl2Flags);

impl MosStatus {
    pub fn new(sys_ctrl2_byte: u8) -> Self {
        MosStatus(SysCtrl2Flags::from_bits_truncate(sys_ctrl2_byte))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(repr = u8), bw(repr = u8))]
pub enum ScdDelay {
    Delay70us,
    Delay100us,
    Delay200us,
    Delay400us,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(repr = u8), bw(repr = u8))]
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
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(repr = u8), bw(repr = u8))]
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
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricCurrent::new::<milliampere>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<milliampere>()))]
    pub scd_limit: ElectricCurrent, // 短路放电电流限制
    pub ocd_delay: OcdDelay,
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricCurrent::new::<milliampere>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<milliampere>()))]
    pub ocd_limit: ElectricCurrent, // 过流放电电流限制
    pub uv_delay: UvOvDelay,
    pub ov_delay: UvOvDelay,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct BatteryConfig {
    pub sys_ctrl1_flags: SysCtrl1Flags,
    pub sys_ctrl2_flags: SysCtrl2Flags,
    pub temp_sensor_selection: TempSensor,

    // Voltage thresholds
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricPotential::new::<millivolt>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<millivolt>()))]
    pub overvoltage_trip: ElectricPotential,
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricPotential::new::<millivolt>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<millivolt>()))]
    pub undervoltage_trip: ElectricPotential,

    // Protection configurations
    pub protection_config: ProtectionConfig,

    // Add Rsense value for current calculation
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricalResistance::new::<milliohm>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<milliohm>()))]
    pub rsense: ElectricalResistance, // 串联电阻值

                                      // Add other configuration fields as needed
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            sys_ctrl1_flags: SysCtrl1Flags::ADC_EN, // ADC_EN is true by default
            sys_ctrl2_flags: SysCtrl2Flags::CC_EN,  // CC_EN is true by default
            temp_sensor_selection: TempSensor::Internal,
            overvoltage_trip: ElectricPotential::new::<millivolt>(4200.0),
            undervoltage_trip: ElectricPotential::new::<millivolt>(2800.0),
            protection_config: ProtectionConfig {
                rsns_enable: true,
                scd_delay: ScdDelay::Delay70us,
                scd_limit: ElectricCurrent::new::<milliampere>(60000.0), // Example default: 60A = 60000mA
                ocd_delay: OcdDelay::Delay10ms,
                ocd_limit: ElectricCurrent::new::<milliampere>(20000.0), // Example default: 20A = 20000mA
                uv_delay: UvOvDelay::Delay1s,
                ov_delay: UvOvDelay::Delay1s,
            },
            rsense: ElectricalResistance::new::<milliohm>(10.0), // Example default: 1mOhm (10 * 100uOhm)
        }
    }
}

/// Represents the BQ76920 measurements.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite))]
pub struct Bq76920Measurements<const N: usize> {
    pub cell_voltages: CellVoltages<N>,
    pub temperatures: TemperatureSensorReadings,
    #[cfg_attr(feature = "binrw", br(map = |x: f32| ElectricCurrent::new::<milliampere>(x)))]
    #[cfg_attr(feature = "binrw", bw(map = |x| x.get::<milliampere>()))]
    pub current: ElectricCurrent,
    pub system_status: SystemStatus,
    pub mos_status: MosStatus,
}

impl<const N: usize> Default for Bq76920Measurements<N> {
    fn default() -> Self {
        Self {
            cell_voltages: CellVoltages::new(),
            temperatures: TemperatureSensorReadings::new(),
            current: ElectricCurrent::new::<milliampere>(0.0),
            system_status: SystemStatus::new(0),
            mos_status: MosStatus::new(0),
        }
    }
}
