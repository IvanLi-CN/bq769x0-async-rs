use crate::units::{
    ElectricCurrent, ElectricPotential, ElectricalResistance, ThermodynamicTemperature,
};
use uom::si::{
    electric_current::milliampere, electric_potential::millivolt, electrical_resistance::milliohm,
    thermodynamic_temperature::kelvin,
};

/// Represents the measured cell voltages.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CellVoltages<const N: usize> {
    /// Voltage of cell 1 to 15.
    /// The number of valid cells depends on the chip model (BQ76920: 5, BQ76930: 10, BQ76940: 15).
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
pub struct Temperatures {
    /// Temperature from TS1 sensor.
    pub ts1: ThermodynamicTemperature,
    /// Temperature from TS2 sensor (BQ76930/40 only).
    pub ts2: Option<ThermodynamicTemperature>,
    /// Temperature from TS3 sensor (BQ76940 only).
    pub ts3: Option<ThermodynamicTemperature>,
    /// Indicates if the temperature readings are Die Temp (false) or Thermistor resistance (true).
    pub is_thermistor: bool,
}

impl Default for Temperatures {
    fn default() -> Self {
        Self::new()
    }
}

impl Temperatures {
    pub fn new() -> Self {
        Self {
            ts1: ThermodynamicTemperature::new::<kelvin>(0.0),
            ts2: None,
            ts3: None,
            is_thermistor: false,
        }
    }
}

/// Represents the measured pack current from the Coulomb Counter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CoulombCounter {
    /// Raw Coulomb Counter value. Needs conversion based on CC_CFG and Rsense.
    pub raw_cc: i16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TempSensor {
    Internal,
    External,
}

/// Represents the system status flags.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SystemStatus {
    pub cc_ready: bool,
    pub ovr_temp: bool,
    pub uv: bool,
    pub ov: bool,
    pub scd: bool,
    pub ocd: bool,
    pub cuv: bool, // Cell Undervoltage
    pub cov: bool, // Cell Overvoltage
}

impl SystemStatus {
    pub fn new(status_byte: u8) -> Self {
        Self {
            cc_ready: (status_byte & 0b1000_0000) != 0,
            ovr_temp: (status_byte & 0b0100_0000) != 0,
            uv: (status_byte & 0b0010_0000) != 0,
            ov: (status_byte & 0b0001_0000) != 0,
            scd: (status_byte & 0b0000_1000) != 0,
            ocd: (status_byte & 0b0000_0100) != 0,
            cuv: (status_byte & 0b0000_0010) != 0,
            cov: (status_byte & 0b0000_0001) != 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScdDelay {
    Delay70us,
    Delay100us,
    Delay200us,
    Delay400us,
}

#[derive(Debug, Clone, Copy, PartialEq)]
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
pub enum UvOvDelay {
    Delay1s,
    Delay2s,
    Delay4s,
    Delay8s,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProtectionConfig {
    pub rsns_enable: bool,
    pub scd_delay: ScdDelay,
    pub scd_limit: ElectricCurrent, // 短路放电电流限制
    pub ocd_delay: OcdDelay,
    pub ocd_limit: ElectricCurrent, // 过流放电电流限制
    pub uv_delay: UvOvDelay,
    pub ov_delay: UvOvDelay,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryConfig {
    // SYS_CTRL1 related
    pub load_present: bool,
    pub adc_enable: bool,
    pub temp_sensor_selection: TempSensor,
    pub shutdown_a: bool,
    pub shutdown_b: bool,

    // SYS_CTRL2 related
    pub delay_disable: bool,
    pub cc_enable: bool,
    pub cc_oneshot: bool,
    pub discharge_on: bool,
    pub charge_on: bool,

    // Voltage thresholds
    pub overvoltage_trip: ElectricPotential,
    pub undervoltage_trip: ElectricPotential,

    // Protection configurations
    pub protection_config: ProtectionConfig,

    // Add Rsense value for current calculation
    pub rsense: ElectricalResistance, // 串联电阻值

                                      // Add other configuration fields as needed
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
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

/// Represents the charge/discharge MOS status.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MosStatus {
    pub charge_on: bool,
    pub discharge_on: bool,
}

impl MosStatus {
    pub fn new(sys_ctrl2_byte: u8) -> Self {
        Self {
            charge_on: (sys_ctrl2_byte & 0b0000_0001) != 0,
            discharge_on: (sys_ctrl2_byte & 0b0000_0010) != 0,
        }
    }
}

/// Represents the BQ76920 measurements.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Bq76920Measurements<const N: usize> {
    pub cell_voltages: CellVoltages<N>,
    pub temperatures: Temperatures,
    pub current: ElectricCurrent,
    pub system_status: SystemStatus,
    pub mos_status: MosStatus,
}

impl<const N: usize> Default for Bq76920Measurements<N> {
    fn default() -> Self {
        Self {
            cell_voltages: CellVoltages::new(),
            temperatures: Temperatures::new(),
            current: ElectricCurrent::new::<milliampere>(0.0),
            system_status: SystemStatus::new(0),
            mos_status: MosStatus::new(0),
        }
    }
}
