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
pub struct SystemStatus {
    pub cc_ready: bool,
    pub device_xready: bool, // Corresponds to SYS_STAT_DEVICE_XREADY (Bit 5)
    pub ovrd_alert: bool,    // Corresponds to SYS_STAT_OVRD_ALERT (Bit 4)
    pub uv: bool,            // Corresponds to SYS_STAT_UV (Bit 3)
    pub ov: bool,            // Corresponds to SYS_STAT_OV (Bit 2)
    pub scd: bool,           // Corresponds to SYS_STAT_SCD (Bit 1)
    pub ocd: bool,           // Corresponds to SYS_STAT_OCD (Bit 0)
    pub ovr_temp: bool,      // Corresponds to SYS_STAT_OVR_TEMP (Bit 6)
}

impl SystemStatus {
    pub fn new(status_byte: u8) -> Self {
        Self {
            cc_ready: (status_byte & 0b1000_0000) != 0,      // Bit 7
            device_xready: (status_byte & 0b0010_0000) != 0, // Bit 5
            ovrd_alert: (status_byte & 0b0001_0000) != 0,    // Bit 4
            uv: (status_byte & 0b0000_1000) != 0,            // Bit 3
            ov: (status_byte & 0b0000_0100) != 0,            // Bit 2
            scd: (status_byte & 0b0000_0010) != 0,           // Bit 1
            ocd: (status_byte & 0b0000_0001) != 0,           // Bit 0
            ovr_temp: (status_byte & 0b0100_0000) != 0,      // Bit 6
        }
    }
}

#[cfg(feature = "binrw")]
impl BinRead for SystemStatus {
    type Args<'a> = ();

    fn read_options<R: binrw::io::Read + binrw::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let status_byte = u8::read_options(reader, endian, ())?;
        Ok(SystemStatus::new(status_byte))
    }
}

#[cfg(feature = "binrw")]
impl BinWrite for SystemStatus {
    type Args<'a> = ();

    fn write_options<W: binrw::io::Write + binrw::io::Seek>(
        &self,
        writer: &mut W,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<()> {
        let mut status_byte = 0u8;
        if self.cc_ready {
            status_byte |= 0b1000_0000;
        }
        if self.device_xready {
            status_byte |= 0b0010_0000;
        }
        if self.ovrd_alert {
            status_byte |= 0b0001_0000;
        }
        if self.uv {
            status_byte |= 0b0000_1000;
        }
        if self.ov {
            status_byte |= 0b0000_0100;
        }
        if self.scd {
            status_byte |= 0b0000_0010;
        }
        if self.ocd {
            status_byte |= 0b0000_0001;
        }
        if self.ovr_temp {
            status_byte |= 0b0100_0000;
        }
        status_byte.write_options(writer, endian, ())
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
    // SYS_CTRL1 related
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub load_present: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub adc_enable: bool,
    pub temp_sensor_selection: TempSensor,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub shutdown_a: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub shutdown_b: bool,

    // SYS_CTRL2 related
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub delay_disable: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub cc_enable: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub cc_oneshot: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub discharge_on: bool,
    #[cfg_attr(feature = "binrw", br(map = |x: u8| x != 0))]
    #[cfg_attr(feature = "binrw", bw(map = |x| *x as u8))]
    pub charge_on: bool,

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

#[cfg(feature = "binrw")]
impl BinRead for MosStatus {
    type Args<'a> = ();

    fn read_options<R: binrw::io::Read + binrw::io::Seek>(
        reader: &mut R,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<Self> {
        let sys_ctrl2_byte = u8::read_options(reader, endian, ())?;
        Ok(MosStatus::new(sys_ctrl2_byte))
    }
}

#[cfg(feature = "binrw")]
impl BinWrite for MosStatus {
    type Args<'a> = ();

    fn write_options<W: binrw::io::Write + binrw::io::Seek>(
        &self,
        writer: &mut W,
        endian: binrw::Endian,
        _args: Self::Args<'_>,
    ) -> binrw::BinResult<()> {
        let mut sys_ctrl2_byte = 0u8;
        if self.charge_on {
            sys_ctrl2_byte |= 0b0000_0001;
        }
        if self.discharge_on {
            sys_ctrl2_byte |= 0b0000_0010;
        }
        sys_ctrl2_byte.write_options(writer, endian, ())
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
