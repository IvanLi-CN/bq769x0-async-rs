extern crate defmt; // Make defmt available for derive macros

/// Represents the measured cell voltages.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CellVoltages<const N: usize> {
    /// Voltage of cell 1 to 15 in mV.
    /// The number of valid cells depends on the chip model (BQ76920: 5, BQ76930: 10, BQ76940: 15).
    pub voltages_mv: [u16; N],
}

/// Represents the measured temperatures.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Temperatures {
    /// Temperature from TS1 sensor in deci-Celsius (if Die Temp) or resistance in 0.1 Ohms (if Thermistor).
    pub ts1: i16,
    /// Temperature from TS2 sensor in deci-Celsius (if Die Temp) or resistance in 0.1 Ohms (if Thermistor) (BQ76930/40 only).
    pub ts2: Option<i16>,
    /// Temperature from TS3 sensor in deci-Celsius (if Die Temp) or resistance in 0.1 Ohms (if Thermistor) (BQ76940 only).
    pub ts3: Option<i16>,
    /// Indicates if the temperature readings are Die Temp (false) or Thermistor resistance (true).
    pub is_thermistor: bool,
}

/// Represents the measured pack current from the Coulomb Counter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Current {
    /// Raw Coulomb Counter value. Needs conversion based on CC_CFG and Rsense.
    pub raw_cc: i16,
}