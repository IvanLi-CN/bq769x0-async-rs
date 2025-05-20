#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Register {
    /// System Status Register
    SysStat = 0x00,
    /// Cell Balance 1 Register (Cells 1-5)
    CELLBAL1 = 0x01,
    /// Cell Balance 2 Register (Cells 6-10) - BQ76930/40 only
    CELLBAL2 = 0x02,
    /// Cell Balance 3 Register (Cells 11-15) - BQ76940 only
    CELLBAL3 = 0x03,
    /// System Control 1 Register
    SysCtrl1 = 0x04,
    /// System Control 2 Register
    SysCtrl2 = 0x05,
    /// Protection 1 Register (SCD)
    PROTECT1 = 0x06,
    /// Protection 2 Register (OCD)
    PROTECT2 = 0x07,
    /// Protection 3 Register (OV/UV Delay)
    PROTECT3 = 0x08,
    /// Overvoltage Trip Register
    OvTrip = 0x09,
    /// Undervoltage Trip Register
    UvTrip = 0x0A,
    /// Coulomb Counter Configuration Register
    CcCfg = 0x0B,
    /// Cell 1 Voltage High Byte
    Vc1Hi = 0x0C,
    /// Cell 1 Voltage Low Byte
    Vc1Lo = 0x0D,
    /// Cell 2 Voltage High Byte
    Vc2Hi = 0x0E,
    /// Cell 2 Voltage Low Byte
    Vc2Lo = 0x0F,
    /// Cell 3 Voltage High Byte
    Vc3Hi = 0x10,
    /// Cell 3 Voltage Low Byte
    Vc3Lo = 0x11,
    /// Cell 4 Voltage High Byte
    Vc4Hi = 0x12,
    /// Cell 4 Voltage Low Byte
    Vc4Lo = 0x13,
    /// Cell 5 Voltage High Byte
    Vc5Hi = 0x14,
    /// Cell 5 Voltage Low Byte
    Vc5Lo = 0x15,
    /// Cell 6 Voltage High Byte - BQ76930/40 only
    Vc6Hi = 0x16,
    /// Cell 6 Voltage Low Byte - BQ76930/40 only
    Vc6Lo = 0x17,
    /// Cell 7 Voltage High Byte - BQ76930/40 only
    Vc7Hi = 0x18,
    /// Cell 7 Voltage Low Byte - BQ76930/40 only
    Vc7Lo = 0x19,
    /// Cell 8 Voltage High Byte - BQ76930/40 only
    Vc8Hi = 0x1A,
    /// Cell 8 Voltage Low Byte - BQ76930/40 only
    Vc8Lo = 0x1B,
    /// Cell 9 Voltage High Byte - BQ76930/40 only
    Vc9Hi = 0x1C,
    /// Cell 9 Voltage Low Byte - BQ76930/40 only
    Vc9Lo = 0x1D,
    /// Cell 10 Voltage High Byte - BQ76930/40 only
    Vc10Hi = 0x1E,
    /// Cell 10 Voltage Low Byte - BQ76930/40 only
    Vc10Lo = 0x1F,
    /// Cell 11 Voltage High Byte - BQ76940 only
    Vc11Hi = 0x20,
    /// Cell 11 Voltage Low Byte - BQ76940 only
    Vc11Lo = 0x21,
    /// Cell 12 Voltage High Byte - BQ76940 only
    Vc12Hi = 0x22,
    /// Cell 12 Voltage Low Byte - BQ76940 only
    Vc12Lo = 0x23,
    /// Cell 13 Voltage High Byte - BQ76940 only
    Vc13Hi = 0x24,
    /// Cell 13 Voltage Low Byte - BQ76940 only
    Vc13Lo = 0x25,
    /// Cell 14 Voltage High Byte - BQ76940 only
    Vc14Hi = 0x26,
    /// Cell 14 Voltage Low Byte - BQ76940 only
    Vc14Lo = 0x27,
    /// Cell 15 Voltage High Byte - BQ76940 only
    Vc15Hi = 0x28,
    /// Cell 15 Voltage Low Byte - BQ76940 only
    Vc15Lo = 0x29,
    /// Battery Voltage High Byte
    BatHi = 0x2A,
    /// Battery Voltage Low Byte
    BatLo = 0x2B,
    /// Temperature Sensor 1 High Byte (or Die Temp)
    Ts1Hi = 0x2C,
    /// Temperature Sensor 1 Low Byte (or Die Temp)
    Ts1Lo = 0x2D,
    /// Temperature Sensor 2 High Byte (or Die Temp) - BQ76930/40 only
    Ts2Hi = 0x2E,
    /// Temperature Sensor 2 Low Byte (or Die Temp) - BQ76930/40 only
    Ts2Lo = 0x2F,
    /// Temperature Sensor 3 High Byte (or Die Temp) - BQ76940 only
    Ts3Hi = 0x30,
    /// Temperature Sensor 3 Low Byte (or Die Temp) - BQ76940 only
    Ts3Lo = 0x31,
    /// Coulomb Counter High Byte
    CcHi = 0x32,
    /// Coulomb Counter Low Byte
    CcLo = 0x33,
    /// ADC Gain 1 Register
    ADCGAIN1 = 0x50,
    /// ADC Offset Register
    ADCOFFSET = 0x51,
    /// ADC Gain 2 Register
    ADCGAIN2 = 0x59,
}

// SYS_STAT register bit masks
pub const SYS_STAT_CC_READY: u8 = 1 << 7;
pub const SYS_STAT_DEVICE_XREADY: u8 = 1 << 5;
pub const SYS_STAT_OVRD_ALERT: u8 = 1 << 4;
pub const SYS_STAT_UV: u8 = 1 << 3;
pub const SYS_STAT_OV: u8 = 1 << 2;
pub const SYS_STAT_SCD: u8 = 1 << 1;
pub const SYS_STAT_OCD: u8 = 1 << 0;

// SYS_CTRL1 register bit masks
pub const SYS_CTRL1_LOAD_PRESENT: u8 = 1 << 7;
pub const SYS_CTRL1_ADC_EN: u8 = 1 << 4;
pub const SYS_CTRL1_TEMP_SEL: u8 = 1 << 3;
pub const SYS_CTRL1_SHUT_A: u8 = 1 << 1;
pub const SYS_CTRL1_SHUT_B: u8 = 1 << 0;

// SYS_CTRL2 register bit masks
pub const SYS_CTRL2_DELAY_DIS: u8 = 1 << 7;
pub const SYS_CTRL2_CC_EN: u8 = 1 << 6;
pub const SYS_CTRL2_CC_ONESHOT: u8 = 1 << 5;
pub const SYS_CTRL2_DSG_ON: u8 = 1 << 1;
pub const SYS_CTRL2_CHG_ON: u8 = 1 << 0;

// PROTECT1 register bit masks (SCD)
pub const PROTECT1_RSNS: u8 = 1 << 7; // RSNS bit
pub const PROTECT1_SCD_DELAY: u8 = 0b11 << 3; // SCD_D1:0 bits
pub const PROTECT1_SCD_THRESH: u8 = 0b111 << 0; // SCD_T2:0 bits

// PROTECT2 register bit masks (OCD)
pub const PROTECT2_OCD_DELAY: u8 = 0b111 << 4; // OCD_D2:0 bits
pub const PROTECT2_OCD_THRESH: u8 = 0b1111 << 0; // OCD_T3:0 bits

// PROTECT3 register bit masks (OV/UV Delay)
pub const PROTECT3_UV_DELAY: u8 = 0b11 << 6; // UV_D1:0 bits
pub const PROTECT3_OV_DELAY: u8 = 0b11 << 4; // OV_D1:0 bits

// CELLBAL1 register bit masks (Cells 1-5)
pub const CELLBAL1_BAL1: u8 = 1 << 0;
pub const CELLBAL1_BAL2: u8 = 1 << 1;
pub const CELLBAL1_BAL3: u8 = 1 << 2;
pub const CELLBAL1_BAL4: u8 = 1 << 3;
pub const CELLBAL1_BAL5: u8 = 1 << 4;

// CELLBAL2 register bit masks (Cells 6-10) - BQ76930/40 only
pub const CELLBAL2_BAL6: u8 = 1 << 0;
pub const CELLBAL2_BAL7: u8 = 1 << 1;
pub const CELLBAL2_BAL8: u8 = 1 << 2;
pub const CELLBAL2_BAL9: u8 = 1 << 3;
pub const CELLBAL2_BAL10: u8 = 1 << 4;

// CELLBAL3 register bit masks (Cells 11-15) - BQ76940 only
pub const CELLBAL3_BAL11: u8 = 1 << 0;
pub const CELLBAL3_BAL12: u8 = 1 << 1;
pub const CELLBAL3_BAL13: u8 = 1 << 2;
pub const CELLBAL3_BAL14: u8 = 1 << 3;
pub const CELLBAL3_BAL15: u8 = 1 << 4;

// Number of cells
#[cfg(feature = "bq76920")]
pub const NUM_CELLS: usize = 5;
#[cfg(feature = "bq76930")]
pub const NUM_CELLS: usize = 10;
#[cfg(feature = "bq76940")]
pub const NUM_CELLS: usize = 15;