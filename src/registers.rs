#[cfg(feature = "binrw")]
use binrw::{BinRead, BinWrite};
use bitflags::bitflags;

#[derive(Clone, Copy, Debug, PartialEq)] // Added Debug and PartialEq for consistency and potential future use
#[repr(u8)]
#[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(repr = u8), bw(repr = u8))]
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

bitflags! {
    /// System Status Register Flags
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct SysStatFlags: u8 {
        /// CC Ready Flag
        const CC_READY = 1 << 7;
        /// Device XREADY Flag
        const DEVICE_XREADY = 1 << 5;
        /// OVRD_ALERT Flag
        const OVRD_ALERT = 1 << 4;
        /// Undervoltage Flag
        const UV = 1 << 3;
        /// Overvoltage Flag
        const OV = 1 << 2;
        /// Short Circuit Detection Flag
        const SCD = 1 << 1;
        /// Overcurrent Detection Flag
        const OCD = 1 << 0;
    }
}

bitflags! {
    /// System Control 1 Register Flags
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct SysCtrl1Flags: u8 {
        /// Load Present Flag
        const LOAD_PRESENT = 1 << 7;
        /// ADC Enable
        const ADC_EN = 1 << 4;
        /// Temperature Sensor Select
        const TEMP_SEL = 1 << 3;
        /// Shutdown A
        const SHUT_A = 1 << 1;
        /// Shutdown B
        const SHUT_B = 1 << 0;
    }
}

bitflags! {
    /// System Control 2 Register Flags
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct SysCtrl2Flags: u8 {
        /// Delay Disable
        const DELAY_DIS = 1 << 7;
        /// Coulomb Counter Enable
        const CC_EN = 1 << 6;
        /// Coulomb Counter One-Shot
        const CC_ONESHOT = 1 << 5;
        /// Discharge FET On
        const DSG_ON = 1 << 1;
        /// Charge FET On
        const CHG_ON = 1 << 0;
    }
}

bitflags! {
    /// Protection 1 Register Flags (SCD)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct Protect1Flags: u8 {
        /// RSNS bit
        const RSNS = 1 << 7;
        /// SCD_D1:0 bits (Short Circuit Delay)
        const SCD_DELAY_MASK = 0b11 << 3;
        /// SCD_T2:0 bits (Short Circuit Threshold)
        const SCD_THRESH_MASK = 0b111;
    }
}

bitflags! {
    /// Protection 2 Register Flags (OCD)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct Protect2Flags: u8 {
        /// OCD_D2:0 bits (Overcurrent Delay)
        const OCD_DELAY_MASK = 0b111 << 4;
        /// OCD_T3:0 bits (Overcurrent Threshold)
        const OCD_THRESH_MASK = 0b1111;
    }
}

bitflags! {
    /// Protection 3 Register Flags (OV/UV Delay)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct Protect3Flags: u8 {
        /// UV_D1:0 bits (Undervoltage Delay)
        const UV_DELAY_MASK = 0b11 << 6;
        /// OV_D1:0 bits (Overvoltage Delay)
        const OV_DELAY_MASK = 0b11 << 4;
    }
}

bitflags! {
    /// Cell Balance 1 Register Flags (Cells 1-5)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct CellBal1Flags: u8 {
        const BAL1 = 1 << 0;
        const BAL2 = 1 << 1;
        const BAL3 = 1 << 2;
        const BAL4 = 1 << 3;
        const BAL5 = 1 << 4;
    }
}

bitflags! {
    /// Cell Balance 2 Register Flags (Cells 6-10)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct CellBal2Flags: u8 {
        const BAL6 = 1 << 0;
        const BAL7 = 1 << 1;
        const BAL8 = 1 << 2;
        const BAL9 = 1 << 3;
        const BAL10 = 1 << 4;
    }
}

bitflags! {
    /// Cell Balance 3 Register Flags (Cells 11-15)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[cfg_attr(feature = "binrw", derive(BinRead, BinWrite), br(map = Self::from_bits_truncate), bw(map = |&s: &Self| s.bits()))]
    pub struct CellBal3Flags: u8 {
        const BAL11 = 1 << 0;
        const BAL12 = 1 << 1;
        const BAL13 = 1 << 2;
        const BAL14 = 1 << 3;
        const BAL15 = 1 << 4;
    }
}
