#![no_std]
extern crate defmt; // Make defmt available for derive macros

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

use core::{marker::PhantomData, ops::Deref};
use heapless::Vec;

pub mod registers;
use registers::*; // Import bit masks

mod crc;
mod data_types;
mod errors;

pub use crc::{calculate_crc, CrcMode, Disabled, Enabled};
pub use data_types::{CellVoltages, Current, Temperatures};
pub use errors::Error;

#[derive(Debug)] // Add Debug for easy printing if needed
pub struct BatteryConfig {
    pub sys_ctrl1_val: u8,
    pub sys_ctrl2_val: u8,
    pub ov_trip_8bit: u8,
    pub uv_trip_8bit: u8,
    pub protect1_rsense: u8,
    pub protect1_scd_delay: u8,
    pub protect1_scd_threshold: u8,
    pub protect2_ocd_delay: u8,
    pub protect2_ocd_threshold: u8,
    pub protect3_uv_delay: u8,
    pub protect3_ov_delay: u8,
    pub cc_cfg_val: u8,
}

/// BQ769x0 driver
pub struct Bq769x0<I2C, M: CrcMode>
where
    I2C: I2c,
{
    address: u8,
    i2c: I2C,
    _crc_mode: core::marker::PhantomData<M>,
}

/// Trait for abstracting register access, with or without CRC.
pub trait RegisterAccess<E>
where
    Self: Sized,
{
    /// The buffer type used for reading multiple registers.
    type ReadBuffer: Deref<Target = [u8]>;

    /// Reads a single byte from the specified register.
    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>>;

    /// Reads multiple bytes starting from the specified register.
    async fn read_registers(
        &mut self,
        reg: Register,
        len: usize,
    ) -> Result<Self::ReadBuffer, Error<E>>;

    /// Writes a single byte to the specified register.
    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>>;

    /// Writes multiple bytes starting from the specified register.
    async fn write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>>;
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Bq769x0<I2C, Disabled>",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> Bq769x0<I2C, Disabled>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new instance of the BQ769x0 driver in Disabled CRC mode.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C peripheral.
    /// * `address` - The I2C address of the BQ769x0 chip.
    pub fn new_without_crc(i2c: I2C, address: u8) -> Self {
        Self {
            address,
            i2c,
            _crc_mode: core::marker::PhantomData,
        }
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Bq769x0<I2C, Disabled>",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> RegisterAccess<E> for Bq769x0<I2C, Disabled>
where
    I2C: I2c<Error = E>,
{
    type ReadBuffer = heapless::Vec<u8, 30>;

    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg as u8], &mut data)
            .await
            .map_err(Error::I2c)?;
        Ok(data[0])
    }

    async fn read_registers(
        &mut self,
        reg: Register,
        len: usize,
    ) -> Result<Self::ReadBuffer, Error<E>> {
        let mut data: heapless::Vec<u8, 30> = heapless::Vec::new();
        data.resize(len, 0).map_err(|_| Error::InvalidData)?; // Ensure buffer is large enough
        self.i2c
            .write_read(self.address, &[reg as u8], &mut data)
            .await
            .map_err(Error::I2c)?;
        Ok(data)
    }

    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg as u8, value])
            .await
            .map_err(Error::I2c)
    }

    async fn write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>> {
        let mut data: heapless::Vec<u8, 31> = heapless::Vec::new(); // Use heapless::Vec
        data.push(reg as u8).map_err(|_| Error::InvalidData)?;
        data.extend_from_slice(values)
            .map_err(|_| Error::InvalidData)?;
        self.i2c
            .write(self.address, &data)
            .await
            .map_err(Error::I2c)
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Bq769x0<I2C, Enabled>",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> Bq769x0<I2C, Enabled>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new instance of the BQ769x0 driver in Enabled CRC mode.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C peripheral.
    /// * `address` - The I2C address of the BQ769x0 chip.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            address,
            i2c,
            _crc_mode: core::marker::PhantomData,
        }
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Bq769x0<I2C, Disabled>",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> RegisterAccess<E> for Bq769x0<I2C, Enabled>
where
    I2C: I2c<Error = E>,
{
    type ReadBuffer = heapless::Vec<u8, 30>;

    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>> {
        let mut data = [0u8; 2]; // Data byte + CRC byte
        self.i2c
            .write_read(self.address, &[reg as u8], &mut data)
            .await
            .map_err(Error::I2c)?;

        let received_data = data[0];
        let received_crc = data[1];

        // CRC calculation for single byte read: slave address + data byte
        let crc_data = [self.address << 1 | 1, received_data];

        let calculated_crc = crc::calculate_crc(&crc_data); // Updated call

        if calculated_crc != received_crc {
            // CRC validation failed
            #[cfg(feature = "defmt")]
            defmt::error!(
                "CRC validation failed. {} Received CRC: {:02x}, Calculated CRC: {:02x}",
                reg as u8,
                received_crc,
                calculated_crc
            );
            return Err(Error::CrcError);
        }

        Ok(received_data)
    }

    async fn read_registers(
        &mut self,
        reg: Register,
        len: usize,
    ) -> Result<Self::ReadBuffer, Error<E>> {
        if len == 0 || len > 30 {
            // BQ769x0 block reads are typically up to 30 bytes (e.g., cell voltages)
            // Adjust max size if needed based on specific use cases, but 30 is common.
            #[cfg(feature = "defmt")]
            defmt::error!("Invalid read length: {}", len);
            return Err(Error::InvalidData);
        }

        let mut raw_data_buffer: heapless::Vec<u8, 60> = heapless::Vec::new(); // len * 2 bytes for data + CRC
        raw_data_buffer
            .resize(len * 2, 0)
            .map_err(|_| Error::InvalidData)?;

        // Read data byte and CRC byte
        self.i2c
            .write_read(self.address, &[reg as u8], &mut raw_data_buffer)
            .await
            .map_err(Error::I2c)?;

        let mut result_buffer: heapless::Vec<u8, 30> = heapless::Vec::new();
        result_buffer.resize(len, 0).map_err(|_| Error::InvalidData)?;


        for i in 0..len {
            let data_byte = raw_data_buffer[i * 2];
            let received_crc = raw_data_buffer[i * 2 + 1];

            let calculated_crc = if i == 0 {
                // First data byte CRC: slave address + data byte
                let crc_data = [
                    self.address << 1 | 1, // Slave address + Write bit
                    data_byte,
                ];
                crc::calculate_crc(&crc_data) // Updated call
            } else {
                // Subsequent data bytes CRC: only the data byte
                crc::calculate_crc(&[data_byte]) // Updated call
            };

            if calculated_crc != received_crc {
                #[cfg(feature = "defmt")]
                defmt::error!(
                     "CRC mismatch for register block read (byte {}), calculated {:02x}, received {:02x}. {:02x}",
                     i,
                     calculated_crc,
                     received_crc,
                     data_byte,
                 );
                return Err(Error::CrcError);
            }

            // Copy the data byte to the result buffer
            result_buffer[i] = data_byte;
        }

        Ok(result_buffer)
    }

    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>> {
        // CRC calculation for single byte write: slave address + register address + data
        let mut crc_data = heapless::Vec::<u8, 3>::new();
        crc_data
            .push(self.address << 1)
            .map_err(|_| Error::InvalidData)?; // Slave address + Write bit
        crc_data.push(reg as u8).map_err(|_| Error::InvalidData)?;
        crc_data.push(value).map_err(|_| Error::InvalidData)?;

        let calculated_crc = crc::calculate_crc(&crc_data); // Updated call

        let mut data_to_write = heapless::Vec::<u8, 3>::new();
        data_to_write
            .push(reg as u8)
            .map_err(|_| Error::InvalidData)?;
        data_to_write.push(value).map_err(|_| Error::InvalidData)?;
        data_to_write
            .push(calculated_crc)
            .map_err(|_| Error::InvalidData)?;

        self.i2c
            .write(self.address, &data_to_write)
            .await
            .map_err(Error::I2c)
    }

    async fn write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>> {
        // CRC calculation for block write:
        // First data byte: slave address + register address + first data byte
        // Subsequent data bytes: only the data byte

        let mut crc_data = heapless::Vec::<u8, 32>::new(); // Max 30 data bytes + address + register
        crc_data
            .push(self.address << 1)
            .map_err(|_| Error::InvalidData)?; // Slave address + Write bit
        crc_data.push(reg as u8).map_err(|_| Error::InvalidData)?;
        if let Some(first_value) = values.get(0) {
            crc_data
                .push(*first_value)
                .map_err(|_| Error::InvalidData)?;
        }

        let mut calculated_crc = crc::calculate_crc(&crc_data); // Updated call

        let mut data_to_write = heapless::Vec::<u8, 62>::new(); // Max 30 data bytes + 30 CRC bytes + register + first CRC
        data_to_write
            .push(reg as u8)
            .map_err(|_| Error::InvalidData)?;
        if let Some(first_value) = values.get(0) {
            data_to_write
                .push(*first_value)
                .map_err(|_| Error::InvalidData)?;
            data_to_write
                .push(calculated_crc)
                .map_err(|_| Error::InvalidData)?;
        }

        for (_i, value) in values.iter().enumerate().skip(1) {
            calculated_crc = crc::calculate_crc(&[*value]); // CRC only on the data byte for subsequent bytes
            data_to_write.push(*value).map_err(|_| Error::InvalidData)?;
            data_to_write
                .push(calculated_crc)
                .map_err(|_| Error::InvalidData)?;
        }

        self.i2c
            .write(self.address, &data_to_write)
            .await
            .map_err(Error::I2c)
    }
}

// Generic impl block for methods that use RegisterAccess
#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "Bq769x0<I2C, M>",),
    async(feature = "async", keep_self)
)]
impl<I2C, M, E> Bq769x0<I2C, M>
where
    I2C: I2c<Error = E>,
    M: CrcMode,
    Self: RegisterAccess<E>,
{
    /// Reads the ADCGAIN and ADCOFFSET registers and calculates the actual GAIN and OFFSET values.
    async fn read_adc_calibration(&mut self) -> Result<(u16, i8), Error<E>> {
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Combine ADCGAIN1 (bits 3-2) and ADCGAIN2 (bits 7-5) to get the 5-bit ADCGAIN<4:0>
        let adc_gain_5bit = (((adc_gain1 >> 2) & 0b11) << 3) | ((adc_gain2 >> 5) & 0b111);
        let gain_uv_per_lsb = 365 + adc_gain_5bit as u16; // GAIN in uV/LSB

        // ADCOFFSET is an 8-bit signed value in mV
        let adc_offset_mv = adc_offset as i8;

        Ok((gain_uv_per_lsb, adc_offset_mv))
    }

    /// Reads and converts cell voltages.
    /// Returns voltages in mV.
    pub async fn read_cell_voltages(&mut self) -> Result<CellVoltages<NUM_CELLS>, Error<E>> {
        // Read all cell voltage registers (15 cells * 2 bytes/cell = 30 bytes)
        let raw_voltages = self.read_registers(Register::Vc1Hi, NUM_CELLS * 2).await?;

        // Read ADC Gain and Offset registers
        let (gain_uv_per_lsb, adc_offset_mv) = self.read_adc_calibration().await?;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "ADC Calibration: gain_uv_per_lsb = {}, adc_offset_mv = {}",
            gain_uv_per_lsb,
            adc_offset_mv
        );

        let mut voltages_mv = [0u16; NUM_CELLS];
        for i in 0..NUM_CELLS {
            let hi_byte = raw_voltages[i * 2];
            let lo_byte = raw_voltages[i * 2 + 1];

            // Combine bytes to get 14-bit raw ADC value
            // According to datasheet Table 8-15:
            // VCx_HI contains bits 13:8
            // VCx_LO contains bits 7:0
            let raw_adc: u16 = (((hi_byte & 0x3f) as u16) << 8) | (lo_byte as u16);

            // Convert raw ADC to voltage (in mV)
            // VCELLn (uV) = (ADC_CELLn * GAIN_uV_per_lsb) + (OFFSET_mV * 1000)
            // VCELLn (mV) = VCELLn (uV) / 1000
            let v_cell_uv: i32 =
                (raw_adc as i32 * gain_uv_per_lsb as i32) + (adc_offset_mv as i32 * 1000);
            voltages_mv[i] = (v_cell_uv / 1000) as u16; // Convert uV to mV and cast to u16
        }
        Ok(CellVoltages::<NUM_CELLS> { voltages_mv })
    }

    /// Reads and converts the total battery pack voltage.
    /// Returns voltage in mV.
    pub async fn read_pack_voltage(&mut self) -> Result<u16, Error<E>> {
        let raw = self.read_registers(Register::BatHi, 2).await?;

        let (gain_uv_per_lsb, adc_offset_mv) = self.read_adc_calibration().await?;

        // Combine bytes to get a signed 16-bit raw voltage value
        let raw_adc: i16 = i16::from_be_bytes([raw[0], raw[1]]);
        let voltage_uv =
            4 * (raw_adc as i32 * gain_uv_per_lsb as i32) + (adc_offset_mv as i32 * 1000);
        Ok((voltage_uv / 1000) as u16)
    }

    /// Reads and converts temperatures from sensors.
    /// Returns temperature in deci-Celsius (if Die Temp) or resistance in 0.1 Ohms (if Thermistor).
    pub async fn read_temperatures(&mut self) -> Result<Temperatures, Error<E>> {
        // Read TEMP_SEL bit from SYS_CTRL1
        let sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        let temp_sel = (sys_ctrl1 & SYS_CTRL1_TEMP_SEL) != 0; // true for Thermistor, false for Die Temp

        let ts1_value: i16;
        let ts2_value: Option<i16> = None;
        let ts3_value: Option<i16> = None;

        // Read TS1_HI and TS1_LO registers (2 bytes)
        let raw_ts1 = self.read_registers(Register::Ts1Hi, 2).await?;
        let raw_adc_ts1: u16 = ((raw_ts1[0] as u16) << 6) | ((raw_ts1[1] as u16) >> 2);

        if temp_sel {
            // Thermistor mode (returns resistance in Ohms * 10 for i16)
            // V_TSX_uV = raw_adc_ts_14bit * 382
            // R_TS_ohm = (10000.0 * V_TSX_V) / (3.3 - V_TSX_V)
            let v_tsx_uv = raw_adc_ts1 as u32 * 382;
            let v_tsx_v = v_tsx_uv as f32 / 1_000_000.0;
            // Avoid division by zero or negative values if V_TSX_V is close to or exceeds 3.3V
            if v_tsx_v >= 3.3 {
                ts1_value = i16::MAX; // Indicate very high resistance (low temp) or error
            } else {
                let r_ts_ohm = (10000.0 * v_tsx_v) / (3.3 - v_tsx_v);
                ts1_value = (r_ts_ohm * 10.0) as i16; // Store resistance * 10 (in 0.1 Ohms)
            }
        } else {
            // Die Temp mode (returns temperature in deci-Celsius)
            // V_TSX_uV = raw_adc_ts_14bit * 382
            // TEMP_C = 25.0 - ((V_TSX_uV - V_25_uV) / 4200.0)
            let v_tsx_uv = raw_adc_ts1 as i32 * 382;
            let v_25_uv = 1_200_000; // 1.200 V nominal
            let temp_c = 25.0 - ((v_tsx_uv - v_25_uv) as f32 / 4200.0);
            ts1_value = (temp_c * 10.0) as i16; // Store temperature in dC
        }

        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // Read TS2_HI and TS2_LO registers (2 bytes) for BQ76930/40
            let raw_ts2 = self.read_registers(Register::Ts2Hi, 2).await?;
            let raw_adc_ts2: u16 = ((raw_ts2[0] as u16) << 6) | ((raw_ts2[1] as u16) >> 2);

            if temp_sel {
                let v_tsx_uv = raw_adc_ts2 as u32 * 382;
                let v_tsx_v = v_tsx_uv as f32 / 1_000_000.0;
                if v_tsx_v >= 3.3 {
                    ts2_value = Some(i16::MAX);
                } else {
                    let r_ts_ohm = (10000.0 * v_tsx_v) / (3.3 - v_tsx_v);
                    ts2_value = Some((r_ts_ohm * 10.0) as i16);
                }
            } else {
                let v_tsx_uv = raw_adc_ts2 as i32 * 382;
                let v_25_uv = 1_200_000;
                let temp_c = 25.0 - ((v_tsx_uv - v_25_uv) as f32 / 4200.0);
                ts2_value = Some((temp_c * 10.0) as i16);
            }
        }

        #[cfg(feature = "bq76940")]
        {
            // Read TS3_HI and TS3_LO registers (2 bytes) for BQ76940
            let raw_ts3 = self.read_registers(Register::Ts3Hi, 2).await?;
            let raw_adc_ts3: u16 = ((raw_ts3[0] as u16) << 6) | ((raw_ts3[1] as u16) >> 2);

            if temp_sel {
                let v_tsx_uv = raw_adc_ts3 as u32 * 382;
                let v_tsx_v = v_tsx_uv as f32 / 1_000_000.0;
                if v_tsx_v >= 3.3 {
                    ts3_value = Some(i16::MAX);
                } else {
                    let r_ts_ohm = (10000.0 * v_tsx_v) / (3.3 - v_tsx_v);
                    ts3_value = Some((r_ts_ohm * 10.0) as i16);
                }
            } else {
                let v_tsx_uv = raw_adc_ts3 as i32 * 382;
                let v_25_uv = 1_200_000;
                let temp_c = 25.0 - ((v_tsx_uv - v_25_uv) as f32 / 4200.0);
                ts3_value = Some((temp_c * 10.0) as i16);
            }
        }

        Ok(Temperatures {
            ts1: ts1_value,
            ts2: ts2_value,
            ts3: ts3_value,
            is_thermistor: temp_sel,
        })
    }

    /// Reads the raw Coulomb Counter value.
    /// Conversion to current (A) requires CC_CFG and Rsense value.
    pub async fn read_current(&mut self) -> Result<Current, Error<E>> {
        // Read CC_HI and CC_LO registers (2 bytes)
        let raw_cc_bytes = self.read_registers(Register::CcHi, 2).await?;

        // Combine bytes to get a signed 16-bit raw CC value
        let raw_cc: i16 = i16::from_be_bytes([raw_cc_bytes[0], raw_cc_bytes[1]]);

        Ok(Current { raw_cc })
    }

    /// Reads the System Status register.
    pub async fn read_status(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::SysStat).await
    }

    /// Clears the specified status flags in the System Status register.
    pub async fn clear_status_flags(&mut self, flags: u8) -> Result<(), Error<E>> {
        // To clear a flag, write a '1' to the corresponding bit.
        self.write_register(Register::SysStat, flags).await
    }

    /// Enables the charging FET.
    pub async fn enable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 |= SYS_CTRL2_CHG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Disables the charging FET.
    pub async fn disable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 &= !SYS_CTRL2_CHG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Enables the discharging FET.
    pub async fn enable_discharging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 |= SYS_CTRL2_DSG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Disables the discharging FET.
    pub async fn disable_discharging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 &= !SYS_CTRL2_DSG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Sets the cell balancing state.
    /// The `mask` is a bitmask where bit n corresponds to cell n+1.
    /// E.g., bit 0 for Cell 1, bit 4 for Cell 5, bit 9 for Cell 10, bit 14 for Cell 15.
    pub async fn set_cell_balancing(&mut self, mask: u16) -> Result<(), Error<E>> {
        // Write to CELLBAL1 (Cells 1-5)
        let cellbal1_mask = (mask & 0x1F) as u8; // Bits 0-4
        self.write_register(Register::CELLBAL1, cellbal1_mask)
            .await?;

        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // Write to CELLBAL2 (Cells 6-10) for BQ76930/40
            let cellbal2_mask = ((mask >> 5) & 0x1F) as u8; // Bits 5-9
            self.write_register(Register::CELLBAL2, cellbal2_mask)
                .await?;
        }

        #[cfg(feature = "bq76940")]
        {
            // Write to CELLBAL3 (Cells 11-15) for BQ76940
            let cellbal3_mask = ((mask >> 10) & 0x1F) as u8; // Bits 10-14
            self.write_register(Register::CELLBAL3, cellbal3_mask)
                .await?;
        }

        Ok(())
    }

    /// Configures the Protection 1 register (SCD).
    /// Rsense: 0 for lower range, 1 for upper range.
    /// ScdDelay: 0=70us, 1=100us, 2=200us, 3=400us.
    /// ScdThreshold: 0-7, see datasheet Table 8-9 for mV values based on Rsense.
    pub async fn configure_protect1(
        &mut self,
        rsense: u8,
        scd_delay: u8,
        scd_threshold: u8,
    ) -> Result<(), Error<E>> {
        let mut protect1_val = 0u8;
        if rsense != 0 {
            protect1_val |= PROTECT1_RSNS;
        }
        protect1_val |= (scd_delay & 0b11) << 3;
        protect1_val |= (scd_threshold & 0b111) << 0;
        self.write_register(Register::PROTECT1, protect1_val).await
    }

    /// Configures the Protection 2 register (OCD).
    /// OcdDelay: 0=8ms, 1=20ms, 2=40ms, 3=80ms, 4=160ms, 5=320ms, 6=640ms, 7=1280ms.
    /// OcdThreshold: 0-15, see datasheet Table 8-10 for mV values based on Rsense.
    pub async fn configure_protect2(
        &mut self,
        ocd_delay: u8,
        ocd_threshold: u8,
    ) -> Result<(), Error<E>> {
        let mut protect2_val = 0u8;
        protect2_val |= (ocd_delay & 0b111) << 4;
        protect2_val |= (ocd_threshold & 0b1111) << 0;
        self.write_register(Register::PROTECT2, protect2_val).await
    }

    /// Configures the Protection 3 register (OV/UV Delay).
    /// UvDelay: 0=1s, 1=4s, 2=8s, 3=16s.
    /// OvDelay: 0=1s, 1=2s, 2=4s, 3=8s.
    pub async fn configure_protect3(&mut self, uv_delay: u8, ov_delay: u8) -> Result<(), Error<E>> {
        let mut protect3_val = 0u8;
        protect3_val |= (uv_delay & 0b11) << 6;
        protect3_val |= (ov_delay & 0b11) << 4;
        // Keep RSVD bits as 0
        self.write_register(Register::PROTECT3, protect3_val).await
    }

    /// Configures the Overvoltage Trip register.
    /// ov_trip_8bit: Middle 8 bits of the 14-bit ADC value (10-ov_trip_8bit-1000).
    /// See datasheet Table 8-12 and Section 8.3.1.2 for calculation based on desired voltage, GAIN, and OFFSET.
    pub async fn configure_ov_trip(&mut self, ov_trip_8bit: u8) -> Result<(), Error<E>> {
        self.write_register(Register::OvTrip, ov_trip_8bit).await
    }

    /// Configures the Undervoltage Trip register.
    /// uv_trip_8bit: Middle 8 bits of the 14-bit ADC value (01-uv_trip_8bit-0000).
    /// See datasheet Table 8-13 and Section 8.3.1.2 for calculation based on desired voltage, GAIN, and OFFSET.
    pub async fn configure_uv_trip(&mut self, uv_trip_8bit: u8) -> Result<(), Error<E>> {
        self.write_register(Register::UvTrip, uv_trip_8bit).await
    }

    /// Enters the SHIP mode.
    /// WARNING: Refer to the datasheet for the exact sequence to enter SHIP mode.
    pub async fn enter_ship_mode(&mut self) -> Result<(), Error<E>> {
        // Datasheet Section 8.4.2:
        // Starting from: [SHUT_A] = 0, [SHUT_B] = 0
        // Write #1: [SHUT_A] = 0, [SHUT_B] = 1 (SYS_CTRL1 = 0b...01)
        // Write #2: [SHUT_A] = 1, [SHUT_B] = 0 (SYS_CTRL1 = 0b...10)

        // Ensure SHUT_A and SHUT_B are initially 0 (reset value) or write 0x00 first if unsure.
        // Let's write 0x00 to SYS_CTRL1 first to be safe, preserving other bits.
        let mut sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        sys_ctrl1 &= !(SYS_CTRL1_SHUT_A | SYS_CTRL1_SHUT_B);
        self.write_register(Register::SysCtrl1, sys_ctrl1).await?;

        // Write #1: [SHUT_A] = 0, [SHUT_B] = 1
        let mut sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        sys_ctrl1 &= !SYS_CTRL1_SHUT_A;
        sys_ctrl1 |= SYS_CTRL1_SHUT_B;
        self.write_register(Register::SysCtrl1, sys_ctrl1).await?;

        // Write #2: [SHUT_A] = 1, [SHUT_B] = 0
        let mut sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        sys_ctrl1 |= SYS_CTRL1_SHUT_A;
        sys_ctrl1 &= !SYS_CTRL1_SHUT_B;
        self.write_register(Register::SysCtrl1, sys_ctrl1).await?;

        Ok(())
    }

    /// Checks if the ALERT pin is being overridden by external control.
    pub async fn is_alert_overridden(&mut self) -> Result<bool, Error<E>> {
        let sys_stat = self.read_status().await?;
        Ok((sys_stat & SYS_STAT_OVRD_ALERT) != 0)
    }

    /// Converts raw Coulomb Counter value to current in mA.
    /// Rsense_mOhm: Sense resistor value in milliOhms.
    /// See datasheet Section 8.3.1.1.3 for formula.
    pub fn convert_raw_cc_to_current_ma(&self, raw_cc: i16, rsense_m_ohm: f32) -> f32 {
        // CC Reading (in μV) = [16-bit 2’s Complement Value] × (8.44 μV/LSB)
        // Current (A) = CC Reading (in V) / Rsense (in Ω)
        // Current (mA) = (CC Reading (in μV) / 1000) / (Rsense (in mΩ) / 1000)
        // Current (mA) = CC Reading (in μV) / Rsense (in mΩ)
        // CC Reading (in μV) = raw_cc * 8.44
        (raw_cc as f32 * 8.44) / rsense_m_ohm
    }

    /// Configures the BQ769x0 with the provided battery parameters.
    pub async fn set_config(&mut self, config: &BatteryConfig) -> Result<(), Error<E>> {
        // Configure SYS_CTRL1 and SYS_CTRL2 (can potentially be a sequential write)
        self.write_register(Register::SysCtrl1, config.sys_ctrl1_val).await?;
        self.write_register(Register::SysCtrl2, config.sys_ctrl2_val).await?;

        // Configure Protection registers
        self.configure_ov_trip(config.ov_trip_8bit).await?;
        self.configure_uv_trip(config.uv_trip_8bit).await?;
        self.configure_protect1(
            config.protect1_rsense,
            config.protect1_scd_delay,
            config.protect1_scd_threshold,
        ).await?;
        self.configure_protect2(
            config.protect2_ocd_delay,
            config.protect2_ocd_threshold,
        ).await?;
        self.configure_protect3(
            config.protect3_uv_delay,
            config.protect3_ov_delay,
        ).await?;

        // Configure CC_CFG
        self.write_register(Register::CcCfg, config.cc_cfg_val).await?;

        Ok(())
    }
}
