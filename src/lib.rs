#![no_std]

#[cfg(feature = "defmt")]
extern crate defmt;

use core::ops::Deref;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

pub mod registers;
use registers::*; // Import all from registers

pub mod crc; // Make crc module public
pub mod data_types;
pub mod errors;

pub use data_types::{
    BatteryConfig, Bq76920Measurements, CellVoltages, CoulombCounter, MosStatus, OcdDelay,
    ProtectionConfig, ScdDelay, SystemStatus, TempSensor, TemperatureData,
    TemperatureSensorReadings, UvOvDelay,
};
use errors::Error;

pub use crc::{CrcMode, Disabled, Enabled, calculate_crc};

/// BQ769x0 driver
pub struct Bq769x0<I2C, M: CrcMode, const N: usize>
where
    I2C: I2c,
{
    address: u8,
    i2c: I2C,
    _crc_mode: core::marker::PhantomData<M>,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "RegisterAccess",),
    async(feature = "async", keep_self)
)]
#[allow(async_fn_in_trait)]
/// Trait for abstracting register access, with or without CRC.
pub trait RegisterAccess<E>
where
    Self: Sized,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq,
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
    sync(cfg(not(feature = "async")), self = "Bq769x0",),
    async(feature = "async", keep_self)
)]
impl<I2C, E, const N: usize> Bq769x0<I2C, Disabled, N>
where
    I2C: I2c<Error = E>,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq, // Add PartialEq constraint for E
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
    sync(cfg(not(feature = "async")), self = "Bq769x0",),
    async(feature = "async", keep_self)
)]
impl<I2C, E, const N: usize> RegisterAccess<E> for Bq769x0<I2C, Disabled, N>
where
    I2C: I2c<Error = E>,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq, // Add PartialEq constraint for E
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
    sync(cfg(not(feature = "async")), self = "Bq769x0",),
    async(feature = "async", keep_self)
)]
impl<I2C, E, const N: usize> Bq769x0<I2C, Enabled, N>
where
    I2C: I2c<Error = E>,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq, // Add PartialEq constraint for E
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
    sync(cfg(not(feature = "async")), self = "Bq769x0",),
    async(feature = "async", keep_self)
)]
impl<I2C, E, const N: usize> RegisterAccess<E> for Bq769x0<I2C, Enabled, N>
where
    I2C: I2c<Error = E>,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq, // Add PartialEq constraint for E
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
        let crc_data = [(self.address << 1) | 1, received_data];

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
            return Err(Error::Crc);
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
        result_buffer
            .resize(len, 0)
            .map_err(|_| Error::InvalidData)?;

        for i in 0..len {
            let data_byte = raw_data_buffer[i * 2];
            let received_crc = raw_data_buffer[i * 2 + 1];

            let calculated_crc = if i == 0 {
                // First data byte CRC: slave address + data byte
                let crc_data = [
                    (self.address << 1) | 1, // Slave address + Write bit
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
                return Err(Error::Crc);
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
        if let Some(first_value) = values.first() {
            crc_data
                .push(*first_value)
                .map_err(|_| Error::InvalidData)?;
        }

        let mut calculated_crc = crc::calculate_crc(&crc_data); // Updated call

        let mut data_to_write = heapless::Vec::<u8, 62>::new(); // Max 30 data bytes + 30 CRC bytes + register + first CRC
        data_to_write
            .push(reg as u8)
            .map_err(|_| Error::InvalidData)?;
        if let Some(first_value) = values.first() {
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
    sync(cfg(not(feature = "async")), self = "Bq769x0",),
    async(feature = "async", keep_self)
)]
impl<I2C, M, E, const N: usize> Bq769x0<I2C, M, N>
where
    I2C: I2c<Error = E>,
    M: CrcMode,
    Self: RegisterAccess<E>,
    E: PartialEq, // Add PartialEq constraint for E
    E: PartialEq, // Add PartialEq constraint for E
{
    /// Reads the ADCGAIN and ADCOFFSET registers and calculates the actual GAIN and OFFSET values.
    /// Reads the ADCGAIN and ADCOFFSET registers and calculates the actual GAIN and OFFSET values.
    /// Returns (adc_gain_uv_per_lsb, adc_offset_mv).
    pub async fn read_adc_calibration(&mut self) -> Result<(u32, i16), Error<E>> {
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Datasheet section 8.1.2.3 ADC Gain and Offset
        // Datasheet section 8.5, Table 8-21 (ADCGAIN1) and Table 8-22 (ADCGAIN2)
        // ADCGAIN<4:3> are bits 3-2 of ADCGAIN1
        // ADCGAIN<2:0> are bits 7-5 of ADCGAIN2
        let adc_gain_raw = ((adc_gain1 & 0b00001100) << 1) | ((adc_gain2 & 0b11100000) >> 5);

        // ADCOFFSET is signed 8-bit
        let adc_offset_signed = adc_offset as i8;

        // Datasheet section 8.1.2.3 ADC Gain and Offset
        // GAIN = 365 μV/LSB + (ADCGAIN<4:0> in decimal) × (1 μV/LSB)
        let adc_gain_uv_per_lsb = 365 + adc_gain_raw as u32;

        // ADCOFFSET is signed 8-bit in mV
        let adc_offset_mv = adc_offset_signed as i16;

        Ok((adc_gain_uv_per_lsb, adc_offset_mv))
    }

    /// Reads and converts cell voltages to mV using ADC calibration values.
    pub async fn read_cell_voltages(&mut self) -> Result<CellVoltages<N>, Error<E>>
    where
        Self: RegisterAccess<E>,
    {
        let (adc_gain_uv_per_lsb, adc_offset_mv) = self.read_adc_calibration().await?;

        let start_reg = Register::Vc1Hi;
        let len = N * 2; // Each cell voltage is 2 bytes (Hi and Lo)
        let raw_data_bytes = self.read_registers(start_reg, len).await?;

        let mut converted_cell_voltages = CellVoltages::new(); // Now expects [i32; N]

        for i in 0..N {
            let hi_byte = raw_data_bytes[i * 2];
            let lo_byte = raw_data_bytes[i * 2 + 1];
            let raw_adc_val = (((hi_byte & 0x3F) as u16) << 8) | (lo_byte as u16);

            // Apply conversion: V_cell_mV = (ADC_raw * Gain_uV_per_LSB / 1000) + Offset_mV
            let converted_mv =
                (raw_adc_val as i32 * adc_gain_uv_per_lsb as i32) / 1000 + adc_offset_mv as i32;
            converted_cell_voltages.voltages[i] = converted_mv;

            // #[cfg(feature = "defmt")]
            // defmt::info!(
            //     "LIB: Cell {}: raw_adc={}, converted_mv={}",
            //     i + 1,
            //     raw_adc_val,
            //     converted_mv
            // );
        }

        Ok(converted_cell_voltages)
    }

    /// Reads the battery pack voltage from the BatHi and BatLo registers.
    /// Returns the pack voltage in mV.
    pub async fn read_pack_voltage(&mut self) -> Result<u32, Error<E>> {
        let raw_data = self.read_registers(Register::BatHi, 2).await?;
        let hi_byte = raw_data[0];
        let lo_byte = raw_data[1];
        let raw_voltage = ((hi_byte as u16) << 8) | (lo_byte as u16);

        let (_adc_gain_uv_per_lsb, _adc_offset_mv) = self.read_adc_calibration().await?;

        // Datasheet section 8.3.1.1.6 16-Bit Pack Voltage
        // V(BAT) = 4 × GAIN × ADC(cell) + (#Cells x OFFSET)
        // The 16-bit value in BAT_HI/LO is already scaled.
        // Nominal LSB is 1.532 mV. Let's use this for conversion.
        // Pack Voltage (mV) = raw_voltage * 1.532
        // To use integer arithmetic, multiply by 1000 to work in μV, then divide by 1000 for mV.
        // Pack Voltage (mV) = (raw_voltage * 1532) / 1000
        // Or, since the LSB is nominal, it might be better to use the sum of individual cell voltages
        // if higher accuracy is needed, but for direct register reading, use the LSB.
        // Let's use the nominal LSB for now and return mV.

        let pack_voltage_mv = (raw_voltage as u32 * 1532) / 1000;

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Pack Voltage: raw_voltage={}, pack_voltage_mv={}",
            raw_voltage,
            pack_voltage_mv
        );

        Ok(pack_voltage_mv)
    }

    /// Reads the temperature sensors (TS1, TS2, TS3 or Die Temp)
    pub async fn read_temperatures(&mut self) -> Result<TemperatureSensorReadings, Error<E>> {
        // The number of temperature sensors depends on the chip variant (BQ76920, BQ76930, BQ76940)
        // and the TEMP_SEL bit in SYS_CTRL1.
        // For BQ76920, there is one external TS pin (TS1) and internal die temp.
        // TEMP_SEL = 0: TS1 is Die Temp
        // TEMP_SEL = 1: TS1 is external thermistor

        // Read SYS_CTRL1 to check TEMP_SEL
        let sys_ctrl1_byte = self.read_register(Register::SysCtrl1).await?;
        let temp_sel =
            SysCtrl1Flags::from_bits_truncate(sys_ctrl1_byte).contains(SysCtrl1Flags::TEMP_SEL);

        let mut temperatures = TemperatureSensorReadings::new();

        // Read TS1
        let ts1_raw_data = self.read_registers(Register::Ts1Hi, 2).await?;
        let ts1_hi = ts1_raw_data[0];
        let ts1_lo = ts1_raw_data[1];
        let ts1_raw = ((ts1_hi as u16) << 8) | (ts1_lo as u16);

        if !temp_sel {
            // Die Temp
            // Datasheet section 8.1.2.4 Temperature
            // V_TSX = ADC(TS) * 382 μV/LSB
            // Store raw ADC value. Conversion to temperature is done in TemperatureSensorReadings::into_temperature_data.
            temperatures.ts1 = ts1_raw;
            temperatures.is_thermistor = false;
        } else {
            // External Thermistor
            // Store raw ADC value. Conversion to resistance/temperature is done in TemperatureSensorReadings::into_temperature_data.
            temperatures.ts1 = ts1_raw;
            temperatures.is_thermistor = true;
        }

        // Handle TS2 and TS3 for BQ76930/40
        if N >= 10 {
            let ts2_raw_data = self.read_registers(Register::Ts2Hi, 2).await?;
            let ts2_hi = ts2_raw_data[0];
            let ts2_lo = ts2_raw_data[1];
            let ts2_raw = ((ts2_hi as u16) << 8) | (ts2_lo as u16);

            if !temp_sel {
                temperatures.ts2 = Some(ts2_raw); // Store raw ADC value
            } else {
                let (adc_gain_uv_per_lsb, _) = self.read_adc_calibration().await?;
                let _v_tsx_mv = (ts2_raw as f32 // _v_tsx_mv is not used, but calculation kept for reference if needed later
                    * adc_gain_uv_per_lsb as f32)
                    / 1000.0;
                temperatures.ts2 = Some(ts2_raw);
            }
        }

        if N >= 15 {
            let ts3_raw_data = self.read_registers(Register::Ts3Hi, 2).await?;
            let ts3_hi = ts3_raw_data[0];
            let ts3_lo = ts3_raw_data[1];
            let ts3_raw = ((ts3_hi as u16) << 8) | (ts3_lo as u16);

            if !temp_sel {
                temperatures.ts3 = Some(ts3_raw); // Store raw ADC value
            } else {
                let (adc_gain_uv_per_lsb, _) = self.read_adc_calibration().await?;
                let _v_tsx_mv = (ts3_raw as f32 // _v_tsx_mv is not used, but calculation kept for reference if needed later
                    * adc_gain_uv_per_lsb as f32)
                    / 1000.0;
                temperatures.ts3 = Some(ts3_raw);
            }
        }

        Ok(temperatures)
    }

    /// Reads the current from the Coulomb Counter registers.
    pub async fn read_current(&mut self) -> Result<CoulombCounter, Error<E>> {
        let raw_data = self.read_registers(Register::CcHi, 2).await?;
        let hi_byte = raw_data[0];
        let lo_byte = raw_data[1];
        let raw_cc = ((hi_byte as i16) << 8) | (lo_byte as i16);

        Ok(CoulombCounter { raw_cc })
    }

    /// Reads all ADC measurements (cell voltages, pack voltage, temperatures, current).
    pub async fn read_all_measurements(&mut self) -> Result<Bq76920Measurements<N>, Error<E>>
    where
        Self: RegisterAccess<E>,
    {
        let cell_voltages = self.read_cell_voltages().await?;
        let temperatures = self.read_temperatures().await?;
        let coulomb_counter = self.read_current().await?;
        let system_status = self.read_status().await?;
        let mos_status = self.read_mos_status().await?;

        Ok(Bq76920Measurements {
            cell_voltages,
            temperatures,
            current: self.convert_raw_cc_to_current_ma(
                coulomb_counter.raw_cc,
                // Use the rsense value from BatteryConfig if available, otherwise use a default.
                // For now, let's use a placeholder or assume it's passed.
                // The set_config method takes BatteryConfig, maybe pass rsense from there?
                // For simplicity in read_all_measurements, let's assume a default rsense or
                // require it to be set elsewhere before calling this.
                // For now, using a hardcoded default as in the original code, but with the new type.
                10u32, // Default Rsense: 10 mΩ
            ),
            system_status,
            mos_status,
        })
    }

    /// Reads the system status register and parses the flags.
    pub async fn read_status(&mut self) -> Result<SystemStatus, Error<E>> {
        let sys_stat = self.read_register(Register::SysStat).await?;
        Ok(SystemStatus::new(sys_stat))
    }

    /// Reads the charge/discharge MOS status from the SYS_CTRL2 register.
    pub async fn read_mos_status(&mut self) -> Result<MosStatus, Error<E>> {
        let sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        Ok(MosStatus::new(sys_ctrl2))
    }

    /// Clears the specified status flags in the SYS_STAT register.
    pub async fn clear_status_flags(&mut self, flags: u8) -> Result<(), Error<E>> {
        self.write_register(Register::SysStat, flags).await
    }

    /// Enables charging by setting the CHG_ON bit in SYS_CTRL2.
    pub async fn enable_charging(&mut self) -> Result<(), Error<E>> {
        let sys_ctrl2_byte = self.read_register(Register::SysCtrl2).await?;
        let mut sys_ctrl2_flags = SysCtrl2Flags::from_bits_truncate(sys_ctrl2_byte);
        sys_ctrl2_flags.insert(SysCtrl2Flags::CHG_ON);
        self.write_register(Register::SysCtrl2, sys_ctrl2_flags.bits())
            .await
    }

    /// Disables charging by clearing the CHG_ON bit in SYS_CTRL2.
    pub async fn disable_charging(&mut self) -> Result<(), Error<E>> {
        let sys_ctrl2_byte = self.read_register(Register::SysCtrl2).await?;
        let mut sys_ctrl2_flags = SysCtrl2Flags::from_bits_truncate(sys_ctrl2_byte);
        sys_ctrl2_flags.remove(SysCtrl2Flags::CHG_ON);
        self.write_register(Register::SysCtrl2, sys_ctrl2_flags.bits())
            .await
    }

    /// Enables discharging by setting the DSG_ON bit in SYS_CTRL2.
    pub async fn enable_discharging(&mut self) -> Result<(), Error<E>> {
        let sys_ctrl2_byte = self.read_register(Register::SysCtrl2).await?;
        let mut sys_ctrl2_flags = SysCtrl2Flags::from_bits_truncate(sys_ctrl2_byte);
        sys_ctrl2_flags.insert(SysCtrl2Flags::DSG_ON);
        self.write_register(Register::SysCtrl2, sys_ctrl2_flags.bits())
            .await
    }

    /// Sets the cell balancing bits.
    /// The mask is a 16-bit value where each bit corresponds to a cell (LSB = Cell 1).
    /// Only the bits relevant to the N (number of cells) will be written to CELLBAL1, CELLBAL2, CELLBAL3.
    pub async fn set_cell_balancing(&mut self, mask: u16) -> Result<(), Error<E>> {
        // CELLBAL1 (Cells 1-5)
        self.write_register(Register::CELLBAL1, (mask & 0x1F) as u8)
            .await?;

        // CELLBAL2 (Cells 6-10) - BQ76930/40 only
        if N >= 10 {
            self.write_register(Register::CELLBAL2, ((mask >> 5) & 0x1F) as u8)
                .await?;
        }

        // CELLBAL3 (Cells 11-15) - BQ76940 only
        if N >= 15 {
            self.write_register(Register::CELLBAL3, ((mask >> 10) & 0x1F) as u8)
                .await?;
        }

        Ok(())
    }

    /// Configures the PROTECT1 register (SCD).
    async fn configure_protect1(
        &mut self,
        rsns_enable: bool,
        scd_delay: ScdDelay,
        scd_threshold_bits: u8,
    ) -> Result<(), Error<E>> {
        let mut protect1: u8 = 0;
        let mut protect1_flags = Protect1Flags::from_bits_truncate(protect1);
        if rsns_enable {
            protect1_flags.insert(Protect1Flags::RSNS);
        }
        protect1 = protect1_flags.bits();
        protect1 |= match scd_delay {
            ScdDelay::Delay70us => 0b00 << 3,
            ScdDelay::Delay100us => 0b01 << 3,
            ScdDelay::Delay200us => 0b10 << 3,
            ScdDelay::Delay400us => 0b11 << 3,
        };
        let mut protect1_flags = Protect1Flags::from_bits_truncate(protect1);
        protect1_flags.insert(
            Protect1Flags::from_bits_truncate(scd_threshold_bits) & Protect1Flags::SCD_THRESH_MASK,
        ); // Ensure only relevant bits are set
        self.write_register(Register::PROTECT1, protect1_flags.bits())
            .await
    }

    /// Configures the PROTECT2 register (OCD).
    async fn configure_protect2(
        &mut self,
        ocd_delay: OcdDelay,
        ocd_threshold_bits: u8,
    ) -> Result<(), Error<E>> {
        let mut protect2: u8 = 0;
        protect2 |= match ocd_delay {
            OcdDelay::Delay10ms => 0b000 << 4,
            OcdDelay::Delay20ms => 0b001 << 4,
            OcdDelay::Delay40ms => 0b010 << 4,
            OcdDelay::Delay80ms => 0b011 << 4,
            OcdDelay::Delay160ms => 0b100 << 4,
            OcdDelay::Delay320ms => 0b101 << 4,
            OcdDelay::Delay640ms => 0b110 << 4,
            OcdDelay::Delay1280ms => 0b111 << 4,
        };
        let mut protect2_flags = Protect2Flags::from_bits_truncate(protect2);
        protect2_flags.insert(
            Protect2Flags::from_bits_truncate(ocd_threshold_bits) & Protect2Flags::OCD_THRESH_MASK,
        ); // Ensure only relevant bits are set
        self.write_register(Register::PROTECT2, protect2_flags.bits())
            .await
    }

    /// Configures the PROTECT3 register (OV/UV Delay).
    async fn configure_protect3(
        &mut self,
        uv_delay: UvOvDelay,
        ov_delay: UvOvDelay,
    ) -> Result<(), Error<E>> {
        let mut protect3: u8 = 0;
        protect3 |= match uv_delay {
            UvOvDelay::Delay1s => 0b00 << 6,
            UvOvDelay::Delay2s => 0b01 << 6,
            UvOvDelay::Delay4s => 0b10 << 6,
            UvOvDelay::Delay8s => 0b11 << 6,
        };
        protect3 |= match ov_delay {
            UvOvDelay::Delay1s => 0b00 << 4,
            UvOvDelay::Delay2s => 0b01 << 4,
            UvOvDelay::Delay4s => 0b10 << 4,
            UvOvDelay::Delay8s => 0b11 << 4,
        };
        self.write_register(Register::PROTECT3, protect3).await
    }

    /// Enters ship mode.
    pub async fn enter_ship_mode(&mut self) -> Result<(), Error<E>> {
        // To enter ship mode, SHUT_A and SHUT_B bits in SYS_CTRL1 must be set to 1.
        // This is typically done by writing 0x00 to SYS_CTRL1 and SYS_CTRL2,
        // then writing 0x03 to SYS_CTRL1 twice.
        self.write_register(Register::SysCtrl1, 0x00).await?;
        self.write_register(Register::SysCtrl2, 0x00).await?;
        self.write_register(Register::SysCtrl1, SysCtrl1Flags::SHUT_B.bits()) // Write #1: SHUT_B = 1
            .await?;
        self.write_register(Register::SysCtrl1, SysCtrl1Flags::SHUT_A.bits()) // Write #2: SHUT_A = 1
            .await?;
        Ok(())
    }

    /// Checks if the ALERT pin is overridden.
    pub async fn is_alert_overridden(&mut self) -> Result<bool, Error<E>> {
        let sys_stat_byte = self.read_register(Register::SysStat).await?;
        Ok(SysStatFlags::from_bits_truncate(sys_stat_byte).contains(SysStatFlags::OVRD_ALERT))
    }

    /// Converts a raw Coulomb Counter value to current in mA.
    /// This is a helper function and requires the Rsense value in mΩ.
    /// Datasheet section 8.3.1.1.3 16-Bit CC
    /// CC Reading (in μV) = [16-bit 2’s Complement Value] × (8.44 μV/LSB)
    /// Current (mA) = Voltage_across_Rsense (mV) / Rsense (mΩ)
    /// Voltage_across_Rsense (mV) = raw_cc * 8.44 μV/LSB / 1000 μV/mV
    /// Current (mA) = (raw_cc * 8.44 / 1000) / (rsense_m_ohm / 1000)
    /// Current (mA) = raw_cc * 8.44 / rsense_m_ohm
    /// To use integer arithmetic, work in 0.01 μV units for precision:
    /// Voltage_across_Rsense (0.01 μV) = raw_cc * 844
    /// Current (mA) = (raw_cc * 844) / (rsense_m_ohm * 100)
    pub fn convert_raw_cc_to_current_ma(
        &self,
        raw_cc: i16,
        rsense_m_ohm: u32, // Rsense value in mΩ
    ) -> i32 {
        let voltage_0_01uv = raw_cc as i32 * 844;
        let rsense_0_01ohm = rsense_m_ohm as i32 * 100; // Convert mΩ to 0.01 Ω for calculation

        // Avoid division by zero
        if rsense_0_01ohm == 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!("Rsense is 0, cannot calculate current.");
            return 0;
        }

        let current_ma = voltage_0_01uv / rsense_0_01ohm;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "Raw CC: {}, Rsense (mOhm): {}, Current (mA): {}",
            raw_cc,
            rsense_m_ohm,
            current_ma
        );

        current_ma
    }

    /// Sets the configuration of the BQ769x0 chip based on the provided BatteryConfig.
    pub async fn set_config(&mut self, config: &BatteryConfig) -> Result<(), Error<E>> {
        // Implement current limit to voltage threshold mapping and register writing here

        // 1. Calculate target voltage thresholds from current limits and Rsense
        // Voltage (mV) = Current (mA) * Resistance (mΩ)
        let scd_target_voltage_mv =
            config.protection_config.scd_limit as i32 * config.rsense as i32;
        let ocd_target_voltage_mv =
            config.protection_config.ocd_limit as i32 * config.rsense as i32;

        // 2. Find the closest supported voltage thresholds and get their register bit values
        let scd_threshold_bits = Self::find_closest_scd_threshold_bits(
            scd_target_voltage_mv,
            config.protection_config.rsns_enable,
        );
        let ocd_threshold_bits = Self::find_closest_ocd_threshold_bits(
            ocd_target_voltage_mv,
            config.protection_config.rsns_enable,
        );

        // 3. Configure registers based on BatteryConfig and mapped thresholds

        // SYS_CTRL1
        let sys_ctrl1_flags = config.sys_ctrl1_flags;
        self.write_register(Register::SysCtrl1, sys_ctrl1_flags.bits())
            .await?;

        // SYS_CTRL2
        let sys_ctrl2_flags = config.sys_ctrl2_flags;
        self.write_register(Register::SysCtrl2, sys_ctrl2_flags.bits())
            .await?;

        // OV_TRIP and UV_TRIP
        // Convert voltage thresholds to raw register values
        // V_OV_TRIP = (OV_TRIP_REG * ADCGAIN / 1000) + ADCOFFSET
        // OV_TRIP_REG = (V_OV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        // OV_TRIP_FULL = (V_OV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        // OV_TRIP_FULL = (V_OV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let ov_trip_full =
            // Datasheet formula: OV_TRIP_FULL = (OV – ADCOFFSET) ÷ ADCGAIN
            // OV and ADCOFFSET are in mV, ADCGAIN is in μV/LSB.
            // OV_TRIP_FULL = ((OV_mV - ADCOFFSET_mV) * 1000 μV/mV) / ADCGAIN_μV_per_LSB
            ((config.overvoltage_trip as i32 - adc_offset as i32) * 1000 / adc_gain as i32) as u16; // Added semicolon

        // Extract middle 8 bits: (ov_trip_full >> 4) & 0xFF
        // Datasheet says upper 2 MSB preset to "10", lower 4 LSB preset to "1000".
        // So the 8 bits are (14-bit value >> 4) & 0xFF.
        let ov_trip_raw = ((ov_trip_full >> 4) & 0xFF) as u8;

        // UV_TRIP_FULL = (V_UV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let uv_trip_full =
            // Datasheet formula: UV_TRIP_FULL = (UV – ADCOFFSET) ÷ ADCGAIN
            // UV and ADCOFFSET are in mV, ADCGAIN is in μV/LSB.
            // UV_TRIP_FULL = ((UV_mV - ADCOFFSET_mV) * 1000 μV/mV) / ADCGAIN_μV_per_LSB
            ((config.undervoltage_trip as i32 - adc_offset as i32) * 1000 / adc_gain as i32) as u16; // Added semicolon

        // Extract middle 8 bits: (uv_trip_full >> 4) & 0xFF
        // Datasheet says upper 2 MSB preset to "01", lower 4 LSB preset to "0000".
        // So the 8 bits are (14-bit value >> 4) & 0xFF.
        let uv_trip_raw = ((uv_trip_full >> 4) & 0xFF) as u8;

        self.write_register(Register::OvTrip, ov_trip_raw).await?;
        self.write_register(Register::UvTrip, uv_trip_raw).await?;

        // Protection registers
        self.configure_protect1(
            config.protection_config.rsns_enable,
            config.protection_config.scd_delay,
            scd_threshold_bits,
        )
        .await?;
        self.configure_protect2(config.protection_config.ocd_delay, ocd_threshold_bits)
            .await?;
        self.configure_protect3(
            config.protection_config.uv_delay,
            config.protection_config.ov_delay,
        )
        .await?;

        // Clear all status flags (SYS_STAT)
        self.write_register(Register::SysStat, 0b11111111).await?;

        // Set CC_CFG to 0x19 for optimal performance
        self.write_register(Register::CcCfg, 0x19).await?;

        Ok(())
    }

    /// Finds the closest SCD threshold register bits for a target voltage in mV.
    fn find_closest_scd_threshold_bits(target_voltage_mv: i32, rsns_enable: bool) -> u8 {
        // Datasheet Table 8-10. SCD_THRESH Settings
        // Values are in mV.
        let thresholds_rsns_0 = [22, 33, 44, 67, 89, 111, 133, 155];
        let thresholds_rsns_1 = [44, 67, 89, 111, 133, 155, 178, 200];

        let thresholds = if rsns_enable {
            &thresholds_rsns_1
        } else {
            &thresholds_rsns_0
        };

        let mut closest_bits = 0;
        let mut min_diff = i32::MAX; // Changed to i32

        for (i, &threshold) in thresholds.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as i32).abs();
            if diff < min_diff {
                // Now i32 < i32
                min_diff = diff; // Now i32 = i32
                closest_bits = i as u8;
            }
        }
        closest_bits
    }

    /// Finds the closest OCD threshold register bits for a target voltage in mV.
    fn find_closest_ocd_threshold_bits(target_voltage_mv: i32, rsns_enable: bool) -> u8 {
        // Datasheet Table 8-11. OCD_THRESH Settings
        // Values are in mV.
        let thresholds_rsns_0 = [
            11, 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94,
        ];
        let thresholds_rsns_1 = [
            17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100,
        ];

        let thresholds = if rsns_enable {
            &thresholds_rsns_1
        } else {
            &thresholds_rsns_0
        };

        let mut closest_bits = 0;
        let mut min_diff = i32::MAX; // Changed to i32

        for (i, &threshold) in thresholds.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as i32).abs();
            if diff < min_diff {
                // Now i32 < i32
                min_diff = diff; // Now i32 = i32
                closest_bits = i as u8;
            }
        }
        closest_bits
    }

    /// Calculates the expected raw 8-bit value for the OV_TRIP or UV_TRIP register.
    /// `trip_voltage_mv` is the desired trip voltage in mV.
    /// `adc_gain_uv_per_lsb` and `adc_offset_mv` are from `read_adc_calibration()`.
    fn calculate_ovuv_trip_raw(
        trip_voltage_mv: u32,
        adc_gain_uv_per_lsb: u32,
        adc_offset_mv: i16,
    ) -> u8 {
        // Datasheet formula: TRIP_FULL = ((TRIP_mV - ADCOFFSET_mV) * 1000 μV/mV) / ADCGAIN_μV_per_LSB
        // Ensure non-negative result before division, then cast to u16 for bitwise operations.
        let numerator = (trip_voltage_mv as i32 - adc_offset_mv as i32) * 1000;
        let trip_full = if numerator < 0 || adc_gain_uv_per_lsb == 0 {
            0 // Avoid division by zero or negative intermediate results leading to large u16
        } else {
            (numerator / adc_gain_uv_per_lsb as i32) as u16
        };
        // Extract middle 8 bits: (14-bit value >> 4) & 0xFF.
        // OV_TRIP: upper 2 MSB preset to "10", lower 4 LSB preset to "1000".
        // UV_TRIP: upper 2 MSB preset to "01", lower 4 LSB preset to "0000".
        // The raw value written to the register is the middle 8 bits of this 14-bit ADC code.
        ((trip_full >> 4) & 0xFF) as u8
    }

    /// Calculates the expected raw 8-bit value for the PROTECT1 register.
    fn calculate_protect1_raw(config: &ProtectionConfig, rsense_m_ohm: u32) -> u8 {
        let scd_target_voltage_mv = config.scd_limit * rsense_m_ohm as i32; // mV = mA * mOhm (scaled by 1000 in practice, but for threshold bits it's direct)
        let scd_threshold_bits =
            Self::find_closest_scd_threshold_bits(scd_target_voltage_mv, config.rsns_enable);

        let mut protect1_val: u8 = 0;
        let mut protect1_flags = Protect1Flags::from_bits_truncate(protect1_val);
        if config.rsns_enable {
            protect1_flags.insert(Protect1Flags::RSNS);
        }
        protect1_val = protect1_flags.bits();
        protect1_val |= match config.scd_delay {
            ScdDelay::Delay70us => 0b00 << 3,
            ScdDelay::Delay100us => 0b01 << 3,
            ScdDelay::Delay200us => 0b10 << 3,
            ScdDelay::Delay400us => 0b11 << 3,
        };
        protect1_flags = Protect1Flags::from_bits_truncate(protect1_val); // Re-assign to use updated protect1_val
        protect1_flags.insert(
            Protect1Flags::from_bits_truncate(scd_threshold_bits) & Protect1Flags::SCD_THRESH_MASK,
        );
        protect1_flags.bits()
    }

    /// Calculates the expected raw 8-bit value for the PROTECT2 register.
    fn calculate_protect2_raw(config: &ProtectionConfig, rsense_m_ohm: u32) -> u8 {
        let ocd_target_voltage_mv = config.ocd_limit * rsense_m_ohm as i32;
        let ocd_threshold_bits =
            Self::find_closest_ocd_threshold_bits(ocd_target_voltage_mv, config.rsns_enable);

        let mut protect2_val: u8 = 0;
        protect2_val |= match config.ocd_delay {
            OcdDelay::Delay10ms => 0b000 << 4,
            OcdDelay::Delay20ms => 0b001 << 4,
            OcdDelay::Delay40ms => 0b010 << 4,
            OcdDelay::Delay80ms => 0b011 << 4,
            OcdDelay::Delay160ms => 0b100 << 4,
            OcdDelay::Delay320ms => 0b101 << 4,
            OcdDelay::Delay640ms => 0b110 << 4,
            OcdDelay::Delay1280ms => 0b111 << 4,
        };
        let mut protect2_flags = Protect2Flags::from_bits_truncate(protect2_val);
        protect2_flags.insert(
            Protect2Flags::from_bits_truncate(ocd_threshold_bits) & Protect2Flags::OCD_THRESH_MASK,
        );
        protect2_flags.bits()
    }

    /// Calculates the expected raw 8-bit value for the PROTECT3 register.
    fn calculate_protect3_raw(config: &ProtectionConfig) -> u8 {
        let mut protect3_val: u8 = 0;
        protect3_val |= match config.uv_delay {
            UvOvDelay::Delay1s => 0b00 << 6,
            UvOvDelay::Delay2s => 0b01 << 6,
            UvOvDelay::Delay4s => 0b10 << 6,
            UvOvDelay::Delay8s => 0b11 << 6,
        };
        protect3_val |= match config.ov_delay {
            UvOvDelay::Delay1s => 0b00 << 4,
            UvOvDelay::Delay2s => 0b01 << 4,
            UvOvDelay::Delay4s => 0b10 << 4,
            UvOvDelay::Delay8s => 0b11 << 4,
        };
        protect3_val
    }

    /// Sets the configuration of the BQ769x0 chip and verifies that key registers were written correctly.
    pub async fn try_apply_config(&mut self, config: &BatteryConfig) -> Result<(), Error<E>> {
        // Step 1: Apply the configuration by calling the existing set_config
        self.set_config(config).await?;

        // Step 2: Verify key configuration registers
        // Read ADC calibration values first, as they are needed for OV/UV trip calculations
        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        // Verify OV_TRIP
        let expected_ov_trip_raw =
            Self::calculate_ovuv_trip_raw(config.overvoltage_trip, adc_gain, adc_offset);
        let actual_ov_trip_raw = self.read_register(Register::OvTrip).await?;
        if actual_ov_trip_raw != expected_ov_trip_raw {
            return Err(Error::ConfigVerificationFailed {
                register: Register::OvTrip,
                expected: expected_ov_trip_raw,
                actual: actual_ov_trip_raw,
            });
        }

        // Verify UV_TRIP
        let expected_uv_trip_raw =
            Self::calculate_ovuv_trip_raw(config.undervoltage_trip, adc_gain, adc_offset);
        let actual_uv_trip_raw = self.read_register(Register::UvTrip).await?;
        if actual_uv_trip_raw != expected_uv_trip_raw {
            return Err(Error::ConfigVerificationFailed {
                register: Register::UvTrip,
                expected: expected_uv_trip_raw,
                actual: actual_uv_trip_raw,
            });
        }

        // Verify PROTECT1
        let expected_protect1_raw =
            Self::calculate_protect1_raw(&config.protection_config, config.rsense);
        let actual_protect1_raw = self.read_register(Register::PROTECT1).await?;
        if actual_protect1_raw != expected_protect1_raw {
            return Err(Error::ConfigVerificationFailed {
                register: Register::PROTECT1,
                expected: expected_protect1_raw,
                actual: actual_protect1_raw,
            });
        }

        // Verify PROTECT2
        let expected_protect2_raw =
            Self::calculate_protect2_raw(&config.protection_config, config.rsense);
        let actual_protect2_raw = self.read_register(Register::PROTECT2).await?;
        if actual_protect2_raw != expected_protect2_raw {
            return Err(Error::ConfigVerificationFailed {
                register: Register::PROTECT2,
                expected: expected_protect2_raw,
                actual: actual_protect2_raw,
            });
        }

        // Verify PROTECT3
        let expected_protect3_raw = Self::calculate_protect3_raw(&config.protection_config);
        let actual_protect3_raw = self.read_register(Register::PROTECT3).await?;
        if actual_protect3_raw != expected_protect3_raw {
            return Err(Error::ConfigVerificationFailed {
                register: Register::PROTECT3,
                expected: expected_protect3_raw,
                actual: actual_protect3_raw,
            });
        }

        // Verify CC_CFG (expected to be 0x19 as set in set_config)
        const EXPECTED_CC_CFG: u8 = 0x19;
        let actual_cc_cfg = self.read_register(Register::CcCfg).await?;
        if actual_cc_cfg != EXPECTED_CC_CFG {
            return Err(Error::ConfigVerificationFailed {
                register: Register::CcCfg,
                expected: EXPECTED_CC_CFG,
                actual: actual_cc_cfg,
            });
        }

        // Verify SYS_CTRL1
        let expected_sys_ctrl1_bits = config.sys_ctrl1_flags.bits();
        let actual_sys_ctrl1_bits = self.read_register(Register::SysCtrl1).await?;
        if actual_sys_ctrl1_bits != expected_sys_ctrl1_bits {
            return Err(Error::ConfigVerificationFailed {
                register: Register::SysCtrl1,
                expected: expected_sys_ctrl1_bits,
                actual: actual_sys_ctrl1_bits,
            });
        }

        // Verify SYS_CTRL2 (base configuration, CHG_ON and DSG_ON are not expected to be set by set_config via BatteryConfig::default)
        let expected_sys_ctrl2_bits = config.sys_ctrl2_flags.bits(); // Should be SysCtrl2Flags::CC_EN.bits() from default
        let actual_sys_ctrl2_bits = self.read_register(Register::SysCtrl2).await?;
        if actual_sys_ctrl2_bits != expected_sys_ctrl2_bits {
            // This check might be too strict if other bits in SYS_CTRL2 can be legitimately set by other means
            // or if the default config changes. For now, assume direct correspondence.
            // We are primarily interested that CC_EN is set and CHG_ON/DSG_ON are NOT set by set_config.
            // A more robust check might be:
            // if (actual_sys_ctrl2_bits & SysCtrl2Flags::CC_EN.bits()) != SysCtrl2Flags::CC_EN.bits() ||
            //    (actual_sys_ctrl2_bits & (SysCtrl2Flags::CHG_ON | SysCtrl2Flags::DSG_ON).bits()) != 0 { ... }
            // However, for a direct verification of what set_config wrote based on BatteryConfig::default():
            return Err(Error::ConfigVerificationFailed {
                register: Register::SysCtrl2,
                expected: expected_sys_ctrl2_bits,
                actual: actual_sys_ctrl2_bits,
            });
        }

        // Note: SYS_STAT is cleared by set_config, verifying it would mean expecting 0x00 (or specific flags cleared).
        // For now, we trust set_config clears it. If specific verification is needed, it can be added.

        Ok(())
    }
}
