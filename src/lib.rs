#![no_std]

#[macro_use]
extern crate uom;

use core::ops::Deref;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

pub mod registers;
use registers::*; // Import bit masks

mod crc;
mod data_types;
mod errors;
pub mod units; // Make the units module public

pub use data_types::{
    BatteryConfig, CellVoltages, CoulombCounter, Temperatures, SystemStatus,
    TempSensor, ScdDelay, OcdDelay, UvOvDelay, ProtectionConfig,
};
use errors::Error;

pub use crc::{calculate_crc, CrcMode, Disabled, Enabled};

use crate::units::{ElectricPotential, ElectricCurrent}; // 从 crate::units 导入基本类型
use uom::si::electric_potential::millivolt;
use uom::si::thermodynamic_temperature::kelvin;
use crate::units::ElectricalResistance; // 从 crate::units 导入 ElectricalResistance
 // 导入 milliohm
// 确保 hundred_micro_ohm 导入路径正确

/// BQ769x0 driver
pub struct Bq769x0<I2C, M: CrcMode>
where
    I2C: I2c,
{
    address: u8,
    i2c: I2C,
    _crc_mode: core::marker::PhantomData<M>,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "RegisterAccess<E>",),
    async(feature = "async", keep_self)
)]
#[allow(async_fn_in_trait)]
/// Trait for abstracting register access, with or without CRC.
pub trait RegisterAccess<E>
where
    Self: Sized,
{
    /// The buffer type used for reading multiple registers.
    type ReadBuffer: Deref<Target = [u8]>;

    /// Reads a single byte from the specified register.
    fn read_register(
        &mut self,
        reg: Register,
    ) -> impl core::future::Future<Output = Result<u8, Error<E>>>;

    /// Reads multiple bytes starting from the specified register.
    fn read_registers(
        &mut self,
        reg: Register,
        len: usize,
    ) -> impl core::future::Future<Output = Result<Self::ReadBuffer, Error<E>>>;

    /// Writes a single byte to the specified register.
    fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> impl core::future::Future<Output = Result<(), Error<E>>>;

    /// Writes multiple bytes starting from the specified register.
    fn write_registers(
        &mut self,
        reg: Register,
        values: &[u8],
    ) -> impl core::future::Future<Output = Result<(), Error<E>>>;
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
    async fn read_adc_calibration(&mut self) -> Result<(ElectricPotential, ElectricPotential), Error<E>> {
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Datasheet section 8.1.2.3 ADC Gain and Offset
        // ADCGAIN = (ADCGAIN1[7:0] << 2) | ADCGAIN2[7:6]
        let adc_gain_raw = ((adc_gain1 as u16) << 2) | ((adc_gain2 as u16 >> 6) & 0b11);

        // ADCOFFSET is signed 8-bit
        let adc_offset_signed = adc_offset as i8;

        // Convert raw ADC gain and offset to uom types
        // ADCGAIN is in units of 10uV/LSB, ADCOFFSET is in mV
        let adc_gain = ElectricPotential::new::<uom::si::electric_potential::microvolt>(adc_gain_raw as f32 * 10.0);
        let adc_offset = ElectricPotential::new::<millivolt>(adc_offset_signed as f32);

        Ok((adc_gain, adc_offset))
    }

    /// Reads the cell voltages from the VCTXHi and VcTxLo registers.
    pub async fn read_cell_voltages(&mut self) -> Result<CellVoltages<NUM_CELLS>, Error<E>> {
        let start_reg = Register::Vc1Hi;
        let len = NUM_CELLS * 2; // Each cell voltage is 2 bytes (Hi and Lo)
        let raw_data = self.read_registers(start_reg, len).await?;

        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        let mut cell_voltages = CellVoltages::new();

        // Define a dimensionless constant for 1000
        let thousand = 1000.0;

        for i in 0..NUM_CELLS {
            let hi_byte = raw_data[i * 2];
            let lo_byte = raw_data[i * 2 + 1];
            let raw_voltage = ((hi_byte as u16) << 8) | (lo_byte as u16);

            // Datasheet section 8.1.2.3 ADC Gain and Offset
            // Cell Voltage (mV) = (raw_voltage * ADCGAIN / 1000) + ADCOFFSET
            // raw_voltage is a dimensionless ADC reading.
            // adc_gain is ElectricPotential (microvolt).
            // adc_offset is ElectricPotential (millivolt).
            let voltage = (adc_gain * raw_voltage as f32) / thousand + adc_offset;

            cell_voltages.voltages[i] = voltage;
        }

        Ok(cell_voltages)
    }

    /// Reads the battery pack voltage from the BatHi and BatLo registers.
    pub async fn read_pack_voltage(&mut self) -> Result<ElectricPotential, Error<E>> {
        let raw_data = self.read_registers(Register::BatHi, 2).await?;
        let hi_byte = raw_data[0];
        let lo_byte = raw_data[1];
        let raw_voltage = ((hi_byte as u16) << 8) | (lo_byte as u16);

        // Pack voltage is sum of cell voltages, no separate calibration needed here.
        // The datasheet implies BatHi/Lo is a direct sum, but it's good practice
        // to verify this or use cell voltages for pack voltage calculation.
        // For simplicity, we'll read the register directly for now.
        // The conversion factor is 1mV per LSB (based on cell voltage conversion).
        Ok(ElectricPotential::new::<millivolt>(raw_voltage as f32))
    }

    /// Reads the temperature sensors (TS1, TS2, TS3 or Die Temp)
    pub async fn read_temperatures(&mut self) -> Result<Temperatures, Error<E>> {
        // The number of temperature sensors depends on the chip variant (BQ76920, BQ76930, BQ76940)
        // and the TEMP_SEL bit in SYS_CTRL1.
        // For BQ76920, there is one external TS pin (TS1) and internal die temp.
        // TEMP_SEL = 0: TS1 is Die Temp
        // TEMP_SEL = 1: TS1 is external thermistor

        // Read SYS_CTRL1 to check TEMP_SEL
        let sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        let temp_sel = (sys_ctrl1 & SYS_CTRL1_TEMP_SEL) != 0;

        let mut temperatures = Temperatures::new();

        // TS1 (always present)
        let raw_ts1_data = self.read_registers(Register::Ts1Hi, 2).await?;
        let raw_ts1 = ((raw_ts1_data[0] as u16) << 8) | (raw_ts1_data[1] as u16);
        // Datasheet section 8.1.2.4 Temperature
        // Temperature (K) = (raw_ts * 0.01) + 273.15
        // Datasheet section 8.1.2.4 Temperature
        // Temperature (K) = (raw_ts * 0.01) + 273.15
        // 由于 uom 使用 u32 作为底层存储，这里将浮点数计算结果乘以 100000 转换为整数，以保留精度
        // 1K = 100000 uK (microkelvin)
        // Temperature (K) = (raw_ts * 0.01) + 273.15
        // Temperature (uK) = (raw_ts * 1000) + (273150000)
        let temp_k_scaled = (raw_ts1 as f32 * 0.01) + 273.15;
        temperatures.ts1 = crate::units::ThermodynamicTemperature::new::<kelvin>(temp_k_scaled);

        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // TS2 (BQ76930/40 only)
            let raw_ts2_data = self.read_registers(Register::Ts2Hi, 2).await?;
            let raw_ts2 = ((raw_ts2_data[0] as u16) << 8) | (raw_ts2_data[1] as u16);
            let temp_k_scaled = (raw_ts2 as f32 * 0.01) + 273.15;
            temperatures.ts2 = Some(crate::units::ThermodynamicTemperature::new::<kelvin>(temp_k_scaled));
        }

        #[cfg(feature = "bq76940")]
        {
            // TS3 (BQ76940 only)
            let raw_ts3_data = self.read_registers(Register::Ts3Hi, 2).await?;
            let raw_ts3 = ((raw_ts3_data[0] as u16) << 8) | (raw_ts3_data[1] as u16);
            let temp_k_scaled = (raw_ts3 as f32 * 0.01) + 273.15;
            temperatures.ts3 = Some(crate::units::ThermodynamicTemperature::new::<kelvin>(temp_k_scaled));
        }

        // Indicate if TS1 is Die Temp or External
        temperatures.is_thermistor = temp_sel;

        Ok(temperatures)
    }

    /// Reads the Coulomb Counter value.
    pub async fn read_current(&mut self) -> Result<CoulombCounter, Error<E>> {
        let raw_data = self.read_registers(Register::CcHi, 2).await?;
        let raw_cc = ((raw_data[0] as i16) << 8) | (raw_data[1] as i16); // CC is signed

        // The conversion from raw CC to current depends on Rsense and ADCGAIN.
        // This conversion should be done by the user or in a higher-level abstraction.
        // We provide the raw value here.
        Ok(CoulombCounter { raw_cc })
    }
    /// Reads the system status flags from the SYS_STAT register.
    pub async fn read_status(&mut self) -> Result<SystemStatus, Error<E>> {
        let status_byte = self.read_register(Register::SysStat).await?;
        Ok(SystemStatus::new(status_byte))
    }

    /// Clears the specified status flags in the SYS_STAT register.
    pub async fn clear_status_flags(&mut self, flags: u8) -> Result<(), Error<E>> {
        // Writing a '1' to a flag bit clears it.
        self.write_register(Register::SysStat, flags).await
    }

    /// Enables charging by setting the CHG_ON bit in SYS_CTRL2.
    pub async fn enable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 |= SYS_CTRL2_CHG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Disables charging by clearing the CHG_ON bit in SYS_CTRL2.
    pub async fn disable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 &= !SYS_CTRL2_CHG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }

    /// Enables discharging by setting the DSG_ON bit in SYS_CTRL2.
    pub async fn enable_discharging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SysCtrl2).await?;
        sys_ctrl2 |= SYS_CTRL2_DSG_ON;
        self.write_register(Register::SysCtrl2, sys_ctrl2).await
    }
    
    /// Sets the cell balancing for cells 1-5 (CELLBAL1 register).
    /// The mask is a bitmask where each bit corresponds to a cell (Bit 0 = Cell 1, Bit 4 = Cell 5).
    pub async fn set_cell_balancing(&mut self, mask: u16) -> Result<(), Error<E>> {
        // Only bits 0-4 are valid for CELLBAL1
        let cellbal1_value = (mask & 0x1F) as u8;
        self.write_register(Register::CELLBAL1, cellbal1_value)
            .await?;

        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // For BQ76930/40, also set CELLBAL2 (Cells 6-10)
            let cellbal2_value = ((mask >> 5) & 0x1F) as u8;
            self.write_register(Register::CELLBAL2, cellbal2_value)
                .await?;
        }

        #[cfg(feature = "bq76940")]
        {
            // For BQ76940, also set CELLBAL3 (Cells 11-15)
            let cellbal3_value = ((mask >> 10) & 0x1F) as u8;
            self.write_register(Register::CELLBAL3, cellbal3_value)
                .await?;
        }

        Ok(())
    }

    /// Configures the PROTECT1 register (SCD).
    /// This method is now internal and used by apply_config.
    async fn configure_protect1(
        &mut self,
        rsns_enable: bool,
        scd_delay: data_types::ScdDelay,
        scd_threshold_bits: u8,
    ) -> Result<(), Error<E>> {
        let mut protect1: u8 = 0;
        if rsns_enable {
            protect1 |= PROTECT1_RSNS;
        }
        protect1 |= match scd_delay {
            data_types::ScdDelay::Delay70us => 0b00 << 3,
            data_types::ScdDelay::Delay100us => 0b01 << 3,
            data_types::ScdDelay::Delay200us => 0b10 << 3,
            data_types::ScdDelay::Delay400us => 0b11 << 3,
        };
        protect1 |= scd_threshold_bits & 0b111; // Ensure only the lower 3 bits are used
        self.write_register(Register::PROTECT1, protect1).await
    }

    /// Configures the PROTECT2 register (OCD).
    /// This method is now internal and used by apply_config.
    async fn configure_protect2(
        &mut self,
        ocd_delay: data_types::OcdDelay,
        ocd_threshold_bits: u8,
    ) -> Result<(), Error<E>> {
        let mut protect2: u8 = 0;
        protect2 |= match ocd_delay {
            data_types::OcdDelay::Delay10ms => 0b000 << 4,
            data_types::OcdDelay::Delay20ms => 0b001 << 4,
            data_types::OcdDelay::Delay40ms => 0b010 << 4,
            data_types::OcdDelay::Delay80ms => 0b011 << 4,
            data_types::OcdDelay::Delay160ms => 0b100 << 4,
            data_types::OcdDelay::Delay320ms => 0b101 << 4,
            data_types::OcdDelay::Delay640ms => 0b110 << 4,
            data_types::OcdDelay::Delay1280ms => 0b111 << 4,
        };
        protect2 |= ocd_threshold_bits & 0b1111; // Ensure only the lower 4 bits are used
        self.write_register(Register::PROTECT2, protect2).await
    }

    /// Configures the PROTECT3 register (UV/OV Delay).
    /// This method is now internal and used by apply_config.
    async fn configure_protect3(
        &mut self,
        uv_delay: data_types::UvOvDelay,
        ov_delay: data_types::UvOvDelay,
    ) -> Result<(), Error<E>> {
        let mut protect3: u8 = 0;
        protect3 |= match uv_delay {
            data_types::UvOvDelay::Delay1s => 0b00 << 6,
            data_types::UvOvDelay::Delay2s => 0b01 << 6,
            data_types::UvOvDelay::Delay4s => 0b10 << 6,
            data_types::UvOvDelay::Delay8s => 0b11 << 6,
        };
        protect3 |= match ov_delay {
            data_types::UvOvDelay::Delay1s => 0b00 << 4,
            data_types::UvOvDelay::Delay2s => 0b01 << 4,
            data_types::UvOvDelay::Delay4s => 0b10 << 4,
            data_types::UvOvDelay::Delay8s => 0b11 << 4,
        };
        self.write_register(Register::PROTECT3, protect3).await
    }

    /// Enters ship mode.
    /// This requires writing 0x00 to the SYS_CTRL1 register twice within 4 seconds.
    pub async fn enter_ship_mode(&mut self) -> Result<(), Error<E>> {
        // Read current SYS_CTRL1 value to restore later if needed (though ship mode is usually permanent until wake)
        // let original_sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;

        // Write 0x00 to SYS_CTRL1
        self.write_register(Register::SysCtrl1, 0x00).await?;

        // Wait for a short period (less than 4 seconds) - a small delay is usually sufficient
        // In a real application, you might need a non-blocking delay here.
        // For this example, we'll assume a blocking delay is acceptable or handled by the async runtime.
        // core::thread::sleep(core::time::Duration::from_millis(100)); // Example blocking delay

        // Write 0x00 to SYS_CTRL1 again
        self.write_register(Register::SysCtrl1, 0x00).await?;

        Ok(())
    }

    /// Checks if the ALERT pin is overridden.
    pub async fn is_alert_overridden(&mut self) -> Result<bool, Error<E>> {
        let sys_stat = self.read_register(Register::SysStat).await?;
        Ok((sys_stat & SYS_STAT_OVRD_ALERT) != 0)
    }

    /// Converts a raw Coulomb Counter value to current in mA.
    /// This is a helper function and requires the ADCGAIN and Rsense values.
    pub fn convert_raw_cc_to_current_ma(&self, raw_cc: i16, rsense: ElectricalResistance) -> ElectricCurrent {
        // Datasheet section 8.1.2.5 Current
        // Current (mA) = raw_cc * 8.44 / Rsense (mOhm)
        // 8.44 is in mV/LSB for CC, so (raw_cc * 8.44) is in mV.
        // Rsense is in mOhm.
        // Current = Voltage / Resistance.
        // Voltage (mV) = raw_cc * 8.44
        // Resistance (mOhm)
        // Result should be in mA.
        // 8.44 mV/LSB
        let voltage_per_lsb = ElectricPotential::new::<uom::si::electric_potential::microvolt>(8440.0); // 8.44 mV = 8440 microvolts
        let raw_voltage = voltage_per_lsb * raw_cc as f32;
        (raw_voltage / rsense).into()
    }

    /// Sets the configuration of the BQ769x0 chip based on the provided BatteryConfig.
    pub async fn set_config(&mut self, config: &BatteryConfig) -> Result<(), Error<E>> {
        // Implement current limit to voltage threshold mapping and register writing here

        // 1. Calculate target voltage thresholds from current limits and Rsense
        // 1. Calculate target voltage thresholds from current limits and Rsense
        let scd_target_voltage =
            config.protection_config.scd_limit * config.rsense;
        let ocd_target_voltage =
            config.protection_config.ocd_limit * config.rsense;

        // 2. Find the closest supported voltage thresholds and get their register bit values
        let scd_threshold_bits = Self::find_closest_scd_threshold_bits(scd_target_voltage.get::<millivolt>() as f32);
        let ocd_threshold_bits = Self::find_closest_ocd_threshold_bits(ocd_target_voltage.get::<millivolt>() as f32);

        // 3. Configure registers based on BatteryConfig and mapped thresholds

        // SYS_CTRL1
        let mut sys_ctrl1: u8 = 0;
        if config.load_present {
            sys_ctrl1 |= SYS_CTRL1_LOAD_PRESENT;
        }
        if config.adc_enable {
            sys_ctrl1 |= SYS_CTRL1_ADC_EN;
        }
        match config.temp_sensor_selection {
            data_types::TempSensor::Internal => { /* TEMP_SEL = 0, default */ }
            data_types::TempSensor::External => {
                sys_ctrl1 |= SYS_CTRL1_TEMP_SEL;
            }
        }
        if config.shutdown_a {
            sys_ctrl1 |= SYS_CTRL1_SHUT_A;
        }
        if config.shutdown_b {
            sys_ctrl1 |= SYS_CTRL1_SHUT_B;
        }
        self.write_register(Register::SysCtrl1, sys_ctrl1).await?;

        // SYS_CTRL2
        let mut sys_ctrl2: u8 = 0;
        if config.delay_disable {
            sys_ctrl2 |= SYS_CTRL2_DELAY_DIS;
        }
        if config.cc_enable {
            sys_ctrl2 |= SYS_CTRL2_CC_EN;
        }
        if config.cc_oneshot {
            sys_ctrl2 |= SYS_CTRL2_CC_ONESHOT;
        }
        if config.discharge_on {
            sys_ctrl2 |= SYS_CTRL2_DSG_ON;
        }
        if config.charge_on {
            sys_ctrl2 |= SYS_CTRL2_CHG_ON;
        }
        self.write_register(Register::SysCtrl2, sys_ctrl2).await?;

        // OV_TRIP
        let ov_trip_8bit = ((config.overvoltage_trip.get::<millivolt>() - 1200.0) / 5.0) as u8;
        self.write_register(Register::OvTrip, ov_trip_8bit).await?;

        // UV_TRIP
        let uv_trip_8bit = ((config.undervoltage_trip.get::<millivolt>() - 700.0) / 5.0) as u8;
        self.write_register(Register::UvTrip, uv_trip_8bit).await?;

        // PROTECT1
        self.configure_protect1(
            config.protection_config.rsns_enable,
            config.protection_config.scd_delay,
            scd_threshold_bits,
        )
        .await?;

        // PROTECT2
        self.configure_protect2(config.protection_config.ocd_delay, ocd_threshold_bits)
            .await?;

        // PROTECT3
        self.configure_protect3(
            config.protection_config.uv_delay,
            config.protection_config.ov_delay,
        )
        .await?;

        // TODO: Implement CC_CFG configuration if needed

        Ok(())
    }

    // Helper function to find the closest SCD threshold bits
    fn find_closest_scd_threshold_bits(target_voltage_mv: f32) -> u8 {
        let thresholds_mv = [20, 30, 40, 50, 60, 70, 80, 90];
        let mut closest_bits = 0;
        let mut min_diff = f32::MAX;

        for (i, &threshold) in thresholds_mv.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as f32).abs();
            if diff < min_diff {
                min_diff = diff;
                closest_bits = i as u8;
            }
        }
        closest_bits
    }

    // Helper function to find the closest OCD threshold bits
    fn find_closest_ocd_threshold_bits(target_voltage_mv: f32) -> u8 {
        let thresholds_mv = [
            10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85,
        ];
        let mut closest_bits = 0;
        let mut min_diff = f32::MAX;

        for (i, &threshold) in thresholds_mv.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as f32).abs();
            if diff < min_diff {
                min_diff = diff;
                closest_bits = i as u8;
            }
        }
        closest_bits
    }
}
