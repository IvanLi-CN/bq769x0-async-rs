#![no_std]

#[macro_use]
extern crate uom;

#[cfg(feature = "defmt")]
extern crate defmt;

use core::ops::Deref;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

pub mod registers;
use registers::*; // Import bit masks

pub mod crc; // Make crc module public
pub mod data_types;
pub mod errors;
pub mod units; // Make the units module public

pub use data_types::{
    BatteryConfig, Bq76920Measurements, CellVoltages, CoulombCounter, MosStatus, OcdDelay,
    ProtectionConfig, ScdDelay, SystemStatus, TempSensor, TemperatureSensorReadings, UvOvDelay,
};
use errors::Error;

pub use crc::{calculate_crc, CrcMode, Disabled, Enabled};

use crate::units::{ElectricCurrent, ElectricPotential, ElectricalResistance};
use uom::si::electric_potential::millivolt;

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
    async fn read_adc_calibration(
        &mut self,
    ) -> Result<(ElectricPotential, ElectricPotential), Error<E>> {
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

        // Convert raw ADC gain and offset to uom types
        // GAIN = 365 μV/LSB + (ADCGAIN<4:0> in decimal) × (1 μV/LSB)
        let adc_gain = ElectricPotential::new::<uom::si::electric_potential::microvolt>(
            365.0 + adc_gain_raw as f32,
        );
        let adc_offset = ElectricPotential::new::<millivolt>(adc_offset_signed as f32);

        Ok((adc_gain, adc_offset))
    }

    /// Reads the cell voltages from the VCTXHi and VcTxLo registers.
    pub async fn read_cell_voltages(&mut self) -> Result<CellVoltages<N>, Error<E>>
    where
        Self: RegisterAccess<E>,
    {
        let start_reg = Register::Vc1Hi;
        let len = N * 2; // Each cell voltage is 2 bytes (Hi and Lo)
        let raw_data = self.read_registers(start_reg, len).await?;

        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        let mut cell_voltages = CellVoltages::new();

        for i in 0..N {
            let hi_byte = raw_data[i * 2];
            let lo_byte = raw_data[i * 2 + 1];
            let raw_voltage = (((hi_byte & 0x3f) as u16) << 8) | (lo_byte as u16);

            // Datasheet section 8.1.2.3 ADC Gain and Offset
            // Cell Voltage (mV) = (raw_voltage * ADCGAIN / 1000) + ADCOFFSET
            // raw_voltage is a dimensionless ADC reading.
            // adc_gain is ElectricPotential (microvolt).
            // adc_offset is ElectricPotential (millivolt).
            // Convert adc_gain to millivolt before calculation to match adc_offset unit.
            let voltage_mv =
                (adc_gain.get::<millivolt>() * raw_voltage as f32) + adc_offset.get::<millivolt>();
            cell_voltages.voltages[i] = ElectricPotential::new::<millivolt>(voltage_mv);
            #[cfg(feature = "defmt")]
            defmt::debug!(
                "Cell {}: raw_voltage={}, adc_gain_mv_per_lsb={}, adc_offset_mv={}, voltage_mv={}",
                i + 1,
                raw_voltage,
                adc_gain.get::<millivolt>(),
                adc_offset.get::<millivolt>(),
                voltage_mv
            );
        }

        Ok(cell_voltages)
    }

    /// Reads the battery pack voltage from the BatHi and BatLo registers.
    pub async fn read_pack_voltage(&mut self) -> Result<ElectricPotential, Error<E>> {
        let raw_data = self.read_registers(Register::BatHi, 2).await?;
        let hi_byte = raw_data[0];
        let lo_byte = raw_data[1];
        let raw_voltage = ((hi_byte as u16) << 8) | (lo_byte as u16);

        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        // Datasheet section 8.3.1.1.6 16-Bit Pack Voltage
        // V(BAT) = 4 × GAIN × ADC(cell) + (#Cells x OFFSET)
        // Assuming NUM_CELLS is defined globally or passed as a parameter.
        // For BQ76920, NUM_CELLS is 5.
        // For BQ76930, NUM_CELLS is 10.
        // For BQ76940, NUM_CELLS is 15.
        // The `NUM_CELLS` constant is defined in `data_types.rs` and used in `read_cell_voltages`.
        // We need to ensure `NUM_CELLS` is accessible here or passed.
        // Given the current structure, `NUM_CELLS` is a const generic parameter for `CellVoltages`,
        // but not directly available in `Bq769x0` impl.
        // For now, let's assume a fixed NUM_CELLS for the BQ76920 (5 cells) for this calculation,
        // or find a way to get the actual NUM_CELLS.
        // The problem statement implies we are working with BQ76920.
        // Let's use a placeholder for NUM_CELLS and address it if it becomes an issue.
        // For the test, NUM_CELLS is 5.

        let num_cells_f32 = N as f32;

        let pack_voltage_mv =
            (4.0 * adc_gain.get::<uom::si::electric_potential::microvolt>() * raw_voltage as f32)
                / 1000.0
                + (num_cells_f32 * adc_offset.get::<millivolt>());
        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Pack Voltage: raw_voltage={}, adc_gain_uv_per_lsb={}, adc_offset_mv={}, num_cells={}, pack_voltage_mv={}",
            raw_voltage,
            adc_gain.get::<uom::si::electric_potential::microvolt>(),
            adc_offset.get::<millivolt>(),
            num_cells_f32,
            pack_voltage_mv
        );

        Ok(ElectricPotential::new::<millivolt>(pack_voltage_mv))
    }

    /// Reads the temperature sensors (TS1, TS2, TS3 or Die Temp)
    pub async fn read_temperatures(&mut self) -> Result<TemperatureSensorReadings, Error<E>> {
        // The number of temperature sensors depends on the chip variant (BQ76920, BQ76930, BQ76940)
        // and the TEMP_SEL bit in SYS_CTRL1.
        // For BQ76920, there is one external TS pin (TS1) and internal die temp.
        // TEMP_SEL = 0: TS1 is Die Temp
        // TEMP_SEL = 1: TS1 is external thermistor

        // Read SYS_CTRL1 to check TEMP_SEL
        let sys_ctrl1 = self.read_register(Register::SysCtrl1).await?;
        let temp_sel = (sys_ctrl1 & SYS_CTRL1_TEMP_SEL) != 0;

        let mut temperatures = TemperatureSensorReadings::new();

        // Read TS1
        let ts1_raw_data = self.read_registers(Register::Ts1Hi, 2).await?;
        let ts1_hi = ts1_raw_data[0];
        let ts1_lo = ts1_raw_data[1];
        let ts1_raw = ((ts1_hi as u16) << 8) | (ts1_lo as u16);

        if !temp_sel {
            // Die Temp
            // Datasheet section 8.1.2.4 Temperature
            // Die Temp
            // Datasheet section 8.1.2.4 Temperature
            // V_TSX = ADC(TS) * 382 μV/LSB
            let v_tsx_uv = ts1_raw as f32 * 382.0; // Calculate V_TSX in μV
            let v_tsx_mv = v_tsx_uv / 1000.0; // Convert V_TSX to mV

            // For internal die temp, we store the raw voltage reading.
            // Conversion to actual temperature (Celsius/Kelvin) will be done by the caller.
            temperatures.ts1 = ElectricPotential::new::<millivolt>(v_tsx_mv);
            temperatures.is_thermistor = false;
        } else {
            // External Thermistor
            // Datasheet section 8.1.2.4 Temperature
            // V_TSX = ADC(TS) * 382 μV/LSB
            let (adc_gain, _) = self.read_adc_calibration().await?; // Use adc_gain for external thermistor calculation
            let v_tsx_mv = (ts1_raw as f32
                * adc_gain.get::<uom::si::electric_potential::microvolt>())
                / 1000.0;
            // For external thermistor, we store the raw voltage reading.
            // Conversion to actual thermistor resistance and then temperature will be done by the caller.
            temperatures.ts1 = ElectricPotential::new::<millivolt>(v_tsx_mv);
            temperatures.is_thermistor = true;
        }

        // Handle TS2 and TS3 for BQ76930/40
        if N >= 10 {
            let ts2_raw_data = self.read_registers(Register::Ts2Hi, 2).await?;
            let ts2_hi = ts2_raw_data[0];
            let ts2_lo = ts2_raw_data[1];
            let ts2_raw = ((ts2_hi as u16) << 8) | (ts2_lo as u16);

            if !temp_sel {
                let v_tsx_mv = (ts2_raw as f32 * 1.265 / 4095.0) * 1000.0;
                temperatures.ts2 = Some(ElectricPotential::new::<millivolt>(v_tsx_mv));
            } else {
                let (adc_gain, _) = self.read_adc_calibration().await?;
                let v_tsx_mv = (ts2_raw as f32
                    * adc_gain.get::<uom::si::electric_potential::microvolt>())
                    / 1000.0;
                temperatures.ts2 = Some(ElectricPotential::new::<millivolt>(v_tsx_mv));
            }
        }

        if N >= 15 {
            let ts3_raw_data = self.read_registers(Register::Ts3Hi, 2).await?;
            let ts3_hi = ts3_raw_data[0];
            let ts3_lo = ts3_raw_data[1];
            let ts3_raw = ((ts3_hi as u16) << 8) | (ts3_lo as u16);

            if !temp_sel {
                let v_tsx_mv = (ts3_raw as f32 * 1.265 / 4095.0) * 1000.0;
                temperatures.ts3 = Some(ElectricPotential::new::<millivolt>(v_tsx_mv));
            } else {
                let (adc_gain, _) = self.read_adc_calibration().await?;
                let v_tsx_mv = (ts3_raw as f32
                    * adc_gain.get::<uom::si::electric_potential::microvolt>())
                    / 1000.0;
                temperatures.ts3 = Some(ElectricPotential::new::<millivolt>(v_tsx_mv));
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
                ElectricalResistance::new::<uom::si::electrical_resistance::milliohm>(10.0),
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
        if rsns_enable {
            protect1 |= PROTECT1_RSNS;
        }
        protect1 |= match scd_delay {
            ScdDelay::Delay70us => 0b00 << 3,
            ScdDelay::Delay100us => 0b01 << 3,
            ScdDelay::Delay200us => 0b10 << 3,
            ScdDelay::Delay400us => 0b11 << 3,
        };
        protect1 |= scd_threshold_bits & PROTECT1_SCD_THRESH; // Ensure only relevant bits are set
        self.write_register(Register::PROTECT1, protect1).await
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
        protect2 |= ocd_threshold_bits & PROTECT2_OCD_THRESH; // Ensure only relevant bits are set
        self.write_register(Register::PROTECT2, protect2).await
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
        self.write_register(Register::SysCtrl1, SYS_CTRL1_SHUT_B) // Write #1: SHUT_B = 1
            .await?;
        self.write_register(Register::SysCtrl1, SYS_CTRL1_SHUT_A) // Write #2: SHUT_A = 1
            .await?;
        Ok(())
    }

    /// Checks if the ALERT pin is overridden.
    pub async fn is_alert_overridden(&mut self) -> Result<bool, Error<E>> {
        let sys_stat = self.read_register(Register::SysStat).await?;
        Ok((sys_stat & SYS_STAT_OVRD_ALERT) != 0)
    }

    /// Converts a raw Coulomb Counter value to current in mA.
    /// This is a helper function and requires the ADCGAIN and Rsense values.
    pub fn convert_raw_cc_to_current_ma(
        &self,
        raw_cc: i16,
        rsense: ElectricalResistance,
    ) -> ElectricCurrent {
        // Datasheet section 8.1.2.5 Current
        // Current (mA) = raw_cc * 8.44 / Rsense (mOhm)
        // 8.44 is in mV/LSB for CC, so (raw_cc * 8.44) is in mV.
        // Rsense is in mOhm.
        // Current = Voltage / Resistance.
        // 8.44 μV/LSB
        let voltage_per_lsb =
            ElectricPotential::new::<uom::si::electric_potential::microvolt>(8.44); // 8.44 μV/LSB
        let raw_voltage = voltage_per_lsb * raw_cc as f32;
        let current_microampere =
            (raw_voltage / rsense).get::<uom::si::electric_current::microampere>(); // Calculate in microampere

        #[cfg(feature = "defmt")]
        defmt::info!(
            "Raw CC: {}, Rsense: {:?}, Current (uA): {}",
            raw_cc,
            rsense.get::<uom::si::electrical_resistance::milliohm>(),
            current_microampere
        );

        // Convert microampere to milliampere
        ElectricCurrent::new::<uom::si::electric_current::milliampere>(current_microampere / 1000.0)
    }

    /// Sets the configuration of the BQ769x0 chip based on the provided BatteryConfig.
    pub async fn set_config(&mut self, config: &BatteryConfig) -> Result<(), Error<E>> {
        // Implement current limit to voltage threshold mapping and register writing here

        // 1. Calculate target voltage thresholds from current limits and Rsense
        // 1. Calculate target voltage thresholds from current limits and Rsense
        let scd_target_voltage = config.protection_config.scd_limit * config.rsense;
        let ocd_target_voltage = config.protection_config.ocd_limit * config.rsense;

        // 2. Find the closest supported voltage thresholds and get their register bit values
        let scd_threshold_bits = Self::find_closest_scd_threshold_bits(
            scd_target_voltage.get::<millivolt>(),
            config.protection_config.rsns_enable,
        );
        let ocd_threshold_bits = Self::find_closest_ocd_threshold_bits(
            ocd_target_voltage.get::<millivolt>(),
            config.protection_config.rsns_enable,
        );

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

        // OV_TRIP and UV_TRIP
        // Convert voltage thresholds to raw register values
        // V_OV_TRIP = (OV_TRIP_REG * ADCGAIN / 1000) + ADCOFFSET
        // OV_TRIP_REG = (V_OV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let (adc_gain, adc_offset) = self.read_adc_calibration().await?;

        // OV_TRIP_FULL = (V_OV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let ov_trip_full =
            ((config.overvoltage_trip.get::<millivolt>() - adc_offset.get::<millivolt>()) * 1000.0
                / adc_gain.get::<uom::si::electric_potential::microvolt>()) as u16; // Use u16 for 14-bit value

        // Extract middle 8 bits: (ov_trip_full >> 4) & 0xFF
        // Datasheet says upper 2 MSB preset to "10", lower 4 LSB preset to "1000".
        // So the 8 bits are (14-bit value >> 4) & 0xFF.
        let ov_trip_raw = ((ov_trip_full >> 4) & 0xFF) as u8;

        // UV_TRIP_FULL = (V_UV_TRIP - ADCOFFSET) * 1000 / ADCGAIN
        let uv_trip_full =
            ((config.undervoltage_trip.get::<millivolt>() - adc_offset.get::<millivolt>()) * 1000.0
                / adc_gain.get::<uom::si::electric_potential::microvolt>()) as u16; // Use u16 for 14-bit value

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

    fn find_closest_scd_threshold_bits(target_voltage_mv: f32, rsns_enable: bool) -> u8 {
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
        let mut min_diff = f32::MAX;

        for (i, &threshold) in thresholds.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as f32).abs();
            if diff < min_diff {
                min_diff = diff;
                closest_bits = i as u8;
            }
        }
        closest_bits
    }

    fn find_closest_ocd_threshold_bits(target_voltage_mv: f32, rsns_enable: bool) -> u8 {
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
        let mut min_diff = f32::MAX;

        for (i, &threshold) in thresholds.iter().enumerate() {
            let diff = (target_voltage_mv - threshold as f32).abs();
            if diff < min_diff {
                min_diff = diff;
                closest_bits = i as u8;
            }
        }
        closest_bits
    }
}
