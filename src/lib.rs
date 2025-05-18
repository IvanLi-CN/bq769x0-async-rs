#![no_std]

#[cfg(not(feature = "async"))]
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

pub mod registers;

/// Represents potential errors when interacting with the BQ769x0 chip.
#[derive(Debug)]
pub enum Error<E> {
    /// An error occurred during I2C communication.
    I2c(E),
    // Add other specific error types as needed later, e.g.:
    // InvalidData,
    // UnsupportedFeature,
}
/// Represents the measured cell voltages.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CellVoltages {
    /// Voltage of cell 1 to 15.
    /// The number of valid cells depends on the chip model (BQ76920: 5, BQ76930: 10, BQ76940: 15).
    pub voltages: [u16; 15],
}

/// Represents the measured temperatures.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Temperatures {
    /// Temperature from TS1 sensor (or Die Temp if configured).
    pub ts1: i16,
    /// Temperature from TS2 sensor (BQ76930/40 only).
    pub ts2: Option<i16>,
    /// Temperature from TS3 sensor (BQ76940 only).
    pub ts3: Option<i16>,
}

/// Represents the measured pack current from the Coulomb Counter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Current {
    /// Raw Coulomb Counter value. Needs conversion based on CC_CFG and Rsense.
    pub raw_cc: i16,
}

/// Represents the hardware protection configuration.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ProtectionConfig {
    /// Short-Circuit Discharge (SCD) threshold.
    pub scd_threshold: u8,
    /// Short-Circuit Discharge (SCD) delay.
    pub scd_delay: u8,
    /// Overcurrent Discharge (OCD) threshold.
    pub ocd_threshold: u8,
    /// Overcurrent Discharge (OCD) delay.
    pub ocd_delay: u8,
    /// Overvoltage (OV) delay.
    pub ov_delay: u8,
    /// Undervoltage (UV) delay.
    pub uv_delay: u8,
}

/// BQ769x0 driver
pub struct Bq769x0<I2C> {
    address: u8,
    i2c: I2C,
}

impl<I2C, E> Bq769x0<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// Creates a new instance of the BQ769x0 driver.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C peripheral.
    /// * `address` - The I2C address of the BQ769x0 chip.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { address, i2c }
    }

    /// Reads a single byte from the specified register.
    ///
    /// # Arguments
    ///
    /// * `reg` - The register to read from.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg as u8], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Reads multiple bytes starting from the specified register.
    ///
    /// # Arguments
    ///
    /// * `reg` - The starting register to read from.
    /// * `len` - The number of bytes to read.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_registers(&mut self, reg: Register, len: usize) -> Result<Vec<u8>, Error<E>> {
        let mut buf = vec![0u8; len];
        self.i2c
            .write_read(self.address, &[reg as u8], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf)
    }

    /// Writes a single byte to the specified register.
    ///
    /// # Arguments
    ///
    /// * `reg` - The register to write to.
    /// * `value` - The byte value to write.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg as u8, value])
            .await
            .map_err(Error::I2c)
    }

    /// Writes multiple bytes starting from the specified register.
    ///
    /// # Arguments
    ///
    /// * `reg` - The starting register to write to.
    /// * `values` - The slice of bytes to write.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>> {
        let mut data = vec![reg as u8];
        data.extend_from_slice(values);
        self.i2c
            .write(self.address, &data)
            .await
            .map_err(Error::I2c)
    }
    /// Reads and converts cell voltages.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_cell_voltages(&mut self) -> Result<CellVoltages, Error<E>> {
        // Read all cell voltage registers (15 cells * 2 bytes/cell = 30 bytes)
        let raw_voltages = self.read_registers(Register::VC1_HI, 30).await?;

        // Read ADC Gain and Offset registers
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Combine ADCGAIN1 and ADCGAIN2 to get the 12-bit gain value
        // ADC_GAIN = (ADCGAIN2[7:4] << 8) | ADCGAIN1[7:0]
        let adc_gain: u16 = (((adc_gain2 & 0xF0) as u16) << 4) | (adc_gain1 as u16); // 12-bit gain
        let adc_offset: u8 = adc_offset; // 8-bit offset (assuming unsigned based on register map)

        let mut voltages = [0u16; 15];
        for i in 0..15 {
            let hi_byte = raw_voltages[i * 2];
            let lo_byte = raw_voltages[i * 2 + 1];

            // Combine bytes to get 14-bit raw ADC value
            // Raw ADC = (VCx_HI << 6) | (VCx_LO >> 2)
            let raw_adc: u16 = ((hi_byte as u16) << 6) | ((lo_byte as u16) >> 2);

            // Convert raw ADC to voltage (in mV)
            // VCELLn (V) = (ADC_CELLn * ADC_GAIN + ADC_OFFSET) * 1e-6
            // VCELLn (mV) = (ADC_CELLn * ADC_GAIN + ADC_OFFSET) * 1e-3
            // VCELLn (uV) = (ADC_CELLn * ADC_GAIN + ADC_OFFSET)
            // VCELLn (mV) = VCELLn (uV) / 1000
            let v_cell_uv: u32 = (raw_adc as u32 * adc_gain as u32) + adc_offset as u32;
            voltages[i] = (v_cell_uv / 1000) as u16; // Convert uV to mV and cast to u16
        }
        Ok(CellVoltages { voltages })
    }

    /// Reads and converts the total battery pack voltage.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_pack_voltage(&mut self) -> Result<u16, Error<E>> {
        // Read BAT_HI and BAT_LO registers (2 bytes)
        let raw_voltage = self.read_registers(Register::BAT_HI, 2).await?;

        // Read ADC Gain and Offset registers
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Combine ADCGAIN1 and ADCGAIN2 to get the 12-bit gain value
        let adc_gain: u16 = (((adc_gain2 & 0xF0) as u16) << 4) | (adc_gain1 as u16);
        let adc_offset: u8 = adc_offset;

        // Combine bytes to get 14-bit raw ADC value
        // Raw ADC = (BAT_HI << 6) | (BAT_LO >> 2)
        let raw_adc: u16 = ((raw_voltage[0] as u16) << 6) | ((raw_voltage[1] as u16) >> 2);

        // Convert raw ADC to voltage (in mV)
        // VPACK (V) = (ADC_PACK * ADC_GAIN + ADC_OFFSET) * 1e-6
        // VPACK (mV) = (ADC_PACK * ADC_GAIN + ADC_OFFSET) * 1e-3
        let v_pack_uv: u32 = (raw_adc as u32 * adc_gain as u32) + adc_offset as u32;
        let pack_voltage_mv = (v_pack_uv / 1000) as u16; // Convert uV to mV and cast to u16

        Ok(pack_voltage_mv)
    }

    /// Reads and converts temperatures from sensors.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_temperatures(&mut self) -> Result<Temperatures, Error<E>> {
        // Read TS1_HI and TS1_LO registers (2 bytes)
        let raw_ts1 = self.read_registers(Register::TS1_HI, 2).await?;

        // Read ADC Gain and Offset registers
        let adc_gain1 = self.read_register(Register::ADCGAIN1).await?;
        let adc_offset = self.read_register(Register::ADCOFFSET).await?;
        let adc_gain2 = self.read_register(Register::ADCGAIN2).await?;

        // Combine ADCGAIN1 and ADCGAIN2 to get the 12-bit gain value
        let adc_gain: u16 = (((adc_gain2 & 0xF0) as u16) << 4) | (adc_gain1 as u16);
        let adc_offset: u8 = adc_offset;

        // Convert raw TS1 ADC to temperature (in dC, deci-Celsius)
        // T_TS1 (°C) = (ADC_TS1 * ADC_GAIN + ADC_OFFSET) * 1e-6
        // T_TS1 (dC) = (ADC_TS1 * ADC_GAIN + ADC_OFFSET) * 1e-5
        let raw_adc_ts1: u16 = ((raw_ts1[0] as u16) << 6) | ((raw_ts1[1] as u16) >> 2);
        let t_ts1_udc: u32 = (raw_adc_ts1 as u32 * adc_gain as u32) + adc_offset as u32;
        let ts1_temp_dc = (t_ts1_udc / 100) as i16; // Convert uV to dC and cast to i16

        let mut ts2_temp_dc: Option<i16> = None;
        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // Read TS2_HI and TS2_LO registers (2 bytes) for BQ76930/40
            let raw_ts2 = self.read_registers(Register::TS2_HI, 2).await?;
            let raw_adc_ts2: u16 = ((raw_ts2[0] as u16) << 6) | ((raw_ts2[1] as u16) >> 2);
            let t_ts2_udc: u32 = (raw_adc_ts2 as u32 * adc_gain as u32) + adc_offset as u32;
            ts2_temp_dc = Some((t_ts2_udc / 100) as i16);
        }

        let mut ts3_temp_dc: Option<i16> = None;
        #[cfg(feature = "bq76940")]
        {
            // Read TS3_HI and TS3_LO registers (2 bytes) for BQ76940
            let raw_ts3 = self.read_registers(Register::TS3_HI, 2).await?;
            let raw_adc_ts3: u16 = ((raw_ts3[0] as u16) << 6) | ((raw_ts3[1] as u16) >> 2);
            let t_ts3_udc: u32 = (raw_adc_ts3 as u32 * adc_gain as u32) + adc_offset as u32;
            ts3_temp_dc = Some((t_ts3_udc / 100) as i16);
        }


        Ok(Temperatures {
            ts1: ts1_temp_dc,
            ts2: ts2_temp_dc,
            ts3: ts3_temp_dc,
        })
    }

    /// Reads the raw Coulomb Counter value.
    /// Conversion to current (A) requires CC_CFG and Rsense value.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_current(&mut self) -> Result<Current, Error<E>> {
        // Read CC_HI and CC_LO registers (2 bytes)
        let raw_cc_bytes = self.read_registers(Register::CC_HI, 2).await?;

        // Combine bytes to get a signed 16-bit raw CC value
        let raw_cc: i16 = i16::from_be_bytes([raw_cc_bytes[0], raw_cc_bytes[1]]);

        // TODO: Implement conversion from raw CC to current (A) based on CC_CFG and Rsense.
        // The exact formula depends on the CC_CFG register settings and the sense resistor value.
        // Refer to the datasheet "16-Bit CC FOR PACK CURRENT MEASUREMENT" section.

        Ok(Current { raw_cc })
    }

    /// Reads the System Status register.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn read_status(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::SYS_STAT).await
    }

    /// Clears the specified status flags in the System Status register.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn clear_status_flags(&mut self, flags: u8) -> Result<(), Error<E>> {
        // To clear a flag, write a '1' to the corresponding bit.
        self.write_register(Register::SYS_STAT, flags).await
    }

    /// Enables the charging FET.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn enable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SYS_CTRL2).await?;
        sys_ctrl2 |= SYS_CTRL2_CHG_ON;
        self.write_register(Register::SYS_CTRL2, sys_ctrl2).await
    }

    /// Disables the charging FET.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn disable_charging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SYS_CTRL2).await?;
        sys_ctrl2 &= !SYS_CTRL2_CHG_ON;
        self.write_register(Register::SYS_CTRL2, sys_ctrl2).await
    }

    /// Enables the discharging FET.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn enable_discharging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SYS_CTRL2).await?;
        sys_ctrl2 |= SYS_CTRL2_DSG_ON;
        self.write_register(Register::SYS_CTRL2, sys_ctrl2).await
    }

    /// Disables the discharging FET.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn disable_discharging(&mut self) -> Result<(), Error<E>> {
        let mut sys_ctrl2 = self.read_register(Register::SYS_CTRL2).await?;
        sys_ctrl2 &= !SYS_CTRL2_DSG_ON;
        self.write_register(Register::SYS_CTRL2, sys_ctrl2).await
    }

    /// Sets the cell balancing state.
    /// The `mask` is a bitmask where bit n corresponds to cell n+1.
    /// E.g., bit 0 for Cell 1, bit 4 for Cell 5, bit 9 for Cell 10, bit 14 for Cell 15.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn set_cell_balancing(&mut self, mask: u16) -> Result<(), Error<E>> {
        // Write to CELLBAL1 (Cells 1-5)
        let cellbal1_mask = (mask & 0x1F) as u8; // Bits 0-4
        self.write_register(Register::CELLBAL1, cellbal1_mask).await?;

        #[cfg(any(feature = "bq76930", feature = "bq76940"))]
        {
            // Write to CELLBAL2 (Cells 6-10) for BQ76930/40
            let cellbal2_mask = ((mask >> 5) & 0x1F) as u8; // Bits 5-9
            self.write_register(Register::CELLBAL2, cellbal2_mask).await?;
        }

        #[cfg(feature = "bq76940")]
        {
            // Write to CELLBAL3 (Cells 11-15) for BQ76940
            let cellbal3_mask = ((mask >> 10) & 0x1F) as u8; // Bits 10-14
            self.write_register(Register::CELLBAL3, cellbal3_mask).await?;
        }

        Ok(())
    }

    /// Enters the SHIP mode.
    /// WARNING: Refer to the datasheet for the exact sequence to enter SHIP mode.
    /// This is a placeholder based on common patterns.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn enter_ship_mode(&mut self) -> Result<(), Error<E>> {
        // Placeholder sequence:
        // 1. Write 0x00 to SYS_CTRL1
        // 2. Write 0x00 to SYS_CTRL2
        // 3. Write 0x00 to SYS_CTRL1
        // 4. Write 0x10 to SYS_CTRL2 (example value, check datasheet)
        // 5. Write 0x00 to SYS_CTRL1
        // 6. Write 0x10 to SYS_CTRL2 (example value, check datasheet)

        self.write_register(Register::SYS_CTRL1, 0x00).await?;
        self.write_register(Register::SYS_CTRL2, 0x00).await?;
        self.write_register(Register::SYS_CTRL1, 0x00).await?;
        // Replace 0x10 with the actual value from the datasheet for SHIP mode entry
        self.write_register(Register::SYS_CTRL2, 0x10).await?;
        self.write_register(Register::SYS_CTRL1, 0x00).await?;
        // Replace 0x10 with the actual value from the datasheet for SHIP mode entry
        self.write_register(Register::SYS_CTRL2, 0x10).await?;

        Ok(())
    }

    /// Checks if the ALERT pin is being overridden by external control.
    #[maybe_async_cfg::maybe(sync, async)]
    pub async fn is_alert_overridden(&mut self) -> Result<bool, Error<E>> {
        let sys_stat = self.read_status().await?;
        Ok((sys_stat & SYS_STAT_OVRD_ALERT) != 0)
    }
}
