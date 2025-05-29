#![no_std]
#![no_main]
const NUM_CELLS: usize = 5;
use defmt::{error, info};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    i2c::{self, Config, I2c},
    peripherals::I2C1,
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use bq769x0_async_rs::{
    BatteryConfig, Bq769x0, RegisterAccess,
    registers::{
        Register, SysCtrl2Flags, SysStatFlags,
    },
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello from STM32G431CB!");

    let scl = p.PB8;
    let sda = p.PB9;

    let i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        Hertz(100_000),
        Config::default(),
    );

    let mut bq = Bq769x0::<_, _, NUM_CELLS>::new(i2c, 0x18);

    let battery_config = BatteryConfig::default();

    info!("Applying battery configuration...");
    if let Err(e) = bq.set_config(&battery_config).await {
        error!("Failed to apply battery configuration: {:?}", e);
        core::panic!("Failed to apply battery configuration: {:?}", e);
    }
    info!("Battery configuration applied successfully.");

    // Set CC_CFG register to 0x19 for optimal performance
    info!("Setting CC_CFG register to 0x19...");
    if let Err(e) = bq.write_register(Register::CcCfg, 0x19).await {
        error!("Failed to set CC_CFG: {:?}", e);
        core::panic!("Failed to set CC_CFG: {:?}", e);
    }
    info!("CC_CFG set successfully.");

    // 4. Clear initial fault flags
    // Write 0xFF to SYS_STAT to clear all flags
    info!("Clearing initial status flags (writing 0xFF to SYS_STAT)...");
    if let Err(e) = bq.clear_status_flags(0xFF).await {
        error!("Failed to clear status flags: {:?}", e);
        core::panic!("Failed to clear status flags: {:?}", e);
    }
    info!("Initial status flags cleared successfully.");

    info!("BQ76920 initialization complete.");

    // --- Main Loop for Data Acquisition ---
    let sense_resistor_m_ohm: u32 = 3; // Your sense resistor value in milliOhms

    loop {
        info!("--- Reading BQ76920 Data ---");

        // Ensure CC_EN is enabled in SYS_CTRL2
        info!("Ensuring CC_EN is enabled in SYS_CTRL2...");
        let sys_ctrl2_val = bq.read_register(Register::SysCtrl2).await.unwrap_or(0);
        if let Err(e) = bq
            .write_register(Register::SysCtrl2, (SysCtrl2Flags::from_bits_truncate(sys_ctrl2_val) | SysCtrl2Flags::CC_EN).bits())
            .await
        {
            error!("Failed to enable CC_EN: {:?}", e);
        }
        info!("CC_EN enable attempt complete.");

        // Read ADC calibration values
        let (adc_gain_uv_per_lsb, adc_offset_mv) = match bq.read_adc_calibration().await {
            Ok(cal) => cal,
            Err(e) => {
                error!("Failed to read ADC calibration: {:?}", e);
                // Use default calibration values if reading fails
                (365, 0) // Default values from datasheet
            }
        };
        info!("ADC Calibration: Gain={} uV/LSB, Offset={} mV", adc_gain_uv_per_lsb, adc_offset_mv);

        // Read Cell Voltages
        match bq.read_cell_voltages().await {
            Ok(cell_voltages) => {
                info!("Cell Voltages:");
                // BQ76920 supports up to 5 cells
                for i in 0..NUM_CELLS {
                    let raw_voltage = cell_voltages.voltages[i];
                    // Convert raw ADC to mV using calibration values
                    // Datasheet formula: V(cell) = GAIN * ADC(cell) + OFFSET
                    // V(cell) in mV = (GAIN_uV_per_LSB * raw_adc / 1000) + OFFSET_mV
                    let voltage_mv = (adc_gain_uv_per_lsb as i32 * raw_voltage as i32 / 1000) + adc_offset_mv as i32;
                    info!("  Cell {}: {} mV (raw {})", i + 1, voltage_mv, raw_voltage);
                }
            }
            Err(e) => {
                error!("Failed to read cell voltages: {:?}", e);
            }
        }

        // Read Pack Voltage
        match bq.read_pack_voltage().await {
            Ok(pack_voltage_mv) => {
                info!("Pack Voltage: {} mV", pack_voltage_mv);
            }
            Err(e) => {
                error!("Failed to read pack voltage: {:?}", e);
            }
        }

        // Read Temperatures
        match bq.read_temperatures().await {
            Ok(raw_temps) => {
                info!("Temperature Sensor Readings (raw ADC):");
                info!("  TS1: {} (is_thermistor: {})", raw_temps.ts1, raw_temps.is_thermistor);
                if let Some(ts2) = raw_temps.ts2 {
                    info!("  TS2: {}", ts2);
                }
                if let Some(ts3) = raw_temps.ts3 {
                    info!("  TS3: {}", ts3);
                }

                // Convert raw temperature readings to temperature data
                // Note: NTC parameters are needed for thermistor conversion,
                // but for Die Temp, conversion is done internally in data_types.rs
                match raw_temps.into_temperature_data(None) { // Pass None for NTC params for now
                    Ok(temps) => {
                        info!("Temperatures (c°C):");
                        info!("  TS1: {}", temps.ts1);
                        if let Some(ts2) = temps.ts2 {
                            info!("  TS2: {}", ts2);
                        }
                        if let Some(ts3) = temps.ts3 {
                            info!("  TS3: {}", ts3);
                        }
                    }
                    Err(e) => {
                        error!("Failed to convert raw temperature data: {}", e);
                    }
                }
            }
            Err(e) => {
                error!("Failed to read temperatures: {:?}", e);
            }
        }

        // Read Current
        match bq.read_current().await {
            Ok(coulomb_counter) => {
                let current_ma = bq.convert_raw_cc_to_current_ma(coulomb_counter.raw_cc, sense_resistor_m_ohm);
                info!("Raw CC: {}, Current: {} mA", coulomb_counter.raw_cc, current_ma);
            }
            Err(e) => {
                error!("Failed to read current: {:?}", e);
            }
        }

        // Read System Status
        match bq.read_status().await {
            Ok(status) => {
                info!("System Status:");
                info!("  CC Ready: {}", status.0.contains(SysStatFlags::CC_READY));
                info!("  Overtemperature: {}", status.0.contains(SysStatFlags::OVRD_ALERT));
                info!("  Undervoltage (UV): {}", status.0.contains(SysStatFlags::UV));
                info!("  Overvoltage (OV): {}", status.0.contains(SysStatFlags::OV));
                info!("  Short Circuit Discharge (SCD): {}", status.0.contains(SysStatFlags::SCD));
                info!("  Overcurrent Discharge (OCD): {}", status.0.contains(SysStatFlags::OCD));
                info!("  Cell Undervoltage (UV): {}", status.0.contains(SysStatFlags::UV));
                info!("  Cell Overvoltage (OV): {}", status.0.contains(SysStatFlags::OV));

                // Clear status flags after reading
                // Only clear flags that are set
                let flags_to_clear = status.0;

                if !flags_to_clear.is_empty() {
                    if let Err(e) = bq.clear_status_flags(flags_to_clear.bits()).await {
                        error!("Failed to clear status flags: {:?}", e);
                    } else {
                        info!("Cleared status flags: {:#010b}", flags_to_clear.bits());
                    }
                }
            }
            Err(e) => {
                error!("Failed to read system status: {:?}", e);
            }
        }

        info!("----------------------------");

        // Wait for 1 second
        Timer::after(Duration::from_secs(1)).await;
    }
}
