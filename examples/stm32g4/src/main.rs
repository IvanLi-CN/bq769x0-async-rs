#![no_std]
#![no_main]
const NUM_CELLS: usize = 5;
use defmt::{error, info};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    i2c::{self, Config, I2c},
    peripherals::I2C1,
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use bq769x0_async_rs::{
    BatteryConfig, Bq769x0, RegisterAccess,
    registers::{
        Register, SYS_CTRL2_CC_EN, SYS_STAT_CC_READY, SYS_STAT_OCD, SYS_STAT_OV,
        SYS_STAT_OVRD_ALERT, SYS_STAT_SCD, SYS_STAT_UV,
    },
    units::ElectricalResistance,
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

type I2cBus<'d> = Mutex<NoopRawMutex, I2c<'d, I2C1>>;
type MyI2cDevice<'d> = I2cDevice<'d, NoopRawMutex, I2c<'d, I2C1>>;

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

    let mut bq = Bq769x0::new(i2c, 0x18);

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
    let sense_resistor = ElectricalResistance::new::<uom::si::electrical_resistance::milliohm>(3.0); // Your sense resistor value in milliOhms

    loop {
        info!("--- Reading BQ76920 Data ---");

        // Ensure CC_EN is enabled in SYS_CTRL2
        info!("Ensuring CC_EN is enabled in SYS_CTRL2...");
        let sys_ctrl2_val = bq.read_register(Register::SysCtrl2).await.unwrap_or(0);
        if let Err(e) = bq
            .write_register(Register::SysCtrl2, sys_ctrl2_val | SYS_CTRL2_CC_EN)
            .await
        {
            error!("Failed to enable CC_EN: {:?}", e);
        }
        info!("CC_EN enable attempt complete.");

        // Read Cell Voltages
        match bq.read_cell_voltages::<NUM_CELLS>().await {
            Ok(voltages) => {
                info!("Cell Voltages (mV):");
                // BQ76920 supports up to 5 cells
                for _i in 0..5 {
                    // Get voltage in millivolts as i32 for printing
                    info!(
                        "  Cell {}: {} mV",
                        _i + 1,
                        voltages.voltages[_i].get::<uom::si::electric_potential::millivolt>()
                    );
                }
            }
            Err(e) => {
                error!("Failed to read cell voltages: {:?}", e);
            }
        }

        // Read Pack Voltage
        match bq.read_pack_voltage().await {
            Ok(voltage) => {
                info!(
                    "Pack Voltage: {} mV",
                    voltage.get::<uom::si::electric_potential::millivolt>()
                );
            }
            Err(e) => {
                error!("Failed to read pack voltage: {:?}", e);
            }
        }

        // Read Temperatures
        match bq.read_temperatures().await {
            Ok(temps) => {
                if temps.is_thermistor {
                    info!("Temperatures (0.1 Ohms):");
                    info!(
                        "  TS1: {} ({} Ohms)",
                        temps
                            .ts1
                            .get::<uom::si::thermodynamic_temperature::kelvin>(),
                        temps
                            .ts1
                            .get::<uom::si::thermodynamic_temperature::kelvin>()
                            / 10.0
                    );
                    // BQ76920 only has TS1
                } else {
                    info!("Temperatures (deci-Celsius):");
                    let ts1_kelvin_integer = temps
                        .ts1
                        .get::<uom::si::thermodynamic_temperature::kelvin>();
                    let ts1_celsius_f32 = ts1_kelvin_integer - 273.15; // Manually convert kelvin to celsius float

                    info!(
                        "  TS1 (Die Temp): kelvin_value={}, celsius_manual_f32={}",
                        ts1_kelvin_integer, ts1_celsius_f32
                    );
                }
            }
            Err(e) => {
                error!("Failed to read temperatures: {:?}", e);
            }
        }

        // Read Current
        match bq.read_current().await {
            Ok(current) => {
                let current_ma = bq.convert_raw_cc_to_current_ma(current.raw_cc, sense_resistor);
                info!(
                    "Raw CC: {}, Current: {} mA",
                    current.raw_cc,
                    current_ma.get::<uom::si::electric_current::milliampere>()
                );
            }
            Err(e) => {
                error!("Failed to read current: {:?}", e);
            }
        }

        // Read System Status
        match bq.read_status().await {
            Ok(status) => {
                info!("System Status:");
                info!("  CC Ready: {}", status.cc_ready);
                info!("  Overtemperature: {}", status.ovr_temp);
                info!("  Undervoltage (UV): {}", status.uv);
                info!("  Overvoltage (OV): {}", status.ov);
                info!("  Short Circuit Discharge (SCD): {}", status.scd);
                info!("  Overcurrent Discharge (OCD): {}", status.ocd);
                info!("  Cell Undervoltage (CUV): {}", status.cuv);
                info!("  Cell Overvoltage (COV): {}", status.cov);

                // Clear status flags after reading
                // Only clear flags that are set
                let flags_to_clear = (status.cc_ready as u8 * SYS_STAT_CC_READY)
                    | (status.ovr_temp as u8 * SYS_STAT_OVRD_ALERT)
                    | (status.uv as u8 * SYS_STAT_UV)
                    | (status.ov as u8 * SYS_STAT_OV)
                    | (status.scd as u8 * SYS_STAT_SCD)
                    | (status.ocd as u8 * SYS_STAT_OCD)
                    | (status.cuv as u8 * SYS_STAT_UV)
                    | (status.cov as u8 * SYS_STAT_OV);

                if flags_to_clear != 0 {
                    if let Err(e) = bq.clear_status_flags(flags_to_clear).await {
                        error!("Failed to clear status flags: {:?}", e);
                    } else {
                        info!("Cleared status flags: {:#010b}", flags_to_clear);
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
