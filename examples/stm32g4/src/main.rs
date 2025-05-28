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
    units::ElectricalResistance,
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
    let sense_resistor = ElectricalResistance::new::<uom::si::electrical_resistance::milliohm>(3.0); // Your sense resistor value in milliOhms

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

        // Read Cell Voltages
        match bq.read_cell_voltages().await {
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
                            .get::<uom::si::electric_potential::millivolt>(), // Assuming raw voltage is read
                        temps
                            .ts1
                            .get::<uom::si::electric_potential::millivolt>() // Assuming raw voltage is read
                            / 10.0 // Assuming conversion factor
                    );
                    // BQ76920 only has TS1
                } else {
                    info!("Temperatures (deci-Celsius):");
                    let ts1_millivolt = temps
                        .ts1
                        .get::<uom::si::electric_potential::millivolt>();
                    let ts1_deci_celsius = ts1_millivolt; // Assuming raw millivolt directly represents deci-celsius
                    let ts1_celsius_f32 = ts1_deci_celsius / 10.0; // Convert deci-celsius to celsius float

                    info!(
                        "  TS1 (Die Temp): kelvin_value={}, celsius_manual_f32={}",
                        ts1_deci_celsius, ts1_celsius_f32
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
