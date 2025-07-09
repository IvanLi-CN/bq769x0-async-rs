#![no_std]
#![no_main]
const NUM_CELLS: usize = 5;
use defmt::{error, info};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    i2c::{self, Config, I2c},
    peripherals::{I2C1, UCPD1},
    time::Hertz,
    ucpd::{self, CcPull, Config as UcpdConfig, Ucpd},
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use bq769x0_async_rs::{
    BatteryConfig, Bq769x0, ProtectionConfig, RegisterAccess,
    crc::Enabled,
    registers::{Register, SysCtrl1Flags, SysCtrl2Flags, SysStatFlags},
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    UCPD1 => ucpd::InterruptHandler<UCPD1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello from STM32G431CB!");

    // --- USB-C Rd Pull-down Configuration ---
    // Based on embassy official UCPD implementation
    info!("Configuring USB-C Rd pull-down resistors...");

    // Initialize UCPD peripheral for USB-C CC pin management
    // For STM32G431CB: CC1=PB6, CC2=PB4 (adjust according to your hardware)
    let ucpd_config = UcpdConfig::default();
    let mut ucpd = Ucpd::new(
        p.UCPD1,
        Irqs,
        p.PB6,  // CC1 pin
        p.PB4,  // CC2 pin
        ucpd_config,
    );

    // Configure as sink (UFP) with Rd pull-down resistors (5.1kΩ)
    // This automatically disables dead battery pull-downs and enables proper Rd
    ucpd.cc_phy().set_pull(CcPull::Sink);

    info!("USB-C Rd pull-down resistors configured successfully.");
    info!("Device configured as UFP (Upstream Facing Port) with 5.1kΩ Rd resistors.");

    let scl = p.PA15;
    let sda = p.PB7;

    let mut i2c_config = Config::default();
    // Enable internal pull-ups for I2C lines
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;

    let i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        Hertz(100_000),
        i2c_config,
    );

    // Sense resistor value in milliOhms (adjust according to your hardware)
    let sense_resistor_m_ohm: u32 = 3; // Your sense resistor value in milliOhms

    let mut bq = Bq769x0::<_, Enabled, NUM_CELLS>::new(i2c, 0x08, sense_resistor_m_ohm, None);

    let battery_config = BatteryConfig {
        overvoltage_trip: 3600u32,  // Set to 3.6V
        undervoltage_trip: 2500u32, // Set to 2.5V
        protection_config: ProtectionConfig {
            ocd_limit: 10_000i32,                         // Set to 10A (10_000 mA)
            ..BatteryConfig::default().protection_config  // Inherit other protection_config fields
        },
        rsense: sense_resistor_m_ohm, // Use mOhms directly as per BatteryConfig field
        ..Default::default()          // Inherit other BatteryConfig fields
    };

    // --- Apply Battery Configuration ---
    info!("Applying battery configuration (including ADC_EN)...");
    if let Err(e) = bq.set_config(&battery_config).await {
        error!("Failed to apply battery configuration: {:?}", e);
        core::panic!("Failed to apply battery configuration: {:?}", e);
    } else {
        info!("Battery configuration applied successfully.");
        info!(
            "  ADC_EN: {}",
            battery_config
                .sys_ctrl1_flags
                .contains(SysCtrl1Flags::ADC_EN)
        );
        info!(
            "  CC_EN: {}",
            battery_config
                .sys_ctrl2_flags
                .contains(SysCtrl2Flags::CC_EN)
        );
    }

    info!("BQ76920 initialization complete.");

    // --- Enable Charge and Discharge MOSFETs ---
    info!("Enabling charge MOSFET...");
    if let Err(e) = bq.enable_charging().await {
        error!("Failed to enable charging: {:?}", e);
    } else {
        info!("Charge MOSFET enabled successfully.");
    }

    info!("Enabling discharge MOSFET...");
    if let Err(e) = bq.enable_discharging().await {
        error!("Failed to enable discharging: {:?}", e);
    } else {
        info!("Discharge MOSFET enabled successfully.");
    }

    // --- Enable Cell Balancing (Example: enable for cells 1 to 5) ---
    info!("Enabling cell balancing...");
    let balancing_mask: u16 = 0b00011111; // Enable balancing for cells 1 to 5
    if let Err(e) = bq.set_cell_balancing(balancing_mask).await {
        error!("Failed to set cell balancing: {:?}", e);
    } else {
        info!("Cell balancing enabled: mask = 0b{:05b}", balancing_mask);
    }

    // // --- Enter Ship Mode ---
    // info!("Entering ship mode...");
    // if let Err(e) = bq.enter_ship_mode().await {
    //     error!("Failed to enter ship mode: {:?}", e);
    // } else {
    //     info!("Ship mode entered successfully.");
    // }

    // --- Main Loop for Data Acquisition ---
    loop {
        info!("--- Reading BQ76920 Data ---");

        // Check system control register status (for debugging)
        info!("Checking system control register status...");
        let sys_ctrl1_val = bq.read_register(Register::SysCtrl1).await.unwrap_or(0);
        let sys_ctrl1_flags = SysCtrl1Flags::from_bits_truncate(sys_ctrl1_val);
        let sys_ctrl2_val = bq.read_register(Register::SysCtrl2).await.unwrap_or(0);
        let sys_ctrl2_flags = SysCtrl2Flags::from_bits_truncate(sys_ctrl2_val);

        info!(
            "SYS_CTRL1 = 0x{:02X}, ADC_EN = {}",
            sys_ctrl1_val,
            sys_ctrl1_flags.contains(SysCtrl1Flags::ADC_EN)
        );
        info!(
            "SYS_CTRL2 = 0x{:02X}, CC_EN = {}",
            sys_ctrl2_val,
            sys_ctrl2_flags.contains(SysCtrl2Flags::CC_EN)
        );

        // Read ADC calibration values
        let (adc_gain_uv_per_lsb, adc_offset_mv) = match bq.read_adc_calibration().await {
            Ok(cal) => cal,
            Err(e) => {
                error!("Failed to read ADC calibration: {:?}", e);
                // Use default calibration values if reading fails
                (365, 0) // Default values from datasheet
            }
        };
        info!(
            "ADC Calibration: Gain={} uV/LSB, Offset={} mV",
            adc_gain_uv_per_lsb, adc_offset_mv
        );

        // Debug UV_TRIP register and calculation
        let uv_trip_register = bq.read_register(Register::UvTrip).await.unwrap();
        info!("UV_TRIP register value: 0x{:02X}", uv_trip_register);

        // Calculate what voltage this UV_TRIP register represents
        // UV threshold format: 01-XXXXXXXX-0000 (14-bit)
        let uv_trip_14bit = (0b01 << 12) | ((uv_trip_register as u16) << 4) | 0b0000;
        let uv_trip_voltage_mv =
            (uv_trip_14bit as i32 * adc_gain_uv_per_lsb as i32) / 1000 + adc_offset_mv as i32;
        info!(
            "Calculated UV trip voltage: {} mV (from 14-bit value: 0x{:04X})",
            uv_trip_voltage_mv, uv_trip_14bit
        );

        // Also check OV_TRIP for comparison
        let ov_trip_register = bq.read_register(Register::OvTrip).await.unwrap();
        info!("OV_TRIP register value: 0x{:02X}", ov_trip_register);
        let ov_trip_14bit = (0b10 << 12) | ((ov_trip_register as u16) << 4) | 0b1000;
        let ov_trip_voltage_mv =
            (ov_trip_14bit as i32 * adc_gain_uv_per_lsb as i32) / 1000 + adc_offset_mv as i32;
        info!(
            "Calculated OV trip voltage: {} mV (from 14-bit value: 0x{:04X})",
            ov_trip_voltage_mv, ov_trip_14bit
        );

        // Read Cell Voltages
        match bq.read_cell_voltages().await {
            Ok(cell_voltages) => {
                info!("Cell Voltages:");
                // BQ76920 supports up to 5 cells
                for i in 0..NUM_CELLS {
                    let voltage_mv = cell_voltages.voltages[i];
                    // Cell voltages are already converted to mV by the library
                    info!("  Cell {}: {} mV", i + 1, voltage_mv);
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
            Ok(temps) => {
                info!("Temperatures (0.01°C):");
                info!("  TS1: {} ({}°C)", temps.ts1, temps.ts1 as f32 / 100.0);
                if let Some(ts2) = temps.ts2 {
                    info!("  TS2: {} ({}°C)", ts2, ts2 as f32 / 100.0);
                }
                if let Some(ts3) = temps.ts3 {
                    info!("  TS3: {} ({}°C)", ts3, ts3 as f32 / 100.0);
                }
            }
            Err(e) => {
                error!("Failed to read temperatures: {:?}", e);
            }
        }

        // Read Current
        match bq.read_current().await {
            Ok(current_ma) => {
                info!("Current: {} mA", current_ma);
            }
            Err(e) => {
                error!("Failed to read current: {:?}", e);
            }
        }

        // Read System Status
        match bq.read_status().await {
            Ok(status) => {
                info!(
                    "System Status (SYS_STAT register: 0x{:02X}):",
                    status.0.bits()
                );
                info!("  CC Ready: {}", status.0.contains(SysStatFlags::CC_READY));
                info!(
                    "  Overtemperature: {}",
                    status.0.contains(SysStatFlags::OVRD_ALERT)
                );

                let uv_fault = status.0.contains(SysStatFlags::UV);
                info!(
                    "  Undervoltage (UV): {} - Expected threshold: {} mV",
                    uv_fault, uv_trip_voltage_mv
                );
                info!(
                    "  Overvoltage (OV): {} - Expected threshold: {} mV",
                    status.0.contains(SysStatFlags::OV),
                    ov_trip_voltage_mv
                );
                info!(
                    "  Short Circuit Discharge (SCD): {}",
                    status.0.contains(SysStatFlags::SCD)
                );
                info!(
                    "  Overcurrent Discharge (OCD): {}",
                    status.0.contains(SysStatFlags::OCD)
                );

                // UV fault management: Enable discharge MOS when UV fault is false (cleared)
                if !uv_fault {
                    // UV fault is not present, ensure discharge MOS is enabled
                    match bq.read_mos_status().await {
                        Ok(mos_status) => {
                            if !mos_status.0.contains(SysCtrl2Flags::DSG_ON) {
                                info!("UV fault cleared, enabling discharge MOSFET...");
                                if let Err(e) = bq.enable_discharging().await {
                                    error!("Failed to enable discharging after UV clear: {:?}", e);
                                } else {
                                    info!("Discharge MOSFET enabled after UV fault cleared.");
                                }
                            }
                        }
                        Err(e) => {
                            error!("Failed to read MOS status for UV management: {:?}", e);
                        }
                    }
                } else {
                    // UV fault is present, discharge MOS should be disabled by hardware
                    info!(
                        "UV fault detected - discharge MOSFET should be disabled by hardware protection"
                    );
                }

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

        // Read MOS Status
        match bq.read_mos_status().await {
            Ok(mos_status) => {
                info!("MOS Status:");
                info!(
                    "  Charge MOSFET (CHG_ON): {}",
                    mos_status.0.contains(SysCtrl2Flags::CHG_ON)
                );
                info!(
                    "  Discharge MOSFET (DSG_ON): {}",
                    mos_status.0.contains(SysCtrl2Flags::DSG_ON)
                );
                info!(
                    "  Coulomb Counter (CC_EN): {}",
                    mos_status.0.contains(SysCtrl2Flags::CC_EN)
                );
                info!(
                    "  CC One-Shot (CC_ONESHOT): {}",
                    mos_status.0.contains(SysCtrl2Flags::CC_ONESHOT)
                );
                info!(
                    "  Delay Disable (DELAY_DIS): {}",
                    mos_status.0.contains(SysCtrl2Flags::DELAY_DIS)
                );
            }
            Err(e) => {
                error!("Failed to read MOS status: {:?}", e);
            }
        }

        info!("----------------------------");

        // Wait for 1 second
        Timer::after(Duration::from_secs(1)).await;
    }
}
