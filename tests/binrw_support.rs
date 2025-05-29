#[cfg(feature = "binrw")]
mod tests {
    // Corrected imports for integration tests
    use binrw::{BinRead, BinWrite, io::Cursor};
    use bq769x0_async_rs::data_types::{
        BatteryConfig, Bq76920Measurements, CellVoltages, CoulombCounter, MosStatus, NtcParameters,
        OcdDelay, ProtectionConfig, ScdDelay, SystemStatus, TempSensor, TemperatureData,
        TemperatureSensorReadings, UvOvDelay,
    };
    use bq769x0_async_rs::registers::{Register, SysCtrl2Flags, SysStatFlags};

    #[test]
    fn test_cell_voltages_binrw() {
        let original = CellVoltages {
            voltages: [
                7853, // Approx raw ADC for 3000mV (3000000uV / 382uV/LSB)
                8116, // Approx raw ADC for 3100mV
                8378, // Approx raw ADC for 3200mV
                8641, // Approx raw ADC for 3300mV
                8903, // Approx raw ADC for 3400mV
            ],
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = CellVoltages::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_temperature_sensor_readings_binrw() {
        let original = TemperatureSensorReadings {
            ts1: 6545,       // Approx raw ADC for 2500mV
            ts2: Some(6808), // Approx raw ADC for 2600mV
            ts3: None,
            is_thermistor: true,
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = TemperatureSensorReadings::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_ntc_parameters_binrw() {
        let original = NtcParameters {
            b_value: 3435.0,
            ref_temp_k: 2500,       // 25.00 °C in c°C
            ref_resistance_ohm: 10, // 10 mΩ
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = NtcParameters::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_temperature_data_binrw() {
        let original = TemperatureData {
            ts1: 2550,       // 25.50 °C in c°C
            ts2: Some(2600), // 26.00 °C in c°C
            ts3: None,
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = TemperatureData::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_coulomb_counter_binrw() {
        let original = CoulombCounter { raw_cc: 12345 };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = CoulombCounter::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_temp_sensor_enum_binrw() {
        let original = TempSensor::Internal;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = TempSensor::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);

        let original = TempSensor::External;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = TempSensor::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_system_status_binrw() {
        let original =
            SystemStatus(SysStatFlags::CC_READY | SysStatFlags::OVRD_ALERT | SysStatFlags::SCD); // Example status byte

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = SystemStatus::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_scd_delay_enum_binrw() {
        let original = ScdDelay::Delay100us;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = ScdDelay::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_ocd_delay_enum_binrw() {
        let original = OcdDelay::Delay320ms;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = OcdDelay::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_uv_ov_delay_enum_binrw() {
        let original = UvOvDelay::Delay4s;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = UvOvDelay::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_protection_config_binrw() {
        let original = ProtectionConfig {
            rsns_enable: true,
            scd_delay: ScdDelay::Delay70us,
            scd_limit: 50000, // 50000 mA
            ocd_delay: OcdDelay::Delay10ms,
            ocd_limit: 15000, // 15000 mA
            uv_delay: UvOvDelay::Delay1s,
            ov_delay: UvOvDelay::Delay1s,
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = ProtectionConfig::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_battery_config_binrw() {
        let original = BatteryConfig::default(); // Use default for simplicity

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = BatteryConfig::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_mos_status_binrw() {
        let original = MosStatus(SysCtrl2Flags::CHG_ON | SysCtrl2Flags::DSG_ON); // Charge and Discharge on

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = MosStatus::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_bq76920_measurements_binrw() {
        // Create a sample Bq76920Measurements with representative raw values
        let original = Bq76920Measurements::<5> {
            cell_voltages: CellVoltages {
                voltages: [8000, 8100, 8200, 8300, 8400],
            }, // Example raw ADC values
            temperatures: TemperatureSensorReadings {
                ts1: 7000,
                ts2: Some(7100),
                ts3: None,
                is_thermistor: false,
            }, // Example raw ADC values
            current: 1000,                          // Example current in mA
            system_status: SystemStatus::new(0x80), // Example status byte
            mos_status: MosStatus::new(0x03),       // Example MOS status byte
        };

        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = Bq76920Measurements::<5>::read_le(&mut buffer).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_register_enum_binrw() {
        let original = Register::SysStat;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = Register::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);

        let original = Register::Vc1Hi;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = Register::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);

        let original = Register::ADCGAIN2;
        let mut buffer = Cursor::new(Vec::new());
        original.write_le(&mut buffer).unwrap();
        buffer.set_position(0);
        let deserialized = Register::read_le(&mut buffer).unwrap();
        assert_eq!(original, deserialized);
    }
}
