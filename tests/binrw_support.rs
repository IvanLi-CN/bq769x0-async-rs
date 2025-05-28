#[cfg(feature = "binrw")]
mod tests {
    // Corrected imports for integration tests
    use bq769x0_async_rs::data_types::{
        BatteryConfig, Bq76920Measurements, CellVoltages, CoulombCounter, MosStatus, NtcParameters,
        OcdDelay, ProtectionConfig, ScdDelay, SystemStatus, TempSensor, TemperatureData,
        TemperatureSensorReadings, UvOvDelay,
    };
    use bq769x0_async_rs::registers::{Register, SysCtrl2Flags, SysStatFlags};
    use bq769x0_async_rs::units::{
        ElectricCurrent, ElectricPotential, ElectricalResistance, TemperatureInterval,
    };

    use binrw::{io::Cursor, BinRead, BinWrite};
    use uom::si::{
        electric_current::milliampere, electric_potential::millivolt, electrical_resistance::ohm,
        temperature_interval::degree_celsius, temperature_interval::kelvin,
    }; // Removed Endian

    #[test]
    fn test_cell_voltages_binrw() {
        let original = CellVoltages {
            voltages: [
                ElectricPotential::new::<millivolt>(3000.0),
                ElectricPotential::new::<millivolt>(3100.0),
                ElectricPotential::new::<millivolt>(3200.0),
                ElectricPotential::new::<millivolt>(3300.0),
                ElectricPotential::new::<millivolt>(3400.0),
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
            ts1: ElectricPotential::new::<millivolt>(2500.0),
            ts2: Some(ElectricPotential::new::<millivolt>(2600.0)),
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
            ref_temp_k: TemperatureInterval::new::<kelvin>(298.15),
            ref_resistance_ohm: ElectricalResistance::new::<ohm>(10000.0),
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
            ts1: TemperatureInterval::new::<degree_celsius>(25.5),
            ts2: Some(TemperatureInterval::new::<degree_celsius>(26.0)),
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
            scd_limit: ElectricCurrent::new::<milliampere>(50000.0),
            ocd_delay: OcdDelay::Delay10ms,
            ocd_limit: ElectricCurrent::new::<milliampere>(15000.0),
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
        let original = Bq76920Measurements::<5>::default(); // Use default for simplicity

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
