# 温度计算功能实现方案 (查表法)

## 目标

在 `TemperatureData` 结构体中存储转换后的温度值。`TemperatureSensorReadings` 结构体存储原始电压读数，并提供方法将电压转换为物理量（内部芯片温度或外部热敏电阻阻值）。热敏电阻阻值到温度的转换使用查表法。

## 现有结构体

```rust
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TemperatureSensorReadings {
    /// Voltage from TS1 sensor (if external thermistor) or Die Temperature (if internal).
    pub ts1: ElectricPotential,
    /// Voltage from TS2 sensor (BQ76930/40 only) or Die Temperature.
    pub ts2: Option<ElectricPotential>,
    /// Voltage from TS3 sensor (BQ76940 only) or Die Temperature.
    pub ts3: Option<ElectricPotential>,
    /// Indicates if the temperature readings are Die Temp (false) or Thermistor resistance (true).
    pub is_thermistor: bool,
}
```

## 数据手册参考 (docs/bq76920.md)

### 内部芯片温度转换

当 `TEMP_SEL = 0` 时，TSx 寄存器存储的是与芯片温度相关的电压读数。转换公式为：

`TEMP_DIE = 25.0 - (V_TSX_volts - 1.2) / 0.0042`

其中 `V_TSX_volts` 是 TSx 引脚的电压值（单位：伏特）。

### 外部热敏电阻电阻值计算

当 `TEMP_SEL = 1` 时，TSx 寄存器存储的是与外部热敏电阻相关的电压读数。使用 10 kΩ NTC 103AT 热敏电阻和内部 10 kΩ 上拉电阻。电阻值计算公式为：

`R_TS = (10000.0 * V_TSX_volts) / (3.3 - V_TSX_volts)`

其中 `V_TSX_volts` 是 TSx 引脚的电压值（单位：伏特），10000.0 是内部上拉电阻的阻值（单位：欧姆），3.3 是内部调节器的标称电压（单位：伏特）。

## 方案设计 (查表法)

1.  `TemperatureSensorReadings` 结构体保持不变。
2.  定义一个新的结构体 `TemperatureData`，用于存储转换后的温度值。

    ```rust
    use uom::si::temperature::ThermodynamicTemperature;

    #[derive(Debug, Copy, Clone, PartialEq)]
    pub struct TemperatureData {
        pub ts1: ThermodynamicTemperature,
        pub ts2: Option<ThermodynamicTemperature>,
        pub ts3: Option<ThermodynamicTemperature>,
    }
    ```

3.  定义一个函数 `convert_resistance_to_temperature`，该函数接收热敏电阻阻值和电阻-温度对照表作为输入，并返回温度。对照表的数据结构需要定义。

    ```rust
    use uom::si::electrical_resistance::ElectricalResistance;
    use uom::si::temperature::ThermodynamicTemperature;

    // Example: Define a simple lookup table as a slice of tuples (resistance_ohm, temperature_celsius)
    // The actual table data needs to be provided by the user or configuration.
    pub type ResistanceTemperatureTable = &'static [(f32, f32)];

    /// Converts thermistor resistance to temperature using a lookup table.
    /// Assumes the table is sorted by resistance in descending order.
    pub fn convert_resistance_to_temperature(
        resistance: ElectricalResistance,
        lookup_table: ResistanceTemperatureTable,
    ) -> Result<ThermodynamicTemperature, &'static str> {
        let r_ohm = resistance.get::<uom::si::electrical_resistance::ohm>();

        if lookup_table.is_empty() {
            return Err("Lookup table is empty");
        }

        // Find the two points in the table that bracket the given resistance
        let mut iter = lookup_table.iter();
        let mut prev_point = iter.next().unwrap(); // Table is not empty

        if r_ohm >= prev_point.0 {
            // Resistance is greater than or equal to the highest resistance in the table
            Ok(ThermodynamicTemperature::new::<uom::si::temperature::degree_celsius>(prev_point.1))
        } else {
            for current_point in iter {
                if r_ohm >= current_point.0 {
                    // The resistance is between prev_point and current_point, perform linear interpolation
                    let r1 = prev_point.0;
                    let t1 = prev_point.1;
                    let r2 = current_point.0;
                    let t2 = current_point.1;

                    let interpolated_temp = t1 + (r_ohm - r1) * (t2 - t1) / (r2 - r1);
                    return Ok(ThermodynamicTemperature::new::<uom::si::temperature::degree_celsius>(interpolated_temp));
                }
                prev_point = current_point;
            }
            // Resistance is less than the lowest resistance in the table
            Ok(ThermodynamicTemperature::new::<uom::si::temperature::degree_celsius>(prev_point.1))
        }
    }
    ```

4.  在 `TemperatureSensorReadings` 结构体中，修改 `into_temperature_data()` 方法：

    ```rust
    use uom::si::electric_potential::volt;
    use uom::si::temperature::{ThermodynamicTemperature, degree_celsius};
    use uom::si::electrical_resistance::ElectricalResistance;

    impl TemperatureSensorReadings {
        /// Converts the raw sensor voltage readings into temperature data.
        /// Requires a resistance-temperature lookup table if the readings are in thermistor mode.
        pub fn into_temperature_data(&self, lookup_table: Option<ResistanceTemperatureTable>) -> Result<TemperatureData, &'static str> {
            if self.is_thermistor {
                // External thermistor: calculate resistance, then temperature using lookup table
                if let Some(table) = lookup_table {
                    let ts1_resistance_ohm = (10000.0 * self.ts1.get::<volt>()) / (3.3 - self.ts1.get::<volt>());
                    let ts2_resistance_ohm = self.ts2.map(|v| (10000.0 * v.get::<volt>()) / (3.3 - v.get::<volt>()));
                    let ts3_resistance_ohm = self.ts3.map(|v| (10000.0 * v.get::<volt>()) / (3.3 - v.get::<volt>()));

                    let ts1_temp = convert_resistance_to_temperature(ElectricalResistance::new::<uom::si::electrical_resistance::ohm>(ts1_resistance_ohm), table)?;
                    let ts2_temp = ts2_resistance_ohm.map(|r| convert_resistance_to_temperature(ElectricalResistance::new::<uom::si::electrical_resistance::ohm>(r), table)).transpose()?;
                    let ts3_temp = ts3_resistance_ohm.map(|r| convert_resistance_to_temperature(ElectricalResistance::new::<uom::si::electrical_resistance::ohm>(r), table)).transpose()?;

                    Ok(TemperatureData {
                        ts1: ts1_temp,
                        ts2: ts2_temp,
                        ts3: ts3_temp,
                    })
                } else {
                    Err("Lookup table is required for thermistor readings")
                }
            } else {
                // Internal die temperature: calculate temperature
                let ts1_temp_celsius = 25.0 - (self.ts1.get::<volt>() - 1.2) / 0.0042;
                let ts2_temp_celsius = self.ts2.map(|v| 25.0 - (v.get::<volt>() - 1.2) / 0.0042);
                let ts3_temp_celsius = self.ts3.map(|v| 25.0 - (v.get::<volt>() - 1.2) / 0.0042);

                Ok(TemperatureData {
                    ts1: ThermodynamicTemperature::new::<degree_celsius>(ts1_temp_celsius),
                    ts2: ts2_temp_celsius.map(|t| ThermodynamicTemperature::new::<degree_celsius>(t)),
                    ts3: ts3_temp_celsius.map(|t| ThermodynamicTemperature::new::<degree_celsius>(t)),
                })
            }
        }
    }
    ```

## 流程图 (查表法)

```mermaid
graph TD
    A[TemperatureSensorReadings struct<br/>(电压, is_thermistor)] --> B{into_temperature_data(lookup_table)};
    B -- 如果 is_thermistor = true --> C[计算阻值 (Ω)];
    C --> D[convert_resistance_to_temperature(阻值, lookup_table)];
    D -- 返回温度 (°C) --> E[TemperatureData<br/>(温度 °C)];
    B -- 如果 is_thermistor = false --> F[计算芯片温度 (°C)];
    F --> E[TemperatureData<br/>(温度 °C)];
    E --> G[调用方 (驱动程序)];
```

请注意，您需要提供实际的电阻-温度对照表数据，并将其作为 `lookup_table` 参数传递给 `into_temperature_data` 方法。
