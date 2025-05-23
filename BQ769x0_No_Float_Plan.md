# BQ769x0 驱动库无浮点数计划

## 目标
将 `bq76920` 驱动库中的所有浮点数使用替换为整数运算，以适应没有硬件浮点支持的 MCU。

## 受影响的文件
*   `bq76920/src/lib.rs`
*   `bq76920/src/units.rs`

## 计划步骤

1.  **修改 `bq76920/src/lib.rs` 中的 `read_temperatures` 函数：**
    *   将温度转换公式 `Temperature (K) = (raw_ts * 0.01) + 273.15` 转换为整数运算。
    *   考虑到 `uom` 库使用 `i32` 作为底层存储，我们将所有值乘以一个足够大的整数比例因子，例如 `10000`，以保留小数精度。
    *   新的公式将类似于：`Temperature (uK) = (raw_ts * 100) + (27315 * 10000)` (其中 `raw_ts` 是原始 ADC 值，结果为微开尔文)。

2.  **修改 `bq76920/src/lib.rs` 中的 `find_closest_scd_threshold_bits` 和 `find_closest_ocd_threshold_bits` 函数：**
    *   将函数签名中的 `target_voltage_mv: f32` 修改为整数类型，例如 `target_voltage_mv: i32` (表示毫伏)。
    *   修改函数内部的逻辑，使其完全使用整数运算进行阈值查找。这可能涉及将内部的浮点常量转换为整数比例因子。

3.  **修改 `bq76920/src/units.rs` 中的 `test_units` 函数：**
    *   将 `assert_eq!(resistance.get::<ohm>(), 0.001);` 修改为使用整数断言，例如 `assert_eq!(resistance.get::<hundred_micro_ohms::hundred_micro_ohm>(), 10);` 或其他合适的整数单位。

## 结构示意图

```mermaid
graph TD
    A[开始] --> B{识别浮点数使用点};
    B --> C[bq76920/src/lib.rs];
    B --> D[bq76920/src/units.rs];

    C --> C1[修改 read_temperatures 函数];
    C --> C2[修改 find_closest_scd_threshold_bits 函数];
    C --> C3[修改 find_closest_ocd_threshold_bits 函数];

    D --> D1[修改 test_units 函数];

    C1 --> E{转换为整数运算};
    C2 --> E;
    C3 --> E;
    D1 --> E;

    E --> F[验证修改];
    F --> G[完成];