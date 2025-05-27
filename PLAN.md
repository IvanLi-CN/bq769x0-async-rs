# BQ769x0 系列设备测试覆盖计划

## 当前测试用例的局限性

当前的 `tests/basic.rs` 文件主要针对 `BQ76920` 设备进行了测试，未能覆盖 BQ769x0 系列的所有型号（BQ76920、BQ76930、BQ76940）。主要体现在：

* 所有测试都使用 `BQ76920_ADDR`。
* `Bq769x0` 实例的泛型参数固定为 `5` 节电池。
* 温度传感器测试断言 `ts2` 和 `ts3` 为 `None`。
* 存在明确的 `TODO` 注释，指出缺少 `BQ76930` 和 `BQ76940` 的电池平衡测试。

## BQ769x0 系列设备差异分析

根据 `bq76920.pdf` 数据手册，BQ769x0 系列的主要差异在于支持的电池节数和温度传感器数量：

* **BQ76920**: 支持 3-5 节电池，1 个温度传感器 (TS1)。
* **BQ76930**: 支持 6-10 节电池，2 个温度传感器 (TS1, TS2)。
* **BQ76940**: 支持 9-15 节电池，3 个温度传感器 (TS1, TS2, TS3)。
* 所有型号都支持 I2C 通信，默认地址为 `0x08`，部分型号有 `0x18` 选项。
* 电池平衡寄存器：`BQ76920` 使用 `CELLBAL1`；`BQ76930` 使用 `CELLBAL1` 和 `CELLBAL2`；`BQ76940` 使用 `CELLBAL1`、`CELLBAL2` 和 `CELLBAL3`。

## 详细测试计划

本计划旨在实现对所有 BQ769x0 系列设备（BQ76920、BQ76930、BQ76940）的全面测试覆盖。

**核心目标：**

1. **扩展现有测试框架：** 利用 `embedded_hal_mock` 模拟 I2C 通信，保持测试的独立性和可重复性。
2. **新增型号特定测试：** 为 `BQ76930` 和 `BQ76940` 添加新的测试用例，覆盖其特有功能和配置。
3. **泛型化通用测试：** 尽可能将现有测试用例泛型化，使其能够轻松应用于所有型号，减少代码重复。
4. **验证不同电池节数：** 针对每个型号支持的电池节数范围，至少测试其最小和最大节数配置。
5. **持续集成验证：** 每次修改后及时运行 `cargo test` 命令，确保代码的正确性。

**具体步骤：**

1. **分析 `src/lib.rs` 和 `src/registers.rs`：**
    * 深入理解 `Bq769x0` 结构体如何通过泛型参数 `CELLS` 处理不同电池节数。
    * 检查 `src/registers.rs` 中是否已定义所有型号特有的寄存器（如 `CELLBAL2`、`CELLBAL3`、`TS2_HI/LO`、`TS3_HI/LO` 等）。如果缺少，需要先添加这些定义。

2. **重构和扩展 `tests/basic.rs`：**
    * **为 `BQ76930` 添加测试：**
        * 定义 `BQ76930_ADDR` (如果与 `BQ76920` 不同，或需要测试 `0x18` 地址)。
        * 创建 `Bq769x0::<_, _, 10>::new_without_crc(...)` 实例（或根据需要测试 6 节电池）。
        * 添加 `test_read_cell_voltages_bq76930`：模拟 10 节电池的电压读取。
        * 添加 `test_read_temperatures_bq76930`：模拟读取 TS1 和 TS2 温度。
        * 添加 `test_set_cell_balancing_bq76930`：测试 `CELLBAL1` 和 `CELLBAL2` 寄存器。
    * **为 `BQ76940` 添加测试：**
        * 定义 `BQ76940_ADDR` (如果与 `BQ76920` 不同，或需要测试 `0x18` 地址)。
        * 创建 `Bq769x0::<_, _, 15>::new_without_crc(...)` 实例（或根据需要测试 9 节电池）。
        * 添加 `test_read_cell_voltages_bq76940`：模拟 15 节电池的电压读取。
        * 添加 `test_read_temperatures_bq76940`：模拟读取 TS1、TS2 和 TS3 温度。
        * 添加 `test_set_cell_balancing_bq76940`：测试 `CELLBAL1`、`CELLBAL2` 和 `CELLBAL3` 寄存器。
    * **泛型化通用测试：** 考虑将 `test_read_register`、`test_write_register`、`test_read_pack_voltage`、`test_read_current`、`test_clear_status_flags`、`test_enable_charging`、`test_disable_charging`、`test_enable_discharging`、`test_is_alert_overridden` 等测试重构为泛型函数或使用宏，以便它们可以针对不同型号和电池节数进行复用。

3. **运行 `cargo test`：** 在每次完成一个型号或一组功能的测试用例编写后，立即运行 `cargo test` 命令，以验证代码的正确性并及时发现问题。

## 测试流程图

```mermaid
graph TD
    A[开始任务] --> B{分析现有测试和数据手册};
    B --> C{确定不同型号差异和测试需求};
    C --> D{规划测试用例结构};
    D --> E{检查并完善 src/registers.rs};
    E --> F{编写 BQ76930 测试用例};
    F --> G{运行 cargo test};
    G -- 成功 --> H{编写 BQ76940 测试用例};
    G -- 失败 --> I{调试并修复};
    I --> F;
    H --> J{运行 cargo test};
    J -- 成功 --> K{泛型化通用测试用例};
    J -- 失败 --> L{调试并修复};
    L --> H;
    K --> M{运行 cargo test};
    M -- 成功 --> N[完成测试覆盖];
    M -- 失败 --> O{调试并修复};
    O --> K;
