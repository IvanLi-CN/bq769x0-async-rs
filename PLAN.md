# 详细计划

## 任务目标
参考 `docs/bq76920.md` 文档，检查 `tests` 目录下的测试用例是否正确无误。

## 阶段 1: 信息收集与初步理解

1.  **阅读 `docs/bq76920.md` 文档**：
    *   重点关注以下章节，提取关键信息：
        *   **7 Specifications** (特别是 `7.5 Electrical Characteristics` 和 `7.6 Timing Requirements`)：获取 ADC 校准、温度测量、电流测量、保护阈值和延迟的电气特性、单位和计算公式。
        *   **8 Detailed Description** (特别是 `8.3.1.1 Measurement Subsystem Overview`, `8.3.1.2 Protection Subsystem`, `8.3.1.3 Control Subsystem`, `8.3.1.4 Communications Subsystem` 和 `8.5 Register Maps`)：获取寄存器地址、位定义、默认值、I2C 通信协议（包括 CRC 算法细节）、不同 BQ769x0 型号的特性（如电池串数、平衡寄存器数量、TSn 引脚功能）。
    *   记录所有相关的计算公式、表格数据和重要说明。

2.  **初步浏览 `tests` 目录下的所有 Rust 文件**：
    *   `tests/common.rs`：理解其提供的 Mock I2C 实现和辅助函数。
    *   `tests/init_basic.rs`：了解基本的 I2C 读写和初始化序列测试。
    *   `tests/other_config.rs`：了解进入 SHIP 模式、ALERT 覆盖、充电/放电控制以及电池平衡的测试。
    *   `tests/protection_config.rs`：了解保护配置（OV, UV, OCD, SCD）的测试。
    *   `tests/status_monitor.rs`：了解各种测量值（电池电压、总电压、温度、电流、状态）的读取测试。

## 阶段 2: 深入分析与交叉验证 (在 `code` 模式下执行)

1.  **I2C CRC 验证**：
    *   根据文档 `8.3.1.4 Communications Subsystem` 中描述的 CRC 多项式 (`x^8 + x^2 + x + 1`) 和初始值 (`0`)，实现 CRC-8 计算函数。
    *   对照 `tests/init_basic.rs` 中的 `test_read_register_enabled_crc_success`, `test_write_register_enabled_crc_success`, `test_read_registers_enabled_crc_success`, `test_write_registers_enabled_crc_success` 测试用例，验证其 CRC 期望值是否与计算结果一致。特别注意单字节和块读写时 CRC 计算的字节范围。

2.  **电池平衡寄存器逻辑验证**：
    *   详细分析 `tests/other_config.rs` 中 `set_cell_balancing` 函数的实现，以及 `test_set_cell_balancing_bq76930` 和 `test_set_cell_balancing_bq76940` 的期望值。
    *   根据 `docs/bq76920.md` 中 `8.5 Register Maps` 章节对 `CELLBAL1`, `CELLBAL2`, `CELLBAL3` 寄存器的定义，以及 `Section 5 Device Comparison Table` 中不同 BQ769x0 型号支持的电池串数，确认 `set_cell_balancing` 函数的逻辑是否正确处理了不同型号的电池平衡位映射。
    *   如果发现 `u16` 类型不足以表示 BQ76940 的所有电池平衡位，将提出相应修改建议。

3.  **保护阈值计算验证**：
    *   根据 `docs/bq76920.md` 中 `8.3.1.2.1 Integrated Hardware Protections` 章节的公式和 `7.5 Electrical Characteristics` 中的表格，精确计算 `tests/protection_config.rs` 中 `test_set_config_basic` 测试用例中 `OV_TRIP`, `UV_TRIP`, `PROTECT1`, `PROTECT2` 寄存器的预期值。
    *   特别关注 `RSNS` 位对 OCD/SCD 阈值的影响，以及 `BatteryConfig` 中 `scd_limit` 和 `ocd_limit` 的值是否在文档规定的范围内。

4.  **测量值转换公式验证**：
    *   **Pack Voltage**: 重点验证 `tests/status_monitor.rs` 中 `test_read_pack_voltage` 测试用例中 `pack_voltage.get::<millivolt>()` 的计算是否正确。对照文档 `8.3.1.1.6 16-Bit Pack Voltage` 中的公式 `V(BAT) = 4 × GAIN × ADC(cell) + (HCells × OFFSET)`，确认 `ADC(cell)` 的含义以及乘以 4 的因子是否正确应用。
    *   **Temperature**: 验证 `tests/status_monitor.rs` 中 `test_read_temperatures_die_temp` 和 `test_read_temperatures_external_thermistor` 测试用例中温度转换的完整性。对照文档 `8.3.1.1.4 External Thermistor` 和 `8.3.1.1.5 Die Temperature Monitor` 中的公式，确保 `V_TSX` 和最终温度的计算都得到了验证。
    *   **Current**: 验证 `tests/status_monitor.rs` 中 `test_read_current` 和 `test_read_all_measurements` 测试用例中电流转换的正确性。对照文档 `8.3.1.1.3 16-Bit CC` 中的公式，确认 `RSENSE` 值在计算中的使用。

5.  **`common::create_driver_with_adc_calibration` 行为确认**：
    *   确认 `tests/common.rs` 中 `MockI2c` 的 `Clone` 实现是否确保了 `driver` 内部使用的 `i2c_mock` 和函数返回的 `i2c_mock` 共享同一个底层 mock 状态，以保证测试的有效性。

## 阶段 3: 总结与建议

1.  **列出发现的问题**：在对比过程中，记录所有不一致、潜在错误或可以改进的地方。
2.  **提供修改建议**：针对发现的问题，提出具体的代码修改建议。
3.  **生成 Mermaid 图**：如果需要，为复杂的逻辑（如 CRC 计算流程、OV/UV 阈值计算流程、电池平衡寄存器映射）创建 Mermaid 图，以增强可读性。

---

### Mermaid 图示例 (CRC 计算流程)

```mermaid
graph TD
    A[开始] --> B{CRC 模式启用?};
    B -- 是 --> C{读操作还是写操作?};
    C -- 读 --> D[读取从设备地址和数据字节];
    C -- 写 --> E[写入从设备地址、寄存器地址和数据字节];
    D --> F{单字节还是块读?};
    E --> G{单字节还是块写?};
    F -- 单字节 --> H[对从设备地址和数据字节计算 CRC];
    F -- 块读 --> I[对第一个数据字节计算 CRC (含从设备地址)];
    I --> J[对后续数据字节单独计算 CRC];
    G -- 单字节 --> K[对从设备地址、寄存器地址和数据计算 CRC];
    G -- 块写 --> L[对第一个数据字节计算 CRC (含从设备地址和寄存器地址)];
    L --> M[对后续数据字节单独计算 CRC];
    H --> N[附加 CRC 到数据];
    I --> N;
    J --> N;
    K --> N;
    M --> N;
    N --> O[发送/接收数据和 CRC];
    O --> P{CRC 校验通过?};
    P -- 否 --> Q[返回 CRC 错误];
    P -- 是 --> R[返回成功];
    B -- 否 --> S[直接发送/接收数据];
    S --> R;
