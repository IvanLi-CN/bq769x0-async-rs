# BQ769x0 驱动重构计划

## 目标

将 `bq76920/src/lib.rs` 文件中 `impl<I2C, E> Bq769x0<I2C, Disabled>` 和 `impl<I2C, E> Bq769x0<I2C, Enabled>` 中重复或相似的功能提取到一个新的泛型 `impl` 块中，以减少代码重复并提高可维护性。特别是将依赖于底层读写寄存器操作的高层方法泛化。

## 计划步骤

1. **定义 `RegisterAccess` Trait:**
    * 创建一个新的 trait `RegisterAccess<E>`，用于抽象读写寄存器的方法。
    * 这个 trait 将包含以下异步方法：
        * `read_register(&mut self, reg: Register) -> Result<u8, Error<E>>`
        * `read_registers(&mut self, reg: Register, len: usize) -> Result<Self::ReadBuffer, Error<E>>`
        * `write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>>`
        * `write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>>`
    * 定义一个关联类型 `ReadBuffer`，用于指定 `read_registers` 方法返回的缓冲区类型，以处理不同 CRC 模式下缓冲区大小的差异。
    * 为 trait 和其方法添加 `#[maybe_async_cfg::maybe(...)]` 属性，以支持同步和异步上下文。

2. **为 `Bq769x0<I2C, Disabled>` 实现 `RegisterAccess` Trait:**
    * 为 `Bq769x0<I2C, Disabled>` 实现 `RegisterAccess<E>` trait。
    * 将 `ReadBuffer` 关联类型设置为 `heapless::Vec<u8, 30>`。
    * 实现 trait 中的方法，使用现有的无 CRC 的读写寄存器逻辑。

3. **为 `Bq769x0<I2C, Enabled>` 实现 `RegisterAccess` Trait:**
    * 为 `Bq769x0<I2C, Enabled>` 实现 `RegisterAccess<E>` trait。
    * 将 `ReadBuffer` 关联类型设置为 `heapless::Vec<u8, 60>`。
    * 实现 trait 中的方法，使用现有的包含 CRC 校验的读写寄存器逻辑。

4. **创建新的泛型 `impl` 块:**
    * 创建一个新的 `impl<I2C, M, E> Bq769x0<I2C, M>` 块。
    * 添加约束 `where I2C: I2c<Error = E>, M: CrcMode, Self: RegisterAccess<E>`。
    * 为这个新的 impl 块添加 `#[maybe_async_cfg::maybe(...)]` 属性。

5. **将通用方法移动到新的泛型 `impl` 块:**
    * 将 `impl<I2C, E> Bq769x0<I2C, Enabled>` 中除了 `new` 方法之外的所有方法移动到新的泛型 `impl` 块中。这些方法包括：
        * `read_adc_calibration`
        * `read_cell_voltages`
        * `read_pack_voltage`
        * `read_temperatures`
        * `read_current`
        * `read_status`
        * `clear_status_flags`
        * `enable_charging`
        * `disable_charging`
        * `enable_discharging`
        * `disable_discharging`
        * `set_cell_balancing`
        * `configure_protect1`
        * `configure_protect2`
        * `configure_protect3`
        * `configure_ov_trip`
        * `configure_uv_trip`
        * `enter_ship_mode`
        * `is_alert_overridden`
        * `convert_raw_cc_to_current_ma`
    * 将 `impl<I2C, E> Bq769x0<I2C, Disabled>` 中的 `convert_raw_cc_to_current_ma` 方法也移动到新的泛型 `impl` 块中。

6. **更新移动后的方法:**
    * 在新的泛型 `impl` 块中，修改移动过来的方法，使其调用 `self.read_register`, `self.read_registers`, `self.write_register`, `self.write_registers`。这些调用将通过 `RegisterAccess` trait 动态分派到具体的 `Disabled` 或 `Enabled` 实现。

7. **清理原始 `impl` 块:**
    * 从 `impl<I2C, E> Bq769x0<I2C, Disabled>` 块中移除 `read_register`, `read_registers`, `write_register`, `write_registers` 和 `convert_raw_cc_to_current_ma` 方法。
    * 从 `impl<I2C, E> Bq769x0<I2C, Enabled>` 块中移除所有已移动到泛型 `impl` 块中的方法，以及 `read_register`, `read_registers`, `write_register`, 和 `write_registers` 方法。
    * 最终，原始的两个 `impl` 块将只包含各自的构造函数 (`new_without_crc` 和 `new`)。

8. **审查和完善:**
    * 仔细审查修改后的代码，确保逻辑正确，没有引入新的错误。
    * 运行测试以验证重构的正确性。

## 结构示意图

```mermaid
graph TD
    A[Bq769x0<I2C, M>] --> B{M: CrcMode};
    B --> C[M = Disabled];
    B --> D[M = Enabled];

    C --> E[impl Bq769x0<I2C, Disabled>];
    D --> F[impl Bq769x0<I2C, Enabled>];

    E -- Implements --> G[RegisterAccess Trait];
    F -- Implements --> G;

    G -- Provides methods like --> H[read_register];
    G -- Provides methods like --> I[write_register];
    G -- Provides methods like --> J[read_registers];
    G -- Provides methods like --> K[write_registers];

    L[impl Bq769x0<I2C, M> where Self: RegisterAccess] -- Uses methods from --> G;

    L -- Contains methods like --> M[read_cell_voltages];
    L -- Contains methods like --> N[read_pack_voltage];
    L -- Contains methods like --> O[read_temperatures];
    L -- Contains methods like --> P[read_current];
    L -- Contains methods like --> Q[read_status];
    L -- Contains methods like --> R[clear_status_flags];
    L -- Contains methods like --> S[enable_charging];
    L -- Contains methods like --> T[disable_charging];
    L -- Contains methods like --> U[enable_discharging];
    L -- Contains methods like --> V[disable_discharging];
    L -- Contains methods like --> W[set_cell_balancing];
    L -- Contains methods like --> X[configure_protect1];
    L -- Contains methods like --> Y[configure_protect2];
    L -- Contains methods like --> Z[configure_protect3];
    L -- Contains methods like --> AA[configure_ov_trip];
    L -- Contains methods like --> AB[configure_uv_trip];
    L -- Contains methods like --> AC[enter_ship_mode];
    L -- Contains methods like --> AD[is_alert_overridden];
    L -- Contains methods like --> AE[convert_raw_cc_to_current_ma];

    E -- Contains --> AF[new_without_crc];
    F -- Contains --> AG[new];
