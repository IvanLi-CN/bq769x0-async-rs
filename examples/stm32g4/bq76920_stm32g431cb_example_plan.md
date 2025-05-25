# 为 BQ76920 驱动创建 STM32G431CB 示例项目计划

### 目标

为 `bq76920` 驱动创建一个独立的 `stm32g431cb` 示例项目，该项目将初始化 STM32G431CB 的 I2C 外设，并使用 `bq769x0-async-rs` 库读取 BQ76920 芯片的状态和测量数据。

### 计划步骤

1.  **创建项目目录并初始化 Cargo 项目**: 在当前工作目录 `/Volumes/ExData/Projects/Ivan/bq769x0-async-rs` 下的 `examples/stm32g4` 目录中运行 `cargo init --bin`，创建一个新的 Rust 二进制项目。
2.  **创建计划文档**: 在 `examples/stm32g4` 目录下创建 `bq76920_stm32g431cb_example_plan.md` 文件，并写入详细的计划内容。
3.  **更新 `examples/stm32g4/Cargo.toml`**:
    *   修改 `package` 部分，确保 `name` 和 `edition` 正确。
    *   添加以下依赖：
        ```toml
        embassy-stm32 = { git = "https://github.com/embassy-rs/embassy.git", rev = "94f9b2707486ca3eade5bf4b237edf3d6aa90f35", features = [
          "defmt",
          "time-driver-any",
          "stm32g431cb",
          "memory-x",
          "unstable-pac",
          "exti",
        ] }
        embassy-embedded-hal = { git = "https://github.com/embassy-rs/embassy.git", rev = "94f9b2707486ca3eade5bf4b237edf3d6aa90f35" }
        embassy-executor = { git = "https://github.com/embassy-rs/embassy.git", rev = "94f9b2707486ca3eade5bf4b237edf3d6aa90f35", features = [
          "arch-cortex-m",
          "executor-thread",
          "defmt",
        ] }
        embassy-time = { git = "https://github.com/embassy-rs/embassy.git", rev = "94f9b2707486ca3eade5bf4b237edf3d6aa90f35", features = [
          "defmt",
          "defmt-timestamp-uptime",
          "tick-hz-32_768",
        ] }
        embassy-sync = { git = "https://github.com/embassy-rs/embassy.git", rev = "94f9b2707486ca3eade5bf4b237edf3d6aa90f35", features = [
          "defmt",
        ] }
        defmt = "1.0.1"
        defmt-rtt = "1.0.0"
        panic-probe = { version = "1.0.0", features = ["print-defmt"] }
        static_cell = "2.1.0"
        heapless = { version = "0.8", default-features = false }
        bq769x0-async-rs = { version = "0.1.1", path = "../../.." }
        uom = { version = "0.37", default-features = false, features = ["si", "i32"] }
        cortex-m = { version = "0.7.7", features = ["inline-asm", "critical-section-single-core"] }
        cortex-m-rt = "0.7.5"
        embedded-hal = "1.0.0"
        portable-atomic = { version = "1.11.0", features = ["critical-section"] }
        libm = "0.2.8"
        embedded-io-async = { version = "0.6.1" }
        ```
    *   配置 `[features]`，启用 `defmt` 和 `async`，并指定 `stm32g431cb` 芯片。
    *   添加 `[profile.dev]` 和 `[profile.release]` 配置，以优化嵌入式开发。
4.  **创建 `examples/stm32g4/.cargo/config.toml`**: 配置 Cargo runner，以便使用 `probe-run` 等工具进行烧录和调试。
5.  **编写 `examples/stm32g4/src/main.rs`**:
    *   参考 `ups120` 项目中 `bq76920` 相关的代码作为基础。
    *   调整 `use` 语句以适应新的项目结构和依赖。
    *   简化 `main` 函数，只保留 `bq76920` 的初始化和数据读取逻辑。
    *   添加 `bq76920.init().await?` 调用，执行 BQ76920 的基本初始化。
    *   在主循环中，定期读取 BQ76920 的各种状态和测量寄存器（例如 `SystemStatus`, `CellVoltages`, `Temperatures` 等），并使用 `defmt::info!` 打印其值。
6.  **提供运行示例的说明**: 告知用户如何编译、烧录和运行这个示例项目。

### 流程图

```mermaid
graph TD
    A[开始] --> B{在 examples/stm32g4 目录中初始化 Cargo 项目};
    B --> C[创建 bq76920_stm32g431cb_example_plan.md];
    C --> D[更新 examples/stm32g4/Cargo.toml];
    D --> E[创建 examples/stm32g4/.cargo/config.toml];
    E --> F[编写 examples/stm32g4/src/main.rs];
    F --> G[实现 BQ76920 初始化和数据读取循环];
    H[提供运行示例说明] --> I[完成];