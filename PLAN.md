# Bitflags 集成计划

## 目标

通过引入 `bitflags` 库，提高代码的可读性、可维护性和类型安全性，特别是在处理硬件寄存器和系统状态标志时。

## 步骤

1. **添加 `bitflags` 依赖**
    * 修改 `Cargo.toml` 文件，在 `[dependencies]` 部分添加 `bitflags` crate。

2. **重构 `src/registers.rs` 中的位掩码常量**
    * 将 `src/registers.rs` 中定义的 `pub const` 位掩码（如 `SYS_STAT_CC_READY`、`SYS_CTRL1_LOAD_PRESENT` 等）转换为 `bitflags!` 宏定义的结构体。
    * 为每个相关的寄存器（例如 `SYS_STAT`、`SYS_CTRL1`、`SYS_CTRL2`、`PROTECT1`、`PROTECT2`、`PROTECT3`、`CELLBAL1`、`CELLBAL2`、`CELLBAL3`）创建对应的 `bitflags!` 类型。
    * 示例：将 `SYS_STAT` 相关的常量重构为 `SysStatFlags`。

3. **重构 `src/data_types.rs` 中的 `SystemStatus` 结构体**
    * 将 `SystemStatus` 结构体中的布尔字段替换为单个 `SysStatFlags` 位标志类型。
    * 更新 `SystemStatus::new()` 函数，使其接受一个 `u8` 字节并直接创建 `SysStatFlags` 实例。
    * 更新 `SystemStatus` 的 `BinRead` 和 `BinWrite` 实现，使其直接读写 `SysStatFlags`。

4. **重构 `src/data_types.rs` 中的 `MosStatus` 结构体**
    * 将 `MosStatus` 结构体中的布尔字段替换为单个位标志类型（例如 `MosControlFlags`）。
    * 更新 `MosStatus::new()` 函数，使其接受一个 `u8` 字节并直接创建 `MosControlFlags` 实例。
    * 更新 `MosStatus` 的 `BinRead` 和 `BinWrite` 实现，使其直接读写 `MosControlFlags`。

5. **更新 `BatteryConfig` 结构体**
    * 如果 `BatteryConfig` 中有直接映射到 `SYS_CTRL1` 或 `SYS_CTRL2` 位的布尔字段，考虑将其替换为对应的 `bitflags!` 类型，以保持一致性。

6. **更新所有引用了这些位掩码或结构体的地方**
    * 在整个项目中搜索并替换所有对旧位掩码常量和 `SystemStatus`/`MosStatus` 布尔字段的引用，改为使用新的 `bitflags!` 类型和其提供的方法（如 `contains()`, `insert()`, `remove()`）。

7. **编写或更新测试**
    * 确保所有受影响的功能在引入 `bitflags` 后仍然正常工作。
    * 特别关注涉及寄存器读写和状态判断的测试用例。

## Mermaid 图示

```mermaid
graph TD
    A[开始] --> B(分析现有位操作);
    B --> C{识别位字段和状态标志};
    C --> D[修改 Cargo.toml 添加 bitflags 依赖];
    D --> E[重构 src/registers.rs 中的位掩码为 bitflags];
    E --> F[重构 src/data_types.rs 中的 SystemStatus 为 bitflags];
    F --> G[重构 src/data_types.rs 中的 MosStatus 为 bitflags];
    G --> H[更新 BatteryConfig 结构体];
    H --> I[更新所有引用了旧位掩码的代码];
    I --> J[编写/更新测试];
    J --> K[完成];
