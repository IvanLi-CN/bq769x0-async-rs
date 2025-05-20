# BQ769x0 代码拆分计划

## 目标

将 `bq76920/src/lib.rs` 文件中的部分 `struct` 和 `enum` 定义拆分到单独的文件中，以改善项目结构，提高可读性和可维护性。

## 拆分方案

根据对 `bq76920/src/lib.rs` 文件内容的分析，建议将以下定义拆分到新的文件中：

* 将 `Error<E>` enum 移动到 `bq76920/src/errors.rs`。
* 将 `CrcMode` trait, `Disabled` struct, `Enabled` struct, 和 `calculate_crc` 函数移动到 `bq76920/src/crc.rs`。
* 将 `CellVoltages<const N: usize>` struct, `Temperatures` struct, 和 `Current` struct 移动到 `bq76920/src/data_types.rs`。

## 详细步骤

1. **创建新文件**:
    * 在 `bq76920/src/` 目录下创建名为 `errors.rs` 的新文件。
    * 在 `bq76920/src/` 目录下创建名为 `crc.rs` 的新文件。
    * 在 `bq76920/src/` 目录下创建名为 `data_types.rs` 的新文件。

2. **迁移代码**:
    * 将 `bq76920/src/lib.rs` 中定义 `Error<E>` enum 的代码块（从第 15 行到第 26 行）及其相关注释移动到 `bq76920/src/errors.rs` 文件中。
    * 将 `bq76920/src/lib.rs` 中定义 `CrcMode` trait, `Disabled` struct, `Enabled` struct, 和 `calculate_crc` 函数的代码块（从第 28 行到第 94 行）及其相关注释移动到 `bq76920/src/crc.rs` 文件中。
    * 将 `bq76920/src/lib.rs` 中定义 `CellVoltages<const N: usize>` struct, `Temperatures` struct, 和 `Current` struct 的代码块（从第 39 行到第 65 行）及其相关注释移动到 `bq76920/src/data_types.rs` 文件中。

3. **更新 `bq76920/src/lib.rs`**:
    * 从 `bq76920/src/lib.rs` 中删除步骤 2 中已迁移的代码块。
    * 在 `bq76920/src/lib.rs` 中添加对新模块的声明。通常，这会在文件顶部完成：

        ```rust
        mod errors;
        mod crc;
        mod data_types;
        ```

    * 在 `bq76920/src/lib.rs` 中添加 `pub use` 语句，以便从外部可以访问新模块中定义的公共项：

        ```rust
        pub use errors::Error;
        pub use crc::{CrcMode, Disabled, Enabled};
        pub use data_types::{CellVoltages, Temperatures, Current};
        ```

    * 更新 `Bq769x0` 的实现块中对 `calculate_crc` 函数的调用，将其前缀更改为 `crc::`，例如 `crc::calculate_crc(...)`。

4. **更新新文件中的代码**:
    * 在 `bq76920/src/errors.rs` 中，确保包含必要的 crate 引用和导入，例如 `extern crate defmt;` 和 `use defmt::Format;`。
    * 在 `bq76920/src/crc.rs` 中，确保包含必要的 crate 引用和导入。
    * 在 `bq76920/src/data_types.rs` 中，确保包含必要的 crate 引用和导入，例如 `extern crate defmt;` 和 `use defmt::Format;`。

## 文件结构变化可视化

```mermaid
graph TD
    A[bq76920/src/lib.rs] --> B[bq76920/src/errors.rs]
    A --> C[bq76920/src/crc.rs]
    A --> D[bq76920/src/data_types.rs]
