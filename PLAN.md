# 为 `bq769x0-async-rs` 项目添加 `binrw` 支持

## 目标
为所有数据类型支持 `binrw` 序列化和反序列化，版本为 `0.15.0`，作为可选依赖，并通过 `binrw` feature 启用。同时，为该特性增加测试用例。

## 计划步骤

1.  **修改 `Cargo.toml` 文件**
    *   在 `[features]` 部分添加 `binrw` 特性，并将其与 `binrw` 依赖关联。
    *   在 `[dependencies]` 部分添加 `binrw = { version = "0.15.0", optional = true }`。

2.  **识别并修改数据类型文件**
    *   需要检查 `src/data_types.rs` 和 `src/registers.rs` 文件，以及其他可能包含数据结构的文件。
    *   对于每个需要支持 `binrw` 的 `struct` 或 `enum`，添加 `#[derive(BinRead, BinWrite)]` 宏。
    *   根据 `binrw` 的要求，可能需要调整数据结构的字段顺序、字节序（endianness）或其他属性。

3.  **添加测试用例**
    *   在 `tests/` 目录下创建一个新的测试文件，例如 `tests/binrw_support.rs`。
    *   编写测试用例，确保在启用 `binrw` 特性时，数据类型能够正确地进行序列化和反序列化。
    *   测试用例应覆盖各种数据类型和边界情况。

4.  **将计划写入 `PLAN.md`**
    *   将上述计划的详细步骤和说明写入 `PLAN.md` 文件。

## 流程图

```mermaid
graph TD
    A[开始] --> B{修改 Cargo.toml};
    B --> C{识别数据类型文件};
    C --> D{为数据类型添加 BinRead/BinWrite 宏};
    D --> E{添加 binrw 测试用例};
    E --> F{将计划写入 PLAN.md};
    F --> G[完成];
