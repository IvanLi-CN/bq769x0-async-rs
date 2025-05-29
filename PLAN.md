# 重构计划：移除 tests/common.rs 依赖

## 目标

重构测试代码，消除对 `tests/common.rs` 文件的依赖，并最终删除该文件。

## 计划步骤

1. **分析依赖**: 确定依赖 `tests/common.rs` 的文件：
    * `tests/status_monitor.rs`
    * `tests/protection_config.rs`
    * `tests/other_config.rs`
    * `tests/init_basic.rs`
2. **修改测试文件**: 针对每个依赖文件，进行以下修改：
    * 移除 `#[path = "common.rs"] mod common;` 引用。
    * 移除所有对 `common::` 前缀的引用。
    * 直接使用 `embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};`。
    * 将 `tests/common.rs` 中 `MockI2c` 的功能（创建 mock、验证事务）直接在每个测试文件中实现或使用 `embedded_hal_mock` 提供的功能替代。
    * 将 `tests/common.rs` 中的 helper 函数 (`create_driver_with_adc_calibration`, `create_bq769x0_driver_disabled_crc`) 的逻辑直接整合到测试用例中，或者在每个测试文件中重新实现这些 helper 函数（如果它们足够通用且不违反您的意图）。
    * 将常量 `BQ76920_ADDR` 直接定义在每个测试文件中。
3. **逐步测试**: 每修改一个文件后，运行 `cargo test` 确保测试通过。
4. **修复错误**: 如果测试失败，根据错误信息调整代码，直到测试通过。
5. **删除 common.rs**: 在所有依赖都移除且测试通过后，删除 `tests/common.rs` 文件。
6. **最终验证**: 再次运行 `cargo test` 确认整个项目测试通过。

## 实施说明

* 在修改每个测试文件时，需要仔细检查并替换所有对 `tests/common.rs` 中定义的函数、结构体和常量的引用。
* `MockI2c` 的实现需要根据 `embedded_hal_mock` 库的用法进行调整。
* Helper 函数的逻辑需要根据具体测试用例的需求进行整合或重新实现。
* 在每个文件修改完成后，立即运行 `cargo test` 进行验证，以便及时发现和修复问题。
* 在删除 `tests/common.rs` 之前，务必确保所有测试文件都已成功重构并通过测试。
