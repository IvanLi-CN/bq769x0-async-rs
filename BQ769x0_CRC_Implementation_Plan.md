# BQ769x0 CRC 功能实现计划

## 目标

为 `bq76920/src/lib.rs` 中的 BQ769x0 驱动增加 CRC 功能的支持，通过泛型参数控制 CRC 的启用状态，并在读写寄存器函数中实现两种不同的逻辑。

## CRC 算法细节

根据数据手册 [`bq76920/bq76920.pdf`](bq76920/bq76920.pdf)，BQ769x0 使用的是一个 8-bit CRC，具体参数如下：

* **多项式:** `x^8 + x^2 + x + 1` (即 0x07)
* **初始值:** 0x00
* **输入反射:** 无
* **输出异或:** 无

CRC 的计算方式取决于 I2C 事务类型（单字节读写或块读写）。

* **单字节写:** CRC 计算覆盖 slave 地址, register 地址, 和 data。
* **块写:** 第一个数据字节的 CRC 计算覆盖 slave 地址, register 地址, 和 data。后续数据字节的 CRC 计算仅覆盖数据字节。
* **单字节读:** CRC 计算在第二个 start 后进行，覆盖 slave 地址 和 data byte。
* **块读:** 第一个数据字节的 CRC 计算在第二个 start 后进行，覆盖 slave 地址 和 data byte。后续数据字节的 CRC 计算仅覆盖数据字节。

## 实现计划步骤

1. **定义 CRC 启用状态枚举：**
    * 创建一个新的枚举类型，例如 `CrcMode`，包含两个变体：`Enabled` 和 `Disabled`。

2. **修改 `Bq769x0` 结构体：**
    * 在 `Bq769x0` 结构体中增加一个泛型参数，例如 `M: CrcMode`，用来表示 CRC 的启用状态。

3. **实现 CRC 计算函数：**
    * 实现一个独立的函数，例如 `calculate_crc(data: &[u8]) -> u8`，该函数根据 BQ769x0 的 CRC 算法计算给定数据的 CRC 值。

4. **为不同的 CRC 模式实现读写寄存器函数：**
    * 为了清晰地分离逻辑，我们将为 `Bq769x0<I2C, CrcMode::Disabled>` 和 `Bq769x0<I2C, CrcMode::Enabled>` 分别实现读写寄存器的函数。
    * **禁用 CRC 模式 (`Bq769x0<I2C, CrcMode::Disabled>`)：**
        * `read_register` 和 `write_register` 函数将保持当前的实现，不包含 CRC 计算和验证。
        * `read_registers` 和 `write_registers` 函数也将保持当前的实现。
    * **启用 CRC 模式 (`Bq769x0<I2C, CrcMode::Enabled>`)：**
        * `read_register(&mut self, reg: Register) -> Result<u8, Error<E>>`：
            * 发送寄存器地址。
            * 读取一个字节的数据和一个字节的 CRC。
            * 使用 `calculate_crc` 函数计算接收到的数据和地址的 CRC。
            * 比较计算出的 CRC 和接收到的 CRC。如果不匹配，返回 `Error::InvalidData`。
            * 返回读取到的数据。
        * `write_register(&mut self, reg: Register, value: u8) -> Result<(), Error<E>>`：
            * 构建包含寄存器地址和数据的数据包。
            * 使用 `calculate_crc` 函数计算数据包的 CRC。
            * 发送数据包和计算出的 CRC。
            * 检查 I2C 事务是否成功。
        * `read_registers(&mut self, reg: Register, len: usize) -> Result<heapless::Vec<u8, 30>, Error<E>>`：
            * 发送起始寄存器地址。
            * 循环读取 `len` 个数据字节，每个数据字节后跟随一个 CRC 字节。
            * 对于第一个数据字节，计算地址和数据的 CRC。对于后续数据字节，仅计算数据字节的 CRC。
            * 验证每个 CRC。
            * 将读取到的数据存储到 `heapless::Vec` 中。
            * 返回数据向量。
        * `write_registers(&mut self, reg: Register, values: &[u8]) -> Result<(), Error<E>>`：
            * 构建包含起始寄存器地址和所有数据字节的数据包。
            * 对于第一个数据字节，计算地址和数据的 CRC。对于后续数据字节，仅计算数据字节的 CRC。
            * 发送数据包和计算出的 CRC。
            * 检查 I2C 事务是否成功。

5. **更新其他依赖读写寄存器函数的函数：**
    * `read_adc_calibration`、`read_cell_voltages`、`read_pack_voltage`、`read_temperatures`、`read_current`、`read_status`、`clear_status_flags`、`enable_charging`、`disable_charging`、`enable_discharging`、`disable_discharging`、`set_cell_balancing`、`configure_protect1`、`configure_protect2`、`configure_protect3`、`configure_ov_trip`、`configure_uv_trip`、`enter_ship_mode`、`is_alert_overridden` 等函数将通过调用带有正确泛型参数的 `read_register`、`write_register`、`read_registers` 或 `write_registers` 来实现。

## 结构体和实现方式可视化

```mermaid
classDiagram
    direction TB
    class Bq769x0<I2C, M> {
        address: u8
        i2c: I2C
        +new(i2c: I2C, address: u8)
    }

    class CrcMode {
        <<enumeration>>
        Disabled
        Enabled
    }

    class Bq769x0_Disabled {
        <<impl Bq769x0<I2C, CrcMode::Disabled>>>
        +read_register(...)
        +write_register(...)
        +read_registers(...)
        +write_registers(...)
        // ... other functions
    }

    class Bq769x0_Enabled {
        <<impl Bq769x0<I2C, CrcMode::Enabled>>>
        +read_register(...)
        +write_register(...)
        +read_registers(...)
        +write_registers(...)
        // ... other functions
    }

    Bq769x0 --|> Bq769x0_Disabled : where M = Disabled
    Bq769x0 --|> Bq769x0_Enabled : where M = Enabled
    Bq769x0_Enabled ..> CrcMode : uses
    Bq769x0_Disabled ..> CrcMode : uses
