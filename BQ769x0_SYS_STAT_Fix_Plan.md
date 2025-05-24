# BQ769x0 SYS_STAT 寄存器位定义修复计划

## 计划目标：
修复 `src/registers.rs` 中 `SYS_STAT` 寄存器的位掩码定义，使其与数据手册 `bq76920.pdf` 完全一致。

## 计划步骤：

1.  **确认 `SYS_STAT` 寄存器的正确位定义：**
    *   再次查阅数据手册 `bq76920.pdf` 第 29 页的 Table 8-3: SYS_STAT (0x00)，确保获取到每个位对应的正确名称和功能。

2.  **修改 `SYS_STAT` 寄存器的位掩码定义：**
    *   在 `src/registers.rs` 中，删除所有不正确的 `SYS_STAT` 位掩码定义。
    *   根据数据手册，重新定义 `SYS_STAT` 寄存器的所有位掩码，确保每个位都对应正确的名称和位位置。
        *   `CC_READY` (Bit 7)
        *   `DEVICE_XREADY` (Bit 5)
        *   `OVRD_ALERT` (Bit 4)
        *   `UV` (Bit 3)
        *   `OV` (Bit 2)
        *   `SCD` (Bit 1)
        *   `OCD` (Bit 0)
    *   对于数据手册中标记为 `RSVD` 的位（Bit 6），可以不显式定义，或者添加注释说明其为保留位。

3.  **验证修复：**
    *   修复完成后，需要重新检查 `src/registers.rs` 文件，确保 `SYS_STAT` 寄存器的位掩码定义现在与数据手册完全一致。
    *   在切换到代码模式后，需要进行编译和单元测试来验证这些更改没有引入新的问题。

4.  **切换到代码模式：**
    *   在用户批准计划后，将请求切换到 `code` 模式来执行修复。

## Mermaid 流程图：

```mermaid
graph TD
    A[用户请求修复问题] --> B{确认SYS_STAT寄存器正确位定义};
    B --> C[修改src/registers.rs中SYS_STAT位掩码定义];
    C --> D[验证修复后的位定义是否与数据手册一致];
    D --> E{用户是否同意计划?};
    E -- 是 --> F[询问是否写入Markdown文件];
    F --> G{用户是否同意写入?};
    G -- 是 --> H[写入Markdown文件];
    G -- 否 --> I[准备切换模式];
    H --> I;
    I --> J[请求切换到Code模式];
    E -- 否 --> A;