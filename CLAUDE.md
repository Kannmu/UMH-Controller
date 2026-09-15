# UMH V7

- 四颗 WS2812 并联在同一根 DI 线上，FPGA 发送一组 24 位 GRB 数据，四颗灯始终显示相同颜色和亮度。
- _RGB_DATA 就是 RGB_DATA，在两块板之间通过排针连接到了一起。
- 只维护可运行的开发调试工程，不创建额外指南文件。
- 不要使用Computer Use，全部通过CLI命令行工具来进行。
- 不要使用SubAgents

ARM_GCC_PATH = D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN
OPENOCD = D:\Software\OpenOCD\bin\openocd.exe

## FPGA Diamond 构建

FPGA 使用 Lattice Diamond 3.13，目标器件为 `LCMXO2-2000HC-4MG132C`。固定入口是 `FPGA/run_diamond.cmd`，不要直接运行 `synthesis.exe`，也不要使用 `pnmainc -batch`。脚本会设置 `FOUNDRY`、`LSC_DIAMOND`、`PATH` 和 `LM_LICENSE_FILE`，调用 `build_full.tcl`，并将完整日志写入 `FPGA/UMH_7_1/pnmainc.log`。

```powershell
Set-Location 'D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\FPGA'
cmd /d /c run_diamond.cmd
```

Tcl 流程依次执行 Synthesis、Map、PAR、Bitgen 和 Jedecgen。成功标准同时包括：脚本返回码为 0，日志含 `UMH_7_1 Diamond build completed`、`PAR_SUMMARY::Run status = Completed`、`PAR_SUMMARY::Number of errors = 0`，并生成最新的 `.ncd`、`.par`、`.twr`、`.bit`、`.jed` 文件。编程文件位于 `FPGA/UMH_7_1/UMH_7_UMH_7_1.bit` 和 `FPGA/UMH_7_1/UMH_7_UMH_7_1.jed`。

中断构建后若 `.ngd` 或 `.ncd` 被锁定，先关闭 Diamond 及其子进程，再使用清理模式：

```powershell
Set-Location 'D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\FPGA'
$env:CLEAN_DIAMOND = '1'; cmd /d /c run_diamond.cmd; Remove-Item Env:CLEAN_DIAMOND
```

清理模式只删除 `UMH_7_1/` 下的生成实现文件，不删除 RTL、约束或工程文件。若仍失败，检查 `pnmainc.log` 最后一条错误以及 `.mrp`、`.twr` 报告。一次只运行一个 Diamond 实例。PLL 必须保持 `FEEDBK_PATH("INT_DIVA")` 和相位匹配的 `CLKOP_CPHASE`，源时钟约束保留在 `UMH_7.lpf`。
