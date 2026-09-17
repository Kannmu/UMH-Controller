# UMH V7

- 四颗 WS2812 并联在同一根 DI 线上，FPGA 发送一组 24 位 GRB 数据，四颗灯始终显示相同颜色和亮度。
- _RGB_DATA 就是 RGB_DATA，在两块板之间通过排针连接到了一起。
- 只维护可运行的开发调试工程，不创建额外指南文件。
- 不要使用Computer Use，全部通过CLI命令行工具来进行。
- 不要使用SubAgents
- PA9 是对外输出的触发信号，用于连接LDV等等外部仪器的时序触发，和超声发射部分没有关系。

ARM_GCC_PATH = D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN
OPENOCD = D:\Software\OpenOCD\bin\openocd.exe

## 已知硬件事实与固件约束（重要，修改前必读）

- **PLL**：STM32 PA8 输出 8 MHz HSE MCO，FPGA `EHXPLLJ` 固定为 `CLKI_DIV=1`、`CLKFB_DIV=8`、`CLKOP_DIV=8`、`CLKOP_CPHASE=7`、`FEEDBK_PATH="INT_DIVA"`，即 64 MHz 主时钟、512 MHz VCO。`UMH_7.lpf` 中 `pll_clk` 必须约束为 64 MHz。
- **PLL LOCK 引脚不可用于输出使能**：本板实测 `LOCK` 输出一直为低，但 `fpga_time` 证明 PLL 频率正确且稳定。`us_tx` 输出只能由 `running` / `stop_event` 控制，禁止重新加入 `!pll_locked` 门控，否则 84 路超声会被永久清零。
- **`level` 语义**：通道 `level` 是 256 个 40 kHz slot 中的高电平 slot 数。`level=0` 关闭；`level=128` 为 50% 占空比；`level=255` 是 99.6% 直流。空间点幅度满量程必须映射到 `level=128`（`spatial_renderer_finalize` 中为 `magnitude * 128.0f`），不能再写回 `* 255.0f`，否则 demo 没有超声、功耗不变。直接 `CHANNEL_STATE` 输入仍把 `level` 原样作为占空比。
- **超声输出提交结构不可回退**：`us_tx` 必须保持"组合 `us_tx_next` + 无条件寄存器"的写法；不要恢复成带 gated enable/异步 clear 的 `if/else` 更新，LSE 在硅上的门控行为曾导致输出不翻转。
- **事件表写入不可回退**：通道掩码必须使用 `ev_ch` 组合译码得到的 `ev_ch_bit`；不要恢复 84-bit one-hot `ev_bit` 移位寄存器，其 LSE 上电/移位行为曾在硅上导致事件表全零。
- **WS2812C-2020-V6 复位时间**：RESET 低电平必须 >=280 us。当前 64 MHz 下使用 20000 个周期 = 312.5 us；位时序为 T0H ~= 312.5 ns、T1H ~= 625 ns、位周期 1.25 us。禁止把 RESET 改短。
- **SPI1 实际时钟为 21.25 MHz**：170 MHz PCLK2 / 8。`UMH_7.lpf` 中 `spi1_sck` 约束是 21.25 MHz，不是 42.5 MHz。
- **烧录后必须从 Flash 重新加载**：`pgrcmd -infile UMH_7_Programmer_File.xcf` 之后，再执行 `pgrcmd -infile UMH_7_Programmer_Refresh.xcf`。只做 Erase/Program/Verify 时，运行中的 FPGA 可能仍执行旧镜像，导致"改了代码但症状完全不变"。
- **时序验收标准**：64 MHz 约束下 Diamond/TRACE 必须为 `0 setup errors, 0 hold errors`。不要再用 128 MHz 的 LPF 约束去"通过"构建；64 MHz 是这块板实际可稳定收敛的频点。
- **84 路超声数量与 PA9**：84 路 `us_tx` 为 40 kHz 数字方波；PA9 只是外部仪器触发，与超声发射无关，不要把它接到超声控制逻辑里。

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

## 四麦克风回波自校准（2026-09-16 增加）

- **麦克风**：4 颗 SPH0641LU4H-1，Ultrasonic Mode（fCLOCK 3.072..4.8 MHz）。上电/唤醒必须按定序：**上电后 mic_clk 先保持低 10 ms → 200 kHz 运行 20 ms → 4 MHz**；禁止直接上电到 4 MHz。FPGA 已内置该定序器（`MIC_BOOT_CYCLES` / `MIC_WARM_EDGES`）。
- **PDM 配对与槽位**（U181/U204 SELECT 接 VDD，U180/U182 SELECT 接 GND）：
  - slot 0 = MIC_DATA_0 上升沿（U181），slot 1 = MIC_DATA_0 下降沿（U180）；
  - slot 2 = MIC_DATA_1 上升沿（U204），slot 3 = MIC_DATA_1 下降沿（U182）。
  麦克风坐标由 `Core/Src/us_calibration.c` 的 `cal_mic_*_mm` 按上述槽位顺序维护，不能按编号顺序交换。
- **FPGA 复解调**：4 MHz PDM 流与整数 40 kHz ±1 本振混频，40 点 boxcar（输出率 100 kHz）；本振/混频/boxcar 全为加、异或，不使用乘法器或 ROM。门累积放在 512×16 单 EBR 中（`umh_mic_iq_ram`，1 个 EBR）。
- **新增 SPI1 命令**（`fpga_link.h`）：
  - `0x14 MIC_CONFIG`: six extension bytes `{gate_count, start_lo, start_hi, step_lo, step_hi, width}`; the three time fields are in 40 kHz samples (25 us), matching the FPGA one-carrier-period I/Q integrator.
  - `0x15 MIC_READ`：门号放在 header 的 frame-sequence 字段（字节 6..9），事务长度 40 字节；MISO 前 16 字节为常规 status，随后 24 字节为 `{status, block_count, gate_count, reserved, I0..I3, Q0..Q3}`（16 位大端，见 `fpga_link.h`）。
- **EBR 占用 = 7/8**：事件表 5 + 麦克风 IQ 1 + 通道 staging 1。新增逻辑必须保持该预算，不能再引入 EBR。
- **Calibration flow**: UI CALIB RUN -> 1 s settle -> coarse/fine balanced-random echo survey (FMAC matched-filter leading edge) -> silent floor -> up to 9 s / 2048 i.i.d. +-1 random projections (full-rank R; stream b=R^H y only, never store A=R^H R) -> CG convex reconstruction of the 84 complex channel responses per microphone -> alternating rank-1 + common-mode mu fit and pose correction -> gauge fix -> one-pattern array-gain verification -> AT24C16 write. Only the high byte of record->phase[i] is written; spatial_renderer picks it up automatically, while direct CHANNEL_STATE/BLOCK phase commands are unaffected.
- **EEPROM 记录版本 2**：`eeprom_profile_record_t` 在 `reserved[3]` 后增加 `cal_meta_valid..cal_time_ms`，payload 长度 = `offsetof(crc32)`；旧版本 1 记录在 `eeprom_profile_load()` 中自动升级。commit 流程仍是双副本 + generation + CRC32。
- **烧录后刷新**：`pgrcmd -infile UMH_7_Programmer_File.xcf` 之后必须 `pgrcmd -infile UMH_7_Programmer_Refresh.xcf`；Refresh XCF 已补齐。
- **Calibration projections**: `cal_cs_row()` uses splitmix64 i.i.d. +-1 (no exact 42/42 balancing) so the 84-dimensional measurement matrix is full rank and A=C^H C is well conditioned; `cal_balanced_row()` (42/42 zero-mean) is used only by the echo survey to cancel common-mode LC ring-down. The final static-phase rank-1 fit carries a per-microphone common-mode nuisance mu (`cal_mu_*`) and alternates "rank-1+mu fit -> subtract mu -> pose refinement", so residual common-mode LC ring cannot corrupt the channel phases. The code matrix is regenerated from (pattern,channel) at acquisition and reconstruction time and is never stored. Do not revert to the old Paley/truncated-Hadamard scheme.

