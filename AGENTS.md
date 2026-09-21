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

## 内置近场耦合相位自校准（2026-09-17 v2）

- **目标**: 只把 84 路在麦克风声孔处看到的静态相位基准校准到一致，不求解绝对相位、不依赖环境反射面。四颗 SPH0641 接收 84 路阵元的平面内近场直达声；门放在每个发射 burst 的稳态内部，15 cm 以外反射来不及到达。
- **物理坐标**: 换能器 GU1008C-40TR 的压电陶瓷在 PCB 麦克风声孔平面之上 7.0 mm；Core/Src/device_profile.c 因此把 84 路 z_um 设为 7000。麦克风声孔仍取钻孔坐标表: slot0=U181(-43.305,24.994)、slot1=U180(43.298,24.994)、slot2=U204(0,0)、slot3=U182(-0.004,-49.994)。不得退回全 z=0。
- **采集**: cal_cs_row 使用 splitmix64 i.i.d. ±1，不要改成 Paley/均衡 H。每个图案在 FPGA pattern swap 后按 MIC_CONFIG(gate_count=1,start=gate_start,step=gate_width,width=16) 在 burst 稳态内取一个 400 us I/Q 门；C 阶段累计 Sy_m、Sk_i、Sxy_mi，用中心化最小二乘 + 复数 CG 解 84x4 的 Z_mi。矩阵按 pattern index 现场重生成，不存 R^H R。
- **发射档位**: A1 使用 5 档 {128,64,32,16,8}，按幅度-电平线性度、单图案重复相干性和 SNR 选择档位。本硬件近场耦合很强，正常工作点常在 level=16/8，不要强行固定 128。
- **门自整定**: B 阶段发 8 次长 burst，MIC_CONFIG(64,0,step=4,width=2) 覆盖 0..6.4 ms；取 burst 尾段稳定窗，幅度距 tail 均值不超过 15%，相邻相位差不超过 5 度，至少 6 个 profile gate，避免把 LC 起振瞬态当稳态。
- **相位拟合**: E 阶段对 Z_mi 乘 exp(±j*k*r_mi) 做逐麦克风共模 mu 加 rank-1 a_i*rho_m 交替加权拟合。cal_fit_direct 当前为 24 轮乘 8 次迭代，共 192 次，并在每轮更新 mu；早期 32 次迭代版本会停在约 20 度 RMS，不要退回。拟合后由 cal_gauge 去掉公共相位和平面项，得到修正字节约 q=负 arg(a_i)。
- **四候选消歧**: 直接枚举 {解旋+ 乘 q, 解旋+ 乘 -q, 解旋- 乘 q, 解旋- 乘 -q}。F 阶段用生产 spatial_renderer_point 分别聚焦到四个麦克风声孔，实测 4 颗接收功率；所有候选都不优于基线才失败。rms_before 小于 20 度时不强制 6 dB 增益和 3 麦正增益，否则按严格门限。
- **原始数据回放**: UMH_MSG_CAL_RAW=0x83 可只做 C 阶段发射采集并把原始 [pattern][mic][I,Q] int16 按 64 图案每帧流式回传到 PC，用于离线算法迭代。参数 8 字节: level u8, gate_start u16, gate_width u8, burst_us u16, patterns u16。离线工具: python Utiles/umh_cal_dump_check.py --raw Utiles/cal_capture/umh_raw_level16_gate232.bin --patterns 1024 --sign both。
- **EEPROM v3**: EEPROM_PROFILE_VERSION=3，布局仍与 v2 相同，phase 高字节为修正。cal_meta_valid=1，cal_level 为 level_used，cal_rms_deg_x10 为 fit_rms；cal_tilt_x_x10 槽位存 verify_gain_db 乘 10，cal_tilt_y_x10 存 mic_consistency 乘 10；cal_reserved 存 patterns、gate_start、gate_width、level、geom、sign、band_trend。
- **调试命令**: UMH_MSG_CAL_START=0x82 用于 CLI 触发一次正常校准，等价于 GUI 的 CALIB RUN。UMH_MSG_CAL_DUMP=0x80 支持 6/7 字节请求，section 0 为重构 Z float32、section 1 为 B 剖面 int16、section 2 为拟合和候选指标。
- **不退回**: 不要恢复平面回波 pose 搜索、FISTA/L1 分支、旧 Paley 码、z=0、32 次 rank 迭代或只验证一种候选；FPGA RTL 不动。
