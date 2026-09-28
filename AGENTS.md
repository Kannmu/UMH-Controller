# UMH V7

- 四颗 WS2812 并联在同一根 DI 线上，FPGA 发送一组 24 位 GRB 数据，四颗灯始终显示相同颜色和亮度。
- `motion_engine` 是实时焦点运动生产者：只允许在 render task 中 service，路径点/配置在协议任务中用互斥锁写入；它必须继续使用 `spatial_renderer` 和 `fpga_link_submit`，不要另写一套相位、载波或 SPI 逻辑。运动输出与块播放、Demo、校准和聚焦 AM 互斥；`MOTION_START` 可以清理普通计划后接管，其他生产者看到 `motion_engine_owns_output` 必须返回 BUSY。
- `frame_ring` 的 LOOP_RAM 快照就地复用同一个 slots 池（`loop_start_index` + `loop_iteration_last`），不要再恢复第二份 `loop_slots` 拷贝；生产者由 plan.running 和 BLOCK_BEGIN 检查锁在外面。
- _RGB_DATA 就是 RGB_DATA，在两块板之间通过排针连接到了一起。
- 只维护可运行的开发调试工程，不创建额外指南文件。
- 不要使用Computer Use，全部通过CLI命令行工具来进行。
- 不要使用SubAgents
- PA9 是对外输出的触发信号，用于连接LDV等等外部仪器的时序触发，和超声发射部分没有关系。
- 暗核涡旋（trap_mode 6）的聚焦深度就是指令 z；阵列朝桌面时小球高度由桌面钉住的驻波台阶决定，聚焦深度不能连续移动小球高度。GUI 悬浮必须保持 `UMH_LEVITATION_TRAP_Z_UM=100000`。主机运动消息只在涡旋消泡程序运行时 BUSY（`UMH_SYSTEM_VORTEX_MODE_MASK`），不要恢复成对悬浮也 BUSY，否则上位机无法不掉球接管。

ARM_GCC_PATH = D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN
OPENOCD = D:\Software\OpenOCD\bin\openocd.exe

## 已知硬件事实与固件约束（重要，修改前必读）

- **PLL**：STM32 PA8 输出 8 MHz HSE MCO，FPGA `EHXPLLJ` 固定为 `CLKI_DIV=1`、`CLKFB_DIV=8`、`CLKOP_DIV=8`、`CLKOP_CPHASE=7`、`FEEDBK_PATH="INT_DIVA"`，即 64 MHz 主时钟、512 MHz VCO。`UMH_7.lpf` 中 `pll_clk` 必须约束为 64 MHz。
- **PLL 环路滤波属性不可删除**：`EHXPLLJ` 实例必须带 scuba 为 8→64 MHz 生成的 `FREQUENCY_PIN_CLKI="8.000000"`、`FREQUENCY_PIN_CLKOP="64.000000"`、`ICP_CURRENT="9"`、`LPF_RESISTOR="72"` 综合属性。缺少它们时 Map 报 `Output Clock(P) Frequency: NA`，PLL 实际从未锁定：LOCK 恒低、pll_clk 相对 STM32 HSE 慢约 700 ppm 并有宽带相位游走（载波 PM rms 约 1.5–3 rad），经 LC/换能器谐振转成 AM，就是所有声场都有的"沙沙"白噪声。加属性后 LOCK=1、频差与 HSE 一致、PM rms 降到约 0.02 rad。LOCK 同步后作为状态位 13（0x2000）仅供诊断。
- **LOCK 不可用于输出使能**：`us_tx` 输出只能由 `running` / `stop_event` 控制，禁止加入 `!pll_locked` 门控。
- **声场纯度复测**：`python Utiles/umh_noise_probe.py --cases off,single41_128,all_16 --secs 6 --save x.npz` 后 `python Utiles/umh_am_pm.py x.npz`；PC 麦克风需先用 `Utiles/mic_volume.ps1 -Level 0.02` 降增益防削波，测完恢复 `-Level 1.0`。
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

## 相位自校准：轴向桌面回波（2026-09-28 v3，生产路径）

- **目标**: 只校准 84 路之间的相对静态相位（公共相位、平面项、x²+y² 项都被 gauge 掉），并检测坏通道。阵列朝桌面，桌面距发射面 5..25 cm（实测 93 mm），只需阵列正下方约 10 cm 直径的中心区域空旷；侧面支架腿等障碍在时间窗外，不影响结果。CAL_START(0x82)、OLED CALIB RUN 和上位机校准按钮都走 `us_calibration_run` → `cal_echo_run`。
- **为什么不用近场直达声**: 平面内近场耦合是 60..90° 离轴方向，实测其相位与轴向发射相位相差 75..84° RMS（所有符号组合），不能代表阵列实际声场。旧近场 EEPROM 图在真实聚焦上比零校准差 2.9 dB（40 组里 38 组更差）。近场拟合只保留为诊断函数 `us_calibration_nearfield_run`，结果永不写 EEPROM；不要把它接回生产路径。
- **物理坐标**: 换能器压电陶瓷在 PCB 麦克风声孔平面之上 7.0 mm，device_profile.c 的 z_um=7000，不得退回 z=0。麦克风声孔: slot0=U181(-43.305,24.994)、slot1=U180(43.298,24.994)、slot2=U204(0,0)、slot3=U182(-0.004,-49.994)（身份映射经置换/镜像测试确认最佳）。
- **A 桌面距离**: 半径 25..35 mm 的前 6 个通道单独发 level 16，`us_calibration_probe` 64×25 us 门（200..1775 us）、diff、4 次；对 rd≥25 mm 的对取回波台阶 t50（偏离前 3 门均值达峰值一半），固定检测延迟 95 us，由 r_e=sqrt(h²+(2V+7)²) 反解 V 取中位数。V 须在 40..250 mm，否则 CAL_Q_GEOM 失败（没有反射面）。
- **B 单通道回波**: 每通道单独 level 16（单通道回波线性；level≥48 起麦克风压缩、相位偏移），16 门×50 us 从 t_on-350 us 起，8 次 diff 重复，settle 20 ms，约 45 s。基线 = t_on-300..-50 us 的近场稳态，回波 = t_on+100..min(+400, 0.8×二次回波间隔)；d=回波-基线。窗口位置在 +25..+800 us 内移动时静态相位变化仅 3..5°。|基线| ≥ 0.6|回波| 的对剔除。
- **C 求解**: u=d·exp(-jk r_e)/|d|，只用 h<60 mm（出射角 <17°）且 rd>14 mm 的对：18..25° 以外阵元指向性有通道相关的零点和 ±180° 翻转，曾把拟合拉到 40° RMS。u_mi=rho_m·a_i 交替投影 30 次；gauge 用 (1,x,y,x²+y²) 加权最小二乘并排除 |ph|≥90° 的大故障通道，x²+y² 吸收桌面距离误差（V 偏 ±8 mm 结果只变 2..4°），平面项作为桌面倾角诊断。然后按每通道标准误做 Wiener 收缩 q=-max(0,1-σ²/(n·ph²))·ph：噪声级差异保持 0，只修显著偏差（如 41/51/79 约 +50..65°）。离线注入 90° RMS + 3 个翻转可恢复到 16.5° 内。
- **幅度/坏通道**: 各通道最大 |d| < 0.15×中位数判为坏通道（ch61 实测无输出），EEPROM enabled 清零、相位写 0。增益不均衡：实测 p10/p90≈0.83/1.16，与 level 8/16 重复性（12..15%）同量级，均衡只会降低总输出；gain 保持原值，相对幅度只在 CAL_RESULT 里报告。
- **质量门与验证**: 覆盖 ≥70 通道、≥3 麦各 ≥10 对、pair sd ≤35°、坏通道 ≤8。然后用生产 `spatial_renderer_point` 聚焦到每个麦克风的桌面镜像点（z=2V+14 mm），每麦 4 组×3 个锥内通道 level 8，候选与零校准交替实测回波幅度；平均增益 < -0.5 dB 则 CAL_Q_VERIFY 失败、不写 EEPROM。通过即覆盖已存校准（不再有“已有有效校准就不写”的规则）。
- **实测（桌面 93 mm）**: 连续 4 次运行 V=93.2..93.4 mm，相位字节互差 4..5° RMS（最大 18°），静态 RMS 约 18°，施加修正约 14° RMS。独立主机聚焦检查（`python Utiles/umh_focus_check.py`）：随机 4 通道 -0.03 dB、8 通道 +0.28 dB、含 41/51/79 的组 +0.68 dB（24/36 更好）；旧近场图为 -2.9 dB。结论：本板静态相位本来就较一致，校准主要价值是抓大偏差/坏通道并保证不引入噪声。
- **SPI 降速不可退回**: 校准 session 必须调用 `fpga_link_calibration_link_begin/end`，把 SPI1 从 /8 降到 /32 再恢复；FRAME/MIC_CONFIG/MIC_READ 都要在降速期间发送。
- **CAL_RESULT(0x81)**: 前 135 字节兼容旧布局，含义改为: reserved=当前 cal_state（device_gui_cal_state_t，5=OK，6=FAIL），progress 在运行中实时更新；rms_before=gauge 后静态 RMS，rms_after/mic_consistency=pair sd，residual=饱和块数，distance_m=V(m)，tilt_x/y=桌面倾角(°)，echo_ratio=回波幅度中位数，mic_ratio=幅度 p90/p10(dB)，verify_gain_db=聚焦验证增益。其后追加 20 字节 correction_rms、amp_p10、amp_p90(float)、pairs u16、covered、dead、verify_wins、verify_sets、committed、method(2=桌面回波) 和 amplitude[84]（128=中位数）、coverage[84]（锥内对数，0xFF=坏通道），共 323 字节。CAL_START 时 progress 清 0，主机轮询到 progress=100 且 state∈{5,6} 即完成。
- **EEPROM v3**: 布局不变。phase 高字节为修正；cal_level=16；cal_rms_deg_x10=pair sd；cal_tilt_x_x10=verify_gain×10；cal_tilt_y_x10=correction_rms×10；cal_reserved: [0..1] 锥内对数，[2..4] SELFTEST 近场门固定 20/16，[5] level，[6] method=2，[7] 验证胜出组数，[8..9] V×10，[10] 坏通道数，[11] B 门起点。
- **工具**: `python Utiles/umh_cal_run.py --runs 3`（触发并打印/比较结果），`python Utiles/umh_focus_check.py`（独立聚焦检查），`python Utiles/umh_echo_cal_ref.py --scan <npz>`（与固件同算法的离线参考），`python Utiles/umh_echo_scan.py`（84 通道 64 门剖面采集），`python Utiles/umh_eeprom_phase_zero.py`（清零 EEPROM 相位）。UMH_MSG_CAL_PROBE=0x86 为台架时间剖面探针。
- **不退回**: 不要恢复近场相位作为生产校准、平面回波 pose 搜索、FISTA/L1、Paley 码、z=0；不要用 level≥48 或多通道同时发射做单通道相位测量；不要去掉锥角限制或 Wiener 收缩；FPGA RTL 不动。
