# UMH V7

- 四颗 WS2812 并联在同一根 DI 线上，FPGA 发送一组 24 位 GRB 数据，四颗灯始终显示相同颜色和亮度。
- `motion_engine` 是实时焦点运动生产者：只允许在 render task 中 service，路径点/配置在协议任务中用互斥锁写入；它必须继续使用 `spatial_renderer` 和 `fpga_link_submit`，不要另写一套相位、载波或 SPI 逻辑。运动输出与块播放、Demo、校准和聚焦 AM 互斥；`MOTION_START` 可以清理普通计划后接管，其他生产者看到 `motion_engine_owns_output` 必须返回 BUSY。
- `frame_ring` 的 LOOP_RAM 快照就地复用同一个 slots 池（`loop_start_index` + `loop_iteration_last`），不要再恢复第二份 `loop_slots` 拷贝；生产者由 plan.running 和 BLOCK_BEGIN 检查锁在外面。
- _RGB_DATA 就是 RGB_DATA，在两块板之间通过排针连接到了一起。
- 只维护可运行的开发调试工程，不创建额外指南文件。
- 不要使用Computer Use，全部通过CLI命令行工具来进行。
- 除非特殊情况，否则代码中不写注释，让极高质量的代码本身就作为注释的效果。
- PA9 是外接的开关接口，用于连接一个扳机开关，来实现将设备当成一个“超声枪”来使用，松开就停止超声发射。PA9 必须始终是 GPIO 输入 + 内部上拉（`TRIGGER` 经 22 Ω 的 R176 串到 SMA 中心脚 U174.5，网络两端没有外部上拉）：按键对地短路即按下，任何对 ODR 写 0 都会把上拉变成下拉，引脚会读成常按。**上电初始模式为模式 1：阵列轴线上全功率单焦点**（`UMH_GUN_FOCUS_UM` = 250 mm，见 `gun_trigger.h`）。发射只走 `gun_trigger` + `spatial_renderer` + `fpga_link`：模式 1 填一个 `umh_spatial_point_t{x=y=0, z=UMH_GUN_FOCUS_UM+阵元 z(7000), level=255, phase=0}`，调用 `spatial_renderer_accumulate_point()` 再 `spatial_renderer_finalize`，即命令相位 = 该路校准修正 − 该路到焦点的传播相位，每路 `level=128`（50% 占空比才是满幅度；总发射功率与平面波相同）；不要另写包络或 SPI 逻辑，也不要退回 `level=240` 之类的写法（93% 直流几乎没有 40 kHz 分量）。焦点距离由阵列孔径决定，不要随手改：84 路六角孔径在 40 kHz 的自然焦距是 279 mm，`python Utiles/umh_focus_plan.py` 用实测阵元坐标做瑞利积分给出全套数字（250 mm 时 200–350 mm 目标都在最佳值 0.3 dB 内，焦深约 300 mm；1 m 处聚焦只比平面波高 0.3 dB 却比 250 mm 低 12 dB，所以 `(0,0,1m)` 是差选择）。按下时接管输出（取消块流、停计划、中止运动/全息/调制孔径、`STOP` 后提交一帧），松开即 `STOP` 回到默认静音；按住期间 `BLOCK_BEGIN/SET_PLAN/START_PLAN/SET_DEMO/MOTION_START/HOLOGRAM_START/MOD_CONFIGURE/CAL_*` 返回 BUSY，校准会话运行期间按下推迟到会话结束。状态位 bit10 `UMH_SYSTEM_TRIGGER` 是去抖后的按键电平，`PLAYING` 表示正在发射。四颗 WS2812 的**蓝色**亮度必须等于当前发射强度的指示灯（`frame.rgb` 由 `gun_trigger` 写入，`255 * level / 128`，关断时为 0），不要再单独调 `fpga_link_set_ws2812`，否则会覆盖电平。
- **超声枪菜单（6 个发射模式）**：长按最后一个键（`INPUT_KEY3`，即返回键）3 s 进入/退出枪菜单（`DEVICE_GUI_GUN`），菜单内每行一个模式、短按返回键循环选择（到末行回卷到第一行）、PA9 按当前行发射；退出菜单保留当前 `gun_trigger.mode`；上电初始值仍为模式 1。`input_events.c` 的 `INPUT_LONG_PRESS_MS=3000`，长按在**按住期间**就上报 `INPUT_EVENT_LONG`（不再等松开），随后的松开不再产生事件，所以一次长按 = 一次切换、其后的短按 = 一次模式切换。注意 KEY0 和 KEY3 的 PRESS 都延迟到松开才上报（short_pending），避免长按刚开始时的短按事件先做一次页面跳转、把随后的 LONG 变成退出（从 HOME 会跳到 GUN 页再退出，表现为长按无变化）。模式 2–5 是纯相位场，全部在 `gun_trigger_build()` 里按 1/1024 周转（q10）算每路相位后交给 `spatial_renderer_accumulate_q10()` + `spatial_renderer_finalize()`，每路幅度都是满量程 1.0（→ wire `level=128`），所以模式 1–5 总发射功率相同、只有波前不同：2=平面波（只留校准相位，**不加涡旋缠绕**）；3=聚焦 l=1 涡旋（球面会聚 + 方位角缠绕一整圈，250 mm 处 40 mm 暗核 / 80 mm 亮环 / −55 dB 轴上零点）；4=l=1 贝塞尔涡旋（`theta − k·rho·sin(beta)`，`UMH_GUN_BESSEL_BETA_DEG=1.35` 由 J1 首零点定）；5=动态瓶（同 3 的定律，但腰深从 600 mm 平滑（smoothstep）收缩到 100 mm、耗时 2 s，之后一直保持到松开；腰平面内暗核直径 300 mm → 27 mm，轴上零点全程 −59…−63 dB）。6=语音模式：按住 PA9 静音采集四麦，最多 2 s、8 kHz/8-bit，松开经校准平面波 AM 回放。瓶模式由 render task 每 `UMH_GUN_BOTTLE_PERIOD_MS=10 ms` 重建一次（`gun_trigger_refresh`，用 `base_prev[]` 回滚失败的重建），字段不变时靠 `fpga_link_submit_allow_hold()` 抑制 SPI 帧；其余模式是静态图，只在 keep-alive 时重发。模式 2–5 的相位律用 `python Utiles/umh_gun_phase_check.py` 与 `Utiles/umh_gun_modes.py` 对齐验证（每个模式误差 < 1 个 wire LSB），场形用 `python Utiles/umh_gun_modes.py` / `--mode N` 复算；**不要**把模式 3–5 的方位角缠绕加到模式 2 上（那就变成涡旋束而不是平面波），也不要用“两个贝塞尔叠加”去凑瓶径（已试过，环半径会随焦距单调跑掉，只有 l=1 零点才是可控量）。
- 模式 1–5 的强度只有一个量：`umh_gun_trigger_t.level`，`0`=静音、`UMH_GUN_LEVEL(128)`=满功率，**按下/松开都是硬边沿：没有渐变、没有包络状态机、没有释放看门狗**。别再把 `energy`/`slew`/`ramp_start_ms`/`envelope_state`/`refresh_pending`/`releasing` 之类的斜坡机制加回来。电平必须每次从 `base_level[]` 重算写进帧，禁止就地缩放 `frame.channels[i].level`（render task 按住期间每 ~1 ms 就 `gun_trigger_refresh` 一次，就地缩放会把电平连乘到 uint8 截断归零，表现为“灯亮但完全没有超声”）；`base_ready` 同时充当“接管已 emit”的判据（`gun_trigger_holds_image`），`claim_output` 清 `level`/`level_sent`/`base_ready`，保证接管后第一帧一定是重建过的场图。`gun_trigger_release` 先 `mute`（清 `engaged`、level 归 0、`safe_stop`，并复位 `collapse_t0_ms`/`render_ms` 让下次按下重新从宽端收缩）再返回；render task 的 `refresh` 在提交后复查 `engaged` 并补一次 `safe_stop`，防止并发的帧把 STOP 顶掉。`UMH_GUN_DEBOUNCE_SAMPLES 3`（30 ms）保留，防止接触抖动写出连串边沿。台架验证：`python Utiles/umh_gun_probe.py --watch 40`（SWD 读 `gun_trigger`，地址由 `arm-none-eabi-nm` 从 ELF 解析，打印模式/电平/`fires`/通道极值/蓝色 LED 亮度），离线复算用 `python Utiles/umh_gun_phase_check.py`（相位律）与 `python Utiles/umh_gun_modes.py`（场形）。探针的结构体偏移由 `gun_trigger.c` 顶部的 `_Static_assert` 钉死（`level`=4、`mode`=5、`base_ready`=104、`base_prev`=106、`fires`=276、`last_submit_ms`=284、`sequence`=288、`frame`=292、`frame.channels[0].level`=313、`frame.rgb[0].blue`=482），改结构体会编译报错而不是静默读错地址；改完结构体要同步 `Utiles/umh_gun_probe.py` 里的同名常量。

ARM_GCC_PATH = D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN
OPENOCD = D:\Software\OpenOCD\bin\openocd.exe

## STM32 烧录与"改了代码没变化"的排查

- **只烧 `build/UMH_Controller.elf`（或同目录的 `.bin`/`.hex`）**。`build/debug/` 和 `build/release/` 是历史构建产物，里面的 ELF 是旧镜像；IDE/脚本按目录名挑文件时很容易挑到它们，症状就是"改了代码但行为完全没变"。
  `D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN\arm-none-eabi-gdb` 或 OpenOCD 都可以：
  `openocd -f openocd.cfg -c "program build/UMH_Controller.elf verify reset exit"`。
- **烧完先确认板子跑的是新镜像**：`python Utiles/umh_gun_probe.py` 会先读 `gun_trigger_build_id_seen`（由 `gun_trigger_build_id()` 在 `gun_trigger_init()` 里写入），打印 `gun build id ... -> OK/MISMATCH`。枪状态的每个字段上电都是 0，**旧镜像读出来就是一个"看起来很正常"的空闲枪**，光看模式/电平分辨不出新旧，只有这个 id 可以。改枪相关代码时把 `UMH_GUN_BUILD_ID` 加一。
- **ST-Link 被 OpenOCD 占用/枚举丢失时** `init` 报 `Error: open failed`：先 `Get-Process openocd | Stop-Process -Force`，再拔插 ST-Link 的 USB。正常的 `init` 会打印 `STLINK V2J...` 和 `Target voltage`。
- **调试时不要长时间/反复 halt**：反复 halt 会让 IWDG 超时把 MCU 复位，现象是"变量神秘归零、PC 停在 `Reset_Handler`"；在 I2C 传输中途 halt 还会把 OLED 事务打断成 NACK，制造出并不存在的显示故障。
- **读状态用 `arm-none-eabi-gdb` 的 `p <变量名>`（走 DWARF），不要手工算 `nm` 地址**：本项目的 `nm` 与调试信息地址可能不一致（实测 `key_state` nm=0x200149cc、DWARF=0x200149e4），照 `nm` 去读会读到别的变量，从而得出"按键没反应"的错误结论。本项目 `input_events.c` 的静态量当前为：`key_state`=0x200149e4、`long_reported`=0x200149e5、`stable_samples`=0x20014a6c。
- **OLED 刷新不能在 ui_task 里做**：一次页传输是阻塞 I2C，面板不 ACK 时每页要等满 100 ms 超时，一次刷新最多 ~800 ms；这段时间 ui_task 既不处理按键也不换页，表现就是"长按返回键没反应"（长按其实已经被输入任务检测到了）。现在由独立 `display_task` 负责 OLED，`oled_ssd1315_refresh` 连续失败 3 次后退避 `OLED_BACKOFF_MS=2000`；ui_task 只做事件分发和渲染。**不要**把刷新挪回 ui_task。

## 已知硬件事实与固件约束（重要，修改前必读）

- **PLL**：STM32 PA8 输出 8 MHz HSE MCO，FPGA `EHXPLLJ` 固定为 `CLKI_DIV=1`、`CLKFB_DIV=8`、`CLKOP_DIV=8`、`CLKOP_CPHASE=7`、`FEEDBK_PATH="INT_DIVA"`，即 64 MHz 主时钟、512 MHz VCO。`UMH_7.lpf` 中 `pll_clk` 必须约束为 64 MHz。
- **PLL 环路滤波属性不可删除**：`EHXPLLJ` 实例必须带 scuba 为 8→64 MHz 生成的 `FREQUENCY_PIN_CLKI="8.000000"`、`FREQUENCY_PIN_CLKOP="64.000000"`、`ICP_CURRENT="9"`、`LPF_RESISTOR="72"` 综合属性。缺少它们时 Map 报 `Output Clock(P) Frequency: NA`，PLL 实际从未锁定：LOCK 恒低、pll_clk 相对 STM32 HSE 慢约 700 ppm 并有宽带相位游走（载波 PM rms 约 1.5–3 rad），经 LC/换能器谐振转成 AM，就是所有声场都有的"沙沙"白噪声。加属性后 LOCK=1、频差与 HSE 一致、PM rms 降到约 0.02 rad。LOCK 同步后作为状态位 13（0x2000）仅供诊断。
- **LOCK 不可用于输出使能**：`us_tx` 输出只能由 `running` / `stop_event` 控制，禁止加入 `!pll_locked` 门控。
- **声场纯度复测**：`python Utiles/umh_noise_probe.py --cases off,single41_128,all_16 --secs 6 --save x.npz` 后 `python Utiles/umh_am_pm.py x.npz`；PC 麦克风需先用 `Utiles/mic_volume.ps1 -Level 0.02` 降增益防削波，测完恢复 `-Level 1.0`。
- **`level` 语义**：通道 `level` 是 256 个 40 kHz slot 中的高电平 slot 数。`level=0` 关闭；`level=128` 为 50% 占空比；`level=255` 是 99.6% 直流。空间点幅度满量程必须映射到 `level=128`（`spatial_renderer_finalize` 中为 `magnitude * 128.0f`），不能再写回 `* 255.0f`，否则 demo 没有超声、功耗不变。直接 `CHANNEL_STATE` 输入仍把 `level` 原样作为占空比。
- **超声输出提交结构不可回退**：`us_tx` 必须保持"组合 `us_tx_next` + 无条件寄存器"的写法；不要恢复成带 gated enable/异步 clear 的 `if/else` 更新，LSE 在硅上的门控行为曾导致输出不翻转。
- **事件表写入不可回退**：通道掩码必须使用 `ev_ch` 组合译码得到的 `ev_ch_bit`；不要恢复 84-bit one-hot `ev_bit` 移位寄存器，其 LSE 上电/移位行为曾在硅上导致事件表全零。
- **WS2812C-2020-V6 复位时间**：RESET 低电平必须 >=280 us。当前 64 MHz 下使用 20000 个周期 = 312.5 us；位时序为 T0H ~= 312.5 ns、T1H ~= 625 ns、位周期 1.25 us。禁止把 RESET 改短。
- **SPI1 实际时钟为 42.5 MHz**：170 MHz PCLK2 / 4（`UMH_SPI1_LINK_DIV=4`）。`UMH_7.lpf` 中 `spi1_sck` 约束是 42.5 MHz。MISO 必须由 SCK 下降沿的 `status_miso_q` 驱动，禁止恢复 128:1 组合状态 mux；否则 42.5 MHz 下状态位会在 STM32 采样前未建立（实测 flags 0x7800 而非 0x7000）。
- **烧录后必须从 Flash 重新加载**：`pgrcmd -infile UMH_7_Programmer_File.xcf` 之后，再执行 `pgrcmd -infile UMH_7_Programmer_Refresh.xcf`。只做 Erase/Program/Verify 时，运行中的 FPGA 可能仍执行旧镜像，导致"改了代码但症状完全不变"。
- **时序验收标准**：64 MHz 约束下 Diamond/TRACE 必须为 `0 setup errors, 0 hold errors`。不要再用 128 MHz 的 LPF 约束去"通过"构建；64 MHz 是这块板实际可稳定收敛的频点。
- **84 路超声数量**：84 路 `us_tx` 为 40 kHz 数字方波；

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

