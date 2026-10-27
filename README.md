# UMH Controller v7

面向空气中 84 路超声相控阵设备的 STM32G491KCU 固件，当前版本 `v7.5.1`，USB 协议版本 7。协议本身只表达设备能力、时间、空间、通道和外设状态；设备侧另有若干本机生产者（Demo、实时焦点运动、全息关键帧、调制孔径、PA9 超声枪和 USB 音频），它们全部复用同一套空间渲染、校准、帧环和 FPGA 提交路径，不存在第二套相位、载波或 SPI 实现。实时帧只放在静态 RAM 和 FPGA FIFO 中，NOR 只保存显式上传的对象和序列；协议与输出路径不分配动态堆。

字段级定义见 `Docs/UMH_v7_Protocol.md`、`Docs/UMH_v7_FPGA_Interface.md` 和 `Docs/UMH_v7_Storage.md`；FPGA 侧说明见 `FPGA/README.md`。**文档与代码不一致时以代码为准**，已知差异列在文末。

## 硬件

* **STM32G491KCU**：170 MHz（8 MHz HSE × PLL），256 KB Flash，96 KB SRAM1 + 16 KB CCM（共 112 KB），FreeRTOS 静态任务模型，`configTOTAL_HEAP_SIZE` 仅 128 B。
* **LCMXO2-2000HC-4MG132C**：8 MHz HSE 经 PA8 MCO 进入 `EHXPLLJ`，生成 64 MHz 主时钟（512 MHz VCO）；`UMH_7.lpf` 按 64 MHz 约束。PLL 属性（`FREQUENCY_PIN_*`、`ICP_CURRENT`、`LPF_RESISTOR`）不可删除，否则 PLL 实际不锁定并产生宽带相位噪声。LOCK 不参与输出使能，输出只由 `running`/`stop_event` 控制。
* **84 路发射**：`GU1008C-40TR` + `DRV8220DSGR` + 2.2 mH 网络，`us_tx` 为 40 kHz 数字方波。通道 `level` 是 256 个载波 slot 中的高电平数：`0` 关闭、`128` 为 50% 占空比（空间满幅度）、`255` 为 99.6% 直流。空间渲染器把幅度 1.0 映射到 `level=128`，不要再改回 `255`。
* **4 颗 SPH0641LU4H-1**：共用 `MIC_CLK`，分布在两条 PDM 数据线上，由 FPGA 采样；原始流经 `SPI_MIC_*` 送到 CH347T，再由 CH347T 作为独立 USB 设备上传，不占用 STM32 控制链路。
* **4 颗 WS2812C-2020-V6**：并联在同一根 `rgb_data` 上，FPGA 只发一组 24 位 GRB，四颗灯始终同色同亮度。RESET 低电平必须 ≥280 µs，当前 64 MHz 下为 20000 周期 = 312.5 µs。
* **外置 NOR**：`W25Q128`（16 MiB）挂在 SPI3（PB3/PB4/PB5）。
* **I2C1**：`SSD1315` OLED（128×64，地址 0x78）与 `AT24C16` EEPROM（2 KiB，地址 0x50）共享总线，由互斥锁和总线恢复逻辑保护。
* **SPI1**：8 bit、CPOL=0、CPHA=0、软件片选（PA4），42.5 MHz（170 MHz PCLK2 / 4）。完整帧 36 字节头 + 168 字节通道 + 12 字节 RGB = 216 字节，约 40.7 µs；相位帧 36 + 84 = 120 字节。
* **PA9 / TRIGGER**：扳机输入，内部上拉；按键经 22 Ω 的 R176 串到 SMA 中心脚，按下对地短路。任何对 ODR 写 0 都会把上拉变成下拉，因此始终保持 GPIO 输入 + 上拉。
* 两块板之间的 `_RGB_DATA` 是同一个网络，通过排针连通。
* 硬件资料中 FPGA 料号（BOM `LCMXO2-2000HC-4MG132C`）与网表封装字符串（`LCMXO640C-4MN132I`）仍不一致，生产前必须复核；84 路阵元坐标已改为实测值，见下文。

## 软件结构

| 模块 | 职责 |
| --- | --- |
| `device_profile` | 通道数、阵元坐标、载波、时间基准、FPGA FIFO、RAM 额度、能力位和自校准代数；`GET_PROFILE` 的 1118 字节线上结构 |
| `spatiotemporal_block` / `block_parser` | 可组合的块头、轨道描述符和记录解析；把时间线编译成 84 路帧 |
| `spatial_renderer` | 用实测阵元坐标、介质声速和 EEPROM 校准把空间点合成为 84 路相位与幅度（q10 相位、1/1024 转） |
| `frame_ring` / `playback_plan` | 24 个 232 字节静态帧槽；计划启动、重复、欠载策略和 `LOOP_RAM` 快照 |
| `fpga_link` | SPI1 DMA 事务、序号/截止时间/FIFO 信用、相位/电平保持优化和安全 STOP |
| `demo_engine` | 3 个内建 Demo（ULM/LML/LMC），编译成 24 帧后按 `LOOP_RAM` 播放 |
| `motion_engine` | 实时焦点运动生产者：路径或实时目标、双焦点/环/涡旋/双环、调色板和 ULM |
| `hologram_engine` | 设备端全息关键帧执行器：在 84 路完整图像之间插值 |
| `aperture_mod` | 调制孔径：固定 84 路相位 + 20 kHz 公共幅度包络（聚焦 AM、UAC、枪语音共用） |
| `gun_trigger` | PA9 超声枪：6 种发射模式、枪菜单、蓝色 LED 强度指示 |
| `usb_audio` / `usb_audio_dsp` | UAC1 自适应单声道播放：48 kHz → 20 kHz 包络 DSP |
| `us_calibration` | 台面回波自校准、自检、原始 I/Q 采集和探针 |
| `flash_nor` / `flash_store` / `flash_library` | NOR 驱动、低频对象库和只读序列库 |
| `eeprom_profile` / `i2c_bus` / `oled_ssd1315` | 双副本校准记录、I2C1 互斥与恢复、OLED 页刷新 |
| `device_gui` / `input_events` / `system_status` / `task_stats` | 10 页 OLED 界面、按键事件、故障/状态快照和任务 CPU 统计 |

任务全部静态创建，共 9 个（含 IDLE）：

| 任务 | 优先级 | 职责 |
| --- | --- | --- |
| `render` | Realtime | 最高优先级；按计划提交帧、驱动 motion/hologram/gun/调制孔径，并维护 20 kHz 包络 |
| `protocol` | High | USB 接收、协议解析、块编译、输出接管和 UAC 供数 |
| `storage` | AboveNormal | NOR 擦写、对象库、序列库上传/回放泵 |
| `cal` | Normal | 自校准、自检、`CAL_RAW`/`CAL_PROBE` 和 SWD 台架邮箱 |
| `input` | Normal | 10 ms 采样 4 个按键和 PA9 |
| `ui` | BelowNormal | 事件分发和 GUI 渲染 |
| `disp` | BelowNormal | 独立 OLED 刷新（阻塞 I2C 不能放在 `ui` 里） |
| `health` | Low | 心跳、运行时间、4 Hz 任务统计快照 |

## 播放链路

`USB CDC -> ProtocolTask -> BlockParser(协议任务内) -> FrameRing -> RenderTask -> FPGA Link`。

USB 回调只把字节复制进 4 KiB 单生产者环形缓冲并通知协议任务；响应池为 2 个 2064 B 静态槽。协议任务校验 16 字节头（`0x55 0xAA`、版本 7、长度 ≤2048），在协议任务上下文里完成块解析和空间解算，把完整帧写入帧环；渲染任务只从帧环取帧，按计划截止时间提交 FPGA。这样块解析不会和实时输出抢栈，也保证 `LOOP_RAM` 的快照在生产者被锁在外时完成。

`PlaybackPlan` 为 32 字节：块 ID、开始方式（立即/设备时间/触发/下一帧边界）、重复方式（单次/`LOOP_RAM`/`LOOP_STREAM`/保持/停止）、欠载策略（保持/关闭/仅报告）、路由（当前必须为 0）、有理数速率和预缓冲。`ONCE` 和 `HOLD_LAST` 在最后一帧后停止调度但保留 FPGA 最后状态；`STOP` 策略、`STOP_PLAN`、`CLEAR_PLAN` 才会主动关闭输出。`LOOP_RAM` 直接复用帧环同一组 slots（`loop_start_index` + `loop_iteration_last`），没有第二份帧池。

`SET_DEMO=0x61` 的载荷是 Demo ID（`0=ULM`、`1=LML`、`2=LMC`）：设备以 `(0, 0, 100 mm)` 为中心生成 24 帧轨迹（ULM 30 mm 单向扫、LML 15 mm 端到端往返、LMC 半径 4.775 mm 整圆），整条轨迹 5 ms（200 Hz）循环，按 `LOOP_RAM` 播放；Demo 会先停止计划并清空块流，被其他生产者占用时返回 `BUSY`，成功 ACK 的载荷是 ASCII 名称。

FPGA 事务在同一提交边界更新超声、RGB、数字和扩展字段；完整帧、仅相位帧（`0x1B`，电平和使能不变时使用）和短命令 `MOD_FEED`（2+N 字节）分别覆盖不同场景。图像与上一次完全相同时，链路只每 10 ms 发一次状态事务，不重复写帧；任何事务的片选都会复位 FPGA 约 30 ms 的链路看门狗，超时则 FPGA 自行关闭输出。

## 设备端生产者

所有生产者互斥：任一时刻只有一个拥有 84 路输出；与当前所有权冲突的命令返回 `BUSY`，`MOTION_START`/`HOLOGRAM_START` 会先停止播放计划和块流再接管，枪按下则会中止所有其他生产者。`STOP_PLAN` 请求淡出/停止，`CLEAR_PLAN` 立即中止并安全停止。

### PA9 超声枪（`gun_trigger`）

* `gun_trigger` 在输入任务里每 10 ms 采样、连续 3 个相同样本才认边沿（30 ms 去抖），边沿在协议任务里处理，按住期间由渲染任务维持图像。
* 按下即接管：取消块流、停止计划、中止运动/全息/调制孔径，发 `STOP` 后由非语音模式提交一张静态场图（语音模式只 `STOP` 并开始录音）；非语音模式松开即 `STOP` 回到静音，语音模式松开后进入回放（见模式 6）。松开都不会恢复按下前的生产者。按住期间 `BLOCK_BEGIN`、`SET_PLAN`、`START_PLAN`、`SET_DEMO`、`MOTION_START`、`HOLOGRAM_START`、`MOD_CONFIGURE` 和会改动输出的 `CAL_START/CAL_SELFTEST/CAL_RAW/CAL_PROBE` 返回 `BUSY`；校准会话期间按下会被推迟到会话结束。
* 电平只有 `gun_trigger.level` 一个量：`0` 静音、`UMH_GUN_LEVEL=128` 满功率，按下/松开都是硬边沿，没有渐变、包络或释放看门狗。每次提交都从 `base_level[]` 重算，不能在 `frame.channels[]` 上就地缩放。四颗 WS2812 的蓝色亮度由同一个电平写成 `3*level/2`（满功率为 75%），不单独调用 `fpga_link_set_ws2812`。
* 6 种模式（枚举 0..5，枪菜单显示 `SINGLE FOCUS` / `PLANE WAVE` / `VORTEX L1` / `BESSEL L1` / `BOTTLE DYN` / `VOICE SPEAKER`），上电默认第一个模式 `SINGLE FOCUS`。模式 1–5 的每路源幅度都是满量程 1.0（默认校准增益下 → wire `level=128`），所以总发射功率相同，只有波前不同：
  1. 单焦点：轴上 `UMH_GUN_FOCUS_UM`（当前编译值 95 mm，从压电陶瓷面起算，即在 `DeviceProfile` 的 z 上再加 +7000 µm）处一个满幅度点，走普通 `spatial_renderer_accumulate_point()`。
  2. 平面波：只保留各路校准相位，84 路同相；**不加**方位角缠绕。
  3. 聚焦 l=1 涡旋：球面会聚到轴上 95 mm，叠加一圈方位角缠绕。
  4. l=1 贝塞尔涡旋：锥形波前 `theta - k*rho*sin(beta)`，`UMH_GUN_BESSEL_BETA_DEG=1.35°`。
  5. 动态瓶：同 3 的相位律再加锥形项，腰深从 600 mm 用 smoothstep 在 2 s 内收缩到 100 mm 后保持；按住期间每 10 ms 重建一次，场不变时靠链路 hold 省掉 SPI 帧。
  6. 语音：按住时先 `STOP`，FPGA 把四麦汇成 16 kHz PCM；STM32 做一阶去直流（约 80 Hz）、31 抽头 Q15 低通和 2:1 抽取到 8 kHz，按固定 mu-law 压扩存进约 16 KB 的 CCM（最多 2 s）。松开后在渲染任务里分步解码、降噪/选增益，再用校准平面波以 8 kHz AM 包络回放；录音期间状态显示 `REC`。
* 长按 **KEY0 或 KEY3** 3 s 进入/退出枪菜单。进入时把光标放到第一个模式并尝试立即生效（发射期间切换会被拒绝）；菜单内短按 KEY0/KEY3 循环选择（到末行回卷），KEY2 也能下移光标并同步模式，KEY1 无动作。**退出菜单保留当前模式**；只有上电默认值固定为第一个模式。
* `GET_STATUS` bit10 是去抖后的按键电平，bit11 是语音录音中，bit1 表示正在发射。

### 实时焦点运动（`motion_engine`，`0x62..0x68`）

* 路径最多 1024 点，每点 8 字节（`x/y/z` 以 10 µm 为单位、`level`、`palette`）；单帧最多 255 点，长路径用 `CHUNKED` 分块，`KEEP_PHASE` 可在运行中替换路径并保留循环相位。
* `MOTION_CONFIG` 接受 69/76/84 字节；输出帧率 50–4000 Hz，路径周期 1–60000 ms，跟踪器限速/限加速度，`z_offset`，最多 8 个图案源（单焦点、轴向/横向双焦点、环、涡旋、双环），16 色调色板及实时调色板旋转/转台旋转。
* v7.5.1 的 ULM 扩展把 `ulm_axis` 的 bit7 作为波形选择：置 1 时输出单向锯齿 `(2*phase-1)*amplitude`，否则为双向正弦；`ulm_frequency_hz ≤1000 Hz`。
* 显示扩展（`MOTION_DISPLAY`）支持 `SMOOTH_PALETTE`、`POINT_BRIGHTNESS` 和 `0x68 MOTION_RATE` 的平滑变速；路径被限制在 x/y ±100 mm、z 10–300 mm 的安全盒内。
* 状态帧 52 字节，包含规划位置、当前循环相位、DWT 统计的 `service/render/submit` 最坏与平均耗时、丢帧数等；`MOTION_STOP` 先做 20 ms 线性淡出再安全停止。

### 全息关键帧（`hologram_engine`，`0xA0..0xA5`）

* 最多 10 个 84 路完整关键帧（每帧 168 字节，相位 84 + 电平 84），单条 `UPLOAD` 最多 11 帧，可分多条按槽位写入。
* `CONFIG` 12 字节：提交帧率 10–1000 Hz、循环周期、相邻关键帧插值时间（≤60 s）、全局电平 0–128、`HOLD_LAST`/`GAMMA`/`STATIC` 标志。相位按最短 8 位路径插值，电平钳在配置上限而不是 255。
* 设备只做插值，不重算声场；不改变的关键帧会被抑制，静态图像每 100 ms 刷新一次；`STOP` 做 20 ms 淡出。状态帧 42 字节，字段位置与运动状态对齐，便于台架对比。

### 调制孔径（`aperture_mod`，`0x90..0x95`）

* 一次 `MOD_CONFIGURE`（20 字节基础配置，可追加最多 7 个 14 字节焦点）把 84 路相位渲染并装载到 FPGA，之后只流 8 位公共包络；`PLANE_WAVE` 位用于只保留校准相位。
* 包络率 8–20 kHz，2048 字节静态环；渲染任务以 20 kHz 从环中线性插值并做填充度时钟伺服（±3000 ppm），持续向 FPGA 的 256 级 FIFO（目标 192）补数。FPGA 每个载波两周期取一个电平，MCU 抖动进不到输出。
* USB 抖动时最多保持上一包络 25 ms，超过才作为欠载淡出；淡出/启动用 Q7 增益斜坡（满程 6.4 ms）。`MOD_FEED` 是无 ACK 的数据帧，序列号来自帧头 `stream_sequence`，单次最多 32 个电平。
* `RAMP_BURST=0x95` 用同一条公共电平通路做单通道开通瞬态测量：载荷 4 字节（通道、目标电平、上升步数 0–31、保留），每个电平由 FPGA 保持 50 µs，`ramp_ticks=0` 即硬边沿；最后一个电平保持到 `STOP_PLAN`/`CLEAR_PLAN`。

### USB 音频（UAC1）

* 复合设备：CDC（接口 0/1，端点 `0x81/0x01/0x82`）+ UAC1 自适应单声道（AC 接口 2、AS 接口 3、端点 `0x03`），VID/PID 为 `0x0483/0x5740`，产品名 `UMH Speaker`。48 kHz、16 bit、单声道；等时 OUT 端点每帧固定声明 48 个样本（96 字节），避免 49 样本帧在 PMA 重装时丢字节。
* 主机 PCM 经 48 抽头 Kaiser 多相 FIR 从 48 kHz 重采样到 20 kHz，再经 20 Hz 直流阻断、+12 dB、4.8 ms 前瞻限幅（0.98 上限、50 ms 释放）和 3.2 ms 峰值 AGC（目标 0.655 节目峰值），以 68.7% 调制度和 0..128 包络输出；静音门开启 4 ms、静音 120 ms 后关闭。
* 包络进入同一个 `aperture_mod`，焦点固定在 `(0, 0, 1 m)`，预充 1536 个包络样本（76.8 ms）。UAC 的静音控制生效，音量控制不缩放 8 位包络（否则只剩十几个码，产生机械感失真）；40 ms 没有新包或主机停止流即释放输出。
* 这是设备的播放输出，不是对手机扬声器的镜像；需要手机/电脑作为 USB 主机并满足 OTG 供电。

## 存储

| 区域 | 布局 |
| --- | --- |
| NOR 0–8 MiB | 低频对象库：前 64 KiB 元数据日志，其余为两个擦除对齐的数据 bank（各 4 MiB − 64 KiB）；最多 64 个对象。先写数据、再追加未提交元数据、最后写提交标志；CRC32 + generation + tombstone，bank 满时低优先级存储任务擦除备用 bank 并迁移 |
| NOR 8–16 MiB | 只读序列库：两个 4 MiB bank，最多 8 个对象；每个对象 256 B 头和 84 或 168 字节/帧的预编译帧（`level_mode` 为整帧电平或统一电平表）。上传按追加写 + CRC + 提交标志；回放只发 `FAST_READ`，0 页编程、0 擦除，存储任务每块 4 帧泵入帧环，预充 8 帧、高水位 16 帧 |
| AT24C16（2 KiB） | 运行时校准：两个 1 KiB 副本，v3 记录含魔数、版本、generation、84 路相位（历史 16 bit，实际只用高字节）、84 路增益、84 路使能位图、4 路 RGB 修正、数字默认值、CRC32 和提交标志；启动时选 CRC 正确且 generation 最新的副本，EEPROM 是校准唯一权威来源 |
| STM32 RAM | 帧环 24 × 232 B、块解析 512 B 记录缓冲、USB 4 KiB 环、2048 B 包络环、运动路径 8 KiB、全息 10 × 168 B、语音 16 KB（CCM）；全部静态数组 |

块流、Demo 和运动产生的帧本身不写入 NOR；序列库是显式“上传一次、本地循环回放”的独立功能，写入与擦除只发生在低优先级存储任务。

## 板载界面与按键

SSD1315 128×64，5×7 固定 ASCII 字体，21 列 × 6 行正文。界面共 10 页（页标题显示 `n/10`）：

`STATUS`（首页） / `PLAYBACK` / `DEVICE` / `CALIB` / `STORAGE` / `DEBUG` / `SYSTEM` / `DEMOS` / `LED TEST` / `GUN`。

* KEY0 = 返回，KEY1 = 确认，KEY2 = 下/下一页，KEY3 = 上/上一页；输入任务 10 ms 扫描，3 个稳定样本（约 30 ms）去抖，长按 3 s 上报 `LONG`。
* KEY0 和 KEY3 的短按延迟到松开才上报，避免长按起始时的页面跳转把随后的事件变成退出；长按在按住期间就上报，一次长按只产生一个事件。
* `DEBUG` 页是 32 行可滚动诊断列表：故障摘要（严重故障优先）、协议/解析/FPGA/欠载/USB/帧环计数、FPGA 版本/信用/深度/状态字/接受序号、运行时间、帧数、设备时间、型号/固件/协议/RAM/最高帧率/校准代数和 EEPROM 错误数。
* `SYSTEM` 页显示空闲堆、任务数和每任务 CPU 占比（由低优先级 health 任务 4 Hz 采样，渲染路径不遍历任务表）。
* `DEMOS` 页可选择 ULM/LML/LMC；`LED TEST` 页可切换 4 颗 WS2812 的 OFF/RED/GREEN/BLUE/WHITE；`GUN` 页即枪菜单。
* OLED 刷新在独立 `disp` 任务中做；连续失败 3 次后退避 2 s，面板故障不会阻塞按键和输出。

## 校准

* `CAL_START=0x82` 执行生产自校准：阵列正对平坦台面（台面距离约 4–25 cm，由 `CAL_ECHO_V_MIN_MM`/`MAX_MM` 限定），先由若干环通道的回波起始测出台面距离，再用相位循环 0/180 差分（`0.5*(y0-y180)`）提取各通道回波的幅度与相位，按生产渲染器几何拟合并做通道健康判定；最后用真实聚焦对比零校正（正/反相 180° 消除共模）验证。只有所有质量门通过且实际聚焦不劣于零校正时才写 EEPROM v3；没有可测回波的通道在 `enabled[]` 中被关闭。结果由 `CAL_RESULT=0x81` 查询。
* `CAL_SELFTEST=0x84` 是独立的相位自检：用单通道 H 与生产聚焦相位比较，给出聚焦增益、相干性和每麦一致性；结果由 `0x85` 查询。它不写 EEPROM。
* `CAL_RAW=0x83` 只做发射和采集，把原始 `[pattern][mic][I,Q]` 流回主机；`CAL_PROBE=0x86` 做多门 I/Q 时间剖面；`CAL_DUMP=0x80` 分 3 段导出重构矩阵、瞬态剖面和拟合指标。
* 近场平面直接耦合拟合（`us_calibration_nearfield_run`）只保留为诊断，实测它与轴向发射相位相差约 75° RMS，因此从不写入 EEPROM。

## 协议与能力位

* 帧：16 字节头 + ≤2048 字节载荷；标志 `ACK_REQUIRED/FIRST/LAST/RESPONSE/ERROR`；状态码 `OK/BAD_HEADER/BAD_LENGTH/BAD_SEQUENCE/NO_MEMORY/UNSUPPORTED/INVALID_STATE/BUSY/IO/CRC`。
* 消息组：`0x01/0x03` 查询，`0x10..0x13` 块流，`0x20..0x23` 计划，`0x30` FPGA 状态，`0x40..0x42` EEPROM，`0x50..0x53` Flash 对象，`0x54..0x59` 序列库，`0x60` 错误计数，`0x61` Demo，`0x62..0x68` 运动，`0x80..0x86` 校准，`0x90..0x95` 调制孔径，`0xA0..0xA3`/`0xA5` 全息；响应 `0x70 ACK` / `0x71 NACK`。
* `DeviceProfile`（1118 字节）：型号 `UMH-84`、固件 `v7.5.1`、协议 `UMH7`、84 通道 / 4 RGB / 4 麦克风、8 bit 相位与强度、最高 14000 帧/s、1 MHz 时间基准、40 kHz 载波、343 m/s 声速、112 KiB RAM（96 KiB SRAM1 + 16 KiB CCM）、64 帧 FIFO、校准版本/代数、84 个实测坐标（µm，z=+7000）和能力位。
* 能力位：bit0 `CHANNEL_STATE`、1 `SPATIAL_POINT`、2 `RGB`、3 `DIGITAL`、4 `EXTENDED`、5 `LOOP_RAM`、6 `LOOP_STREAM`、7 `GEOMETRY_VALID`、8 `FOCUSED_AM`、9 `MOTION`、10 `FOCUSED_AM_MULTI`、11 `HOLOGRAM`、12 `MOTION_DISPLAY`、13 `MOTION_LONG_PATH`，当前全部置位。
* 状态位：bit0 已连接、bit1 正在发射、bit2 FPGA 就绪、bit3 Flash 就绪、bit4 校准有效、bit5 欠载、bit6 错误、bit10 扳机按下（去抖）、bit11 语音录音、bit12 USB 音频；bit7..9 保留为 0。
* 阵元几何：`device_profile.c` 使用 2026-09 四麦短脉冲飞行时间实测的坐标表，通道 i 即 `us_tx` 位 i（`fpga_link` 当前为 identity 映射）；`Reference/UMH 7 Element Layout` 中的理论六角晶格 `E##` 表只作对照，不再直接装载。

## 构建与烧录

需要 ARM GNU Toolchain 10.3（`arm-none-eabi-gcc`）和 `make`：

```powershell
$env:PATH = 'D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN;' + $env:PATH
make -j 4
```

产物为 `build/UMH_Controller.elf`、`.hex`、`.bin`；当前镜像 text 164,096 B + data 2,788 B（约 164 KB，占 256 KB Flash 的 64%），SRAM1 约 94 KB，CCM 16 KB 由语音缓冲和音频前瞻窗口占满。编译期 `_Static_assert` 会检查 profile、帧、校准记录和网络结构尺寸。清理可直接删除 `build` 目录（`make clean` 需要 POSIX shell）。

烧录只使用 `build/UMH_Controller.elf`（或同目录 `.bin/.hex`），不要按 `debug/`、`release/` 之类目录名挑历史镜像：

```powershell
openocd -f openocd.cfg -c "program build/UMH_Controller.elf verify reset exit"
```

FPGA 侧用 `FPGA/run_diamond.cmd` 构建（成功标准、清理模式和编程文件见 `FPGA/README.md`）；XCF 烧录后还要执行 `UMH_7_Programmer_Refresh.xcf`，否则运行中的 FPGA 可能仍执行旧镜像。

## 台架与调试

* `Utiles/mic_volume.ps1` 调整 PC 麦克风增益（声场测量前先降到 `-Level 0.02`，测完恢复 `-Level 1.0`）；`Utiles/uac_tone.ps1` 产生 UAC 测试音。
* `Utiles/bench/` 下是本地 OpenOCD/GDB 台架脚本（未纳入版本控制）：`ocd.py` 复位与快照、`bench_stream.py` 稳态负载与 CPU 占用、`bench_loads.py` 运动/全息负载、`bench_uac.py` UAC 统计、`bench_flash.py` 序列库上传/回放、`spi_stress.py` SPI 压力、`dump_oled.py` 帧缓冲导出、`umh_client.py` 最小主机客户端。
* 读状态优先用 `arm-none-eabi-gdb` 的 `p <符号>`（走 DWARF）；本项目的 `nm` 地址与调试信息可能不一致，不要手工按 `nm` 地址读。常用符号：`gun_trigger`、`gun_trigger_build_id_seen`、`umh_usb_audio_stats`、`umh_task_stats`、`umh_bench_cmd/status/arg`。
* 改枪相关代码时递增 `UMH_GUN_BUILD_ID`，并在台架上核对 `gun_trigger_build_id_seen`：旧镜像的枪状态全为 0，只看模式/电平分辨不出新旧。

## 已知不一致（以代码为准）

1. **枪聚焦距离**：`UMH_GUN_FOCUS_UM` 与 `UMH_GUN_VORTEX_FOCUS_UM` 当前都是 95 mm；`gun_trigger.h` 的注释和 `Docs/UMH_v7_Protocol.md` 仍按 250 mm 描述。
2. **校准章节**：协议文档把近场直接耦合写成生产流程；代码中 `CAL_START` 走台面回波，近场拟合只是诊断。
3. **FPGA 接口文档**：仍写 SPICLK 21.25 MHz（PCLK2/8）和 16 字节 `AUDIO_LEVEL` 回退；实际是 42.5 MHz（/4）和 `MOD_FIXED`+`MOD_FEED`。
4. **存储文档**：仍写 16 KiB USB 环、32 × 400 B 帧环和独立 `LOOP_RAM` 池；实际是 4 KiB、24 × 232 B、快照复用同一 slots。
5. **通道映射**：`umh_device_logical_to_physical` 表已定义但未接入，`fpga_link` 当前使用 identity 映射（`fpga_channel_map_identity=1`）；`device_gui.c` 中“退出枪菜单恢复默认模式”的注释也已过时，代码保留所选模式。
6. 文档中引用的 `Utiles/umh_*.py`（`umh_focus_plan.py`、`umh_gun_modes.py`、`umh_gun_phase_check.py`、`umh_gun_probe.py`、`umh_noise_probe.py`、`umh_am_pm.py`、`umh_cal_dump_check.py` 等）在当前工作树中已不存在，其中一部分曾纳入版本控制后被删除。
