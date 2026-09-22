# UMH v7 FPGA SPI1 接口

STM32 通过 SPI1 以 8 bit、CPOL=0、CPHA=0、软件片选方式连接 LCMXO2-2000HC-4MG132C。控制链路时钟为 21.25 MHz，由 STM32G4 的 170 MHz PCLK2 通过 `/8` 分频得到，SPI 全双工阻塞传输保证整帧连续。PA8 输出 8 MHz HSE MCO，驱动 FPGA 内部 PLL 生成 64 MHz 主时钟。PA4 为软件 CS。包含 84 路超声和 4 路 RGB 的完整帧为 216 字节，线缆传输时间约为 40.7 us。

FPGA 负责 84 路超声驱动状态、输出帧定时、载波生成、4 路 PDM 麦克风采样和四颗 WS2812C-2020-V6 的串行 GRB 输出。STM32 只提交通道状态和同步字段，不在实时 SPI 事务中生成载波或复制麦克风数据。

FPGA 使用分数累加器生成 40 kHz 载波，通道线格式固定为 `phase(uint8), level(uint8)`。相位把一个载波周期划分为 256 个码（10.24 MHz 码率，单码 97.65625 ns），实际输出边沿仍对齐到 15.625 ns 的 64 MHz 时钟。`level=0` 表示关闭，`level=1..254` 为对应占空比，`level=255` 表示 100% 高电平。`us_tx` 始终是 0/1 数字信号，对应外部 0 V/3.3 V 方波。空间渲染幅度映射到占空比 0..128，满强度对应 50% 占空比；`level=255` 仍是 99.6% 高电平，仅用于直接通道状态控制。本板 PLL 的 LOCK 输出始终为低，但实测 PLL 频率正确；FPGA 输出使能不依赖 LOCK，只使用 running/stop_event。EEPROM 可以继续保存旧的 16 位校准字，STM32 在载入时转换到 8 位运行格式。

## STM32 到 FPGA 帧

所有命令共享以下 little-endian 前缀：

```text
uint8  command                 // STATUS=0x01, FRAME=0x10, STOP=0x11, RESET=0x12
uint8  protocol_version        // 1
uint32 transaction_sequence
uint32 frame_sequence
uint64 deadline
uint16 update_flags
uint8[11] ultrasound_bitmap
uint8  rgb_bitmap
uint8  digital_mask
uint8  digital_state
uint16 extension_length
```

当 `update_flags` 包含超声位时，位图低 84 位有效，随后发送 84 个 `phase(uint8), level(uint8)`，共 168 字节。当前 STM32 提交完整 84 路状态，位图为全选；FPGA 仍按位图定义接口，便于后续稀疏提交。包含 RGB 位时，随后发送 4 组三字节 RGB 值，顺序为 R、G、B；STM32 已完成颜色与亮度合成。包含扩展位时，发送 `extension_length` 字节，最大 32 字节。数字字段在所有类型帧中固定出现，更新由 `digital_mask` 指示。超声、RGB、数字和扩展数据只在同一提交边界生效。

FPGA 必须拒绝版本、长度、位图越界、序号非法或 FIFO 无信用的帧，并在安全状态关闭 84 路输出、将 RGB 置为安全值。`STOP` 复位触发输出并清空运行状态；SPI 失联、欠载策略要求停止或检测到输出故障时执行同样的安全动作。

## FPGA 到 STM32 状态

每个事务返回固定 16 字节：

```text
uint8  protocol_version
uint8  reserved
uint16 fifo_credit
uint16 fifo_depth
uint16 status_flags
uint32 fpga_time
uint32 accepted_sequence
```

`fifo_credit` 是可继续提交的帧槽数量，`fifo_depth` 是当前深度，`fpga_time` 为 FPGA 输出时间基准，`accepted_sequence` 是最近接受的事务或帧序号。状态位至少应覆盖欠载、溢出、非法帧、输出故障和运行状态；bit15 为 `AUDIO_MODE` 诊断位，bit14 为 `AUDIO_SHORT` 诊断位；STM32 在进入聚焦 AM 后据此确认 FPGA 位流支持 `0x16/0x17`，并优先启用 3 字节 `0x19` 热路径。STM32 在每次提交后解析状态，在后台轮询状态；协议版本不匹配会清除 FPGA 就绪标志并进入错误状态。

## 时钟、DMA 和边界

FPGA 应在明确的参考时钟域内锁存 SPI 接收数据，在输出帧边界一次性切换通道、RGB 和数字状态。SPI DMA 事务由 STM32 的 FPGA 链路任务发起，DMA 完成中断只置位完成标志，超时则终止事务并把输出置于安全状态。单帧必须小于 STM32 的 512 字节 TX 缓冲；当前 84 路完整通道状态、RGB 和扩展区均受该上限约束。

## 聚焦 AM 紧凑音频事务

普通超声帧仍然用于装载 84 路相位和通道使能标记。聚焦 AM 模式新增紧凑
命令，不再重复发送 216 字节完整帧：

```text
16 字节命令：AUDIO_MODE=0x17 或 AUDIO_LEVEL=0x16
byte 0   command             MODE=0x17 / LEVEL=0x16
byte 1   protocol_version    1
byte 2   data                MODE: 0/1；LEVEL: 0..128
byte 6..9 frame_sequence     STM32 音频样本序号（可选）
其余字节为 0；MISO 仍返回标准 16 字节状态

3 字节热路径：AUDIO_LEVEL_SHORT=0x19
byte 0   command             0x19
byte 1   protocol_version    1
byte 2   level               0..128
```

新位流在状态字 bit14 置 `AUDIO_SHORT`，STM32 配置时检测到后，20 kHz 流
使用 3 字节命令；旧位流没有该位时自动回退到 16 字节 `0x16`。这样旧 FPGA
镜像仍能工作，新镜像则把每个音频样本的 SPI 线时间从约 6 us 降到约 1.1 us。

STM32 按安全顺序装载：先提交一次 `level=0` 的静音相位帧，再发送
`AUDIO_MODE=1`（公共电平 0），最后在音频模式下提交带通道使能标记的真实
相位帧；因此切换过程中不会出现直流满输出。`audio_mode=1` 时，任何触发
重建的普通 `FRAME` 也使用当前公共电平，staging 的 `level` 字节只作为
使能标记（0=静音，非 0=启用）。`AUDIO_LEVEL` 可在音频模式下每 50 us 更新
一次（20 kHz），`AUDIO_MODE=0` 退出。重建过程中当前活动 bank 继续输出，
bank 在载波周期边界交换，更新不会造成输出缺口。

## 麦克风和 USB 拓扑

四颗 `SPH0641LU4H-1` 共用 `MIC_CLK`，数据分布在两条 PDM 数据线上，由 FPGA 统一采样。采样流经 `SPI_MIC_*` 进入 CH347T，再由 CH347T 作为独立 USB 设备上传。CH347T 不并入 STM32 v7 控制协议，因此采样流不会抢占控制链路的解析 RAM 或 USB CDC 带宽。

## 设备端麦克风直通设计（v8 计划）

当前 v7 定向音频 Demo 只能使用主机/耳麦麦克风：CH347 原始 PDM 流没有上位机驱动，16 Mbit/s 的 PDM 速率也不适合走 SPI1 控制链路。要让举着设备在嘴边使用的设备麦克风直通工作，最省逻辑的方案是复用现有 40 kHz 麦克风相关器。

关键点是：把 mic_lo_i 固定为 1、mic_lo_q 固定为 0，现有 100 个 PDM 样本积分窗就变成一个 25 us 滑动平均（boxcar）。它对 40 kHz 载波恰好是零点，定向载波及其 AM 边带被强烈抑制，而 0..8 kHz 人声以约 1 dB 以内平坦度通过。FPGA 不需要新的 DSP，只需把现有 mic_win_i 连续写入一个 EBR FIFO。

STM32 按 5 ms 块读取 4 路 40 kHz 样本，在软件里做 40 k 到 16 k 抽取、以已知发射包络为参考的 1..2 抽头回声抵消（抑制设备自身近场串扰）、噪声门、AGC 和限幅，再喂给现有 audio_engine 发送 AUDIO_LEVEL_SHORT。

由于 LCMXO2-2000 当前 LUT4/SLICE 已用到 96%/97%，只余 1 个 EBR，该功能需要先做两项 RTL 清理：把 spi_mic_stream 的 96 位组合 mux 改成逐字节输出（约省 110 LUT4），并把连续采集/块读地址并入现有 mic_iq_ram 读口。建议作为 v8 位流发布，并与 v7 协议新增的 0x95/0x96 设备麦克风消息一起启用。

## 硬件资料一致性

BOM 指定 FPGA 为 `LCMXO2-2000HC-4MG132C`，现有网表封装字符串却为 `LCMXO640C-4MN132I`。出厂前必须在原理图、网表、PCB 封装和生产资料中统一料号与引脚；固件只依据原理图网络名和 BOM 料号。PA9 为 `TRIGGER` 推挽数字输出。84 个发射单元由 `GU1008C-40TR + DRV8220DSGR + 2.2 mH` 构成。


麦克风采样补充：MIC_CLK 约为 4 MHz，FPGA 在每个完整周期的两个边沿分别锁存两条数据线，形成四个 16 位窗口。CH347T 每次片选读取固定 12 字节快照，由四个三字节记录组成：source_id、样本高字节、样本低字节。source_id 0/1 对应 MIC_DATA_0 上升沿/下降沿，2/3 对应 MIC_DATA_1 上升沿/下降沿。主机应按标记分发样本，阵列几何位置由设备配置中的麦克风坐标描述。

## 内置麦克风自校准数据路径

麦克风不仅可经 CH347T 独立上传，FPGA 还提供 `MIC_CONFIG=0x14` 和 `MIC_READ=0x15` SPI1 命令给 STM32 自校准使用。`MIC_CONFIG` 的三个时间字段单位为 40 kHz 采样（25 us）：门数、起始、步进、宽度。`MIC_READ` 用帧序号字段作为门号，单事务返回 40 字节，其中前 16 字节为常规状态，之后为 `{status, block_count, gate_count, reserved, I0..I3, Q0..Q3}` 大端 16 位。

自校准在发射 burst 的稳态内部开一个 400 us 门；门内只有直达声和静态本底，不依赖反射面。换能器 GU1008C-40TR 的压电陶瓷在 PCB 麦克风声孔平面之上 7.0 mm，固件 DeviceProfile 的 z 坐标因此为 7000 um；麦克风声孔在 PCB 平面，聚焦到声孔的几何距离必须包含这 7 mm。原始每图案数据可由 `UMH_MSG_CAL_RAW=0x83` 从 STM32 USB CDC 流式回传到 PC，FPGA RTL 不需要改动。
