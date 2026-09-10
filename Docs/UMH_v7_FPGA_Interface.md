# UMH v7 FPGA SPI1 接口

STM32 通过 SPI1 以 8 bit、CPOL=0、CPHA=0、软件片选方式连接 LCMXO2-2000HC-4MG132C。控制链路时钟为 42.5 MHz，由 STM32G4 的 170 MHz PCLK2 通过 `/4` 分频得到，SPI DMA 保证整帧连续传输。PA8 输出 8 MHz HSE MCO，驱动 FPGA 内部 PLL 生成 128 MHz 主时钟。PA4 为软件 CS。包含 84 路超声和 4 路 RGB 的完整帧为 216 字节，线缆传输时间约为 40.7 us。

FPGA 负责 84 路超声驱动状态、输出帧定时、载波生成、4 路 PDM 麦克风采样和四颗 WS2812C-2020-V6 的串行 GRB 输出。STM32 只提交通道状态和同步字段，不在实时 SPI 事务中生成载波或复制麦克风数据。

FPGA 使用 32 位 DDS 生成载波，通道线格式固定为 `phase(uint8), level(uint8)`。相位把一个载波周期划分为 256 个码，128 MHz 时钟下相邻码约为 97.65625 ns，实际输出边沿仍对齐到 7.8125 ns 时钟。`level=0` 表示关闭，`level=1..254` 为对应占空比，`level=255` 表示 100% 高电平。`us_tx` 始终是 0/1 数字信号，对应外部 0 V/3.3 V 方波。EEPROM 可以继续保存旧的 16 位校准字，STM32 在载入时转换到 8 位运行格式。

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

`fifo_credit` 是可继续提交的帧槽数量，`fifo_depth` 是当前深度，`fpga_time` 为 FPGA 输出时间基准，`accepted_sequence` 是最近接受的事务或帧序号。状态位至少应覆盖欠载、溢出、非法帧、输出故障和运行状态。STM32 在每次提交后解析状态，在后台轮询状态；协议版本不匹配会清除 FPGA 就绪标志并进入错误状态。

## 时钟、DMA 和边界

FPGA 应在明确的参考时钟域内锁存 SPI 接收数据，在输出帧边界一次性切换通道、RGB 和数字状态。SPI DMA 事务由 STM32 的 FPGA 链路任务发起，DMA 完成中断只置位完成标志，超时则终止事务并把输出置于安全状态。单帧必须小于 STM32 的 512 字节 TX 缓冲；当前 84 路完整通道状态、RGB 和扩展区均受该上限约束。

## 麦克风和 USB 拓扑

四颗 `SPH0641LU4H-1` 共用 `MIC_CLK`，数据分布在两条 PDM 数据线上，由 FPGA 统一采样。采样流经 `SPI_MIC_*` 进入 CH347T，再由 CH347T 作为独立 USB 设备上传。CH347T 不并入 STM32 v7 控制协议，因此采样流不会抢占控制链路的解析 RAM 或 USB CDC 带宽。

## 硬件资料一致性

BOM 指定 FPGA 为 `LCMXO2-2000HC-4MG132C`，现有网表封装字符串却为 `LCMXO640C-4MN132I`。出厂前必须在原理图、网表、PCB 封装和生产资料中统一料号与引脚；固件只依据原理图网络名和 BOM 料号。PA9 为 `TRIGGER` 推挽数字输出。84 个发射单元由 `GU1008C-40TR + DRV8220DSGR + 2.2 mH` 构成。


麦克风采样补充：MIC_CLK 约为 4 MHz，FPGA 在每个完整周期的两个边沿分别锁存两条数据线，形成四个 16 位窗口。CH347T 每次片选读取固定 12 字节快照，由四个三字节记录组成：source_id、样本高字节、样本低字节。source_id 0/1 对应 MIC_DATA_0 上升沿/下降沿，2/3 对应 MIC_DATA_1 上升沿/下降沿。主机应按标记分发样本，阵列几何位置由设备配置中的麦克风坐标描述。
