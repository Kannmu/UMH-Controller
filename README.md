# UMH Controller v7

UMH Controller v7 是面向空气中通用型超声相控阵设备的 STM32G491KCU 固件。固件只描述设备能力、时间、空间、通道和外设状态，不包含任何应用模式或演示逻辑。主控通过 USB CDC 接收统一的 `SpatiotemporalBlock` 与 `PlaybackPlan`，在 RAM 中编译输出帧，再经 SPI1 提交给 FPGA。

## 硬件职责

STM32G491KCU 运行 FreeRTOS，负责 USB 控制链路、协议解析、时空数据编译、空间点解算、播放调度、EEPROM 参数、低频 Flash 对象、OLED、按键和设备状态。LCMXO2-2000HC-4MG132C 负责 84 路超声实时输出、4 路 PDM 麦克风采样和四颗 WS2812C-2020-V6 的时序发送。CH347T 通过独立 USB 链路上传 FPGA 采集的麦克风数据，控制链路不会复制这一路数据。

84 个发射单元由 GU1008C-40TR、DRV8220DSGR 和 2.2 mH 网络组成。SPI1 使用 8 bit、CPOL=0、CPHA=0、软件片选，SPI3 连接 25Q128M/TR 外置 NOR Flash。OLED（SSD1315，128x64）与 AT24C16 共用 I2C1。PA8 输出 FPGA 参考时钟，PA9 为通用触发输出。按键、心跳灯、电源灯和 CH347T 指示灯均作为设备级状态资源管理。

## 软件结构

`device_profile` 描述通道、阵元坐标、载波、时间基准、FPGA FIFO、RAM 额度、麦克风和能力位。`spatiotemporal_block` 与 `block_parser` 处理可组合的时间记录和轨道载荷。内建载荷包括通道状态、空间点、RGB、亮度、数字状态，以及可转发的标量、向量和原始扩展数据。`spatial_renderer` 使用阵元坐标、介质参数和 AT24C16 校准数据，将一个或多个空间源合成为 84 路相位与强度。

播放链路为 `USB RX -> ProtocolTask -> BlockParser -> RenderTask -> RAM Frame Ring -> FPGA Link`。USB 回调只复制字节并通知任务，所有帧槽和队列均为静态分配。FPGA 接口使用带序号、截止时间和信用额度的 FIFO 事务，在同一提交边界更新超声、RGB 和数字字段。欠载时按照播放计划选择保持、关闭或仅报告。

## 存储策略

播放块、相位帧和波形永远不写入外置 Flash，只保存在 STM32 RAM 和 FPGA FIFO。25Q128M/TR 仅提供低频对象库，采用追加记录、CRC32、提交标志、双数据 bank 和低优先级回收。AT24C16 保存运行时校准的双副本记录，启动时按 CRC 和 generation 选择最新有效副本；EEPROM 是校准参数唯一权威来源。

## 协议和文档

USB v7 帧为版本化、长度定界的二进制事务，支持块流、播放计划、设备描述、FPGA 状态、EEPROM 事务和低频 Flash 对象操作。协议和硬件接口的字段定义见：

* `Docs/UMH_v7_Protocol.md`
* `Docs/UMH_v7_FPGA_Interface.md`
* `Docs/UMH_v7_Storage.md`

84 路阵元坐标和 `E## -> Tn -> FPGA ball` 映射直接来自 `Reference/UMH 7 Element Layout`，固件按 `E01` 至 `E84` 顺序装载理论六角晶格坐标并宣称 `GEOMETRY_VALID`。实际声学中心误差仍需加工后校准；硬件生产资料中的 FPGA 料号和网表封装字符串不一致仍需在生产前复核。

板载 GUI 使用 SSD1315 的固定 ASCII 字体，提供首页、播放、设备、校准、存储、Debug、Demo 和控制页面。Demo 页可选择 ULM、LML 或 LMC，并以 `(0,0,0.1m)` 为中心焦点循环播放；它们复用标准空间渲染、校准、帧环和 FPGA 播放链路。KEY0 至 KEY3 依次为 Return、Confirm、Down、UP；控制页可执行通用的计划启动、输出停止、计划清空和触发等待操作。Debug 页按 KEY1 轮换故障摘要、错误计数、FPGA 链路、运行时和设备识别视图。故障摘要保留最后一次故障的短原因、参数、发生时刻和严重等级；严重故障另行保留，不会被后续普通告警覆盖。这样在没有主机连接时，也能仅通过四个按键查看 FPGA 协议、FIFO、播放帧、USB 丢包和存储错误。

## 构建

项目使用 ARM GCC 和 STM32CubeMX 生成的 HAL/FreeRTOS 源码。Windows 环境可直接运行 `make -j 4` 完成镜像构建；若需要清理，请删除 `build` 目录后重新构建。验收范围为编译、静态断言和镜像容量检查，不包含硬件联调。
