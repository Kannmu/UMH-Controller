# UMH v7 通用时空数据协议

本文档描述 STM32 控制链路的 v7 二进制协议。协议只表达设备能力、时间、空间、通道和外设状态，不包含任何应用模式名称。所有多字节整数均为 little-endian。USB CDC 已提供链路级校验，控制帧不再附加应用层 CRC。

## USB 帧

每个帧由 16 字节固定头和 `payload_length` 字节载荷组成。`header_length` 当前必须为 16，`payload_length` 最大为 2048。

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 1 | `sync0`，固定 `0x55` |
| 1 | 1 | `sync1`，固定 `0xAA` |
| 2 | 1 | `protocol_version`，固定 `7` |
| 3 | 1 | `message_type` |
| 4 | 1 | `flags` |
| 5 | 1 | `header_length`，固定 `16` |
| 6 | 2 | `payload_length` |
| 8 | 4 | `transaction_id` |
| 12 | 4 | `stream_sequence` |

标志位为 `ACK_REQUIRED=0x01`、`FIRST=0x02`、`LAST=0x04`、`RESPONSE=0x08` 和 `ERROR=0x10`。事务号用于把响应关联到请求；块流的连续性由 `stream_sequence` 检查，USB 包边界不参与块解析。

请求消息类型为：`GET_PROFILE=0x01`、`GET_STATUS=0x03`、`BLOCK_BEGIN=0x10`、`BLOCK_DATA=0x11`、`BLOCK_END=0x12`、`BLOCK_CANCEL=0x13`、`SET_PLAN=0x20`、`START_PLAN=0x21`、`STOP_PLAN=0x22`、`CLEAR_PLAN=0x23`、`FPGA_STATUS=0x30`、`EEPROM_READ=0x40`、`EEPROM_WRITE=0x41`、`EEPROM_COMMIT=0x42`、`FLASH_LIST=0x50`、`FLASH_READ=0x51`、`FLASH_WRITE=0x52`、`FLASH_DELETE=0x53` 和 `ERROR_COUNTERS=0x60`。

成功响应通常为 `ACK=0x70`，查询类响应也可使用对应的响应类型，例如 `PROFILE=0x02`、`STATUS=0x04`、`FPGA_STATUS=0x30`。失败响应为 `NACK=0x71`。ACK/NACK 的无额外载荷形式包含 4 字节：状态码、当前帧数量、剩余帧槽数量和保留字节。状态码为 `OK=0`、`BAD_HEADER=1`、`BAD_LENGTH=2`、`BAD_SEQUENCE=3`、`NO_MEMORY=4`、`UNSUPPORTED=5`、`INVALID_STATE=6`、`BUSY=7`、`IO=8`、`CRC=9`。

## SpatiotemporalBlock

`BLOCK_BEGIN` 的载荷由 36 字节块头和 `track_count` 个 22 字节轨道描述符组成。块头字段依次为：

```text
uint32 block_id
uint32 timebase_hz          // 当前必须为 1,000,000
uint64 start_time
uint64 duration
uint32 record_count
uint32 output_period
uint16 track_count           // 1..16
uint16 flags
```

`start_time + duration` 必须不溢出，记录数最大为 512。当前设备只接受 1 MHz 逻辑时间基准；时间单位因此为微秒。

轨道描述符字段依次为：`track_id(uint16)`、`payload_type(uint8)`、`value_type(uint8)`、`encoding(uint8)`、`interpolation(uint8)`、`component_mask(uint16)`、`target_mode(uint8)`、`unit(uint8)`、`target_count(uint16)`、`quantization_bits(uint8)`、`attributes(uint8)`、`target_start(uint16)`、`payload_stride(uint16)` 和 `reserved(uint32)`。轨道号在一个块内唯一且范围为 0..15。

目标选择方式为 `ALL=0`、`RANGE=1`、`BITMAP=2`、`SPARSE=3`。通道位图固定 11 字节，对应 84 路通道，最后一个字节只有低 4 位有效。RGB 目标位图为 1 字节，对应 4 个输出。`RANGE` 使用 `target_start` 和 `target_count`；`SPARSE` 的索引在记录载荷中携带；`ALL` 的空数量表示全部目标。

编码方式为 `CONSTANT=1`、`DENSE=2`、`SPARSE=3`、`DELTA=4`、`ZERO_SUPPRESS=5` 和 `VARIABLE=6`。定长编码不重复发送长度字段；稀疏或差分编码只传输发生变化的目标；`VARIABLE` 由记录前缀的长度字段定界。当前 MCU 解释器实现了常量、密集、稀疏、差分和零抑制，通用可变载荷通过扩展区转发。

插值属性为 `HOLD=0`、`STEP=1` 和 `LINEAR=2`。相位线性插值使用 16 bit 模环绕差值，从而选择最短相位路径；复杂曲线应由主机细分为更多记录或直接发送通道状态。

## 记录流

`BLOCK_DATA` 可以在任意 USB 帧边界切分。设备先以记录为单位重组字节，再解析：

```text
varuint delta_time
uint16   track_id
uint8    record_flags
[varuint payload_length] // only when the track has variable length
uint8[payload_length] payload
```

`delta_time` 和可变载荷长度都是无符号 varuint，低 7 位为数据、最高位表示后续字节，最多 5 字节。常量和密集轨道在描述符已经确定目标数量时由设备推导载荷长度，不重复携带长度字段。稀疏、差分、零抑制、位图密集以及 `VARIABLE` 轨道携带 varuint 长度，长度受设备记录缓冲限制。固定 84 通道密集状态记录最大 336 字节，设备记录重组缓冲为 512 字节。记录时间等于上一记录时间加 `delta_time`，必须落在块起始时间到结束时间内。同一时间戳允许连续记录，解析器在提交输出帧前将其合并。记录标志为 `KEYFRAME=0x01` 和 `END_OF_FRAME=0x02`。
当目标选择为 `SPARSE` 时，载荷长度始终显式携带；颜色轨道的每项为 `index,r,g,b`，亮度轨道的每项为 `index,level`，常量编码也沿用该逐目标形式。

内建载荷类型如下：

* `CHANNEL_STATE=1`：每个选中通道为 `phase(uint16), level(uint8), enabled(uint8)`，支持全量、稀疏和差分更新。
* `SPATIAL_POINT=2`：`x_um, y_um, z_um(int32)`、源强度 `level(uint8)`、源相位 `phase(uint16)` 和 `source_id(uint8)`，共 16 字节。STM32 根据阵元坐标、声速、校准参数解算通道域状态；多个源先进行复数叠加。
* `COLOR_RGB8=3`：每个选中 RGB 输出 3 字节 RGB 值。
* `LIGHT_LEVEL8=4`：每个选中 RGB 输出 1 字节亮度。
* `DIGITAL_STATE=5`：2 字节 `mask,state`，用于通用数字输出，其中 bit0 对应 `TRIGGER`。
* `SCALAR=6`、`VECTOR=7`、`RAW=8`：不由 MCU 解释的扩展数据，按记录顺序追加到 32 字节帧扩展区并转发到 FPGA 扩展端点。

颜色和亮度可由不同轨道独立更新。WS2812 没有独立亮度寄存器，STM32 在帧提交时计算 `rgb_out = rgb_color * level / 255`，FPGA 只发送 GRB 时序。

`DeviceProfile` 的 84 路坐标来自 `Reference/UMH 7 Element Layout`，按 `E01` 至 `E84` 稳定 ID 顺序装载，`GEOMETRY_VALID` 已开启。坐标是理论六角晶格的整数微米值，实际声学中心误差仍由加工后的校准参数修正；生产资料中的 FPGA 封装信息仍需独立复核。

## PlaybackPlan

`SET_PLAN` 的载荷是 32 字节：`block_id(uint32)`、`start_mode(uint8)`、`repeat_mode(uint8)`、`underrun_policy(uint8)`、`route(uint8)`、`rate_numerator(uint32)`、`rate_denominator(uint32)`、`start_time(uint64)`、`prebuffer_frames(uint32)` 和 `loop_count(uint32)`。

当前硬件只有一个 FPGA 输出端点，因此 `route` 必须为 `0`（默认路由）；其他值会返回 `INVALID_STATE`，不会被静默忽略。计划时间映射以块头的 `start_time` 为源时间原点，首条记录的时间差会在立即、设备时间和触发启动模式下保留，并按有理数倍率换算到设备输出时间。

开始方式为立即、设备时间、触发事件或下一帧边界。倍率为有理数且分子分母均非零。重复方式为单次、`LOOP_RAM`、`LOOP_STREAM`、保持最后状态和停止。`ONCE` 在提交块的最后一帧后停止计划调度但保持 FPGA 的最后状态，用于静态固定输出；显式 `STOP_PLAN` 或 `STOP` 重复策略才会主动关闭 FPGA 输出。`LOOP_RAM` 必须在开始前把帧快照复制到独立 RAM 池；`LOOP_STREAM` 允许块继续接收，由主机按信用额度持续发送。欠载策略为保持、关闭输出或仅报告。

设备达到预缓冲阈值后才运行 FPGA。触发方式等待通用触发事件；帧提交以 FPGA 接受的序号和截止时间为边界。`CLEAR_PLAN` 会停止 FPGA、取消块、清空帧环。播放数据只存在 STM32 RAM 和 FPGA FIFO，不能写入外置 Flash。

## 数据路径和能力

USB CDC 回调只把数据复制到 16 KiB 单生产者环形缓冲并通知协议任务。协议任务校验帧头、处理事务和块序号；编译任务执行轨道解析与空间解算；帧环使用 32 个固定 400 字节槽；FPGA 链路任务以 DMA 和 FIFO 信用额度提交原子输出帧。

`GET_PROFILE` 返回 1118 字节 `DeviceProfile`，含 84 个整数微米坐标、84 路通道、4 个 RGB 输出、4 个麦克风、时间基准、载波、FPGA 时钟、RAM/FIFO 额度、校准生成号和能力位。麦克风数据不经 STM32 控制链路转发，而由 FPGA 采样后经 CH347T 独立 USB 链路上传。
