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

请求消息类型为：`GET_PROFILE=0x01`、`GET_STATUS=0x03`、`BLOCK_BEGIN=0x10`、`BLOCK_DATA=0x11`、`BLOCK_END=0x12`、`BLOCK_CANCEL=0x13`、`SET_PLAN=0x20`、`START_PLAN=0x21`、`STOP_PLAN=0x22`、`CLEAR_PLAN=0x23`、`FPGA_STATUS=0x30`、`EEPROM_READ=0x40`、`EEPROM_WRITE=0x41`、`EEPROM_COMMIT=0x42`、`FLASH_LIST=0x50`、`FLASH_READ=0x51`、`FLASH_WRITE=0x52`、`FLASH_DELETE=0x53`、`ERROR_COUNTERS=0x60`、`SET_DEMO=0x61`、`CAL_DUMP=0x80`、`CAL_RESULT=0x81`、`CAL_START=0x82`、`CAL_RAW=0x83`、`AUDIO_CONFIGURE=0x90`、`AUDIO_START=0x91`、`AUDIO_DATA=0x92`、`AUDIO_STOP=0x93` 和 `AUDIO_STATUS=0x94`。

`SET_DEMO=0x61` 的载荷是一个 Demo ID：`0=ULM`、`1=LML`、`2=LMC`。设备以 `(0,0,100000µm)` 为中心焦点，使用当前校准和空间渲染器生成 24 帧轨迹，并在 5 ms（200 Hz）内播放完整轨迹后以 `LOOP_RAM` 循环。`ULM` 为 15 mm 单向扫描，`LML` 为 7.5 mm 往返扫描，`LMC` 为 4.77 mm 圆周；成功 ACK 的载荷为 ASCII 名称。演示播放与普通块互斥，停止或清空计划后才可接收新的块。

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

`delta_time` 和可变载荷长度都是无符号 varuint，低 7 位为数据、最高位表示后续字节，最多 5 字节。常量和密集轨道在描述符已经确定目标数量时由设备推导载荷长度，不重复携带长度字段。稀疏、差分、零抑制、位图密集以及 `VARIABLE` 轨道携带 varuint 长度，长度受设备记录缓冲限制。固定 84 通道密集状态记录最大 168 字节，设备记录重组缓冲为 512 字节。记录时间等于上一记录时间加 `delta_time`，必须落在块起始时间到结束时间内。同一时间戳允许连续记录，解析器在提交输出帧前将其合并。记录标志为 `KEYFRAME=0x01` 和 `END_OF_FRAME=0x02`。
当目标选择为 `SPARSE` 时，载荷长度始终显式携带；颜色轨道的每项为 `index,r,g,b`，亮度轨道的每项为 `index,level`，常量编码也沿用该逐目标形式。

内建载荷类型如下：

* `CHANNEL_STATE=1`：每个选中通道为 `phase(uint8), level(uint8)`，支持全量、稀疏和差分更新。`level=0` 表示关闭，不再发送独立 enable 字段。
* `SPATIAL_POINT=2`：`x_um, y_um, z_um(int32)`、源强度 `level(uint8)`、源相位 `phase(uint8)` 和 `source_id(uint8)`，共 15 字节。STM32 根据阵元坐标、声速、校准参数解算通道域状态；多个源先进行复数叠加。
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

## 聚焦 AM 音频扩展（0x90..0x94）

`DeviceProfile.capability_flags` 的 bit8（`FOCUSED_AM`）表示设备支持聚焦 AM
音频扩展。该扩展把空间渲染器只用于一次：`AUDIO_CONFIGURE` 根据给定焦点计算
84 路相位字节并把它们作为普通 FRAME 提交给 FPGA；随后音频数据只携带一个
公共 8 位包络电平。FPGA 在非活动事件表中用固定相位和公共电平重建 256 slot
载波表，因此音频流期间焦点、相位校准和通道掩码保持不变。

| 消息 | 值 | 方向 | 载荷 |
| ---: | ---: | --- | --- |
| `AUDIO_CONFIGURE` | `0x90` | 请求 | 20 字节配置，成功返回 ACK/NACK |
| `AUDIO_START` | `0x91` | 请求 | 空，进入预充状态 |
| `AUDIO_DATA` | `0x92` | 无 ACK 数据 | 1..2048 个包络字节；序列号使用帧头的 `stream_sequence` |
| `AUDIO_STOP` | `0x93` | 请求 | 空，淡出后返回 ACK |
| `AUDIO_STATUS` | `0x94` | 查询 | 空，响应类型 `0x94`，32 字节状态 |

`AUDIO_CONFIGURE` 载荷字段（全部 little-endian）：

```text
int32  x_um                 焦点 x，微米
int32  y_um                 焦点 y，微米
int32  z_um                 焦点 z，微米
uint8  phase                附加公共相位（0..255）
uint8  level                空间源幅度（255 表示满幅度；相位计算仍会执行）
uint16 envelope_rate_hz     包络更新率，8000..20000
uint16 prebuffer_samples    启动前需要的包络样本数，1..2048
uint16 flags                保留，必须为 0
```

`AUDIO_DATA` 的载荷是连续的 8 位包络电平，取值范围 `0..128`：0 关闭载波，
128 是 50% 占空比空间满幅度。数据帧不需要 ACK；STM32 用 `stream_sequence`
检查连续性并统计丢包。设备内部 2048 字节静态环形缓冲，主机按 256 样本
一包发送，把 USB 写调用次数减半；渲染任务以 20 kHz 从环中插值出设备采样
时钟，并根据环填充度做 ±3000 ppm 的异步时钟修正。USB/USB 主机偶发间隙时
最多保持上一包络 25 ms，超过后才进入真正的欠载恢复。固件优先使用 FPGA 的
3 字节 `AUDIO_LEVEL_SHORT=0x19` 热路径，旧位流自动回退 16 字节 `0x16`。

`AUDIO_STATUS` 响应字段：

```text
uint8  state                0=OFF,1=CONFIGURED,2=PRIMING,3=RUNNING,4=STOPPING,5=FAULT
uint8  flags                1=CONFIGURED,2=PRIMING,4=RUNNING,8=REFILLING,16=UNDERRUN
uint16 ring_fill
uint16 ring_capacity
uint16 prebuffer
uint32 underrun_count
uint32 overrun_count
uint32 packet_loss_count
uint32 rendered_samples
int32  clock_correction_ppm
uint32 max_service_us
```

音频模式与普通块流、播放计划、Demo 和校准互斥：音频输出拥有 FPGA 时，
`BLOCK_BEGIN`、`SET_PLAN`、`START_PLAN`、`SET_DEMO` 和 `CAL_*` 请求会返回
BUSY。重新发送 `AUDIO_CONFIGURE` 会中止上一路音频并重新装载相位。`AUDIO_STOP`
由渲染任务先线性淡出到 0，再发送 FPGA `AUDIO_MODE=0` 和 STOP。

## 内置近场耦合相位自校准

设备内置四颗 SPH0641LU4H-1，坐标由钻孔文件确定，换能器 GU1008C-40TR 的压电陶瓷位于 PCB 声孔平面之上 7.0 mm。STM32 通过 SPI1 的 `MIC_CONFIG=0x14` 和 `MIC_READ=0x15` 在发射 burst 稳态内部设置 400 us I/Q 门；不依赖反射面，也不要求 FPGA 改动。

`CAL_START=0x82`：无载荷，触发一次完整自校准。流程为发射电平自检、burst 尾段稳定门选择、逐通道 phase-0/180 差分复响应 H 测量、以生产空间渲染器几何为约束的四麦克风相位一致优化，然后对候选修正做一次差分聚焦实测（正/反相 180° 消除共模本底），并仅在所有麦克风实际相干性达标后写 EEPROM v3。成功结果通过 `CAL_RESULT=0x81` 请求读取。

`CAL_DUMP=0x80`：载荷为 `offset u32, length u16, section u8`（section 可省略，默认 0）。分三次返回：

- section 0：重构后的 `[mic][channel][I,Q]` float32。
- section 1：B 阶段 64 点瞬态剖面 `[gate][mic][I,Q]` int16。
- section 2：拟合指标、各麦克风 mu/rho、通道 a、量化修正字节和 4 个候选实测增益。

`CAL_RAW=0x83`：仅做 C 阶段发射和采集并把原始门采样流式回传，用于离线算法开发。载荷 8 字节：`level u8, gate_start u16, gate_width u8, burst_us u16, patterns u16`。数据帧仍为 `CAL_RAW=0x83`，`stream_sequence` 为块内第一个图案索引，载荷 layout 为 `[pattern][mic][I,Q]` int16 little-endian，每帧 64 个图案。投影矩阵不发送，PC 端用与固件相同的 splitmix64 种子按 pattern index 重生成。离线工具见 `Utiles/umh_cal_dump_check.py`。
