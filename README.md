# UMH Controller (Ultrasound Mid-Air Haptics)

UMH Controller 是 **UMH (Ultrasound Mid-Air Haptics)** 项目的核心固件，运行于 **STM32H750VBTx** 高性能微控制器上。本项目通过精确控制 61 个超声波换能器的相位和占空比，在空气中生成可感知的触觉反馈（声辐射压力）。

## 1. 项目概述 (Overview)

本固件旨在实现一个基于超声相控阵的触觉生成系统。通过控制 40kHz 超声波的相位延迟，可以在空间中形成高声压焦点（Focal Point），进而产生触觉 sensation。

**主要特性：**

* **高精度相位控制**：利用 STM32H7 的 DMA + Timer 机制生成精确的 PWM 波形。
* **多种刺激模式**：支持单点聚焦 (Point)、多点离散 (Discrete)、线性轨迹 (Linear)、圆形轨迹 (Circular)、双阱 (TwinTrap) 等模式。
* **内置演示模式**：通过按键即可切换预设的触觉演示效果。
* **实时通信**：通过 USB CDC (虚拟串口) 与上位机进行高频数据交互。
* **模块化设计**：软件架构分层清晰，包含独立的刺激生成、换能器管理、通信协议和 DMA 驱动模块。

---

## 2. 硬件架构 (Hardware Architecture)

### 2.1 核心控制器

* **MCU**: STM32H750VBTx (ARM Cortex-M7, 480MHz)
* **Flash**: 128KB (内部)
* **RAM**: 1MB
* **时钟源**: 外部高速晶振 (HSE) 经过 PLL 倍频。

### 2.2 换能器阵列 (Transducer Array)

* **数量**: 61 个超声波换能器 (Transducers)。
  * 60 个用于实际物理输出。
  * 1 个保留用于校准计算的虚拟通道。
* **排列方式**: 紧密六边形堆积。
* **工作频率**: 40 kHz (周期 25us)。
* **驱动方式**: 通过 GPIO 直接驱动或经由 MOSFET 驱动电路（固件侧输出 PWM 信号）。
* **物理参数**:
  * 换能器直径: 10mm
  * 阵列间距: 10mm

### 2.3 外设资源分配

* **DMA (Direct Memory Access)**:
  * **DMA1 / DMA2**: 配置为 **Circular Mode** (循环模式)。
  * 用于将内存中的波形数据 (`Waveform_Storage`) 自动传输到 GPIO 输出寄存器 (ODR)，以极低的 CPU 占用率生成 61 路独立的 PWM 信号。
* **Timers**:
  * `TIM1`: 主驱动定时器，同步生成 40kHz 载波，并触发 DMA 请求。
* **Connectivity**:
  * **USB FS**: 模拟 CDC 设备，用于指令接收和状态回传。
* **User Interface**:
  * **KEY0**: 切换校准模式 (Calibration Mode)。
  * **KEY1**: 切换演示模式 (Demo Mode)。

---

## 3. 固件架构 (Firmware Architecture)

软件采用**前后台系统**设计：

* **后台 (Background)**: 主循环 (`main.c`) 处理低优先级任务（模式切换、状态机更新、LED 指示）。
* **前台 (Foreground)**: 中断和 DMA 处理高实时性任务（波形更新、USB 数据接收）。

### 3.1 核心模块

#### 3.1.1 刺激控制模块 (`stimulation.c/h`)

负责计算和管理当前的触觉反馈模式。

* **Point**: 静态焦点。
* **Discrete**: 多点离散刺激。
* **Linear**: 线性轨迹渲染。
* **Circular**: 圆形轨迹渲染。
* **TwinTrap**: 双阱模式，用于悬浮或特殊触觉效果。
* **调制**: 支持 200Hz 的调幅 (AM) 刺激以匹配人体触觉敏感度。

#### 3.1.2 换能器管理 (`transducer.c/h`)

物理层的抽象，包含相控阵算法。

* **几何映射**: `TransducerArray` 结构体存储了每个换能器的 `(x, y, z)` 坐标及 GPIO 映射。
* **聚焦算法**: `Set_Point_Focus` 等函数计算每个换能器到达目标点所需的相位延迟。
  * 公式: $\Delta \phi_i = \frac{2\pi f}{c} (d_{focus} - d_i)$
* **校准支持**: 支持加载每个换能器的相位校准数据。

#### 3.1.3 通信模块 (`communication.c/h`)

基于自定义协议帧的数据解析器。

* **接口**: USB CDC。
* **接收逻辑**: 使用状态机 (`rx_state_t`) 解析数据包，防止粘包或断包。
* **发送逻辑**: 异步发送状态报告或配置确认。

#### 3.1.4 DMA 管理 (`dma_manager.c/h`)

本项目的技术核心。由于需要同时控制 60+ 路 IO 的相位，纯软件翻转 IO 无法满足精度要求。

* **实现原理**: 在内存中开辟一段缓冲区 (`Waveform_Storage`)，存放对应 PWM 周期的 GPIO 状态。DMA 控制器在定时器更新事件触发下，自动将数据搬运至 GPIO ODR 寄存器。
* **缓冲区结构**: `Waveform_Storage[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE]`。
* **特殊配置**: 需修改 Linker Script (`.ld` 文件) 将缓冲区放置在 D2 域 SRAM 中 (`.storage_buffer` section)，以优化总线访问性能。

#### 3.1.5 校准模块 (`calibration.c/h`)

管理换能器的相位校准数据。

* **数据存储**: `Transducer_Calibration_Array` 存储了每个通道的延迟时间 (us)。
* **模式切换**: 提供接口在校准模式（全零相位）和正常模式（加载校准数据）之间切换。

---

## 4. 通信协议 (Communication Protocol)

设备与上位机通过 USB 虚拟串口通信，波特率自适应（USB 协议）。

### 4.1 帧结构

数据包采用定长头部的变长帧格式：

| Header (2B) | Cmd Type (1B) | Length (1B) | Data (N Bytes) | Checksum (1B) | Footer (2B) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xAA 0x55` | `Cmd` | `N` | Payload... | Sum | `0x0D 0x0A` |

* **Checksum**: `(Cmd + Length + Sum(Data)) & 0xFF`

### 4.2 指令集 (Command Set)

**上位机 -> 控制器 (PC -> UMH):**

| Cmd ID | Name | Description | Payload |
| :--- | :--- | :--- | :--- |
| `0x01` | `CMD_ENABLE_DISABLE` | 启用/禁用刺激输出 | `[1Byte] 1: Enable, 0: Disable` |
| `0x02` | `CMD_PING` | 心跳检测 | 任意数据 (原样返回) |
| `0x03` | `COM_GET_CONFIG` | 获取设备配置信息 | 无 |
| `0x04` | `CMD_GET_STATUS` | 获取设备实时状态 | 无 |
| `0x05` | `CMD_SET_STIMULATION` | 设置刺激模式参数 | `[Type][Params...]` (见源码结构) |
| `0x06` | `CMD_SET_PHASES` | 直接设定换能器相位 | `[float phase_0, float phase_1, ...]` |
| `0x07` | `CMD_SET_DEMO` | 设置演示模式 | `[1Byte] Demo Index` |

**控制器 -> 上位机 (UMH -> PC):**

| Rsp ID | Name | Description | Payload |
| :--- | :--- | :--- | :--- |
| `0x80` | `RSP_ACK` | 通用确认 | 无 |
| `0x81` | `RSP_NACK` | 通用否认 | 无 |
| `0x82` | `RSP_PING_ACK` | Ping 响应 | 原样返回 Ping 数据 |
| `0x83` | `RSP_RETURN_CONFIG` | 返回配置信息 | `device_config` 结构体 |
| `0x84` | `RSP_RETURN_STATUS` | 返回状态信息 | `device_status` 结构体 (含电压、温度等) |
| `0x85` | `RSP_SACK` | 刺激设置确认 | 无 |
| `0x86` | `RSP_DEMO_ACK` | 演示模式确认 | 无 |
| `0xFF` | `RSP_ERROR_CODE` | 错误代码 | 无 |

---

## 5. 操作说明 (Operation)

### 5.1 正常模式

上电后默认为正常模式，等待 USB 指令。可以通过上位机发送 `CMD_SET_STIMULATION` 来控制触觉输出。

### 5.2 校准模式 (Calibration Mode)

* **进入/退出**: 按下板载 **KEY0** 键。
* **功能**: 进入校准模式后，所有换能器的校准相位将被清零。再次按下退出，重新加载校准数据。

### 5.3 演示模式 (Demo Mode)

* **切换**: 按下板载 **KEY1** 键。
* **功能**: 循环切换内置的演示效果 (如 Point, DLM, Linear, Circular 等)。
* **注意**: 仅在非校准模式下有效。

---

## 6. 开发与编译 (Development)

### 6.1 环境要求

* **IDE**: STM32CubeIDE 或 Keil MDK.
* **Toolchain**: ARM GCC.
* **Configurator**: STM32CubeMX (用于底层初始化代码生成).

### 6.2 关键配置修改

⚠️ **注意**: 本项目对 CubeMX 生成的代码进行了特定修改，重新生成代码时需注意保护：

1. **Linker Script**: 必须使用修改过的 `.ld` 文件，确保大数组分配在正确的 RAM 区域 (`.storage_buffer`)。
2. **DMA Settings**: 强制开启 `DMA_CIRCULAR` 模式。
3. 详见 `Docs/Custom_Changes.md`。

---

## 7. 目录结构 (Directory Structure)

```
UMH Controller/
├── Core/
│   ├── Inc/            # 头文件 (API 定义)
│   └── Src/            # 源文件 (业务逻辑)
│       ├── main.c          # 程序入口
│       ├── stimulation.c   # 刺激模式实现
│       ├── transducer.c    # 相控阵算法
│       ├── communication.c # 通信协议栈
│       ├── dma_manager.c   # DMA底层驱动
│       ├── calibration.c   # 校准管理
│       ├── custom_math.c   # 数学库
│       └── utiles.c        # 通用工具
├── Drivers/            # STM32 HAL 库 & CMSIS
├── Docs/               # 项目文档
└── .stm32env           # 环境配置文件
```

## 8. 许可证与版权

Copyright (c) UMH Project. All rights reserved.
