# OLED 花屏与按键无显示响应修复

本次依据当前工作树、用户上电照片及硬件网表分析。照片不能确定 I2C 波形、OLED 实际应答或实板焊接状态；以下区分已确认的软件缺陷与尚未验证的硬件现象。

## 根因

`oled_ssd1315.c` 使用 `HAL_I2C_Master_Transmit_DMA()`。工程原先只有 DMA2 Channel3 中断，没有 I2C1 EV/ER 的 NVIC 配置和处理函数。STM32G4 HAL 在 DMA 搬完数据后仍通过 I2C 事件中断处理 STOP、NACK 和事务完成。没有这条路径，即使 OLED 已收到初始化命令并执行 AF 开屏，HAL 状态也不能正常回到 READY。驱动约 100 ms 后超时，将 `initialized` 置零，随后所有刷新直接失败。控制器上电 RAM 没有被覆盖，能够解释照片中的随机花纹。

原超时处理调用异步 Abort 后立即释放共享总线，而 Abort 本身也依赖 I2C 事件中断。另一个独立缺陷是发送函数只检查 READY，未检查 HAL 错误码：补全中断后，NACK 同样会返回 READY，原代码仍会将失败当成成功并清除脏页标志。

四个按键的 GPIO 映射和低有效极性与网表一致。原 UI 任务每次刷新后休眠 100 ms，按键又要求连续三次采样，短按约需跨越 200–300 ms，刷新耗时还会进一步加长窗口。即使检测到按键，OLED 已被标记为未初始化时也不会呈现页面变化。Heart 是独立任务每 250 ms 翻转，显示失败不会阻止它运行。这一现象不支持再次套用旧版“调度器启动前死锁”的结论。

## 已实现修改

补全 I2C1 EV/ER 中断及 HAL 转发，优先级设为 5，并同步 `.ioc`。传输完成后检查错误码；超时时在仍持有 I2C 总线锁期间执行 DeInit/Init，项目的 MSP DeInit 会停止并清理两个 DMA 通道，避免未完成事务继续使用发送缓冲区。

OLED 初始化先关屏，写完全部 8 页零显存后才发送 AF。初始化或刷新失败后，UI 每秒尝试重新初始化；正常刷新失败沿用现有故障记录机制，脏页和 shadow 只在传输成功后更新。

屏幕排针在下边沿，使用 A0/C0 替换原 A1/C8，相对原显示方向旋转 180°。两个轴的扫描方向都改变，不再进行 framebuffer 或字模的二次旋转。上层像素坐标及按键编号保持原有定义。

按键改由静态分配的独立任务每 10 ms 扫描，沿用三次消抖和现有单生产者/单消费者事件队列。UI 消费事件和刷新仍由原 UI 任务执行。扫描不再等待 OLED I2C 事务，新增任务栈为 512 字节。

## CH347 的结论与实机定位

网表中 LED3 阴极直接连接 U226.15 (`CH347_LED`)，CH347T 的 USB DM/DP 分别连接 U225 Hub 的端口 1；其 SPI 与 FPGA 相连，STM32 没有控制该灯的 GPIO。因此 OLED 修复不能保证 CH347 指示灯恢复，也不能仅凭灯灭判定 CH347 损坏。

本次查询当前电脑的已连接 USB 设备，未发现 CH347 或 STM32 CDC；这只能说明本次没有可用的设备通信证据，不能据此断言板卡的 USB 硬件故障。尚未烧录、读取实板寄存器或采集总线波形。

后续若 CH347 仍异常，应先接通板卡的数据 USB 与主机，查看 Hub 和 CH347 是否枚举。如果 Hub 也未枚举，先查上游数据线及 Hub 供电；若 Hub 正常、仅 CH347 缺失，查 U226 供电、12 MHz 晶振及端口 1 的 DM/DP。若已枚举且能通信，则继续按 CH347 的指示输出状态及 LED3/R179 电路定位，不能将灯灭等同于 STM32 程序异常。

此前 `STM32_Bringup_2026-09-08.md` 已记录 U173 第 32 脚 VSS 在设计资料中误接 +3.3 V。本次没有重新验证或修复实物；如果该问题尚未返修，应先断电处理，再进行上电验证。如果实板已经修正，不需要重复返修。

## 验证与产物

ARM GCC 15.2.1 完整构建通过：text 84316、data 480、bss 99544 字节。链接器仍有已有的 RWX LOAD segment 警告。`nm` 确认 I2C1 EV/ER 均为实际强符号处理函数，与 Default_Handler 地址不同。

原生测试编译真实 OLED 和按键源码，用 HAL 模拟成功、启动失败、NACK 和超时，验证先清屏后点亮、A0/C0、边角像素、不重复刷新干净页、失败保留脏页与 shadow、超时后释放总线、重新初始化，以及四个键的 50 ms 短按和消抖。全部通过。此测试不模拟真实电气时序，也不替代实板验证。

在项目根目录的 PowerShell 中复现：

```powershell
New-Item -ItemType Directory -Force build/oled-input-fix | Out-Null
make -j4 BUILD_DIR=build/oled-input-fix
gcc -std=c11 -Wall -Wextra -Werror -ITests/host -ICore/Inc -include Tests/host/hal_mock.h Tests/test_oled_input.c Core/Src/oled_ssd1315.c Core/Src/input_events.c -o build/oled-input-fix/test_oled_input.exe
& ./build/oled-input-fix/test_oled_input.exe
```

烧录文件为 `build/oled-input-fix/UMH_Controller.hex`，同目录提供 `.elf` 与 `.bin`；BIN 的 Flash 起始地址为 `0x08000000`。构建包含进入本次任务前工作树中已有的演示功能修改，未撤销这些修改，也未改动 FPGA。

实机上应看到排针朝下时文字正向，KEY2/KEY3 短按切换页面，KEY0 返回主页。KEY1 在主页本来就没有动作，应在诊断页验证其切换详细视图，避免将既有交互定义误判为故障。Heart 应保持约 2 Hz。如果仍花屏，下一步读取 `oled.initialized`、`hi2c1.State/ErrorCode` 并抓取 PA15/PB7 的初始化与页写事务；若完整事务成功而画面仍错误，再核对实际模块控制器型号和供电，不能凭外观擅自换成另一种 OLED 驱动。
