# 烧录FPGA固件步骤

## 方法1：使用Lattice Diamond Programmer

1. 打开 Lattice Diamond Programmer
2. 选择设备：MachXO2-2000HC
3. 加载固件文件：`UMH_7_1/UMH_7_UMH_7_1.jed`
4. 点击 "Program" 按钮烧录

## 方法2：使用命令行

```bash
cd FPGA
"D:\Software\Lattice Diamond\diamond\3.13\bin\nt64\pgrcmd.exe" -infile UMH_7_Programmer_File.xcf
```

## 验证

烧录完成后：
1. 复位STM32（按复位按钮）
2. 进入 LED TEST 菜单
3. 选择颜色（RED/GREEN/BLUE/WHITE）
4. 按下 ACTIVATE
5. 所有4颗WS2812应该同时显示相同的颜色
