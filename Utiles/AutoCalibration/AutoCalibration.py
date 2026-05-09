import time
import math
import struct
import serial
import pyvisa
import numpy as np

# ==========================================
# 硬件与通信配置参数
# ==========================================
SERIAL_PORT = 'COM11'     # 请根据实际情况修改
BAUD_RATE = 115200
VISA_ADDR = 'USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR' # 你的示波器 VISA 地址

NUM_TRANSDUCERS = 60
FREQ = 40000.0
PERIOD_US = 1e6 / FREQ
C_SOUND = 343.2
RX_POS = np.array([0.0, 0.0, 0.1]) # 麦克风接收位置 (100mm)

# ==========================================
# 通信与几何计算辅助函数
# ==========================================
def calc_checksum(cmd_type, data):
    return (cmd_type + len(data) + sum(data)) & 0xFF

def send_phases(ser, phases):
    """将相位数组打包并通过 USB CDC 发送给相控阵"""
    data = struct.pack(f'<{NUM_TRANSDUCERS}f', *phases)
    frame = bytearray([0xAA, 0x55, 0x06, len(data)])
    frame.extend(data)
    frame.append(calc_checksum(0x06, data))
    frame.extend([0x0D, 0x0A])
    ser.write(frame)

def get_transducer_positions():
    """获取所有换能器的三维坐标"""
    row_lengths = [5, 6, 7, 8, 9, 8, 7, 6, 5]
    dy = 10.0e-3 * 0.86602540378
    spacing = 10.0e-3
    positions = []
    for i in range(NUM_TRANSDUCERS):
        k = i
        for r in range(9):
            row_count = 8 if r == 4 else row_lengths[r]
            if k < row_count: break
            k -= row_count
        j = k + 1 if (r == 4 and k >= 4) else k
        x = (j - (row_lengths[r] - 1.0) / 2.0) * spacing
        y = (4 - r) * dy
        positions.append(np.array([x, y, 0.0]))
    return positions

def get_geometric_phase_delays(positions):
    """计算理想状态下的几何相位延迟 k*d"""
    wave_k = 2.0 * math.pi * FREQ / C_SOUND
    return [(np.linalg.norm(pos - RX_POS) * wave_k) % (2.0 * math.pi) for pos in positions]

# ==========================================
# 核心：数字正交解调提取复数相量 (Complex Phasor)
# ==========================================
def get_complex_phasor(scope):
    """
    通过 IQ 解调，提取 40kHz 载波的复数相量（包含绝对幅度和绝对相位）。
    由于触发源绑定在 PC10（DMA起点），提取出的相位是绝对时间锁定的！
    """
    scope.write(':CLEAR')
    time.sleep(0.5) # 等待示波器完成 16 次平均
    
    # 读取时间偏移量和缩放系数
    xinc = float(scope.query(":WAV:XINC?"))
    xorig = float(scope.query(":WAV:XOR?"))
    yinc = float(scope.query(":WAV:YINC?"))
    yorig = float(scope.query(":WAV:YOR?"))
    yref = float(scope.query(":WAV:YREF?"))
    
    # 读取波形原始数据
    raw_data = scope.query_binary_values(":WAV:DATA?", datatype='B', container=np.ndarray, header_fmt='ieee')
    
    # 还原真实电压与绝对时间数组 (锁定到触发点)
    volts = (raw_data - yref) * yinc - yorig
    t = np.arange(len(volts)) * xinc + xorig
    
    # 截断为整数个 40kHz 周期以完美消除直流偏置和频谱泄漏
    period = 1.0 / FREQ
    duration = t[-1] - t[0]
    num_periods = int(duration / period)
    if num_periods > 0:
        valid_length = int(num_periods * period / xinc)
        volts = volts[:valid_length]
        t = t[:valid_length]
    
    # IQ 正交解调提取复数相量: 2 * mean(V(t) * e^{-jwt})
    omega = 2.0 * math.pi * FREQ
    complex_signal = volts * np.exp(-1j * omega * t)
    phasor = 2.0 * np.mean(complex_signal)
    
    return phasor

def get_complex_phasor_safe(scope, retries=3):
    for r in range(retries):
        try:
            return get_complex_phasor(scope)
        except Exception as e:
            print(f"    [!] 示波器通信失败 (尝试 {r+1}/{retries}): {e}")
            time.sleep(1)
    raise RuntimeError("无法读取示波器波形，校准终止。")

# ==========================================
# 主校准逻辑：全息 4 步相移干涉测量
# ==========================================
def optimize_phases():
    print("="*70)
    print(" 🚀 超声相控阵：全息干涉相控校准系统 (Holographic Calibration)")
    print("="*70)
    print("【硬件状态要求】")
    print(" 1. 探头接在 100mm 聚焦点，CH1 必须接 PC10 并作为触发源。")
    print(" 2. 示波器开启 Average 模式（16次），且波形无严重削顶(Clipping)。")
    print(" 3. 固件已烧录全零的 Transducer_Calibration_Array。\n")
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(VISA_ADDR)
    scope.timeout = 5000
    
    scope.write(':RUN')
    scope.write(':WAV:SOUR CHAN2')
    scope.write(':WAV:MODE NORM')
    scope.write(':WAV:FORM BYTE')
    
    geo_phases = get_geometric_phase_delays(get_transducer_positions())
    optimized_calib_us = np.zeros(NUM_TRANSDUCERS)
    
    # 【核心技巧】生成固定的随机散斑背景！
    # 这让背景声压大幅下降，单通道的信号变化在示波器上更加明显，解决量化噪声问题
    np.random.seed(42)
    fixed_bg = np.random.uniform(0, 2*math.pi, NUM_TRANSDUCERS)
    
    print("开始执行 4-Step 绝对相位提取...")
    for i in range(NUM_TRANSDUCERS):
        Z = []
        # 四步相移: 0, 90, 180, 270 度
        for p in [0.0, math.pi/2, math.pi, 3*math.pi/2]:
            work_phases = fixed_bg.copy()
            work_phases[i] = p
            send_phases(ser, work_phases)
            Z.append(get_complex_phasor_safe(scope))
            
        Z0, Z1, Z2, Z3 = Z
        
        # 核心全息干涉公式：完美消除背景 B 的影响，提取纯净的 Tx_i 向量
        C_pos = (Z0 - 1j*Z1 - Z2 + 1j*Z3) / 4.0
        C_neg = (Z0 + 1j*Z1 - Z2 - 1j*Z3) / 4.0
        
        # 自动判定固件相位映射极性，并提取当前换能器的纯物理属性
        if abs(C_pos) > abs(C_neg):
            Tx_vector = C_pos
            sign = +1
        else:
            Tx_vector = C_neg
            sign = -1
            
        theta_i = np.angle(Tx_vector) # 换能器的绝对接收相位 (含天生体质与几何路径)
        amp_i = abs(Tx_vector)        # 换能器的绝对声压贡献
        
        # 【神奇的数学补偿】
        # 这里生成的 p_calib 不仅补偿了硬件体质差异，还直接在数学上抵消了
        # 固件中 Distance_to_Phase = 2PI - fmod() 造成的逆向散焦 BUG！
        if sign == +1:
            p_calib = (geo_phases[i] - theta_i) % (2 * math.pi)
        else:
            p_calib = (geo_phases[i] + theta_i) % (2 * math.pi)
            
        calib_time_us = (p_calib / (2 * math.pi)) * PERIOD_US
        optimized_calib_us[i] = calib_time_us
        
        print(f" [CH {i+1:02d}/{NUM_TRANSDUCERS}] 振幅: {amp_i*1000:6.1f} mV | 绝对初相: {math.degrees(theta_i):6.1f}° | 补偿延迟: {calib_time_us:5.2f} us")

    # ==========================================
    # 输出可直接复制到 C 代码中的数组
    # ==========================================
    print("\n" + "="*50)
    print(" 🎉 校准完成！请将以下代码覆盖到 calibration.c 中：")
    print("="*50 + "\n")
    
    lines = []
    for i in range(0, NUM_TRANSDUCERS, 5): 
        chunk = optimized_calib_us[i:i+5]
        lines.append("    " + ", ".join([f"{val:5.2f}" for val in chunk]) + ",")
    if lines: lines[-1] = lines[-1].rstrip(',') 
    
    print("float Transducer_Calibration_Array[] = {")
    for line in lines: print(line)
    print("    0\n};")
    
    ser.close()
    scope.close()

if __name__ == '__main__':
    optimize_phases()