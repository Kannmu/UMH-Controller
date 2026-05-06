import time
import math
import struct
import serial
import pyvisa
import numpy as np

# ==========================================
# 配置参数
# ==========================================
SERIAL_PORT = 'COM11'
BAUD_RATE = 115200
VISA_ADDR = 'USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR'

NUM_TRANSDUCERS = 60
FREQ = 40000.0
PERIOD_US = 1e6 / FREQ
C_SOUND = 343.2
RX_POS = np.array([0.0, 0.0, 0.1])

# ==========================================
# 通信与几何计算辅助函数 (保持不变)
# ==========================================
def calc_checksum(cmd_type, data):
    return (cmd_type + len(data) + sum(data)) & 0xFF

def send_phases(ser, phases):
    data = struct.pack(f'<{NUM_TRANSDUCERS}f', *phases)
    frame = bytearray([0xAA, 0x55, 0x06, len(data)])
    frame.extend(data)
    frame.append(calc_checksum(0x06, data))
    frame.extend([0x0D, 0x0A])
    ser.write(frame)
    time.sleep(0.02) 

def get_transducer_positions():
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
    wave_k = 2.0 * math.pi * FREQ / C_SOUND
    return [(np.linalg.norm(pos - RX_POS) * wave_k) % (2.0 * math.pi) for pos in positions]

# ==========================================
# 【核心升级】获取示波器原始波形并进行数字IQ解调
# ==========================================
def get_waveform_amplitude(scope):
    """
    通过读取原始波形，利用傅里叶分量计算出绝对可靠的幅度信号
    极大压制随机相位抖动和宽带噪声
    """
    # 清空平均缓存，强制捕获最新的平均波形
    scope.write(':CLEAR')
    time.sleep(0.6) # 给定示波器执行16次平均的时间
    
    # 设定读取参数
    scope.write(":WAV:SOUR CHAN2")
    scope.write(":WAV:MODE NORM")
    scope.write(":WAV:FORM BYTE")
    
    # 读取时间偏移量和缩放系数
    xinc = float(scope.query(":WAV:XINC?"))
    xorig = float(scope.query(":WAV:XOR?"))
    yinc = float(scope.query(":WAV:YINC?"))
    yorig = float(scope.query(":WAV:YOR?"))
    yref = float(scope.query(":WAV:YREF?"))
    
    # 读取波形原始数据 (带TMC头，需切片)
    raw_data = scope.query_binary_values(":WAV:DATA?", datatype='B', container=np.ndarray, header_fmt='ieee')
    
    # 还原真实电压与时间数组
    volts = (raw_data - yref) * yinc - yorig
    t = np.arange(len(volts)) * xinc + xorig
    
    # 【数字正交解调 (IQ Demodulation)】提取40kHz基波幅度
    # 这相当于一个中心频率为40kHz的超窄带数字滤波器
    omega = 2.0 * math.pi * FREQ
    I_comp = np.mean(volts * np.cos(omega * t))
    Q_comp = np.mean(volts * np.sin(omega * t))
    
    # 提取出的绝对有效波形幅度 (与环境低频无关的真实贡献值)
    amp = 2.0 * math.sqrt(I_comp**2 + Q_comp**2)
    return amp

# ==========================================
# 主校准逻辑
# ==========================================
def optimize_phases():
    print("="*60)
    print("  超声相控阵终极校准系统 (Raw-Data IQ Demodulation)")
    print("="*60)
    print("【必须遵守的硬件设置】")
    print(" 1. 务必将 CH1 接入 PC10，并将触发源(Trigger Source)设为 CH1！")
    print(" 2. 避免气流扰动，保持安静环境。")
    print(" 3. 示波器开启平均(Average)模式，次数16。\n")
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(VISA_ADDR)
    scope.timeout = 5000
    print(f"成功连接示波器: {scope.query('*IDN?').strip()}")
    
    # 初始化读取参数
    scope.write(':RUN')
    scope.write(':CHANnel2:DISPlay ON')
    
    geo_phases = get_geometric_phase_delays(get_transducer_positions())
    background_phases = np.zeros(NUM_TRANSDUCERS)
    optimized_calib_us = np.zeros(NUM_TRANSDUCERS)
    
    STEPS = 16 
    phase_sweep = np.linspace(0, 2*np.pi, STEPS, endpoint=False)
    
    for i in range(NUM_TRANSDUCERS):
        print(f"\n---> [通道 {i+1:02d}/{NUM_TRANSDUCERS}] 扫描中...")
        amplitudes = []
        valid_sweep = True
        
        work_phases = np.copy(background_phases)
        
        for p in phase_sweep:
            work_phases[i] = p
            send_phases(ser, work_phases)
            
            try:
                amp = get_waveform_amplitude(scope)
                amplitudes.append(amp)
            except Exception as e:
                print(f"  [!] 读取波形异常: {e}")
                valid_sweep = False
                break
            
        if not valid_sweep:
            continue
            
        amps = np.array(amplitudes)
        
        mean_amp = np.mean(amps)
        x_comp = np.sum(amps * np.cos(phase_sweep)) * (2.0 / STEPS)
        y_comp = np.sum(amps * np.sin(phase_sweep)) * (2.0 / STEPS)
        
        v_mod = math.sqrt(x_comp**2 + y_comp**2) 
        phi_opt = math.atan2(y_comp, x_comp)
        if phi_opt < 0: phi_opt += 2 * math.pi
            
        fitted_curve = mean_amp + v_mod * np.cos(phase_sweep - phi_opt)
        ss_res = np.sum((amps - fitted_curve)**2)
        ss_tot = np.sum((amps - mean_amp)**2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
        
        modulation_depth = (2 * v_mod / mean_amp * 100) if mean_amp > 0 else 0
        
        phi_calib = (geo_phases[i] - phi_opt) % (2 * math.pi)
        calib_time_us = (phi_calib / (2 * math.pi)) * PERIOD_US
        optimized_calib_us[i] = calib_time_us
        
        print(f"  [数据] 解调幅度: {v_mod:.4f} V | 调制深度: {modulation_depth:.1f}%")
        if r_squared > 0.85:
            print(f"  [质量] R² = {r_squared:.3f} (✅ 极优)")
        elif r_squared > 0.60:
            print(f"  [质量] R² = {r_squared:.3f} (⚠️ 尚可，空气轻微扰动)")
        else:
            print(f"  [质量] R² = {r_squared:.3f} (❌ 差，环境扰动剧烈)")
            
        print(f"  => 驱动补偿角: {math.degrees(phi_opt):5.1f}° | 最终静态延迟: {calib_time_us:5.2f} us")

    print("\n========== 最终校准参数矩阵 ==========")
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