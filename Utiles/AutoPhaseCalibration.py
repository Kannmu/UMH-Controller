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
# 辅助函数
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
    time.sleep(0.05)

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
# 示波器测量函数
# ==========================================
def measure_robust_amplitude(scope):
    # 清空平均缓存，强制重新采集
    scope.write(':CLEAR')
    # 给足够的时间让示波器完成 16次 平均
    time.sleep(0.8) 
    
    vpp_str = scope.query(':MEASure:ITEM? VPP,CHANnel2').strip()
    vpp = float(vpp_str)
    
    # Rigol 示波器超量程时的特殊返回值
    if vpp > 9e30:
        print("    [!] 警告: 示波器量程爆表，正在尝试恢复...")
        return -1.0 # 返回无效标志
        
    return vpp

# ==========================================
# 主校准逻辑
# ==========================================
def optimize_phases():
    print("初始化硬件和通信...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(VISA_ADDR)
    scope.timeout = 5000
    print(f"成功连接示波器: {scope.query('*IDN?').strip()}")
    scope.write(':CHANnel2:DISPlay ON')
    
    geo_phases = get_geometric_phase_delays(get_transducer_positions())
    
    # 所有的通道保持为 0，作为一个固定不变的“公共声学背景”
    background_phases = np.zeros(NUM_TRANSDUCERS)
    optimized_calib_us = np.zeros(NUM_TRANSDUCERS)
    
    STEPS = 8 
    phase_sweep = np.linspace(0, 2*np.pi, STEPS, endpoint=False)
    
    for i in range(NUM_TRANSDUCERS):
        print(f"\n---> 正在测量通道 {i+1}/{NUM_TRANSDUCERS} <---")
        amplitudes = []
        valid_sweep = True
        
        # 将工作数组复制自背景数组
        work_phases = np.copy(background_phases)
        
        for p in phase_sweep:
            work_phases[i] = p
            send_phases(ser, work_phases)
            
            amp = measure_robust_amplitude(scope)
            if amp < 0:
                valid_sweep = False
                break
                
            amplitudes.append(amp)
            print(f"  扫相 {math.degrees(p):5.1f}° -> 幅度: {amp:.4f} V")
            
        if not valid_sweep:
            print("  [!] 测量失效，可能是超量程，该通道跳过。")
            continue
            
        amps = np.array(amplitudes)
        amp_ptp = np.max(amps) - np.min(amps)
        mean_amp = np.mean(amps)
        modulation_depth = (amp_ptp / mean_amp * 100) if mean_amp > 0 else 0
        
        print(f"  [Debug] 调制深度: {modulation_depth:.2f}% (幅度波动: {amp_ptp:.4f} V)")
        
        # 利用离散傅里叶变换(FFT)提取基波相位
        x_comp = np.sum(amps * np.cos(phase_sweep))
        y_comp = np.sum(amps * np.sin(phase_sweep))
        phi_opt = math.atan2(y_comp, x_comp)
        if phi_opt < 0: 
            phi_opt += 2 * math.pi
            
        phi_calib = (geo_phases[i] - phi_opt) % (2 * math.pi)
        calib_time_us = (phi_calib / (2 * math.pi)) * PERIOD_US
        optimized_calib_us[i] = calib_time_us
        
        # 【关键修改】：这里绝不再把 phi_opt 写入 background_phases
        # 背景必须永远保持原样！防止滚雪球效应！
        
        print(f"  => 最优相位: {math.degrees(phi_opt):.1f}° | 校准延迟: {calib_time_us:.2f} us")

    print("\n========== 校准完成 ==========")
    lines = []
    for i in range(0, NUM_TRANSDUCERS, 5): 
        chunk = optimized_calib_us[i:i+5]
        lines.append("    " + ", ".join([f"{val:.2f}" for val in chunk]) + ",")
    if lines: lines[-1] = lines[-1].rstrip(',') 
    
    print("float Transducer_Calibration_Array[] = {")
    for line in lines: print(line)
    print("    0\n};")
    
    ser.close()
    scope.close()

if __name__ == '__main__':
    optimize_phases()