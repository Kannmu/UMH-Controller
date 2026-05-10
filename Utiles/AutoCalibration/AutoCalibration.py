import time
import math
import struct
import serial
import pyvisa
import numpy as np

# ==========================================
# 配置参数
# ==========================================
SERIAL_PORT = 'COM6'  # 替换为你的串口
BAUD_RATE = 115200
VISA_ADDR = 'USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR' # 替换为你的示波器VISA地址

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

def send_enable_disable(ser, enable):
    """控制阵列的使能状态，用于控制功耗和发热"""
    cmd_type = 0x01
    data = [1 if enable else 0]
    frame = bytearray([0xAA, 0x55, cmd_type, len(data)])
    frame.extend(data)
    frame.append(calc_checksum(cmd_type, data))
    frame.extend([0x0D, 0x0A])
    ser.write(frame)
    state_str = "ENABLE (启动阵列)" if enable else "DISABLE (禁用阵列)"
    print(f"[CMD] 发送阵列状态控制: {state_str}")
    time.sleep(0.5)  # 给 MCU 留出响应和稳定的时间

def send_transducers(ser, phases, duties):
    """发送 60 个通道的相位和占空比给 MCU"""
    data = bytearray()
    for i in range(NUM_TRANSDUCERS):
        phase_val = float(phases[i])
        duty_val = float(duties[i])
        
        # 限制范围
        phase_val = phase_val % (2.0 * math.pi)
        if duty_val < 0.0: duty_val = 0.0
        if duty_val > 1.0: duty_val = 1.0
        
        # 量化
        phase_raw = int((phase_val / (2.0 * math.pi)) * 65535.0)
        duty_raw = int(duty_val * 255.0)
        
        if phase_raw > 65535: phase_raw = 65535
        if phase_raw < 0: phase_raw = 0
        if duty_raw > 255: duty_raw = 255
        if duty_raw < 0: duty_raw = 0
        
        # 存入 buffer (低字节在前)
        data.append(phase_raw & 0xFF)
        data.append((phase_raw >> 8) & 0xFF)
        data.append(duty_raw & 0xFF)
        
    frame = bytearray([0xAA, 0x55, 0x06, len(data)])
    frame.extend(data)
    frame.append(calc_checksum(0x06, data))
    frame.extend([0x0D, 0x0A])
    ser.write(frame)
    # 等待 DMA 更新并让示波器完成 64 次平均 (200Hz 触发下 64次大约需要 320ms)
    time.sleep(0.35) 

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

# ==========================================
# 核心：数字正交解调 (IQ Demodulation)
# ==========================================
def read_oscilloscope_iq(scope):
    """
    暂停示波器，读取差分波形，使用数字锁相放大器提取 40kHz 复数向量
    """
    scope.write(':STOP') 
    
    def get_channel_data(channel):
        scope.write(f":WAV:SOUR CHAN{channel}")
        scope.write(":WAV:MODE NORM")
        scope.write(":WAV:FORM BYTE")
        xinc = float(scope.query(":WAV:XINC?"))
        xorig = float(scope.query(":WAV:XOR?"))
        yinc = float(scope.query(":WAV:YINC?"))
        yorig = float(scope.query(":WAV:YOR?"))
        yref = float(scope.query(":WAV:YREF?"))
        raw_data = scope.query_binary_values(":WAV:DATA?", datatype='B', container=np.ndarray, header_fmt='ieee')
        volts = (raw_data - yref) * yinc - yorig
        return xinc, xorig, volts

    xinc, xorig, v1 = get_channel_data(1)
    _, _, v2 = get_channel_data(2)
    scope.write(':RUN')
    
    # 计算差分信号
    volts = v1 - v2
    
    # 构造绝对时间轴 (相对于 PC10 触发点的真实时间)
    t = np.arange(len(volts)) * xinc + xorig
    
    # 数字 IQ 解调提取基波向量
    omega = 2.0 * math.pi * FREQ
    I_comp = np.mean(volts * np.cos(omega * t))
    Q_comp = np.mean(volts * np.sin(omega * t))
    
    return I_comp + 1j * Q_comp

# ==========================================
# 单次校准流程
# ==========================================
def perform_single_calibration(ser, scope, positions, wave_k, run_idx):
    print(f"\n" + "="*60)
    print(f" ▶ 开始第 {run_idx} 次校准扫描 (时分复用提取)...")
    print("="*60)
    
    optimized_calib_us = np.zeros(NUM_TRANSDUCERS)
    
    for i in range(NUM_TRANSDUCERS):
        try:
            # 步骤 1：净化声场，关闭其他换能器，只打开第 i 个
            phases = np.zeros(NUM_TRANSDUCERS)
            duties = np.zeros(NUM_TRANSDUCERS)
            duties[i] = 0.5  # 50% 占空比最大输出
            
            # 步骤 2：测量正相声学矢量 Z_0
            phases[i] = 0.0
            send_transducers(ser, phases, duties)
            Z_0 = read_oscilloscope_iq(scope)
            
            # 步骤 3：测量反相声学矢量 Z_180
            phases[i] = math.pi
            send_transducers(ser, phases, duties)
            Z_180 = read_oscilloscope_iq(scope)
            
        except Exception as e:
            print(f"[!] 读取示波器失败: {e}")
            raise e

        # 步骤 4：完美的背景对消：提取第 i 个换能器的独立纯声学矢量，消除共模电磁干扰
        V_i = (Z_0 - Z_180) / 2.0
        
        amplitude = abs(V_i) * 2.0 # 实际差分电压峰峰值
        measured_arrival_phase = np.angle(V_i) # 测得的原生到达相位 (-pi 到 pi)
        
        # 步骤 5：物理闭环计算
        dist = np.linalg.norm(positions[i] - RX_POS)
        fw_phase = (dist * wave_k) % (2.0 * math.pi)
        
        calib_phase = (-measured_arrival_phase - fw_phase) % (2.0 * math.pi)
        calib_us = (calib_phase / (2.0 * math.pi)) * PERIOD_US
        optimized_calib_us[i] = calib_us
        
        # 详细打印中间变量
        status = "✅ 极优" if amplitude > 0.003 else ("⚠️ 偏弱" if amplitude > 0.001 else "❌ 极弱(检查换能器是否损坏)")
        print(f"通道 [{i+1:02d}/60] | 幅值: {amplitude*1000:6.2f} mV | 原生相位: {math.degrees(measured_arrival_phase):+6.1f}° | 理论前馈: {math.degrees(fw_phase):+6.1f}° | 补偿延迟: {calib_us:5.2f} us | {status}")

    return optimized_calib_us

# ==========================================
# 两次结果一致性分析
# ==========================================
def analyze_similarity(arr1, arr2):
    print("\n" + "="*60)
    print(" 📊 两次校准结果一致性分析")
    print("="*60)
    
    diff = arr1 - arr2
    # 考虑到相位的周期性，延迟差值可能在接近 1 周期时出现跳变
    # 例如：0.1us 和 24.9us (周期 25us) 实际上相差 0.2us，因此我们需要计算环形差值
    diff_wrapped = (diff + PERIOD_US / 2) % PERIOD_US - PERIOD_US / 2
    
    max_diff = np.max(np.abs(diff_wrapped))
    mean_diff = np.mean(np.abs(diff_wrapped))
    std_diff = np.std(diff_wrapped)
    
    print(f"最大绝对偏差: {max_diff:5.2f} us (约 {(max_diff/PERIOD_US)*360:5.1f}°)")
    print(f"平均绝对偏差: {mean_diff:5.2f} us (约 {(mean_diff/PERIOD_US)*360:5.1f}°)")
    print(f"偏差标准差:   {std_diff:5.2f} us")
    
    print("\n[逐通道对比 - 偏差 > 0.5 us 的通道]:")
    has_large_diff = False
    for i in range(NUM_TRANSDUCERS):
        if abs(diff_wrapped[i]) > 0.5:
            print(f"通道 [{i+1:02d}/60] | 运行1: {arr1[i]:5.2f} us | 运行2: {arr2[i]:5.2f} us | 偏差: {abs(diff_wrapped[i]):5.2f} us")
            has_large_diff = True
            
    if not has_large_diff:
        print("所有通道一致性良好！(所有偏差均 <= 0.5 us)")
        
    if max_diff < 1.0:
        print("\n结论：✅ 两次测量高度一致，校准值属于固有硬件偏差，数据极其可靠。")
    elif max_diff < 3.0:
        print("\n结论：⚠️ 存在轻微波动，可能受到环境气流或温度影响，但基本可用。")
    else:
        print("\n结论：❌ 存在较大偏差，校准值包含较多随机误差，请检查麦克风固定装置是否稳定，或考虑增加示波器平均次数。")

# ==========================================
# 主校准逻辑
# ==========================================
def run_calibration():
    print("="*65)
    print(" 🎯 超声相控阵终极校准系统 (时分复用 TDM + 正反相减去电磁串扰)")
    print("="*65)
    print("[!] 运行前请确保：")
    print("    1. 固件已支持 CMD_SET_TRANSDUCERS 指令。")
    print("    2. 示波器量程已调至最小 (建议 2mV/div 或 5mV/div)，并开启 64 次平均模式。")
    print("    3. 烧录的 calib 数组为全 0。\n")
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(VISA_ADDR)
    scope.timeout = 5000
    print(f"[+] 成功连接示波器: {scope.query('*IDN?').strip()}")
    
    try:
        # 开始校准前 Enable 阵列
        send_enable_disable(ser, True)
        
        scope.write(':RUN')
        time.sleep(1)
        
        positions = get_transducer_positions()
        wave_k = 2.0 * math.pi * FREQ / C_SOUND
        
        # 连续运行两次校准
        calib_1 = perform_single_calibration(ser, scope, positions, wave_k, 1)
        
        print("\n等待 2 秒后进行第二次验证校准...")
        time.sleep(2.0)
        
        calib_2 = perform_single_calibration(ser, scope, positions, wave_k, 2)
        
        # 相似性分析
        analyze_similarity(calib_1, calib_2)
        
        # 计算两次结果的环形均值
        c1 = np.exp(1j * (calib_1 / PERIOD_US * 2 * np.pi))
        c2 = np.exp(1j * (calib_2 / PERIOD_US * 2 * np.pi))
        c_mean = c1 + c2
        mean_phase = np.angle(c_mean) % (2 * np.pi)
        final_calib_us = (mean_phase / (2 * np.pi)) * PERIOD_US
        
        print("\n" + "="*60)
        print(" 🎉 校准完成！请将以下代码复制并替换 calibration.c 中的数组：")
        print(" (注：此处结果为两次校准的平均值，已过滤部分随机误差)")
        print("="*60 + "\n")
        
        lines = []
        for i in range(0, NUM_TRANSDUCERS, 5): 
            chunk = final_calib_us[i:i+5]
            lines.append("    " + ", ".join([f"{val:5.2f}" for val in chunk]) + ",")
        if lines: lines[-1] = lines[-1].rstrip(',') 
        
        print("float Transducer_Calibration_Array[] = {")
        for line in lines: 
            print(line)
        print("    0\n};")
        
        # 恢复状态：归零相位并开启所有通道占空比
        send_transducers(ser, np.zeros(NUM_TRANSDUCERS), np.ones(NUM_TRANSDUCERS) * 0.5)
        
    except Exception as e:
        print(f"\n[!] 运行过程中发生错误: {e}")
    finally:
        # 无论成功还是异常退出，都确保 Disable 阵列，降低功耗和发热
        send_enable_disable(ser, False)
        ser.close()
        scope.close()
        print("\n[+] 串口和仪器已断开连接，阵列已禁用。")

if __name__ == '__main__':
    run_calibration()

