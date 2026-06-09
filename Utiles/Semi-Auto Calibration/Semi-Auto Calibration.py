import time
import math
import pyvisa
import numpy as np
import msvcrt
import sys

# ==========================================
# 配置参数
# ==========================================
VISA_ADDR = 'USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR' # 替换为你的示波器VISA地址

NUM_TRANSDUCERS = 60
FREQ = 40000.0
PERIOD_US = 1e6 / FREQ

# ==========================================
# 核心：数字正交解调 (IQ Demodulation)
# ==========================================
def read_oscilloscope_iq(scope):
    """
    暂停示波器，读取差分波形(CH1-CH2)和参考波形(CH3)，使用数字锁相放大器提取 40kHz 复数向量。
    已修正时间量纲，并增加了严格的边界截断和调试输出。
    """
    scope.write(':STOP') 
    
    # 增加微小延时，确保示波器内部缓冲处理完毕，防止读取到跨界残波
    time.sleep(0.05) 
    
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
    _, _, v3 = get_channel_data(3)
    scope.write(':RUN')
    
    # 检查数据点是否一致
    if not (len(v1) == len(v2) == len(v3)):
        raise ValueError(f"通道数据长度不匹配! CH1:{len(v1)}, CH2:{len(v2)}, CH3:{len(v3)}")
    
    # 计算差分信号和参考信号
    volts_diff = v1 - v2
    volts_ref = v3
    
    # 构造绝对时间轴 (单位：秒)
    t = np.arange(len(volts_diff)) * xinc + xorig
    
    # ==========================================
    # 数据诊断与量纲统一 (统一使用秒 [s])
    # ==========================================
    t_span_s = t[-1] - t[0]         # 屏幕总时长 (秒)
    period_s = 1.0 / FREQ           # 物理目标周期 (秒，40kHz对应 25e-6)
    
    cycles = int(t_span_s / period_s) # 包含的完整整数周期数
    
    # 仅在第一次采集或发生异常时打印底部诊断信息（可根据需要修改条件，这里全部打印）
    # print(f"\n[DEBUG] 点数:{len(t)} | XINC:{xinc:.2e}s | 屏幕跨度:{t_span_s*1e6:.1f}us | 整数周期数:{cycles}")

    if cycles < 1:
        raise ValueError(f"示波器屏幕时间过短 ({t_span_s*1e6:.1f} us)，无法捕获完整周期 ({period_s*1e6:.1f} us)！")
    
    # 核心截断逻辑：只取完全覆盖整数个周期的有效点数
    valid_points = int(cycles * period_s / xinc)
    
    t_valid = t[:valid_points]
    volts_diff_valid = volts_diff[:valid_points]
    volts_ref_valid = volts_ref[:valid_points]
    
    # 数字 IQ 解调提取基波向量
    omega = 2.0 * math.pi * FREQ
    
    I_diff = np.mean(volts_diff_valid * np.cos(omega * t_valid))
    Q_diff = np.mean(volts_diff_valid * np.sin(omega * t_valid))
    V_diff = I_diff + 1j * Q_diff
    
    I_ref = np.mean(volts_ref_valid * np.cos(omega * t_valid))
    Q_ref = np.mean(volts_ref_valid * np.sin(omega * t_valid))
    V_ref = I_ref + 1j * Q_ref
    
    return V_diff, V_ref

def flush_keyboard_buffer():
    while msvcrt.kbhit():
        msvcrt.getch()

# ==========================================
# 主校准逻辑
# ==========================================
def run_semi_auto_calibration():
    print("="*65)
    print(" 🎯 超声相控阵 半自动校准系统")
    print("="*65)
    print("[!] 运行前请确保：")
    print("    1. 设备固件已进入 calibration_mode (KEY0 按下切换)。")
    print("    2. 示波器 CH1 和 CH2 通过差分连接到接收换能器。")
    print("    3. 示波器 CH3 连接到 PC10 作为相位基准。")
    print("    4. 示波器量程已调好，并开启合适的平均模式。\n")
    
    rm = pyvisa.ResourceManager()
    try:
        scope = rm.open_resource(VISA_ADDR)
        scope.timeout = 5000
        print(f"[+] 成功连接示波器: {scope.query('*IDN?').strip()}")
    except Exception as e:
        print(f"[-] 连接示波器失败: {e}")
        return

    final_calib_us = np.zeros(NUM_TRANSDUCERS)

    print("\n[操作说明]")
    print(" - 将接收换能器紧贴在目标通道上方。")
    print(" - 按下【空格键】开始当前通道的测量（自动采集2秒后结束并保存）。")
    print(" - 按下【q】键退出程序并输出已测数据。\n")

    try:
        for i in range(NUM_TRANSDUCERS):
            print("="*50)
            print(f" ▶ 准备校准通道 [{i+1:02d}/60]")
            print("="*50)
            print("请将接收器放置在当前通道上，然后按【空格键】开始测量...")
            
            flush_keyboard_buffer()
            # 等待开始按键
            exit_flag = False
            while True:
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key == b' ':
                        break
                    elif key.lower() == b'q':
                        exit_flag = True
                        break
                time.sleep(0.05)
                
            if exit_flag:
                break
                
            print("\n开始测量... (自动采集 2 秒数据)")
            measurements_cpx = []
            
            start_time = time.time()
            # 采集 2 秒数据
            while time.time() - start_time < 2.0:
                
                try:
                    V_diff, V_ref = read_oscilloscope_iq(scope)
                    
                    V_rel = V_diff / V_ref
                    amplitude = abs(V_diff) * 2.0
                    measured_arrival_phase = np.angle(V_rel)
                    
                    calib_phase = (-measured_arrival_phase) % (2.0 * math.pi)
                    calib_us = (calib_phase / (2.0 * math.pi)) * PERIOD_US
                    
                    # 保存复数形式以便求平均，避免相位卷绕问题
                    measurements_cpx.append(np.exp(1j * calib_phase))
                    
                    status = "✅" if amplitude > 0.003 else ("⚠️" if amplitude > 0.001 else "❌")
                    print(f"实时测量 | 幅值: {amplitude*1000:6.2f} mV | 相对相位: {math.degrees(measured_arrival_phase):+6.1f}° | 补偿延迟: {calib_us:5.2f} us {status}")
                    
                except Exception as e:
                    print(f"[!] 读取失败: {e}")
                    time.sleep(0.5)
                    
                time.sleep(0.05)
                
            if len(measurements_cpx) > 0:
                c_mean = np.mean(measurements_cpx)
                mean_phase = np.angle(c_mean) % (2 * np.pi)
                mean_calib_us = (mean_phase / (2 * np.pi)) * PERIOD_US
                final_calib_us[i] = mean_calib_us
                print(f"\n通道 [{i+1:02d}/60] 校准完成！平均补偿延迟: {mean_calib_us:5.2f} us\n")
            else:
                print(f"\n[!] 通道 [{i+1:02d}/60] 未获取到有效数据，已记录为 0 us。\n")

            if exit_flag:
                break

    except KeyboardInterrupt:
        print("\n[!] 用户强制中断校准。")
    finally:
        print("\n" + "="*60)
        print(" 🎉 校准数据收集结束！请将以下代码复制并替换 calibration.c 中的数组：")
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
        
        try:
            scope.close()
        except:
            pass
        print("\n[+] 仪器已断开连接。")

if __name__ == '__main__':
    run_semi_auto_calibration()
