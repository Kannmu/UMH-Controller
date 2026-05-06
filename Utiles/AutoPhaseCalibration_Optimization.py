import time
import math
import struct
import serial
import pyvisa
import numpy as np
import matplotlib.pyplot as plt

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

# SPSA 超参数 (根据实际电压反馈可能需要微调)
ITERATIONS = 150       # 总迭代次数
A_GAIN = 10           # 梯度更新步长增益 (控制学习率)
C_PERTURB = 0.5        # 微扰大小 (弧度，约等于28度)
ALPHA = 0.602
GAMMA = 0.101
A_OFFSET = ITERATIONS * 0.1

# ==========================================
# 通信与几何计算
# ==========================================
def calc_checksum(cmd_type, data):
    return (cmd_type + len(data) + sum(data)) & 0xFF

def send_phases(ser, phases):
    # 保证相位在 0~2pi 之间
    phases_wrapped = np.mod(phases, 2.0 * np.pi)
    data = struct.pack(f'<{NUM_TRANSDUCERS}f', *phases_wrapped)
    frame = bytearray([0xAA, 0x55, 0x06, len(data)])
    frame.extend(data)
    frame.append(calc_checksum(0x06, data))
    frame.extend([0x0D, 0x0A])
    ser.write(frame)
    time.sleep(0.02)

def get_geometric_phase_delays():
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
        
    wave_k = 2.0 * math.pi * FREQ / C_SOUND
    return np.array([(np.linalg.norm(pos - RX_POS) * wave_k) % (2.0 * math.pi) for pos in positions])

# ==========================================
# 硬件测量 (VRMS)
# ==========================================
def measure_amplitude(scope):
    scope.write(':CLEAR')
    time.sleep(0.4) # 等待示波器刷新(建议将平均次数设为 8 或 16)
    try:
        val = float(scope.query(':MEASure:ITEM? VRMS,CHANnel2').strip())
        if val > 9e30: return 0.0 # 超量程保护
        return val
    except Exception as e:
        return 0.0
# ==========================================
# 实时可视化设置 (续)
# ==========================================
def setup_plots():
    plt.ion()
    # 创建上下两个子图：上图显示幅度趋势，下图显示当前的校准微秒值分布
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title('SPSA 全局相控阵联合优化器')
    
    # 上图：目标声压 (VRMS) 进化曲线
    line_amp, = ax1.plot([], [], 'b-', linewidth=2, label='Current Vrms')
    line_best, = ax1.plot([], [], 'r--', linewidth=2, label='Best Vrms')
    ax1.set_xlim(0, ITERATIONS)
    ax1.set_ylim(0, 5) # 初始化Y轴，后续自动拉伸
    ax1.set_title('Global Focal Amplitude Optimization (SPSA)', fontsize=14)
    ax1.set_ylabel('Amplitude (Vrms)')
    ax1.set_xlabel('Iteration')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend()
    
    # 下图：实时校准参数 (0~25 us)
    bars = ax2.bar(range(NUM_TRANSDUCERS), np.zeros(NUM_TRANSDUCERS), color='teal', alpha=0.7)
    ax2.set_xlim(-1, NUM_TRANSDUCERS)
    ax2.set_ylim(0, 25.0)
    ax2.set_title('Real-time Transducer Calibration Delays', fontsize=14)
    ax2.set_ylabel('Delay (us)')
    ax2.set_xlabel('Transducer Channel Index')
    ax2.grid(axis='y', linestyle='--', alpha=0.6)
    
    plt.tight_layout()
    return fig, ax1, ax2, line_amp, line_best, bars

# ==========================================
# 主优化逻辑 (SPSA核心算法)
# ==========================================
def optimize_phases():
    print("="*60)
    print("  超声相控阵全局声场联合优化器 (SPSA 神级算法)")
    print("="*60)
    print("【准备工作】")
    print(" 1. 请在示波器上将获取(Acquire)模式设为平均(Average)，推荐 8 次。")
    print(" 2. 将 CH1 接入 PC10 并设为触发源(Trigger Source)。")
    print(" 3. 脚本将同时向60个通道施加微扰，全面逼近绝对最强点！\n")
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rm = pyvisa.ResourceManager()
    scope = rm.open_resource(VISA_ADDR)
    scope.timeout = 5000
    print(f"成功连接示波器: {scope.query('*IDN?').strip()}")
    scope.write(':CHANnel2:DISPlay ON')
    
    # 提前计算几何飞行相位
    geo_phases = get_geometric_phase_delays()
    
    # 图表初始化
    fig, ax1, ax2, line_amp, line_best, bars = setup_plots()
    history_amp = []
    history_best = []
    
    # SPSA 初始状态：可以将所有通道初始化为理论焦点相位，或者全部赋 0 盲搜
    # 这里我们使用 0 作为初始猜测值
    theta = np.zeros(NUM_TRANSDUCERS)
    
    best_theta = np.copy(theta)
    best_amp = -1.0
    
    print("\n🚀 开始执行 SPSA 并行迭代...")
    
    for k in range(ITERATIONS):
        # 1. 计算当前迭代的步长 (学习率) 和 微扰半径
        ak = A_GAIN / (k + 1.0 + A_OFFSET)**ALPHA
        ck = C_PERTURB / (k + 1.0)**GAMMA
        
        # 2. 生成伯努利随机微扰向量 (同时对60个通道施加 +1 或 -1 的随机翻转)
        # SPSA的灵魂所在：仅用1个随机向量，就能算出60维的伪梯度
        delta = np.random.choice([-1, 1], size=NUM_TRANSDUCERS)
        
        # 3. 施加正向微扰并测量
        theta_plus = theta + ck * delta
        send_phases(ser, theta_plus)
        y_plus = measure_amplitude(scope)
        
        # 4. 施加反向微扰并测量
        theta_minus = theta - ck * delta
        send_phases(ser, theta_minus)
        y_minus = measure_amplitude(scope)
        
        # 5. 计算梯度估计并更新参数 (注意：我们是求最大值，所以是加上梯度)
        # gk 包含了 60 个通道各自的调整方向和幅度
        gk = (y_plus - y_minus) / (2.0 * ck) * delta
        theta = theta + ak * gk
        
        # 将相位规范化到 [0, 2pi) 之间
        theta = np.mod(theta, 2.0 * math.pi)
        
        # 6. 测量当前更新后的中心状态 (可选，但为了绘图和记录最佳值)
        send_phases(ser, theta)
        y_current = measure_amplitude(scope)
        
        # 记录史上最强声压
        if y_current > best_amp:
            best_amp = y_current
            best_theta = np.copy(theta)
            
        # ==========================================
        # 动态更新可视化与计算校准参数
        # ==========================================
        history_amp.append(y_current)
        history_best.append(best_amp)
        
        # 动态拉伸Y轴
        if best_amp * 1.2 > ax1.get_ylim()[1]:
            ax1.set_ylim(0, best_amp * 1.2)
            
        line_amp.set_data(range(len(history_amp)), history_amp)
        line_best.set_data(range(len(history_best)), history_best)
        
        # 实时计算校准延迟 (us) 并更新柱状图
        calib_phases = (geo_phases - theta) % (2.0 * math.pi)
        calib_us = (calib_phases / (2.0 * math.pi)) * PERIOD_US
        
        for bar, val in zip(bars, calib_us):
            bar.set_height(val)
            
        fig.canvas.draw()
        fig.canvas.flush_events()
        
        print(f"[Iter {k+1:03d}/{ITERATIONS}] 步长ak:{ak:.3f} | 微扰ck:{ck:.3f} | 当前Vrms: {y_current:.3f} V | 历史极值: {best_amp:.3f} V")

    # ==========================================
    # 迭代完成，输出最终的最优结果
    # ==========================================
    print("\n🎉 SPSA 迭代优化完毕！")
    print(f"最终在焦点处寻找到的绝对最强声压为: {best_amp:.4f} Vrms")
    
    # 使用最佳的 theta 计算最终的 Calibration Array
    final_calib_phases = (geo_phases - best_theta) % (2.0 * math.pi)
    final_calib_us = (final_calib_phases / (2.0 * math.pi)) * PERIOD_US
    
    # 把相控阵设定到这个最强状态供你欣赏
    send_phases(ser, best_theta)
    
    print("\n========== 最终校准参数矩阵 ==========")
    lines = []
    for i in range(0, NUM_TRANSDUCERS, 5): 
        chunk = final_calib_us[i:i+5]
        lines.append("    " + ", ".join([f"{val:5.2f}" for val in chunk]) + ",")
    if lines: lines[-1] = lines[-1].rstrip(',') 
    
    print("float Transducer_Calibration_Array[] = {")
    for line in lines: print(line)
    print("    0\n};")
    
    ser.close()
    scope.close()
    
    print("\n按回车键退出并关闭动态图表...")
    input()

if __name__ == '__main__':
    optimize_phases()