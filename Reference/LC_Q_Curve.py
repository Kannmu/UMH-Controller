import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. 设置电路参数 (根据你的描述)
# ==========================================
L = 6.8e-3        # 串联电感: 6.8 mH
C = 2.2e-9        # 换能器静态电容: 2.2 nF
R = 330 + 330     # 串联总电阻: 660 欧姆
Vcc = 12         # 假设供电电压为 5V (你可以根据实际情况修改)

# BTL 驱动下，施加在 LRC 网络两端的方波峰峰值约为 2 * Vcc
# 这里为了方便理解，我们按正弦波基波等效来计算频响，直接看电压放大倍数
V_in_pp = 2 * Vcc 

# ==========================================
# 2. 计算理论参数
# ==========================================
# 理论谐振频率 f0 = 1 / (2 * pi * sqrt(LC))
f0_hz = 1 / (2 * np.pi * np.sqrt(L * C))

# 理论品质因数 Q = (1/R) * sqrt(L/C)
Q_factor = (1 / R) * np.sqrt(L / C)

# ==========================================
# 3. 频率扫描计算 (20kHz 到 70kHz)
# ==========================================
freqs = np.linspace(20000, 70000, 2000) # 生成 20kHz 到 70kHz 的频率数组
omega = 2 * np.pi * freqs               # 角频率

# 计算系统传递函数的幅值 |H(jw)| = | V_c / V_in |
# H(jw) = (1 / jwC) / (R + jwL + 1/jwC) = 1 / (1 - w^2*LC + jwRC)
mag_H = 1 / np.sqrt((1 - omega**2 * L * C)**2 + (omega * R * C)**2)

# 计算实际施加在换能器上的峰峰值电压
V_out_pp = V_in_pp * mag_H

# ==========================================
# 4. 提取关键点 (谐振点、-3dB 带宽)
# ==========================================
max_idx = np.argmax(V_out_pp)
peak_f = freqs[max_idx]
peak_V = V_out_pp[max_idx]
peak_mag = mag_H[max_idx] # 谐振时的放大倍数，近似等于 Q 值

# 寻找 -3dB 点 (即峰值电压的 1/sqrt(2) 处)
threshold_V = peak_V / np.sqrt(2)
# 找到左侧和右侧最接近阈值的点
left_idx = np.argmin(np.abs(V_out_pp[:max_idx] - threshold_V))
right_idx = max_idx + np.argmin(np.abs(V_out_pp[max_idx:] - threshold_V))

f_low = freqs[left_idx]
f_high = freqs[right_idx]
bandwidth = f_high - f_low

# ==========================================
# 5. 绘制精美的可视化图表
# ==========================================
# 设置 matplotlib 全局字体和样式
plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial'] # 支持中文
plt.rcParams['axes.unicode_minus'] = False
plt.style.use('bmh') # 使用美观的内置样式

fig, ax1 = plt.subplots(figsize=(10, 6), dpi=120)

# 绘制 Vpp 曲线 (左侧 Y 轴)
color1 = 'tab:blue'
ax1.set_xlabel('频率 Frequency (kHz)', fontsize=12, fontweight='bold')
ax1.set_ylabel(f'换能器两端驱动电压 Vpp (V) \n[基于 Vcc={Vcc}V BTL驱动]', color=color1, fontsize=12, fontweight='bold')
line1, = ax1.plot(freqs / 1000, V_out_pp, color=color1, linewidth=2.5, label='换能器电压 Vpp')
ax1.tick_params(axis='y', labelcolor=color1)
ax1.set_xlim(20, 70)
ax1.grid(True, linestyle='--', alpha=0.7)

# 创建共享 X 轴的第二个 Y 轴，用于显示放大倍数/Q值
ax2 = ax1.twinx()  
color2 = 'tab:red'
ax2.set_ylabel('电压放大倍数 |H(f)| (约为 Q 值)', color=color2, fontsize=12, fontweight='bold')
line2, = ax2.plot(freqs / 1000, mag_H, color=color2, linewidth=2.5, linestyle='-.', alpha=0.5, label='电压放大倍数')
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_ylim(0, peak_mag * 1.2) # 留出顶部空间

# ==========================================
# 6. 添加标注和辅助线
# ==========================================
# 标记谐振峰
ax1.plot(peak_f / 1000, peak_V, marker='o', markersize=8, color='orange', zorder=5)
ax1.annotate(f'谐振点: {peak_f/1000:.2f} kHz\n最大电压: {peak_V:.1f} Vpp\n放大倍数(Q): {peak_mag:.2f}', 
             xy=(peak_f / 1000, peak_V), xytext=(peak_f / 1000 + 2, peak_V - (peak_V*0.1)),
             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="orange", lw=1.5),
             arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2", color='orange'),
             fontsize=11)

# 标记 -3dB 带宽
ax1.hlines(threshold_V, f_low / 1000, f_high / 1000, colors='green', linestyles='dashed', linewidth=2)
ax1.plot([f_low / 1000, f_high / 1000], [threshold_V, threshold_V], 'go', markersize=6)
ax1.annotate(f'-3dB 带宽: {bandwidth/1000:.2f} kHz\n({f_low/1000:.1f}k ~ {f_high/1000:.1f}k)', 
             xy=(peak_f / 1000, threshold_V), xytext=(peak_f / 1000 - 12, threshold_V - (peak_V*0.05)),
             bbox=dict(boxstyle="round,pad=0.3", fc="lightgreen", ec="green", alpha=0.8),
             fontsize=10)

# 图表标题和图例
plt.title('超声波换能器 LC 串联谐振频率响应分析', fontsize=16, fontweight='bold', pad=15)
fig.legend(handles=[line1, line2], loc='upper left', bbox_to_anchor=(0.12, 0.88), fontsize=11)

plt.tight_layout()
plt.show()