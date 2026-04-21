import math
import numpy as np

# ==========================================
# 配置参数
# ==========================================
PERIOD_US = 25.0  # 40kHz 的周期为 25 微秒

# 请将你【手动测量】得到的校准参数填入此列表 (共60个换能器，不含最后一个0)
MANUAL_CALIB_ARRAY = [
    22.5, 0, 24.7, 24, 5.5,
    0, 15.2, 10, 22.5, 16, 12.5,
    9.6, 22.6, 2.6, 12.4, 2.3, 10.5, 9.5,
    24, 24, 24.5, 6.9, 17.9, 24.3, 8, 22.2,
    7.9, 16.2, 3.1, 5.9, 6.9, 14.3, 8.5, 22.8,
    24.5, 18.9, 11, 5.5, 18.3, 13.3, 21.8, 0,
    8.6, 23.1, 0, 24.5, 1.5, 23, 24.6,
    0, 16, 7.9, 23, 1.5, 0,
    6, 0, 19, 12.5, 20.6
]

# 请将你【自动校准脚本】算出的校准参数填入此列表
AUTO_CALIB_ARRAY = [
    21.62, 9.37, 6.08, 14.36, 13.18,
    10.50, 11.59, 4.30, 21.10, 8.08,
    21.98, 23.48, 21.38, 10.32, 23.05,
    11.43, 10.10, 1.30, 13.03, 23.85,
    7.89, 23.76, 8.80, 10.51, 21.69,
    22.72, 7.64, 7.76, 11.37, 0.05,
    1.73, 23.38, 23.03, 24.35, 7.55,
    23.54, 22.13, 23.37, 9.01, 18.78,
    21.55, 10.84, 0.00, 22.30, 13.01,
    9.42, 9.76, 22.72, 11.79, 14.88,
    10.88, 12.84, 21.60, 22.46, 10.68,
    9.99, 5.05, 16.18, 23.08, 23.75
]

# ==========================================
# 核心计算函数
# ==========================================
def calculate_cyclic_error(val1, val2, period):
    """计算考虑周期性的绝对误差"""
    raw_diff = abs(val1 - val2)
    # 实际误差是直接差值与跨越周期差值中的较小者
    return min(raw_diff, period - raw_diff)

def evaluate_calibration(manual, auto, period):
    if len(manual) != len(auto):
        print(f"警告：数组长度不一致！手动:{len(manual)}，自动:{len(auto)}")
        return
    
    errors = []
    for i in range(len(manual)):
        err = calculate_cyclic_error(manual[i], auto[i], period)
        errors.append(err)
        
    errors = np.array(errors)
    
    # 统计指标
    max_error = np.max(errors)
    mean_error = np.mean(errors)  # 平均绝对误差 (MAE)
    rmse = np.sqrt(np.mean(errors**2)) # 均方根误差 (RMSE)
    
    # 最大可能误差为半个周期 (12.5 us)
    max_possible_error = period / 2.0
    
    # 计算分数 (基于RMSE，100分为完全一致，0分为全部完全反相)
    # 公式：Score = 100 * (1 - RMSE / 最大可能误差)
    score = 100.0 * (1.0 - (rmse / max_possible_error))
    score = max(0, min(100, score)) # 限制在 0-100 之间
    
    # 打印详细结果
    print("="*50)
    print("        自动校准效果评估报告")
    print("="*50)
    print(f"通道总数      : {len(manual)}")
    print(f"评估周期      : {period} us (40kHz)")
    print("-" * 50)
    print(f"最大通道误差  : {max_error:.3f} us (位于通道 {np.argmax(errors)+1})")
    print(f"平均绝对误差  : {mean_error:.3f} us")
    print(f"均方根误差    : {rmse:.3f} us")
    print("-" * 50)
    print(f"⭐️ 综合校准得分: {score:.2f} / 100.00")
    print("="*50)
    
    # 打印异常大的通道（例如误差超过 1.0 us 的通道）
    tolerance = 1.0
    bad_channels = np.where(errors > tolerance)[0]
    if len(bad_channels) > 0:
        print(f"\n⚠️ 发现 {len(bad_channels)} 个误差较大的通道 ( > {tolerance} us):")
        for idx in bad_channels:
            print(f"  通道 {idx+1:02d}: 手动={manual[idx]:5.2f} us, 自动={auto[idx]:5.2f} us -> 误差={errors[idx]:5.2f} us")
    else:
        print(f"\n✅ 所有通道的一致性都非常高 (误差均 < {tolerance} us)！自动校准算法表现完美。")

if __name__ == "__main__":
    evaluate_calibration(MANUAL_CALIB_ARRAY, AUTO_CALIB_ARRAY, PERIOD_US)