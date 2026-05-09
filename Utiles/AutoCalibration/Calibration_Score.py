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
20.43,  8.57,  8.93, 14.69, 11.57,
     8.63, 11.73, 11.55, 22.81, 10.97,
    20.52, 24.01, 20.55,  8.62, 22.61,
     9.88, 11.79,  1.30, 10.54, 22.19,
     9.25, 25.00, 10.66, 11.24, 22.00,
    22.94,  7.94, 12.19, 11.52,  0.34,
    23.42, 24.84, 23.42,  0.33, 12.94,
    21.72, 22.11, 24.10,  7.63, 20.42,
    22.76, 13.50, 23.43, 21.78,  9.11,
    11.06,  9.49,  2.23, 11.71, 13.74,
    10.54, 13.48, 23.15,  1.44, 11.17,
    10.43, 22.52, 18.61, 21.94, 21.21
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