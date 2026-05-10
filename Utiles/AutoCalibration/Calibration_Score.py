import math
import numpy as np

PERIOD_US = 25.0

ARRAY_A = [
    18.57, 12.54, 22.36,  1.34,  4.99,
    10.08, 18.76,  5.22, 22.90,  9.44,
    17.35,  4.07, 20.98, 22.01, 19.02,
    24.13, 20.69,  7.88, 19.58, 23.13,
    4.95,  5.91, 19.30,  2.90,  8.00,
    14.51,  1.04,  6.52, 20.84,  7.03,
    4.89, 21.89,  8.81, 23.29, 13.62,
    23.63,  3.09, 16.09, 18.66,  3.64,
    4.63,  8.95,  4.99,  3.33, 19.20,
    13.19, 19.76,  8.32, 13.93, 24.39,
    5.82, 15.78,  4.58, 23.29, 20.76,
    4.08,  9.23,  3.37, 10.86,  3.53
]

ARRAY_B = [
    23.23, 17.31, 18.92, 8.48, 21.21,
    7.08, 15.13, 12.35, 16.95, 7.33,
    12.32, 2.85, 15.51, 5.70, 13.94,
    6.21, 15.40, 14.06, 3.78, 17.85,
    11.42, 4.67, 3.85, 10.15, 14.13,
    11.05, 14.55, 7.49, 5.13, 23.81,
    0.91, 6.14, 7.64, 16.81, 7.23,
    15.42, 8.85, 0.93, 1.55, 11.26,
    22.50, 15.48, 21.91, 1.88, 15.31,
    16.03, 13.26, 24.63, 0.60, 18.92,
    13.70, 1.56, 3.97, 17.46, 15.94,
    21.61, 14.91, 1.06, 16.68, 23.36
]

def calculate_cyclic_diff(val1, val2, period):
    raw_diff = abs(val1 - val2)
    return min(raw_diff, period - raw_diff)

def compare_arrays(name1, arr1, name2, arr2, period):
    if len(arr1) != len(arr2):
        print(f"警告：数组长度不一致！{name1}:{len(arr1)}，{name2}:{len(arr2)}")
        return
    
    diffs = []
    for i in range(len(arr1)):
        diff = calculate_cyclic_diff(arr1[i], arr2[i], period)
        diffs.append(diff)
        
    diffs = np.array(diffs)
    
    max_diff = np.max(diffs)
    mean_diff = np.mean(diffs)
    rmse = np.sqrt(np.mean(diffs**2))
    
    max_possible_diff = period / 2.0
    
    score = 100.0 * (1.0 - (rmse / max_possible_diff))
    score = max(0, min(100, score))
    
    print("="*50)
    print(f"        {name1} vs {name2} 相似度评估报告")
    print("="*50)
    print(f"通道总数      : {len(arr1)}")
    print(f"评估周期      : {period} us")
    print("-" * 50)
    print(f"最大通道差异  : {max_diff:.3f} us (位于通道 {np.argmax(diffs)+1})")
    print(f"平均绝对差异  : {mean_diff:.3f} us")
    print(f"均方根差异    : {rmse:.3f} us")
    print("-" * 50)
    print(f"⭐️ 相似度得分: {score:.2f} / 100.00")
    print("="*50)
    
    tolerance = 1.0
    bad_channels = np.where(diffs > tolerance)[0]
    if len(bad_channels) > 0:
        print(f"\n⚠️ 发现 {len(bad_channels)} 个差异较大的通道 ( > {tolerance} us):")
        for idx in bad_channels:
            print(f"  通道 {idx+1:02d}: {name1}={arr1[idx]:5.2f} us, {name2}={arr2[idx]:5.2f} us -> 差异={diffs[idx]:5.2f} us")
    else:
        print(f"\n✅ 所有通道的一致性都非常高 (差异均 < {tolerance} us)！")

def compare_multiple_arrays(arrays_dict, period):
    names = list(arrays_dict.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            compare_arrays(names[i], arrays_dict[names[i]], names[j], arrays_dict[names[j]], period)
            print("\n")

if __name__ == "__main__":
    arrays_to_compare = {
        "ArrayA": ARRAY_A,
        "ArrayB": ARRAY_B
    }
    compare_multiple_arrays(arrays_to_compare, PERIOD_US)
    