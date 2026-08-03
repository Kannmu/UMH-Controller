import seaborn as sns
import math
import numpy as np
import matplotlib.pyplot as plt

Voltage_Vpp = 24 # Voltage in Vpp
Voltages = np.linspace(0.01, 40, 100)  # 避免0值导致log10(0)错误
Base_SPL = 110

SPL_OUT_SINE = Base_SPL + 20 * np.log10(Voltages / 14.14)
SPL_OUT_SQUARE = Base_SPL + 20 * np.log10(0.09 * Voltages)

sns.lineplot(x=Voltages, y=SPL_OUT_SINE, label="Sine")
sns.lineplot(x=Voltages, y=SPL_OUT_SQUARE, label="Square")
plt.legend()
plt.xlabel("Voltage (Vpp)")
plt.ylabel("SPL (dB)")
plt.title("Voltage to SPL Conversion")
plt.show()

