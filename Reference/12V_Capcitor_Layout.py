import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np

start_angle = 315
end_angle = 225 # Degree
gap_angle = 270/30
total_pin = 31


for i in range(total_pin):
    print(f"Pin {i}: Angle: {np.fmod(start_angle + i * gap_angle, 360):.2f}°")


