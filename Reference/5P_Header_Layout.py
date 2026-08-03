import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np

total_pin = 21

start_rotation = -30
end_rotation = 210
gap_rotation = 15

start_angle = 240
end_angle = 300 # Degree
gap_angle = 15

ring_radius = 58

for i in range(total_pin):
    if(i==2 or i==5 or i == 8 or i==11 or i==14):
        continue
    print(f"Pin {i} Position:{ring_radius*np.cos(np.fmod(start_rotation + i * gap_rotation, 360)*np.pi/180):.2f},{ring_radius*np.sin(np.fmod(start_rotation + i * gap_rotation, 360)*np.pi/180):.2f}", f"Angle: {np.fmod(start_angle + i * gap_angle, 360):.2f}°")


