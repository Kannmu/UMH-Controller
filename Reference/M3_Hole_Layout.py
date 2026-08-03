import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np

start_angle = -90+54.010
end_angle = 215.99 # Degree
gap_angle = 28 # Degree
total_pin = 10

angle_offset = 5
radius = 68

postion_dict = {}

for i in range(total_pin):
    current_center_angle = start_angle + i * gap_angle
    current_left_angle = current_center_angle - angle_offset
    current_right_angle = current_center_angle + angle_offset
    
    postion_dict[i] = {
        
        'center_angle': current_center_angle,
        'left_angle': current_left_angle,
        'right_angle': current_right_angle,
        
        'center_position_x': radius * np.cos(np.deg2rad(current_center_angle)),
        'center_position_y': radius * np.sin(np.deg2rad(current_center_angle)),

        'left_position_x': radius * np.cos(np.deg2rad(current_left_angle)),
        'left_position_y': radius * np.sin(np.deg2rad(current_left_angle)),

        'right_position_x': radius * np.cos(np.deg2rad(current_right_angle)),
        'right_position_y': radius * np.sin(np.deg2rad(current_right_angle)),
    }

df = pd.DataFrame(postion_dict).T
print(df.iloc[:,5:])




# Visualization

plt.figure(figsize=(10, 10))
sns.scatterplot(x='center_position_x', y='center_position_y', data=df,color='red')
sns.scatterplot(x='left_position_x', y='left_position_y', data=df,color='blue')
sns.scatterplot(x='right_position_x', y='right_position_y', data=df,color='green')
plt.axis('equal')
# plt.show()


