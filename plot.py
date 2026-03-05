import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1. Read data from CSV file
# Update the filename to your own path if needed (e.g. 'vehicle_data.csv')
try:
    df = pd.read_csv('states.csv')
except FileNotFoundError:
    print("Error: 'states.csv' not found. Please check the filename.")
    exit()

# optional: read obstacles
obs_df = None
try:
    obs_df = pd.read_csv('obstacles.csv')
except FileNotFoundError:
    pass

# 2. create plotting area (3 rows, 3 columns)
fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(16, 12))
fig.suptitle('Vehicle Dynamics Simulation Data', fontsize=18, fontweight='bold')

# define a color palette
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

# --- PLOTS ---

# 1. Trajectory (XY plot)
axes[0, 0].plot(df['x'], df['y'], color=colors[0], linewidth=2)
if obs_df is not None:
    import matplotlib.patches as patches
    for _, row in obs_df.iterrows():
        # draw oriented rectangle (dimensions chosen arbitrarily, e.g. 2x1)
        w = 2.0; h = 1.0
        rect = patches.Rectangle((row['x']-w/2, row['y']-h/2), w, h,
                                 angle=np.degrees(row['psi']),
                                 edgecolor='red', facecolor='red', alpha=0.3)
        axes[0,0].add_patch(rect)
axes[0, 0].set_title('Vehicle Trajectory (XY)', fontweight='bold')
axes[0, 0].set_xlabel('X Position [m]')
axes[0, 0].set_ylabel('Y Position [m]')
axes[0, 0].grid(True)

# 2. Longitudinal and lateral speeds (vx, vy)
axes[0, 1].plot(df['time'], df['vx'], label='$V_x$', color=colors[1])
axes[0, 1].plot(df['time'], df['vy'], label='$V_y$', color=colors[2])
axes[0, 1].set_title('Velocity Components', fontweight='bold')
axes[0, 1].set_xlabel('Time [s]')
axes[0, 1].set_ylabel('Speed [m/s]')
axes[0, 1].legend()

# 3. Yaw angle (psi)
axes[0, 2].plot(df['time'], df['psi'], color=colors[3])
axes[0, 2].set_title('Yaw Angle ($\psi$)', fontweight='bold')
axes[0, 2].set_xlabel('Time [s]')
axes[0, 2].set_ylabel('Radians')

# 4. Side slip angle (beta)
axes[1, 0].plot(df['time'], df['beta'], color=colors[4])
axes[1, 0].set_title('Side Slip Angle ($\\beta$)', fontweight='bold')
axes[1, 0].set_xlabel('Time [s]')
axes[1, 0].set_ylabel('Radians')

# 5. Yaw Rate (r)
axes[1, 1].plot(df['time'], df['r'], color=colors[5])
axes[1, 1].set_title('Yaw Rate ($r$)', fontweight='bold')
axes[1, 1].set_xlabel('Time [s]')
axes[1, 1].set_ylabel('rad/s')

# 6. Steering angle (delta)
axes[1, 2].plot(df['time'], df['delta'], color=colors[6])
axes[1, 2].set_title('Steering Angle ($\delta$)', fontweight='bold')
axes[1, 2].set_xlabel('Time [s]')
axes[1, 2].set_ylabel('Radians')

# 7. Boyuna Kuvvet (Fx)
axes[2, 0].plot(df['time'], df['Fx'], color='black')
axes[2, 0].set_title('Longitudinal Force ($F_x$)', fontweight='bold')
axes[2, 0].set_xlabel('Time [s]')
axes[2, 0].set_ylabel('Newton [N]')

# hide unused empty subplots
axes[2, 1].axis('off')
axes[2, 2].axis('off')

# adjust layout so plots don't overlap
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# display the plot
plt.show()