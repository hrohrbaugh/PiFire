import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import time

from controller.pids_fo_protect import Controller

# === Simulation Parameters ===
dt = 10.0  # seconds
duration = 300  # seconds
set_point = 225  # target grill temp
time_axis = np.arange(0, duration, dt)

# === Generate synthetic temperature profile ===
# # Example: starts below setpoint, spikes up, then crashes (simulates overshoot then fireout risk)
# temperature_profile = []
# temp = 200
# for t in time_axis:
#     if t < 60:
#         temp += 0.5  # heating up
#     elif t < 120:
#         temp += 1.0  # rapid overshoot
#     elif t < 180:
#         temp -= 1.5  # fire weakens, temp drops fast
#     else:
#         temp -= 0.3  # slow decline
#     temperature_profile.append(temp)

# === Load Temperature Profile from Excel ===
excel_path = 'temperature_profile.xlsx'  # Change this to your Excel file path

# Expecting columns: Time (s), Temperature (°F)
df = pd.read_excel(excel_path)

# Ensure columns exist
if 'Time' not in df.columns or 'Temperature' not in df.columns:
    raise ValueError("Excel file must have 'Time' and 'Temperature' columns.")

time_axis = df['Time'].values
temperature_profile = df['Temperature'].values
dt = np.mean(np.diff(time_axis))  # Infer timestep

# === Controller Config ===
config = {
    "PB": 100.0,
    "Ti": 120.0,
    "Td": 20.0,
    "Ks": 0.05,
    "Kf": 1.0,
    "slope_thresh": 0.15,
    "decay_rate": 0.015,
    "target_fire_strength": 0.1,
    "fire_strength_thresh": 0.05
}

# === Dummy args for controller base
units = {}
cycle_data = {}

# === Initialize Controller ===
controller = Controller(config, units, cycle_data)
controller.set_target(set_point)

# === Run Controller on Simulated Data ===
pid_outputs = []
mod_outputs = []
fire_strengths = []
temps = []
p = []
i = []
d = []
s = []
f = []

# Reset fire strength for testing
controller.fire_strength = 0.0
controller.temp_last = temperature_profile[0]

# Downsample to the specified cycle time
time_axis_10s = time_axis[::10]
temperature_10s = temperature_profile[::10]

for t, temp in zip(time_axis_10s, temperature_10s):
    # Simulate elapsed time
    controller.last_update -= dt  # force dt = 1s
    u = controller.update(temp)
    pid_outputs.append(controller.u)
    mod_outputs.append(controller.mod_u)
    fire_strengths.append(controller.fire_strength)
    temps.append(temp)
    p.append(controller.p)
    i.append(controller.i)
    d.append(controller.d)
    s.append(controller.slope_comp)
    # f.append(controller.fire_correction)



# === Plotting ===
plt.figure(figsize=(12, 8))

plt.subplot(4, 1, 1)
plt.plot(time_axis, temperature_profile, label='Temperature (°F)', color='orange')
plt.axhline(set_point, color='gray', linestyle='--', label='Setpoint')
plt.ylabel("Temp (°F)")
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time_axis_10s, pid_outputs, label='Raw PID Output', linestyle='--')
plt.plot(time_axis_10s, mod_outputs, label='Modified Output (mod_u)', linewidth=2)
plt.ylim(-1,2)
plt.ylabel("Duty Cycle (u)")
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time_axis_10s, fire_strengths, label='Fire Strength', color='red')
plt.ylim(-1,2)
plt.ylabel("Fire Strength")
plt.xlabel("Time (s)")
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_axis_10s, p, label='P Gain', color='red')
plt.plot(time_axis_10s, i, label='I Gain', color='blue')
plt.plot(time_axis_10s, d, label='D Gain', color='green')
plt.plot(time_axis_10s, s, label='S Gain', color='orange')
# plt.plot(time_axis, f, label='F Gain', color='purple')
plt.ylim(-1,2)
plt.ylabel("PID Gains")
plt.xlabel("Time (s)")
plt.legend()

plt.tight_layout()
plt.show()
