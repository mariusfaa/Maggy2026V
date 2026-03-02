import matplotlib.pyplot as plt
import re
from parser import parse

# Parse data
data = parse('data_step.txt')

# Convert time to relative seconds
t0 = data['t'][0] if data['t'] else 0

time_s = [(t - t0) / 1e6 for t in data['t']]  # assuming microseconds

# Create subplots
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Plot magnetic field
axes[0].plot(time_s, data['bx'], label='bx')
axes[0].plot(time_s, data['by'], label='by')
axes[0].plot(time_s, data['bz'], label='bz')
axes[0].set_ylabel('Magnetic Field')
axes[0].legend()
axes[0].grid(True)
axes[0].set_title('Magnetic Field vs Time')

# Plot currents
axes[1].plot(time_s, data['ux'], label='ux')
axes[1].plot(time_s, data['uy'], label='uy')
axes[1].plot(time_s, data['i1'], label='i1')
# axes[1].plot(time_s, data['i2'], label='i2')
axes[1].plot(time_s, data['i3'], label='i3')
# axes[1].plot(time_s, data['i4'], label='i4')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Current')
axes[1].legend()
axes[1].grid(True)
axes[1].set_title('Current vs Time')

plt.tight_layout()
plt.show()
