import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === Load data ===
file_path = "~/MSc/Maggy2026V/observer/PCobserver/results/simulation_results.csv"
df = pd.read_csv(file_path)

# === Extract variables ===
t = df["t"].values

# True states
x = df["x"].values
y = df["y"].values
theta = df["z"].values
v = df["alpha"].values
omega = df["beta"].values

# Estimated states
x_est = df["x_est"].values
y_est = df["y_est"].values
theta_est = df["z_est"].values
v_est = df["alpha_est"].values
omega_est = df["beta_est"].values

# Standard deviations (absolute values assumed)
x_std = df["x_std"].values
y_std = df["y_std"].values
theta_std = df["z_std"].values
v_std = df["alpha_std"].values
omega_std = df["beta_std"].values

# === Figure layout ===
fig = plt.figure(figsize=(12, 6))
gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1])

# === Left: 2D trajectory ===
ax_traj = fig.add_subplot(gs[:, 0])
ax_traj.plot(x, y, label="True", linewidth=2)
ax_traj.plot(x_est, y_est, "--", label="Estimated")

# Estimated Start/stop
ax_traj.scatter(x_est[0], y_est[0], color='green', s=50, 
           marker='^', label='Estimated Start')
ax_traj.scatter(x_est[-1], y_est[-1], color='red', s=50, 
           marker='s', label='Estimated Stop')
           

# Anchor positions
anchor_posx = np.array([-1, 1, 0])
anchor_posy = np.array([-1, -1, 1])
ax_traj.scatter(anchor_posx, anchor_posy, color='blue', s=100, 
           marker='*', label='Anchor positions')

# Optional: confidence ellipse approximation (diagonal only)
ax_traj.fill_betweenx(
    y_est,
    x_est - x_std,
    x_est + x_std,
    alpha=0.1,
    label="±1σ (x)"
)

ax_traj.set_xlabel("x [m]")
ax_traj.set_ylabel("y [m]")
ax_traj.set_title("2D Position")
ax_traj.axis("equal")
ax_traj.grid()
ax_traj.legend()

# === Top right: theta ===
ax_theta = fig.add_subplot(gs[0, 1])
ax_theta.plot(t, theta, label="True θ [rad]", linewidth=2)
ax_theta.plot(t, theta_est, "--", label="Estimated θ [rad]")

ax_theta.fill_between(
    t,
    theta_est - theta_std,
    theta_est + theta_std,
    alpha=0.2,
    label="±1σ"
)

ax_theta.set_title("Heading")
ax_theta.set_xlabel("")
ax_theta.set_ylabel("θ [rad]")
ax_theta.grid()
ax_theta.legend()

# === Bottom right: v and omega ===
ax_vel = fig.add_subplot(gs[1, 1])

# v
ax_vel.plot(t, v, label="True v [m/s]", linewidth=2)
ax_vel.plot(t, v_est, "--", label="Estimated v")
ax_vel.fill_between(
    t,
    v_est - v_std,
    v_est + v_std,
    alpha=0.2
)

# omega
ax_vel.plot(t, omega, label="True ω [rad/s]", linewidth=2)
ax_vel.plot(t, omega_est, "--", label="Estimated ω")
ax_vel.fill_between(
    t,
    omega_est - omega_std,
    omega_est + omega_std,
    alpha=0.2
)

ax_vel.set_title("Velocity and Angular Rate")
ax_vel.set_xlabel("Time [s]")
ax_vel.grid()
ax_vel.legend()

# === Final layout ===
plt.savefig('test_sim_results.pdf', bbox_inches='tight')
plt.tight_layout()
plt.show()