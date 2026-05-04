"""
PID-Controlled Differential Drive Robot — Lissajous (∞) Trajectory Tracking
=============================================================================
Pure Python simulation.  Run with:
    python pid_lissajous_sim.py

Dependencies: numpy, matplotlib  (pip install numpy matplotlib)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.animation as animation

# ─────────────────────────── Simulation parameters ───────────────────────────
DT = 0.02          # time-step  (s)
T_TOTAL = 40.0     # total sim time (s)
STEPS = int(T_TOTAL / DT)

# Lissajous (infinity / figure-8) parameters
#   x(t) = A·sin(ω·t)
#   y(t) = B·sin(2·ω·t)
A = 3.0             # amplitude x
B = 1.5             # amplitude y
OMEGA = 0.3         # angular frequency  (rad/s)

# Robot physical limits
V_MAX = 2.0         # m/s
W_MAX = 3.0         # rad/s

# ─────────────────────────── PID controller class ────────────────────────────
class PID:
    """Simple PID with anti-windup clamp."""
    def __init__(self, kp, ki, kd, out_min=-np.inf, out_max=np.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integral = 0.0
        self._prev_err = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def compute(self, error, dt):
        self._integral += error * dt
        # Anti-windup clamp
        self._integral = np.clip(self._integral,
                                 self.out_min / max(self.ki, 1e-9),
                                 self.out_max / max(self.ki, 1e-9))
        derivative = (error - self._prev_err) / dt if dt > 0 else 0.0
        self._prev_err = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return np.clip(output, self.out_min, self.out_max)


# ─────────────────────── Reference trajectory ────────────────────────────────
def lissajous_ref(t):
    """Return (x, y, θ_ref) on the Lissajous curve at time t."""
    x = A * np.sin(OMEGA * t)
    y = B * np.sin(2.0 * OMEGA * t)
    # Analytical derivatives for reference heading
    dx = A * OMEGA * np.cos(OMEGA * t)
    dy = B * 2.0 * OMEGA * np.cos(2.0 * OMEGA * t)
    theta = np.arctan2(dy, dx)
    return x, y, theta


# ─────────── Differential-drive kinematics (unicycle model) ──────────────────
def diff_drive_step(state, v, w, dt):
    """Propagate  [x, y, θ]  by one time-step."""
    x, y, theta = state
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt
    # Wrap angle to [-π, π]
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return np.array([x, y, theta])


def angle_wrap(a):
    """Wrap angle to [-π, π]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


# ─────────────────────────── Main simulation ─────────────────────────────────
def run_simulation():
    # PID gains — tuned for smooth tracking
    pid_v = PID(kp=2.0, ki=0.1, kd=0.5, out_min=-V_MAX, out_max=V_MAX)
    pid_w = PID(kp=6.0, ki=0.05, kd=1.0, out_min=-W_MAX, out_max=W_MAX)

    # Initial robot state  (start at origin, facing +x)
    state = np.array([0.0, 0.0, 0.0])

    # Storage for plotting
    t_hist = np.zeros(STEPS)
    x_hist = np.zeros(STEPS)
    y_hist = np.zeros(STEPS)
    th_hist = np.zeros(STEPS)
    xr_hist = np.zeros(STEPS)
    yr_hist = np.zeros(STEPS)
    v_hist = np.zeros(STEPS)
    w_hist = np.zeros(STEPS)
    err_dist_hist = np.zeros(STEPS)
    err_head_hist = np.zeros(STEPS)

    for k in range(STEPS):
        t = k * DT
        xr, yr, thr = lissajous_ref(t)

        # ── Error computation ──
        ex = xr - state[0]
        ey = yr - state[1]
        dist_err = np.hypot(ex, ey)

        # Desired heading toward the reference point
        angle_to_ref = np.arctan2(ey, ex)
        heading_err = angle_wrap(angle_to_ref - state[2])

        # If the robot is very close, use the reference heading directly
        # to avoid oscillations at the reference point
        if dist_err < 0.05:
            heading_err = angle_wrap(thr - state[2])

        # ── PID outputs ──
        # Linear velocity proportional to distance error,
        # but reduced when heading is off to avoid overshooting
        cos_factor = np.cos(heading_err)  # project forward
        v_cmd = pid_v.compute(dist_err * cos_factor, DT)
        w_cmd = pid_w.compute(heading_err, DT)

        # Clamp
        v_cmd = np.clip(v_cmd, -V_MAX, V_MAX)
        w_cmd = np.clip(w_cmd, -W_MAX, W_MAX)

        # ── Propagate robot ──
        state = diff_drive_step(state, v_cmd, w_cmd, DT)

        # ── Store ──
        t_hist[k] = t
        x_hist[k] = state[0]
        y_hist[k] = state[1]
        th_hist[k] = state[2]
        xr_hist[k] = xr
        yr_hist[k] = yr
        v_hist[k] = v_cmd
        w_hist[k] = w_cmd
        err_dist_hist[k] = dist_err
        err_head_hist[k] = np.degrees(heading_err)

    return (t_hist, x_hist, y_hist, th_hist,
            xr_hist, yr_hist,
            v_hist, w_hist,
            err_dist_hist, err_head_hist)


# ─────────────────────────── Visualisation ───────────────────────────────────
def animate_results(data):
    (t_hist, x_hist, y_hist, th_hist,
     xr_hist, yr_hist,
     v_hist, w_hist,
     err_dist_hist, err_head_hist) = data

    # ── Full reference curve for background ──
    t_full = np.linspace(0, T_TOTAL, 2000)
    xref_full = A * np.sin(OMEGA * t_full)
    yref_full = B * np.sin(2.0 * OMEGA * t_full)

    # ── Figure layout ──
    fig = plt.figure(figsize=(16, 9), facecolor="#0f0f1a")
    fig.suptitle("PID Differential-Drive Robot  ·  Lissajous ∞ Tracking",
                 color="white", fontsize=16, fontweight="bold", y=0.97)

    gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.35,
                          left=0.06, right=0.97, top=0.91, bottom=0.08)

    # ── Trajectory panel (large) ──
    ax_traj = fig.add_subplot(gs[:, 0:2])
    ax_traj.set_facecolor("#151528")
    ax_traj.set_title("Trajectory", color="white", fontsize=13, pad=10)
    ax_traj.set_xlabel("x  (m)", color="#aaa")
    ax_traj.set_ylabel("y  (m)", color="#aaa")
    ax_traj.set_aspect("equal")
    ax_traj.grid(True, color="#252545", linewidth=0.5)
    ax_traj.tick_params(colors="#888")
    for spine in ax_traj.spines.values():
        spine.set_color("#333")

    # Reference path (faint)
    ax_traj.plot(xref_full, yref_full, '--', color="#555588", linewidth=1.2,
                 label="Reference ∞", zorder=1)

    # Robot trail (will grow during animation)
    trail_line, = ax_traj.plot([], [], '-', color="#00e5ff", linewidth=1.8,
                               label="Robot path", zorder=2)
    # Robot marker
    robot_dot, = ax_traj.plot([], [], 'o', color="#ff4081", markersize=8,
                              markeredgecolor="white", markeredgewidth=1.2,
                              zorder=4)
    # Heading arrow (will be updated)
    arrow_len = 0.4
    heading_line, = ax_traj.plot([], [], '-', color="#ff4081", linewidth=2,
                                 zorder=3)
    # Reference dot
    ref_dot, = ax_traj.plot([], [], 'o', color="#ffd740", markersize=6,
                            markeredgecolor="white", markeredgewidth=0.8,
                            zorder=4, label="Ref point")

    margin = 0.8
    ax_traj.set_xlim(-A - margin, A + margin)
    ax_traj.set_ylim(-B - margin, B + margin)
    ax_traj.legend(loc="upper right", fontsize=9, framealpha=0.4,
                   facecolor="#222", edgecolor="#555", labelcolor="white")

    # ── Distance-error panel ──
    ax_err = fig.add_subplot(gs[0, 2])
    ax_err.set_facecolor("#151528")
    ax_err.set_title("Tracking Error", color="white", fontsize=11, pad=8)
    ax_err.set_ylabel("dist (m)", color="#aaa", fontsize=9)
    ax_err.grid(True, color="#252545", linewidth=0.5)
    ax_err.tick_params(colors="#888", labelsize=8)
    for spine in ax_err.spines.values():
        spine.set_color("#333")
    err_line, = ax_err.plot([], [], '-', color="#76ff03", linewidth=1.2)
    ax_err.set_xlim(0, T_TOTAL)
    ax_err.set_ylim(0, max(err_dist_hist.max() * 1.2, 0.5))

    # ── Velocity commands panel ──
    ax_vel = fig.add_subplot(gs[1, 2])
    ax_vel.set_facecolor("#151528")
    ax_vel.set_title("Control Inputs", color="white", fontsize=11, pad=8)
    ax_vel.set_xlabel("time (s)", color="#aaa", fontsize=9)
    ax_vel.set_ylabel("cmd", color="#aaa", fontsize=9)
    ax_vel.grid(True, color="#252545", linewidth=0.5)
    ax_vel.tick_params(colors="#888", labelsize=8)
    for spine in ax_vel.spines.values():
        spine.set_color("#333")
    v_line, = ax_vel.plot([], [], '-', color="#00e5ff", linewidth=1.0, label="v (m/s)")
    w_line, = ax_vel.plot([], [], '-', color="#ff4081", linewidth=1.0, label="ω (rad/s)")
    ax_vel.set_xlim(0, T_TOTAL)
    ax_vel.set_ylim(min(w_hist.min(), v_hist.min()) * 1.2,
                    max(w_hist.max(), v_hist.max()) * 1.2)
    ax_vel.legend(loc="upper right", fontsize=8, framealpha=0.4,
                  facecolor="#222", edgecolor="#555", labelcolor="white")

    # ── Time text ──
    time_text = ax_traj.text(0.02, 0.96, "", transform=ax_traj.transAxes,
                             color="white", fontsize=11,
                             verticalalignment="top",
                             bbox=dict(boxstyle="round,pad=0.3",
                                       facecolor="#222", alpha=0.7,
                                       edgecolor="#555"))

    # ── Animation function ──
    skip = 4  # speed-up factor (render every Nth frame)

    def init():
        trail_line.set_data([], [])
        robot_dot.set_data([], [])
        heading_line.set_data([], [])
        ref_dot.set_data([], [])
        err_line.set_data([], [])
        v_line.set_data([], [])
        w_line.set_data([], [])
        time_text.set_text("")
        return (trail_line, robot_dot, heading_line, ref_dot,
                err_line, v_line, w_line, time_text)

    def update(frame):
        idx = frame * skip
        if idx >= STEPS:
            idx = STEPS - 1

        # Trail
        trail_line.set_data(x_hist[:idx], y_hist[:idx])

        # Robot position
        robot_dot.set_data([x_hist[idx]], [y_hist[idx]])

        # Heading arrow
        hx = x_hist[idx] + arrow_len * np.cos(th_hist[idx])
        hy = y_hist[idx] + arrow_len * np.sin(th_hist[idx])
        heading_line.set_data([x_hist[idx], hx], [y_hist[idx], hy])

        # Reference point
        ref_dot.set_data([xr_hist[idx]], [yr_hist[idx]])

        # Error plot
        err_line.set_data(t_hist[:idx], err_dist_hist[:idx])

        # Velocity plots
        v_line.set_data(t_hist[:idx], v_hist[:idx])
        w_line.set_data(t_hist[:idx], w_hist[:idx])

        # Time annotation
        time_text.set_text(f"t = {t_hist[idx]:.1f} s\n"
                           f"err = {err_dist_hist[idx]:.3f} m")

        return (trail_line, robot_dot, heading_line, ref_dot,
                err_line, v_line, w_line, time_text)

    n_frames = STEPS // skip
    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True,
                                  interval=DT * skip * 1000,  # ms
                                  repeat=False)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Running PID Lissajous infinity tracking simulation ...")
    data = run_simulation()

    # Print summary stats
    err = data[8]
    print(f"  Tracking error -- mean: {err.mean():.4f} m | "
          f"max: {err.max():.4f} m | final: {err[-1]:.4f} m")
    print("Launching animation ...")
    animate_results(data)
    print("Done.")
