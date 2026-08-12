import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as patches

# --- Robot Parameters ---
WHEELBASE = 1.542
MAX_STEER = 1.5708  # 90 degrees

# --- Setup Plot ---
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.35)
ax.set_xlim(-2.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_title("Reverse Tricycle Kinematics (Top-Down View)")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")

# --- Draw Static Robot Body ---
# Center of rotation is (0,0). Rear wheel is at (-1.542, 0).
body = patches.Rectangle((-WHEELBASE, -0.4), WHEELBASE, 0.8, fill=False, edgecolor='blue', lw=2)
ax.add_patch(body)
ax.plot(0, 0, 'go', markersize=8, label="Front Axle (Center of Rotation)")
ax.plot(-WHEELBASE, 0, 'ko', markersize=5, label="Rear Pivot")

# --- Dynamic Elements ---
# The steerable rear wheel
wheel_width = 0.4
wheel_line, = ax.plot([], [], 'k-', lw=6, label="Rear Wheel")

# Velocity Vectors
vec_front_v = ax.quiver(0, 0, 0, 0, color='green', scale=1, scale_units='xy', angles='xy', width=0.008)
vec_rear_vx = ax.quiver(-WHEELBASE, 0, 0, 0, color='gray', scale=1, scale_units='xy', angles='xy', width=0.005)
vec_rear_vy = ax.quiver(-WHEELBASE, 0, 0, 0, color='gray', scale=1, scale_units='xy', angles='xy', width=0.005)
vec_rear_result = ax.quiver(-WHEELBASE, 0, 0, 0, color='red', scale=1, scale_units='xy', angles='xy', width=0.008, label="Resulting Drive Vector")

text_display = ax.text(-2.4, 1.1, "", fontsize=10, family='monospace', bbox=dict(facecolor='white', alpha=0.8))
ax.legend(loc='lower right', fontsize=8)

# --- Sliders ---
ax_v = plt.axes([0.2, 0.2, 0.65, 0.03])
ax_omega = plt.axes([0.2, 0.1, 0.65, 0.03])

slider_v = Slider(ax_v, 'Linear v (m/s)', -2.0, 2.0, valinit=0.5)
slider_omega = Slider(ax_omega, 'Angular $\omega$ (rad/s)', -1.5, 1.5, valinit=0.2)

def update(val):
    v = slider_v.val
    omega = slider_omega.val

    # ==========================================
    # YOUR EXACT CONTROLLER MATH
    # ==========================================
    v_x = v
    v_y = -omega * WHEELBASE

    if v_x == 0.0 and v_y == 0.0:
        target_steering = 0.0
        v_drive_linear = 0.0
    else:
        target_steering = math.atan2(v_y, v_x)
        v_drive_linear = math.hypot(v_x, v_y)

    flipped = False
    # Anti-flip optimization
    if target_steering > math.pi / 2:
        target_steering -= math.pi
        v_drive_linear = -v_drive_linear
        flipped = True
    elif target_steering < -math.pi / 2:
        target_steering += math.pi
        v_drive_linear = -v_drive_linear
        flipped = True

    # Clamp steering (visual only)
    target_steering = max(-MAX_STEER, min(MAX_STEER, target_steering))
    # ==========================================

    # Update Wheel Graphic (rotate a line around the pivot)
    dx = (wheel_width / 2) * math.cos(target_steering)
    dy = (wheel_width / 2) * math.sin(target_steering)
    wheel_line.set_data([-WHEELBASE - dx, -WHEELBASE + dx], [0 - dy, 0 + dy])

    # Update Vectors
    vec_front_v.set_UVC(v, 0)
    vec_rear_vx.set_UVC(v_x, 0)
    vec_rear_vy.set_UVC(0, v_y)
    
    # Resulting vector points in the direction of the wheel travel
    res_dx = v_drive_linear * math.cos(target_steering)
    res_dy = v_drive_linear * math.sin(target_steering)
    vec_rear_result.set_UVC(res_dx, res_dy)

    # Update Text
    flip_text = "YES" if flipped else "NO"
    text_display.set_text(
        f"Input v     : {v:+.2f} m/s\n"
        f"Input omega : {omega:+.2f} rad/s\n"
        f"--------------------------\n"
        f"Rear V_x    : {v_x:+.2f} m/s\n"
        f"Rear V_y    : {v_y:+.2f} m/s\n"
        f"180° Flipped: {flip_text}\n"
        f"Steer Angle : {math.degrees(target_steering):+.1f}°\n"
        f"Drive Speed : {v_drive_linear:+.2f} m/s"
    )
    fig.canvas.draw_idle()

slider_v.on_changed(update)
slider_omega.on_changed(update)

# Initialize
update(0)
plt.show()
