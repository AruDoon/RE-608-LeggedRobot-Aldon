import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

# ─── Segment lengths ───────────────────────────────────────────────────────────
L_COXA   = 3.0   # hip → first joint
L_FEMUR  = 3.0   # first joint → knee
L_TIBIA  = 3.0   # knee → foot tip

# ─── Forward Kinematics ────────────────────────────────────────────────────────
def forward_kinematics(theta1, theta2, theta3):
    """
    theta1 : coxa   angle (from X-axis, radians)
    theta2 : femur  angle (relative to coxa segment)
    theta3 : tibia  angle (relative to femur segment)
    Returns: origin, P1 (coxa end), P2 (femur end), P3 (tibia tip)
    """
    origin = np.array([0.0, 0.0])

    # Coxa end
    a1 = theta1
    P1 = origin + L_COXA * np.array([np.cos(a1), np.sin(a1)])

    # Femur end
    a2 = a1 + theta2
    P2 = P1 + L_FEMUR * np.array([np.cos(a2), np.sin(a2)])

    # Tibia tip
    a3 = a2 + theta3
    P3 = P2 + L_TIBIA * np.array([np.cos(a3), np.sin(a3)])

    return origin, P1, P2, P3

# ─── Analytic 2-link IK (femur + tibia) for given coxa angle ──────────────────
def ik_two_link(target, theta1):
    """
    Given target world position and fixed coxa angle, solve femur+tibia angles.
    Returns (theta2, theta3) relative angles, or None if unreachable.
    """
    # Coxa end position
    a1 = theta1
    P1 = L_COXA * np.array([np.cos(a1), np.sin(a1)])

    # Vector from P1 to target
    dx = target[0] - P1[0]
    dy = target[1] - P1[1]
    dist = np.hypot(dx, dy)

    max_reach = L_FEMUR + L_TIBIA
    min_reach = abs(L_FEMUR - L_TIBIA)
    if dist > max_reach:
        dist = max_reach   # clamp
    if dist < min_reach:
        dist = min_reach

    # Law of cosines for knee angle (absolute femur→tibia angle)
    cos_a = (L_FEMUR**2 + dist**2 - L_TIBIA**2) / (2 * L_FEMUR * dist)
    cos_a = np.clip(cos_a, -1, 1)
    alpha = np.arccos(cos_a)   # angle at P1 in triangle

    # Angle from P1 toward target
    gamma = np.arctan2(dy, dx)

    # Absolute angle of femur (elbow-down configuration)
    femur_abs = gamma - alpha

    # theta2 is relative to coxa direction
    theta2 = femur_abs - a1

    # Absolute angle of tibia
    cos_b = (L_FEMUR**2 + L_TIBIA**2 - dist**2) / (2 * L_FEMUR * L_TIBIA)
    cos_b = np.clip(cos_b, -1, 1)
    beta = np.arccos(cos_b)   # angle at knee

    # theta3 (relative, and it "bends" inward → negative)
    theta3 = np.pi - beta   # bend angle (elbow-down)

    return theta2, theta3

# ─── Build animation frames ────────────────────────────────────────────────────
N_FRAMES = 240

# # Target foot traces an ellipse in world space
# t_vals = np.linspace(0, 2 * np.pi, N_FRAMES, endpoint=False)
# foot_x = 7.0 + 3.5 * np.cos(t_vals)
# foot_y = -4.0 + 2.5 * np.sin(t_vals)

# Infinity / lemniscate path  (Bernoulli lemniscate parametric form)
# Centre is at world origin (= coxa base)
a = 6.0   # scale — tweak to taste
t_vals = np.linspace(0, 2 * np.pi, N_FRAMES, endpoint=False)

denom   = 1 + np.sin(t_vals)**2
foot_x  =  a * np.cos(t_vals) / denom
foot_y  =  a * np.sin(t_vals) * np.cos(t_vals) / denom

# Coxa sweeps a small oscillation to make the whole leg look alive
coxa_base = -np.pi / 6          # ~-30° rest angle
coxa_swing = 0.25 * np.sin(t_vals)   # small swing
# theta1_frames = coxa_base + coxa_swing
# Coxa fixed pointing right — IK handles the rest
# theta1_frames = np.full(N_FRAMES, 0.0)   # 0 rad = pointing along +X

# Replace the fixed coxa line with this:
# theta1_frames = np.arctan2(foot_y, foot_x) * 0.4  # coxa tracks foot direction

# Option 1 — bigger swing multiplier
theta1_frames = np.arctan2(foot_y, foot_x) * 0.7  # was 0.4


frames_data = []
for i in range(N_FRAMES):
    target = np.array([foot_x[i], foot_y[i]])
    th1 = theta1_frames[i]
    th2, th3 = ik_two_link(target, th1)
    origin, P1, P2, P3 = forward_kinematics(th1, th2, th3)
    frames_data.append((origin, P1, P2, P3))

# ─── Plot setup ────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 8))
fig.patch.set_facecolor('#0d0d1a')
ax.set_facecolor('#0d0d1a')

# ax.set_xlim(-5, 16)
# ax.set_ylim(-10, 10)
ax.set_xlim(-10, 10)
ax.set_ylim(-6, 6)
ax.set_aspect('equal')
ax.grid(True, color='#1e1e3a', linewidth=0.6, linestyle='--')
ax.tick_params(colors='#555577')
for spine in ax.spines.values():
    spine.set_edgecolor('#1e1e3a')
ax.set_title('Insect Leg – Coxa / Femur / Tibia  (IK)', color='#aaaacc',
             fontsize=13, pad=12)
ax.set_xlabel('X', color='#555577')
ax.set_ylabel('Y', color='#555577')

# Foot trace ghost
ax.plot(foot_x, foot_y, color='#2a2a4a', linewidth=1.2, linestyle=':', zorder=1)

COXA_COLOR  = '#00ffcc'
FEMUR_COLOR = '#ff6680'
TIBIA_COLOR = '#66aaff'
JOINT_COLOR = '#ffffff'

line_coxa,  = ax.plot([], [], color=COXA_COLOR,  linewidth=4, solid_capstyle='round', zorder=3)
line_femur, = ax.plot([], [], color=FEMUR_COLOR, linewidth=3, solid_capstyle='round', zorder=3)
line_tibia, = ax.plot([], [], color=TIBIA_COLOR, linewidth=2.5, solid_capstyle='round', zorder=3)

# Glowing foot trail
TRAIL = 30
trail_x = np.full(TRAIL, np.nan)
trail_y = np.full(TRAIL, np.nan)
trail_line, = ax.plot([], [], color=TIBIA_COLOR, linewidth=1.5, alpha=0.4, zorder=2)

joint_dots = [ax.plot([], [], 'o', color=JOINT_COLOR, markersize=7, zorder=5)[0]
              for _ in range(4)]

# Segment labels
lbl_coxa  = ax.text(0, 0, 'coxa',  color=COXA_COLOR,  fontsize=8, ha='center', zorder=6)
lbl_femur = ax.text(0, 0, 'femur', color=FEMUR_COLOR, fontsize=8, ha='center', zorder=6)
lbl_tibia = ax.text(0, 0, 'tibia', color=TIBIA_COLOR, fontsize=8, ha='center', zorder=6)

def midpoint(A, B):
    return (A + B) / 2

def init():
    line_coxa.set_data([], [])
    line_femur.set_data([], [])
    line_tibia.set_data([], [])
    trail_line.set_data([], [])
    for d in joint_dots:
        d.set_data([], [])
    return line_coxa, line_femur, line_tibia, trail_line, *joint_dots

def update(frame):
    origin, P1, P2, P3 = frames_data[frame]

    line_coxa.set_data([origin[0], P1[0]], [origin[1], P1[1]])
    line_femur.set_data([P1[0], P2[0]], [P1[1], P2[1]])
    line_tibia.set_data([P2[0], P3[0]], [P2[1], P3[1]])

    for dot, pt in zip(joint_dots, [origin, P1, P2, P3]):
        dot.set_data([pt[0]], [pt[1]])

    # Rolling trail
    trail_x[:-1] = trail_x[1:]
    trail_y[:-1] = trail_y[1:]
    trail_x[-1] = P3[0]
    trail_y[-1] = P3[1]
    trail_line.set_data(trail_x, trail_y)

    # Labels at segment midpoints
    m_c = midpoint(origin, P1)
    m_f = midpoint(P1, P2)
    m_t = midpoint(P2, P3)
    lbl_coxa.set_position((m_c[0] + 0.3, m_c[1] + 0.3))
    lbl_femur.set_position((m_f[0] + 0.3, m_f[1] + 0.3))
    lbl_tibia.set_position((m_t[0] + 0.3, m_t[1] + 0.3))

    return line_coxa, line_femur, line_tibia, trail_line, *joint_dots, lbl_coxa, lbl_femur, lbl_tibia

ani = animation.FuncAnimation(
    fig, update, frames=N_FRAMES,
    init_func=init, interval=16, blit=True
)

plt.tight_layout()

# Save as GIF
print("Saving animation… (this takes ~20 s)")
# ani.save('/mnt/user-data/outputs/leg_ik.gif', writer='pillow', fps=60, dpi=100)
plt.show()
print("Saved: leg_ik.gif")
plt.close()