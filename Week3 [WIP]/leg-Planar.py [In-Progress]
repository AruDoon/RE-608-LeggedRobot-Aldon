import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ─── Segment lengths ───────────────────────────────────────────────────────────
L_COXA  = 2.0
L_FEMUR = 3.0
L_TIBIA = 3.0

# ─── Base (hip) position ───────────────────────────────────────────────────────
BASE = np.array([0.0, 3.0])   # hip sits above the ground line (y=0)
GROUND_Y = 0.0

# ─── Forward Kinematics (side-view, all angles in world frame) ─────────────────
def forward_kinematics(base, theta1, theta2, theta3):
    """
    base   : (x, y) of hip joint
    theta1 : coxa  angle from horizontal (world)
    theta2 : femur angle relative to coxa
    theta3 : tibia angle relative to femur
    Returns: base, P1, P2, P3 (foot)
    """
    a1 = theta1
    P1 = base + L_COXA * np.array([np.cos(a1), np.sin(a1)])

    a2 = a1 + theta2
    P2 = P1 + L_FEMUR * np.array([np.cos(a2), np.sin(a2)])

    a3 = a2 + theta3
    P3 = P2 + L_TIBIA * np.array([np.cos(a3), np.sin(a3)])

    return base, P1, P2, P3

# ─── Gait cycle definition ────────────────────────────────────────────────────
#
#  One full cycle (0→1):
#    Stance phase [0, SWING_START): foot on ground, body moves forward → foot slides backward
#    Swing  phase [SWING_START, 1): foot lifts, swings forward, plants again
#
SWING_START = 0.60   # 60% stance, 40% swing

# Foot touchdown and liftoff positions (in world X, relative to hip)
FOOT_PLANT_X  =  2.2   # foot plants here (forward of hip)
FOOT_LIFTOFF_X = -2.2  # foot lifts here (behind hip)
SWING_HEIGHT  =  2.2   # peak height above ground during swing

N_FRAMES = 300
cycle_t = np.linspace(0, 1, N_FRAMES, endpoint=False)


def foot_world_pos(phase):
    """Return (x, y) of foot in world coords for a given gait phase [0,1)."""
    if phase < SWING_START:
        # Stance: foot fixed on ground, slides from plant→liftoff
        s = phase / SWING_START                        # 0→1 during stance
        x = FOOT_PLANT_X + s * (FOOT_LIFTOFF_X - FOOT_PLANT_X)
        y = GROUND_Y
    else:
        # Swing: foot lifts and swings forward with a smooth arc
        s = (phase - SWING_START) / (1.0 - SWING_START)   # 0→1 during swing
        x = FOOT_LIFTOFF_X + s * (FOOT_PLANT_X - FOOT_LIFTOFF_X)
        y = GROUND_Y + SWING_HEIGHT * np.sin(np.pi * s)   # half-sine arc
    return np.array([x, y])


def ik_2link(P1, target, L1, L2):
    """
    Analytical 2-link IK for femur+tibia given knee (P1) as origin.
    Returns (theta_femur_world, theta_tibia_relative), or None if unreachable.
    """
    d = np.linalg.norm(target - P1)
    d = np.clip(d, abs(L1 - L2) + 1e-6, L1 + L2 - 1e-6)

    # Elbow-down: knee bends backward (typical insect stance)
    cos_a = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_a = np.clip(cos_a, -1, 1)
    alpha = np.arccos(cos_a)          # angle at femur root inside triangle

    dir_angle = np.arctan2(target[1] - P1[1], target[0] - P1[0])
    theta_femur = dir_angle - alpha   # elbow-down solution

    # Tibia: from knee (P2) to foot
    P2 = P1 + L1 * np.array([np.cos(theta_femur), np.sin(theta_femur)])
    theta_tibia_world = np.arctan2(target[1] - P2[1], target[0] - P2[0])
    theta_tibia_rel   = theta_tibia_world - theta_femur

    return theta_femur, theta_tibia_rel


# Fixed coxa angle (pointing slightly downward-forward)
THETA1 = np.deg2rad(-30)

# Pre-compute frames
frames_data = []
foot_positions = []

for phase in cycle_t:
    foot_world = foot_world_pos(phase)
    foot_world_abs = BASE + np.array([foot_world[0], 0]) + np.array([0, foot_world[1] - BASE[1]])

    # Coxa tip (P1) in world space
    P1_world = BASE + L_COXA * np.array([np.cos(THETA1), np.sin(THETA1)])

    # IK for femur + tibia to reach the foot
    result = ik_2link(P1_world, foot_world, L_FEMUR, L_TIBIA)
    if result is None:
        frames_data.append(frames_data[-1])
        foot_positions.append(foot_world)
        continue

    theta_femur, theta_tibia_rel = result

    origin, P1, P2, P3 = forward_kinematics(BASE, THETA1, theta_femur - THETA1, theta_tibia_rel)
    frames_data.append((origin, P1, P2, P3))
    foot_positions.append(P3.copy())

foot_trace = np.array(foot_positions)

# ─── Plot setup ───────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 6))
fig.patch.set_facecolor('#0d0d1a')
ax.set_facecolor('#0d0d1a')

ax.set_xlim(-6, 6)
ax.set_ylim(-1.5, 7)
ax.set_aspect('equal')
ax.grid(True, color='#1e1e3a', linewidth=0.6, linestyle='--')
ax.tick_params(colors='#555577')
for spine in ax.spines.values():
    spine.set_edgecolor('#1e1e3a')

ax.set_title('Insect Leg – Side-View Walking Gait (FK + IK)', color='#aaaacc', fontsize=13, pad=12)
ax.set_xlabel('X', color='#555577')
ax.set_ylabel('Y', color='#555577')

# Ground line
ax.axhline(GROUND_Y, color='#445544', linewidth=2, linestyle='-', zorder=1)
ax.fill_between([-6, 6], GROUND_Y - 1.5, GROUND_Y, color='#111a11', zorder=0)
ax.text(-5.7, -1.2, 'ground', color='#335533', fontsize=8)

# Ghost foot path
ax.plot(foot_trace[:, 0], foot_trace[:, 1],
        color='#2a2a4a', linewidth=1.0, linestyle=':', zorder=1)

# Hip marker
ax.plot(*BASE, 'o', color='#ffffff', markersize=9, zorder=6)
ax.text(BASE[0] + 0.15, BASE[1] + 0.15, 'hip', color='#aaaacc', fontsize=8, zorder=7)

# ─── Animated artists ─────────────────────────────────────────────────────────
COXA_COLOR  = '#00ffcc'
FEMUR_COLOR = '#ff6680'
TIBIA_COLOR = '#66aaff'
FOOT_COLOR  = '#ffdd55'
JOINT_COLOR = '#ffffff'

line_coxa,  = ax.plot([], [], color=COXA_COLOR,  linewidth=4,   solid_capstyle='round', zorder=3)
line_femur, = ax.plot([], [], color=FEMUR_COLOR, linewidth=3,   solid_capstyle='round', zorder=3)
line_tibia, = ax.plot([], [], color=TIBIA_COLOR, linewidth=2.5, solid_capstyle='round', zorder=3)

TRAIL = 50
trail_x = np.full(TRAIL, np.nan)
trail_y = np.full(TRAIL, np.nan)
trail_line, = ax.plot([], [], color=FOOT_COLOR, linewidth=1.8, alpha=0.5, zorder=2)

foot_dot, = ax.plot([], [], 'o', color=FOOT_COLOR, markersize=9, zorder=6)

joint_dots = [
    ax.plot([], [], 'o', color=JOINT_COLOR, markersize=7, zorder=5)[0]
    for _ in range(3)   # P1 (coxa tip), P2 (knee)
]

# Phase indicator bar
phase_bar_bg = ax.axhline(-0.9, color='#333355', linewidth=8, xmin=0.02, xmax=0.98, zorder=7)
phase_line,  = ax.plot([], [], color='#00ffcc', linewidth=8, alpha=0.7, zorder=8)
ax.text(-5.5, -0.65, 'stance ◀', color='#00ffcc', fontsize=7)
ax.text( 1.0, -0.65, '▶ swing', color='#ffdd55', fontsize=7)
phase_label  = ax.text(0, -0.65, '', color='#ffffff', fontsize=8, ha='center', zorder=9)

# Swing phase coloring segment
swing_x_start = -6 + 12 * SWING_START
ax.axvline(swing_x_start, color='#333355', linewidth=0.8, ymin=0.0, ymax=0.07, zorder=7)

# Legend
ax.plot([], [], color=COXA_COLOR,  linewidth=3, label='coxa')
ax.plot([], [], color=FEMUR_COLOR, linewidth=3, label='femur')
ax.plot([], [], color=TIBIA_COLOR, linewidth=3, label='tibia')
ax.plot([], [], 'o', color=FOOT_COLOR, markersize=6, label='foot')
ax.legend(loc='upper right', facecolor='#1a1a2e', edgecolor='#333355',
          labelcolor='white', fontsize=9)


def init():
    line_coxa.set_data([], [])
    line_femur.set_data([], [])
    line_tibia.set_data([], [])
    trail_line.set_data([], [])
    foot_dot.set_data([], [])
    phase_line.set_data([], [])
    phase_label.set_text('')
    for d in joint_dots:
        d.set_data([], [])
    return line_coxa, line_femur, line_tibia, trail_line, foot_dot, phase_line, phase_label, *joint_dots


def update(frame):
    origin, P1, P2, P3 = frames_data[frame]
    phase = cycle_t[frame]

    line_coxa.set_data( [origin[0], P1[0]], [origin[1], P1[1]])
    line_femur.set_data([P1[0],     P2[0]], [P1[1],     P2[1]])
    line_tibia.set_data([P2[0],     P3[0]], [P2[1],     P3[1]])

    foot_dot.set_data([P3[0]], [P3[1]])

    for dot, pt in zip(joint_dots, [P1, P2]):
        dot.set_data([pt[0]], [pt[1]])

    # Rolling foot trail
    trail_x[:-1] = trail_x[1:]
    trail_y[:-1] = trail_y[1:]
    trail_x[-1] = P3[0]
    trail_y[-1] = P3[1]
    trail_line.set_data(trail_x, trail_y)
    if phase >= SWING_START:
        trail_line.set_color(FOOT_COLOR)
    else:
        trail_line.set_color('#334455')

    # Phase progress bar
    bar_x = -6 + 12 * phase
    phase_line.set_data([-6, bar_x], [-0.9, -0.9])
    if phase < SWING_START:
        phase_line.set_color('#00ffcc')
        phase_label.set_text(f'STANCE  {phase/SWING_START*100:.0f}%')
        phase_label.set_color('#00ffcc')
    else:
        phase_line.set_color('#ffdd55')
        phase_label.set_text(f'SWING  {(phase-SWING_START)/(1-SWING_START)*100:.0f}%')
        phase_label.set_color('#ffdd55')

    return line_coxa, line_femur, line_tibia, trail_line, foot_dot, phase_line, phase_label, *joint_dots


ani = animation.FuncAnimation(
    fig, update, frames=N_FRAMES,
    init_func=init, interval=16, blit=True
)

plt.tight_layout()
plt.show()