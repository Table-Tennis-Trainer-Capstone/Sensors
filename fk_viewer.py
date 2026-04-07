#!/usr/bin/env python3
"""
Standalone arm FK visualizer — no ROS, no display needed.
Usage:
    python3 fk_viewer.py                       # draws HOME + ZERO + TEST presets
    python3 fk_viewer.py 0.0 0.0 0.0 0.0      # Base Upper Fore Wrist angles (rad)
Outputs: fk_view.png
"""
import sys, math, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── 3-D FK helpers ────────────────────────────────────────────────────────────
def rot_axis(axis, angle):
    ax = np.array(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    x, y, z = ax
    return np.array([
        [t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
        [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
        [t*x*z-s*y, t*y*z+s*x, t*z*z+c  ]])

def rpy_mat(r, p, y):
    return (rot_axis([0,0,1], y) @
            rot_axis([0,1,0], p) @
            rot_axis([1,0,0], r))

def T44(xyz=(0,0,0), rpy=(0,0,0)):
    M = np.eye(4)
    M[:3, 3]  = xyz
    M[:3, :3] = rpy_mat(*rpy)
    return M

def Tjoint(axis, angle):
    M = np.eye(4)
    M[:3, :3] = rot_axis(axis, angle)
    return M

def fk(base_a, upper_a, fore_a, wrist_a, paddle_a=0.0):
    """Return world-frame positions of: root, shoulder, elbow, wrist, tcp"""
    # World frame: X=lateral, Y=into table, Z=up (matches table tennis setup)
    # Robot root is 1.4732 m below world origin → Z = -1.4732 in world
    Tc = T44(xyz=(0, 0, -1.4732))           # world → root

    # root → Base (fixed, identity)
    Tc = Tc @ T44()

    # Base → Shoulder  (BaseRotate_0)
    Tc = Tc @ T44(xyz=(0,0,0), rpy=(0.01712, 0.04808, -0.38348))
    shoulder_pre = Tc[:3, 3].copy()
    Tc = Tc @ Tjoint([0.04806, -0.01710, -0.99869], base_a)
    shoulder = Tc[:3, 3].copy()

    # Shoulder → UpperArm  (UpperArmRotate_0)
    Tc = Tc @ T44(xyz=(-0.01750,-0.03317,0.10511), rpy=(-0.08249,-0.02327,0.02051))
    Tc = Tc @ Tjoint([0.93476,-0.35525,0], upper_a)
    elbow = Tc[:3, 3].copy()

    # UpperArm → Forearm  (ForeArmRotate_0)
    Tc = Tc @ T44(xyz=(-0.03355,-0.08830,0.45530), rpy=(-0.06873,0.02609,-0.00089))
    Tc = Tc @ Tjoint([0.93476,-0.35525,0], fore_a)
    wrist = Tc[:3, 3].copy()

    # Forearm → Wrist  (WristRotate_0)
    Tc = Tc @ T44(xyz=(-0.10957,-0.19888,-0.43388), rpy=(-0.84990,0.27810,3.0151))
    Tc = Tc @ Tjoint([0.93476,-0.35525,0], wrist_a)
    pre_tcp = Tc[:3, 3].copy()

    # Wrist → Paddle
    Tc = Tc @ T44(xyz=(-0.04174,-0.00429,0.02647))
    Tc = Tc @ Tjoint([0.20602,0.54209,-0.81467], paddle_a)

    # Paddle → TCP (fixed)
    Tc = Tc @ T44(xyz=(-0.033,-0.105,0.149), rpy=(-0.587,0.245,-1.933))
    tcp = Tc[:3, 3].copy()

    return shoulder_pre, shoulder, elbow, wrist, tcp

# ── Preset configurations ─────────────────────────────────────────────────────
configs = {
    'HOME':   dict(base=0.0,    upper=-0.2617, fore=0.5235,  wrist=0.2617,  paddle=-1.5708),
    'ZERO':   dict(base=0.0,    upper=0.0,     fore=0.0,     wrist=0.0,     paddle=0.0),
    'TEST':   dict(base=2.3778, upper=0.2513,  fore=-3.4858, wrist=-2.967,  paddle=2.6902),
}

# Override from command line: Base Upper Fore Wrist
if len(sys.argv) >= 5:
    b, u, f, w = map(float, sys.argv[1:5])
    p = float(sys.argv[5]) if len(sys.argv) >= 6 else 0.0
    configs = {'CUSTOM': dict(base=b, upper=u, fore=f, wrist=w, paddle=p)}

colors_map = {'HOME': 'tab:blue', 'ZERO': 'tab:green', 'TEST': 'tab:orange', 'CUSTOM': 'tab:red'}

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor='#111')
for ax in axes:
    ax.set_facecolor('#111')
    ax.tick_params(colors='#aaa')
    for spine in ax.spines.values():
        spine.set_color('#444')

ax_side, ax_top = axes

# Table geometry in world coords
# Table surface at Z=0 (world up), spans Y from -1.37 to +1.37 (into/out of table)
# Net at Y=0, height 0.1525m
# Robot at Y≈negative (behind table), base at Z=-1.4732

legend_patches = []

for name, cfg in configs.items():
    pts = fk(cfg['base'], cfg['upper'], cfg['fore'], cfg['wrist'], cfg['paddle'])
    # pts: shoulder_pre, shoulder, elbow, wrist, tcp
    # World frame: X=lateral, Y=into-table, Z=up
    c = colors_map.get(name, 'white')

    # Side view: Y (into table) vs Z (up/height)
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    ax_side.plot(ys, zs, '-o', color=c, linewidth=2, markersize=6, label=name)
    ax_side.annotate(f'{name}\nTCP y={ys[-1]:.2f} z={zs[-1]:.2f}',
                     xy=(ys[-1], zs[-1]), xytext=(ys[-1]+0.05, zs[-1]+0.05),
                     color=c, fontsize=7)

    # Top view: X (lateral) vs Y (into table)
    xs = [p[0] for p in pts]
    ax_top.plot(ys, xs, '-o', color=c, linewidth=2, markersize=6, label=name)

    legend_patches.append(mpatches.Patch(color=c, label=name))

# ── Side view decorations ─────────────────────────────────────────────────────
# Table surface
ax_side.axhline(0, color='#2a8', linewidth=3, label='table surface')
# Net
ax_side.plot([0, 0], [0, 0.1525], color='#44f', linewidth=3)
ax_side.text(0.02, 0.1525, 'net', color='#44f', fontsize=8)
# Robot column
ax_side.plot([configs.get('HOME',{'base':0})and-1.4732 or -1.4732]*2,
             [-0.2, 0], color='#555', linewidth=8)
ax_side.set_xlabel('Y — into table (m)', color='#aaa')
ax_side.set_ylabel('Z — height above table (m)', color='#aaa')
ax_side.set_title('SIDE VIEW (Y vs Z)', color='white')
ax_side.set_ylim(-1.6, 1.2)
ax_side.set_xlim(-2.2, 1.6)
ax_side.legend(handles=legend_patches, facecolor='#222', labelcolor='white')
ax_side.grid(color='#222', linewidth=0.5)
ax_side.axhline(0, color='#2a8', linewidth=1, alpha=0.3)
# Mark robot base
ax_side.plot([-1.4732], [-1.4732], 'x', color='#888', markersize=12)
ax_side.text(-1.6, -1.5, 'root\n(Z=-1.47)', color='#888', fontsize=7)

# ── Top view decorations ──────────────────────────────────────────────────────
# Table outline: X in [-0.7625, 0.7625], Y in [-1.37, 1.37]
rect_y = [-1.37, 1.37, 1.37, -1.37, -1.37]
rect_x = [-0.7625, -0.7625, 0.7625, 0.7625, -0.7625]
ax_top.plot(rect_y, rect_x, color='#555', linewidth=2)
ax_top.axhline(0, color='#333', linestyle='--')     # lateral center
ax_top.axvline(0, color='#44f', linewidth=2)         # net
ax_top.set_xlabel('Y — into table (m)', color='#aaa')
ax_top.set_ylabel('X — lateral (m)', color='#aaa')
ax_top.set_title('TOP VIEW (Y vs X)', color='white')
ax_top.set_xlim(-2.2, 1.6)
ax_top.set_ylim(-1.2, 1.2)
ax_top.legend(handles=legend_patches, facecolor='#222', labelcolor='white')
ax_top.grid(color='#222', linewidth=0.5)

# Print coordinates to terminal too
print("\n=== FK Results ===")
for name, cfg in configs.items():
    pts = fk(cfg['base'], cfg['upper'], cfg['fore'], cfg['wrist'], cfg['paddle'])
    labels = ['shoulder_pre', 'shoulder', 'elbow', 'wrist', 'TCP']
    print(f"\n{name} (Base={cfg['base']:.3f} Upper={cfg['upper']:.3f} Fore={cfg['fore']:.3f} Wrist={cfg['wrist']:.3f}):")
    for lbl, p in zip(labels, pts):
        print(f"  {lbl:15s}: X={p[0]:+.3f}  Y={p[1]:+.3f}  Z={p[2]:+.3f}  (height={p[2]:.3f}m, table_depth={p[1]:.3f}m)")

plt.tight_layout()
plt.savefig('/home/capstone-nano2/Sensors/fk_view.png', dpi=120, facecolor='#111')
print("\nSaved: /home/capstone-nano2/Sensors/fk_view.png")
