# ── TTT System Parameters ─────────────────────────────────────────────────────
# Single source of truth for all tunable values.
# Imported by jetsonA.launch.py, jetsonB.launch.py, and marty_gui.py.
# Edit here and restart to apply changes everywhere.

PARAMS = {

    # ── Camera Hardware ───────────────────────────────────────────────────────
    'exposure':          10000,   # Increase for more light (Warning: Too high will drop FPS below 240)
    'analogue_gain':      1600,   # Increase for more digital brightness without dropping FPS
    'width':               640,
    'height':              400,
    'fps':                 240,

    # ── Ball Detection (vision_node) ──────────────────────────────────────────
    'min_area':               16,
    'max_area':             295,  # Tightened: ball is rarely >200px area. Rejects paddles/hands.
    'motion_threshold':     15,   # Increased: Rejects faint shadows and background sensor grain
    'min_contrast':         60,   # Increased: Forces moving objects to be distinctly bright
    'dilate_iters':          1,
    'edge_margin':          30,   # Increased: Ignores the outer 30px boundary of the lens to prevent glare tracking
    'kf_gate_px':          120.0, # Widened slightly: Allows the tracker to follow faster sudden movements
    'kf_process_noise':     0.08, # Increased: Reduces smoothing inertia so the tracker feels "snappy" again
    'table_roi_left':     [153, 157, 388, 81, 569, 194, 253, 395],
    'table_roi_right':    [294, 54, 524, 139, 411, 372, 120, 156],

    # ── Stereo Camera Intrinsic Lenses ────────────────────────────────────────
    'fx':                448.0,
    'fy':                448.0,
    'cx':                320.0,
    'cy':                200.0,
    
    # ── Stereo Alignment & Origin Shift (stereo_node) ─────────────────────────
    'baseline_m':        1.397,   # Distance between cameras
    'max_sync_age_ms':      35,   # Increased: Tolerates higher network jitter between Jetson A and B
    'net_dist_z':         0.61,   # Z-distance from cameras to the physical net
    'height_left':        0.7,   # Physical height of Left Cam
    'height_right':       0.53,   # Physical height of Right Cam
    'pan_left_deg':       19.3,
    'pan_right_deg':      23.5,
    'tilt_left_deg':      30.2,
    'tilt_right_deg':     29.8,
    'roll_left_deg':     -12.1,
    'roll_right_deg':      14.1,
    'limit_x_m':           1.5,   # 3D bounding box (width max +/-)
    'limit_y_top_m':       2.3,   # 3D bounding box (height max)
    'limit_y_bottom_m':   -0.2,   # 3D bounding box (height min)
    'limit_z_m':           2.5,   # 3D bounding box (depth max +/-)

    # ── Robust Trajectory Prediction (trajectory_node) ────────────────────────
    'lookahead_ms':        150,   # How far AFTER the bounce to intercept the ball (ms)
    'stage1_min_samples':    3,   # STAGE 1: Fast initial direction (3 samples @ 240fps = 12ms)
    'stage2_min_samples':    8,   # STAGE 2: Smoothed tracking (8 samples @ 240fps = 33ms)
    'max_samples':          15,   # Reduced: 15 samples @ 240fps = ~62ms. Less lag when recalculating arcs.
    'gravity':            9.81,   # m/s^2
    'table_y':             0.0,   # Y coordinate of table surface (0 = table surface is Y=0)
    'camera_tilt_deg':     0.0,   # Camera tilt correction (leave 0 unless cameras are angled)
    'restitution':        0.85,   # Energy retained after bounce (0–1)
    'min_incoming_speed':  0.5,   # MIN SPEED: Ignore shots slower than 0.5 m/s
    'net_margin_z':       -0.2,   # ORIGIN GATE: Shot must have crossed Z > -0.2m
    'max_track_z':         1.5,   # END OF TABLE: Ignore noise beyond this Z depth
    'max_velocity':        25.0,  # SPEED LIMIT: Ignore tracking jumps > 25 m/s (~56mph)

    # ── Arm Control (control_node) ────────────────────────────────────────────
    'update_rate_hz':     60.0,   # How often the control loop checks for new ball targets
    'planning_time_s':     0.05,   # Max time MoveIt spends finding a motion plan (keep short for real-time)
    'return_delay_ms':     50,   # Time (ms) to hold the swing position before returning home
    'speed_multiplier':    4.0,   # Overdrive scaling factor to bypass default URDF velocity limits
}