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
    'min_area':             10,   # Ignore tiny 4x5 pixel noise fragments
    'max_area':             250,  # Tightened: rejects large background objects/paddles, but allows the ball near the net
    'motion_threshold':     12,   # prevents dropping the ball when it motion-blurs
    'min_contrast':         29,   # Mandates the object be bright white/orange. Rejects dark matte paddles.
    'dilate_iters':          2,   # merges broken paddle edge fragments into one giant blob > max_area
    'edge_margin':          10,   # Reclaims the outer edges of the camera lens
    'kf_gate_px':          400.0, # Massive gate ensures the ball is never dropped on sudden fast hits
    'kf_process_noise':     2.0,  # High process noise forces the tracker to instantly adopt new velocities
    'consistency_min':       1,
    'frame_delay':           5,   # Frames back for motion diff (5 = 21ms @ 240fps; less smear)
    'max_aspect_ratio':      8.0, # Relaxed: Extremely fast balls motion-blur into very long streaks
    'table_roi_left':     [231, 212, 436, 181, 560, 266, 255, 351],
    'table_roi_right':    [264, 106, 468, 132, 446, 272, 142, 191],

    # ── Stereo Camera Intrinsic Lenses ────────────────────────────────────────
    'fx':                448.0,
    'fy':                448.0,
    'cx':                320.0,
    'cy':                200.0,
    
    # ── Stereo Alignment & Origin Shift (stereo_node) ─────────────────────────
    'baseline_m':        1.397,   # Distance between cameras
    'max_sync_age_ms':      35,   # Increased: Tolerates higher network jitter between Jetson A and B
    'net_dist_z':         0.93,   # Z-distance from cameras to the physical net
    'height_left':        0.85,   # Physical height of Left Cam
    'height_right':       0.85,   # Physical height of Right Cam
    'pan_left_deg':       13.5,
    'pan_right_deg':      21.6,
    'tilt_left_deg':      20.7,
    'tilt_right_deg':     30.2,
    'roll_left_deg':     -4.3,
    'roll_right_deg':      0.2,
    'limit_x_m':           1.5,   # 3D bounding box (width max +/-)
    'limit_y_top_m':       1.5,   # 3D bounding box (height max, ~2 ft above table)
    'limit_y_bottom_m':   -0.2,   # 3D bounding box (height min)
    'limit_z_m':           1.25,  # 3D bounding box: Ignore the last 12cm of the table to blind the system to the paddle

    # ── Robust Trajectory Prediction (trajectory_node) ────────────────────────
    'lookahead_ms':        50,   # How far AFTER the bounce to intercept the ball (ms)
    'stage1_min_samples':    2,   # STAGE 1: Fast initial direction
    'stage2_min_samples':    5,   # STAGE 2: Smoothed tracking
    'max_samples':          80,   # Reduced: 15 samples @ 240fps = ~62ms. Less lag when recalculating arcs.
    'gravity':            9.81,   # m/s^2
    'table_y':             0.0,   # Y coordinate of table surface (0 = table surface is Y=0)
    'camera_tilt_deg':     0.0,   # Camera tilt correction (leave 0 unless cameras are angled)
    'restitution':        0.85,   # Energy retained after bounce (0–1)
    'min_incoming_speed':  0.5,   # MIN SPEED: Ignore shots slower than 0.5 m/s
    'net_margin_z':       -0.2,   # ORIGIN GATE: Shot must have crossed Z > -0.2m
    'max_track_z':         1.25,  # END OF TABLE: Matches limit_z_m to ignore paddle strikes
    'max_y':               1.5,   # HEIGHT LIMIT: Highest valid ball altitude (m) over the table
    'max_velocity':        60.0,  # SPEED LIMIT: Raised further to prevent dropping frames on fast smashes

    # ── Arm Control (control_node) ────────────────────────────────────────────
    'update_rate_hz':     240.0,   # How often the control loop checks for new ball targets
    'planning_time_s':     0.05,    # Increased to give the KDL IK solver enough time to find constrained goal states
    'return_delay_ms':     500,     # Time (ms) to hold the swing position before returning home
    'speed_multiplier':    5.0,    # Overdrive scaling factor to bypass default URDF velocity limits
    'intercept_x_offset':  0.0,    # Pulls hit point toward robot (negative=closer). Keep at 0 to avoid IK unreachable errors.

}