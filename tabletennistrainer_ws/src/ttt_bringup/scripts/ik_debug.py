#!/usr/bin/env python3
"""
IK Debug Tool — fully standalone, no STM hardware required.

Launches the full robot control stack locally (mock hardware) and lets you
send move commands, watch joint angles update in real time, and preview
the exact motor commands that would be sent to the STM.

Usage (Jetson / local):
  # Basic — uses all DDS interfaces (RViz skipped if no display):
  python3 ik_debug.py --software

  # Force headless (web dashboard only, no RViz window):
  python3 ik_debug.py --software --no-rviz
"""

import sys, os, shlex, tempfile, threading, time, json, math, logging, argparse, subprocess
from collections import deque
from flask import Flask, Response, render_template_string, request

# ── CLI args ───────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description='IK Debug Tool')
parser.add_argument('--software', action='store_true',
                    help='Launch robot stack locally (mock hardware, no Jetsons)')
parser.add_argument('--no-rviz', action='store_true',
                    help='Do NOT launch RViz2 automatically')
parser.add_argument('--ws', default='/home/capstone-nano2/Sensors/tabletennistrainer_ws',
                    help='Path to built workspace (default: Jetson workspace)')
parser.add_argument('--iface', default='',
                    help='NIC IP for CycloneDDS (e.g. 172.26.174.93)')
parser.add_argument('--port', type=int, default=5001, help='Web server port (default 5001)')
args = parser.parse_args()

# _IK_ROS_SOURCED is set before re-exec below; guards build + source from running twice
_ros_sourced = os.environ.get('_IK_ROS_SOURCED') == '1'

# ── 1. Auto-Build Required Packages ────────────────────────────────────────────
if args.software and not _ros_sourced:
    print(f"\n⚙️  Building required packages in {args.ws}...")
    build_cmd = ['colcon', 'build', '--symlink-install', '--packages-up-to', 'ttt_control']
    res = subprocess.run(build_cmd, cwd=args.ws)
    if res.returncode != 0:
        print("❌ Build failed! Check the errors above. Exiting.")
        sys.exit(1)
    print("✅ Build complete!\n")

# ── 2. DDS / ROS env Sourcing ──────────────────────────────────────────────────
_iface_xml = f'<Interfaces><NetworkInterface address="{args.iface}"/></Interfaces>' if args.iface else ''
_cyc_xml = f"""<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any"><General>{_iface_xml}</General></Domain>
</CycloneDDS>"""

_cyc_path = os.path.join(tempfile.gettempdir(), 'cyc_ik.xml')
with open(_cyc_path, 'w') as f:
    f.write(_cyc_xml)

for k in ['CYCLONEDDS_URI']:
    if k in os.environ: del os.environ[k]
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = 'file:///' + _cyc_path.replace('\\', '/')

if not _ros_sourced:
    # LD_LIBRARY_PATH is consumed by the dynamic linker at process start and cannot
    # be patched in a running process.  Re-launch the script under bash so that
    # sourcing ROS sets LD_LIBRARY_PATH *before* Python ever loads any C extension.
    print("🔄 Sourcing ROS and re-launching under bash...")
    _ws_setup = os.path.join(args.ws, 'install', 'setup.bash')
    _argv_q   = ' '.join(shlex.quote(a) for a in sys.argv)
    _bash_cmd = (
        f'source /opt/ros/humble/setup.bash 2>/dev/null; '
        f'source {shlex.quote(_ws_setup)} 2>/dev/null; '
        f'export _IK_ROS_SOURCED=1; '
        f'export ROS_DOMAIN_ID=42; '
        f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; '
        f'export CYCLONEDDS_URI={shlex.quote(os.environ["CYCLONEDDS_URI"])}; '
        f'exec {shlex.quote(sys.executable)} {_argv_q}'
    )
    os.execv('/bin/bash', ['/bin/bash', '-c', _bash_cmd])

# ── 3. Late Imports (Requires sourced environment) ─────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ── STM motor config ──────────────────────────────────────────────────────────
STM_CFG = [
    ('BaseRotate_0',     'M1 Base',        0.0,  1.0, -120.0, 120.0, '1:1'),
    ('UpperArmRotate_0', 'M2 Shoulder',   -15.0,  1.0,    0.0, 170.0, 'URDF -15 = STM 0'),
    ('ForeArmRotate_0',  'M3 Elbow',      -25.0,  1.0,    0.0, 150.0, 'URDF -25 = STM 0'),
    ('WristRotate_0',    'M4 Wrist',      90.0,  2.0,  -90.0,  90.0, 'URDF -90 = STM 0'),
    ('PaddleRotate_0',   'M5 Paddle',      0.0,  1.0,  -90.0,  90.0, '1:1'),
]

def compute_stm_angles(joints):
    result = []
    for ros_name, label, offset, scale, stm_min, stm_max, desc in STM_CFG:
        raw = joints.get(ros_name)
        if raw is None:
            result.append({'label': label, 'joint_deg': None, 'motor_deg': None, 'motor_deg_safe': None,
                           'stm_min': stm_min, 'stm_max': stm_max, 'desc': desc, 'clamped': False})
        else:
            jdeg = math.degrees(raw)
            mdeg = offset + scale * jdeg
            clamped = mdeg < stm_min or mdeg > stm_max
            mdeg_safe = max(stm_min, min(stm_max, mdeg))
            result.append({'label': label, 'joint_deg': round(jdeg, 3),
                           'motor_deg': round(mdeg, 3), 'motor_deg_safe': round(mdeg_safe, 3),
                           'stm_min': stm_min, 'stm_max': stm_max,
                           'desc': desc, 'clamped': clamped})
    return result

# ── Software launcher ──────────────────────────────────────────────────────────
class SoftwareLauncher:
    def __init__(self, ws_path, launch_rviz=True):
        self._ws = ws_path
        self._launch_rviz = launch_rviz
        self._procs = []
        self._log = deque(maxlen=300)
        self._lock = threading.Lock()

    def _source_cmd(self):
        ws_setup = os.path.join(self._ws, 'install', 'setup.bash')
        return f'source /opt/ros/humble/setup.bash && source {ws_setup}'

    def _launch(self, cmd, label):
        full = f'{self._source_cmd()} && export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI="{os.environ["CYCLONEDDS_URI"]}" && {cmd}'
        p = subprocess.Popen(['bash', '-c', full],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True)
        self._procs.append((label, p))
        def _reader():
            for line in p.stdout:
                with self._lock:
                    self._log.append(f'[{label}] {line.rstrip()}')
        threading.Thread(target=_reader, daemon=True).start()
        return p

    def get_log(self):
        with self._lock: return list(self._log)

    def start(self):
        threading.Thread(target=self._start_sequence, daemon=True).start()

    def _start_sequence(self):
        with self._lock: self._log.append('[launcher] Starting robot stack (mock hardware)...')

        self._launch('ros2 launch ttt_control rsp.launch.py', 'rsp')
        time.sleep(3)

        self._launch('ros2 launch ttt_control controllers.launch.py', 'controllers')
        time.sleep(4)

        self._launch('ros2 launch ttt_control move_group.launch.py', 'move_group')
        time.sleep(6)

        # mock_components/GenericSystem publishes joint states with timestamp 0 in Humble,
        # causing MoveIt trajectory execution to abort. Disable the start-state timestamp
        # check so software-mode execution proceeds normally.
        for param, val in [
            ('trajectory_execution.allowed_start_tolerance', '0.0'),
            # Allow wrist to start outside URDF bounds (hardware homes to 0°, URDF min is 2.09 rad).
            # MoveIt clamps the start state to the nearest valid joint value instead of aborting.
            ('start_state_max_bounds_error', '3.15'),
        ]:
            subprocess.run(
                ['bash', '-c',
                 f'{self._source_cmd()} && '
                 f'export ROS_DOMAIN_ID=42 && '
                 f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && '
                 f'ros2 param set /move_group {param} {val}'],
                capture_output=True)

        for frame_args, label in [
            # world → table_center: RSP then handles the full chain (table_center → root → Base → ...)
            ('--x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id table_center', 'tf_world_table_center'),
            # world → table: logical frame used by control_node for ball positions
            ('--x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id table', 'tf_world_table'),
        ]:
            self._launch(f'ros2 run tf2_ros static_transform_publisher {frame_args}', label)
        time.sleep(1)

        self._launch('ros2 run ttt_control control_node', 'control_node')
        
        if self._launch_rviz:
            with self._lock: self._log.append('[launcher] Launching RViz2 GUI...')
            self._launch('ros2 launch ttt_control moveit_rviz.launch.py', 'rviz')

        with self._lock: self._log.append('[launcher] All nodes started. Ready.')

    def stop(self):
        for label, p in self._procs: p.terminate()

_has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
_launch_rviz = not args.no_rviz and _has_display
if args.software and not args.no_rviz and not _has_display:
    print("⚠️  No display detected — skipping RViz (pass --no-rviz to suppress this warning)")
launcher = SoftwareLauncher(args.ws, launch_rviz=_launch_rviz) if args.software else None

# ── Shared state ───────────────────────────────────────────────────────────────
class State:
    def __init__(self):
        self.joints = {}
        self.joint_stamp = None
        self.links = {}
        self.tf_status = 'Waiting for TF...'
        
        self.packet_log = deque(maxlen=20) 
        self._lock = threading.Lock()

    def add_packet(self, packet_str):
        with self._lock:
            ts = time.strftime('%H:%M:%S', time.localtime())
            self.packet_log.appendleft(f'<span style="color:#777">[{ts}]</span> {packet_str}')

    def to_dict(self):
        with self._lock: 
            j = dict(self.joints)
            packets = list(self.packet_log)
            
        stm = compute_stm_angles(j)

        return {
            'joints': j,
            'joint_stamp': self.joint_stamp,
            'links': dict(self.links),
            'tf_status': self.tf_status,
            'stm_angles': stm,
            'packets': packets
        }

state = State()

# ── ROS worker ─────────────────────────────────────────────────────────────────
class IKWorker(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.n = self.test_pub = self.arm_cmd_pub = self.stm_cmd_pub = None

    def run(self):
        rclpy.init()
        self.n = Node('ik_debug')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.n.create_subscription(JointState, '/joint_states', self._cb_joints, q)
        
        # Strictly listen to the REAL packets published by control_node
        self.n.create_subscription(String, '/stm_cmd', self._cb_stm_cmd, 10)
        
        self.test_pub    = self.n.create_publisher(PointStamped, '/ball_trajectory/predicted', 10)
        self.arm_cmd_pub = self.n.create_publisher(String, '/arm_named_target', 10)
        self.stm_cmd_pub = self.n.create_publisher(String, '/stm_cmd', 10)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.n)
        self.n.create_timer(0.05, self._timer_tf)
        rclpy.spin(self.n)

    def _cb_stm_cmd(self, msg):
        if msg.data.startswith('M'):
            state.add_packet(msg.data.strip())

    def _cb_joints(self, msg):
        with state._lock:
            for i, name in enumerate(msg.name): state.joints[name] = msg.position[i]
            state.joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _timer_tf(self):
        links, errors = {}, []
        for link in ['root','Base','Shoulder','UpperArm','Forearm','Wrist','Paddle','paddle_tcp']:
            try:
                t = self.tf_buffer.lookup_transform('root', link, rclpy.time.Time())
                links[link] = {'x': t.transform.translation.x,
                               'y': t.transform.translation.y,
                               'z': t.transform.translation.z}
            except Exception as e: errors.append(f'{link}: {type(e).__name__}')
        with state._lock:
            if links:
                state.links = links
                state.tf_status = f'OK ({len(links)}/8 links)'
            else:
                state.tf_status = 'WAITING | ' + '; '.join(errors[:3])

worker = IKWorker()

# ── HTML ───────────────────────────────────────────────────────────────────────
_HTML = """<!DOCTYPE html>
<html>
<head>
  <title>IK Debug Tool</title>
  <style>
    body{background:#0d0d0d;color:#e0e0e0;font-family:Consolas,monospace;margin:0;padding:16px;}
    h1{text-align:center;color:#00ffcc;margin:0 0 6px;font-size:20px;}
    .mode-badge{text-align:center;font-size:11px;margin-bottom:14px;}
    .sw{color:#0f0;} .hw{color:#fa0;}
    .grid{display:flex;gap:14px;flex-wrap:wrap;justify-content:center;}
    .card{background:#111;border:1px solid #2a2a2a;border-radius:8px;padding:14px;flex:1;min-width:300px;max-width:520px;}
    .card h3{color:#0ff;margin:0 0 10px;font-size:12px;letter-spacing:1px;border-bottom:1px solid #1e1e1e;padding-bottom:6px;}
    .btn{background:#1a1a1a;border:1px solid #0f0;color:#0f0;padding:6px 15px;
         font-family:Consolas,monospace;cursor:pointer;font-size:12px;border-radius:4px;margin:3px;}
    .btn:hover{background:#1a2e1a;}
    .btn-blue{border-color:#0af;color:#0af;} .btn-blue:hover{background:#00161f;}
    .btn-red{border-color:#f55;color:#f55;} .btn-red:hover{background:#1f0000;}
    .btn-yellow{border-color:#fa0;color:#fa0;} .btn-yellow:hover{background:#1a1200;}
    input[type=number]{background:#161616;color:#0fc;border:1px solid #333;
                       font-family:Consolas,monospace;font-size:12px;padding:3px 5px;width:68px;border-radius:3px;}
    label{color:#777;font-size:11px;}
    .jname{color:#888;}
    .fresh{color:#0f0;} .warn{color:#fa0;} .stale{color:#f44;}
    .status{font-size:10px;color:#555;margin-top:6px;}
    canvas{background:#080808;border-radius:4px;width:100%;height:360px;display:block;}
    .logbox{background:#060606;border:1px solid #222;border-radius:4px;
            height:180px;overflow-y:auto;padding:6px;font-size:10px;line-height:1.5;}
    .packetbox{background:#060606;border:1px solid #222;border-radius:4px;
            height:360px;overflow-y:auto;padding:6px;font-size:12px;line-height:1.6;color:#0fc;}
    .log-send{color:#0fc;} .log-result{color:#fa0;} .log-err{color:#f55;} .log-info{color:#555;}
    .nodelog{background:#050505;border:1px solid #1a1a1a;border-radius:4px;
             height:160px;overflow-y:auto;padding:6px;font-size:10px;line-height:1.4;color:#444;}
    .tf-row{display:flex;gap:6px;font-size:10px;padding:2px 0;border-bottom:1px solid #141414;}
    .tf-name{color:#0af;width:90px;flex-shrink:0;} .tf-xyz{color:#777;}
  </style>
</head>
<body>
<h1>IK DEBUG TOOL</h1>
<div class="mode-badge" id="mode-badge">loading...</div>

<div class="grid">
  <div class="card">
    <h3>SEND MOVE COMMAND</h3>
    <div style="margin-bottom:10px;">
      <label>Target in <em>table</em> frame (m) — X=lateral +=right, Y=height, Z=depth neg=robot</label><br><br>
      <label>X:</label> <input id="arm-x" type="number" value="0.0"   step="0.05">
      <label style="margin-left:8px;">Y:</label> <input id="arm-y" type="number" value="0.30"  step="0.05">
      <label style="margin-left:8px;">Z:</label> <input id="arm-z" type="number" value="-1.05" step="0.05">
    </div>
    <button class="btn" onclick="sendMove()">MOVE TO POINT</button>
    <button class="btn btn-blue" onclick="sendCmd('ready','/api/arm_ready')">READY</button>
    <button class="btn btn-blue" onclick="sendCmd('home','/api/arm_home')">HOME</button>
    <button class="btn btn-yellow" onclick="sendCmd('stm_home','/api/stm_home')">STM HOME</button>
    <div class="status" id="cmd-status">idle</div>
    <br>
    <h3>PRESETS</h3>
    <button class="btn" style="font-size:10px;" onclick="setPos(0,0.30,-1.05)">Center Mid</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(-0.3,0.30,-0.9)">Left Near</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(0.3,0.30,-0.9)">Right Near</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(0,0.50,-1.2)">Center High</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(0,0.15,-0.8)">Center Low</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(-0.5,0.30,-0.9)">Far Left</button>
    <button class="btn" style="font-size:10px;" onclick="setPos(0.5,0.30,-0.9)">Far Right</button>
  </div>

  <div class="card" style="max-width:660px;">
    <h3>JOINT STATES <span id="joint-age" class="stale">(waiting)</span></h3>
    <div id="joints-body">
      <div style="color:#333;font-size:11px;padding:16px 0;text-align:center;">Waiting for /joint_states...</div>
    </div>
  </div>
</div>

<div class="grid" style="margin-top:14px;">
  <div class="card" style="min-width:420px;max-width:500px;">
    <h3>ARM VIEW (TF LINKS — root frame)</h3>
    <canvas id="armCanvas" width="450" height="360"></canvas>
    <div id="tf-status" class="stale" style="font-size:10px;margin-top:4px;">Waiting for TF...</div>
  </div>

  <div class="card" style="max-width:350px;">
    <h3>STM32 UDP PACKETS (SINGLE FIRE)</h3>
    <div class="packetbox" id="packetlog">
        <div style="color:#333;text-align:center;margin-top:140px;">Waiting for move command...</div>
    </div>
  </div>

  <div class="card">
    <h3>COMMAND LOG</h3>
    <div class="logbox" id="log"></div>
    <button class="btn btn-red" style="margin-top:6px;font-size:10px;" onclick="clearLog()">Clear</button>

    <h3 style="margin-top:12px;">TF LINKS (root frame)</h3>
    <div id="tf-links"></div>

    <h3 style="margin-top:12px;">NODE LOG <span style="color:#333;font-weight:normal;font-size:10px;">(software mode only)</span></h3>
    <div class="nodelog" id="nodelog">not in software mode</div>
  </div>
</div>

<script>
  var _stats = {}, _log = [], _jointLastT = null, _swMode = false;

  fetch('/api/mode').then(r=>r.json()).then(d=>{
    _swMode = d.software;
    var el = document.getElementById('mode-badge');
    if(d.software) el.innerHTML = '<span class="sw">&#9679; SOFTWARE MODE</span> — mock hardware, no Jetsons needed';
    else el.innerHTML = '<span class="hw">&#9679; HARDWARE MODE</span> — connecting to robot on network';
  });

  function poll(){
    fetch('/api/stats').then(r=>r.json()).then(d=>{ _stats=d; updateUI(d); }).catch(()=>{});
    setTimeout(poll, 80);
  }

  function pollNodeLog(){
    if(!_swMode){ setTimeout(pollNodeLog, 2000); return; }
    fetch('/api/node_log').then(r=>r.json()).then(lines=>{
      var el = document.getElementById('nodelog');
      if(lines.length){ el.innerHTML = lines.slice(-80).join('<br>'); el.scrollTop = el.scrollHeight; }
    }).catch(()=>{});
    setTimeout(pollNodeLog, 1000);
  }

  function updateUI(d){
    var now = Date.now();
    if(d.joint_stamp) _jointLastT = d.joint_stamp * 1000;
    var age = _jointLastT ? (now - _jointLastT) : null;
    var ageEl = document.getElementById('joint-age');
    if(age===null){ ageEl.textContent='(no data)'; ageEl.className='stale'; }
    else if(age<200){ ageEl.textContent='('+age.toFixed(0)+'ms)'; ageEl.className='fresh'; }
    else if(age<1000){ ageEl.textContent='('+age.toFixed(0)+'ms)'; ageEl.className='warn'; }
    else { ageEl.textContent='(STALE '+(age/1000).toFixed(1)+'s)'; ageEl.className='stale'; }

    var jcols = ['#00ff88','#00aaff','#ffaa00','#ff5555','#ff55ff'];
    var stm = d.stm_angles || [];
    var html = '';
    stm.forEach(function(s,i){
      var col = jcols[i];
      if(s.joint_deg === null || s.joint_deg === undefined){
        html += '<div style="padding:6px 0;border-bottom:1px solid #181818;">'
              + '<div style="display:flex;justify-content:space-between;font-size:11px;">'
              + '<span style="color:'+col+';font-weight:bold;">'+s.label+'</span>'
              + '<span style="color:#333">waiting...</span></div>'
              + '<div style="font-size:9px;color:#333;margin-top:1px;">'+s.desc+'</div></div>';
        return;
      }
      var pct = (s.motor_deg_safe - s.stm_min) / (s.stm_max - s.stm_min) * 100;
      var barCol = s.clamped ? '#f55' : (Math.abs(pct-50) > 40 ? '#fa0' : '#0a0');
      var stmCol = s.clamped ? '#f55' : '#fa0';
      var clampNote = s.clamped ? ' <span style="color:#f55;font-size:9px;">⚠ CLAMPED ('+s.motor_deg.toFixed(1)+'°→'+s.motor_deg_safe.toFixed(1)+'°)</span>' : '';
      html += '<div style="padding:5px 0;border-bottom:1px solid #181818;">'
            + '<div style="display:flex;justify-content:space-between;align-items:center;font-size:11px;">'
            + '<span style="color:'+col+';font-weight:bold;width:110px;">'+s.label+'</span>'
            + '<span style="color:'+col+';">'+s.joint_deg.toFixed(1)+'° <span style="color:#444;font-size:9px;">('+( s.joint_deg*Math.PI/180).toFixed(3)+' rad)</span></span>'
            + '<span style="color:'+stmCol+';font-size:12px;font-weight:bold;">→ '+s.motor_deg_safe.toFixed(1)+'°</span>'
            + clampNote + '</div>'
            + '<div style="display:flex;align-items:center;gap:6px;margin-top:3px;">'
            + '<span style="color:#333;font-size:9px;width:30px;text-align:right;">'+s.stm_min+'°</span>'
            + '<div style="flex:1;background:#1a1a1a;height:6px;border-radius:3px;position:relative;">'
            + '<div style="position:absolute;left:'+pct.toFixed(1)+'%;top:-1px;width:2px;height:8px;background:'+barCol+';border-radius:1px;transform:translateX(-50%);"></div>'
            + '</div><span style="color:#333;font-size:9px;width:30px;">'+s.stm_max+'°</span></div>'
            + '<div style="font-size:9px;color:#444;margin-top:1px;">'+s.desc+'</div></div>';
    });
    if(!html) html='<div style="color:#333;padding:12px;text-align:center;">No joints yet</div>';
    document.getElementById('joints-body').innerHTML = html;

    // UDP Packet Display - ONLY Shows what the C++ node sends!
    if(d.packets && d.packets.length > 0) {
        document.getElementById('packetlog').innerHTML = d.packets.join('<br>');
    }

    var links = d.links || {};
    var lnames = ['root','Base','Shoulder','UpperArm','Forearm','Wrist','Paddle','paddle_tcp'];
    var tfHtml = '';
    lnames.forEach(function(n){
      var lk = links[n];
      if(!lk){ tfHtml += '<div class="tf-row"><span class="tf-name">'+n+'</span><span style="color:#222">missing</span></div>'; return; }
      tfHtml += '<div class="tf-row"><span class="tf-name">'+n+'</span>'
              + '<span class="tf-xyz">X:'+lk.x.toFixed(3)+' Y:'+lk.y.toFixed(3)+' Z:'+lk.z.toFixed(3)+'</span></div>';
    });
    document.getElementById('tf-links').innerHTML = tfHtml || '<div style="color:#333;font-size:11px;">No TF data</div>';
    var tfEl = document.getElementById('tf-status');
    tfEl.textContent = d.tf_status || 'Waiting for TF...';
    tfEl.className = (d.tf_status||'').startsWith('OK') ? 'fresh' : 'stale';
  }

  function renderLoop(){ drawArmView(_stats); requestAnimationFrame(renderLoop); }
  poll(); pollNodeLog(); requestAnimationFrame(renderLoop);

  function addLog(cls,msg){
    var ts = new Date().toISOString().substr(11,12);
    _log.unshift('<span class="'+cls+'">['+ts+'] '+msg+'</span>');
    if(_log.length>120) _log.pop();
    document.getElementById('log').innerHTML = _log.join('<br>');
  }
  function clearLog(){ _log=[]; document.getElementById('log').innerHTML=''; }
  function setPos(x,y,z){
    document.getElementById('arm-x').value=x;
    document.getElementById('arm-y').value=y;
    document.getElementById('arm-z').value=z;
  }

  function sendMove(){
    var x=parseFloat(document.getElementById('arm-x').value);
    var y=parseFloat(document.getElementById('arm-y').value);
    var z=parseFloat(document.getElementById('arm-z').value);
    var st=document.getElementById('cmd-status');
    st.textContent='sending...'; st.style.color='#ff0';
    addLog('log-send','MOVE X='+x.toFixed(3)+' Y='+y.toFixed(3)+' Z='+z.toFixed(3));
    fetch('/api/test_arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x,y,z})})
    .then(r=>r.json()).then(res=>{
      st.textContent=res.ok?'sent OK':'ERROR'; st.style.color=res.ok?'#0f0':'#f55';
      addLog(res.ok?'log-result':'log-err','Response: '+(res.ok?'OK':'FAIL'));
    }).catch(e=>{ st.textContent='net error'; st.style.color='#f55'; addLog('log-err','Net err: '+e); });
  }

  function sendCmd(label,url){
    var st=document.getElementById('cmd-status');
    st.textContent='sending '+label+'...'; st.style.color='#ff0';
    addLog('log-send','CMD: '+label.toUpperCase());
    fetch(url,{method:'POST'}).then(r=>r.json()).then(res=>{
      st.textContent=res.ok?label+' OK':'ERROR'; st.style.color=res.ok?'#0f0':'#f55';
      addLog(res.ok?'log-result':'log-err',label+': '+(res.ok?'ACK':'FAIL'));
    }).catch(e=>{ st.textContent='net error'; st.style.color='#f55'; addLog('log-err','Net err: '+e); });
  }

  function drawArmView(d){
    var cv=document.getElementById('armCanvas'); if(!cv) return;
    var ctx=cv.getContext('2d'); var w=cv.width, h=cv.height;
    ctx.clearRect(0,0,w,h);
    var links=d.links||{};
    var linkOrder=['root','Base','Shoulder','UpperArm','Forearm','Wrist','Paddle','paddle_tcp'];
    var segColors=['#777','#00ff88','#00aaff','#ffaa00','#ff5555','#ff55ff','#aaaaff'];

    if(!linkOrder.some(function(k){ return links[k]; })){
      ctx.fillStyle='#444'; ctx.font='12px Consolas'; ctx.textAlign='center';
      ctx.fillText('Waiting for TF...', w/2, h/2-10);
      ctx.textAlign='left'; return;
    }

    var SC=130, cx=w/2;
    var divY = Math.round(h*0.44);       // dividing line between top and side views
    var topCY = Math.round(divY*0.5);    // top view: root is centered in top panel
    var sideCY = Math.round(divY + (h-divY)*0.5); // side view: root (Z=0) centered in side panel

    ctx.strokeStyle='#334'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(0,divY); ctx.lineTo(w,divY); ctx.stroke();
    ctx.fillStyle='#334'; ctx.font='10px Consolas';
    ctx.fillText('TOP VIEW (X-Y)',4,divY-3);
    ctx.fillText('SIDE VIEW (X-Z)',4,h-6);

    // Crosshairs at root origin for each view
    ctx.strokeStyle='#1a2a1a'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(cx-40,topCY); ctx.lineTo(cx+40,topCY); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx,topCY-40); ctx.lineTo(cx,topCY+40); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx-40,sideCY); ctx.lineTo(cx+40,sideCY); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx,sideCY-40); ctx.lineTo(cx,sideCY+40); ctx.stroke();

    var cosA = Math.cos(-0.7854);
    var sinA = Math.sin(-0.7854);
    function rotX(p) { return p.x * cosA - p.y * sinA; }
    function rotY(p) { return p.x * sinA + p.y * cosA; }

    function tx(p){ return cx + rotY(p)*SC; }
    function ty(p){ return topCY + rotX(p)*SC; }
    function sx(p){ return cx - rotX(p)*SC; }
    function sy(p){ return sideCY - p.z*SC; }

    var basePt=links['Base'];
    if(basePt){
      ctx.fillStyle='rgba(0,170,255,0.15)'; ctx.strokeStyle='#00aaff'; ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(tx(basePt),ty(basePt),14,0,Math.PI*2); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.arc(sx(basePt),sy(basePt),14,0,Math.PI*2); ctx.fill(); ctx.stroke();
    }

    for(var i=2;i<linkOrder.length-1;i++){
      var p1=links[linkOrder[i]], p2=links[linkOrder[i+1]]; if(!p1||!p2) continue;
      var col=segColors[i%segColors.length];
      ctx.strokeStyle=col; ctx.lineWidth=4;
      ctx.beginPath(); ctx.moveTo(tx(p1),ty(p1)); ctx.lineTo(tx(p2),ty(p2)); ctx.stroke();
      ctx.fillStyle=col; ctx.beginPath(); ctx.arc(tx(p1),ty(p1),4,0,Math.PI*2); ctx.fill();
      ctx.beginPath(); ctx.moveTo(sx(p1),sy(p1)); ctx.lineTo(sx(p2),sy(p2)); ctx.stroke();
      ctx.fillStyle=col; ctx.beginPath(); ctx.arc(sx(p1),sy(p1),4,0,Math.PI*2); ctx.fill();
    }

    var tcp=links['paddle_tcp'];
    if(tcp){
      ctx.fillStyle='#f22'; ctx.strokeStyle='#fff'; ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(tx(tcp),ty(tcp),7,0,Math.PI*2); ctx.fill(); ctx.stroke();
      ctx.fillStyle='#fff'; ctx.font='bold 9px Consolas'; ctx.fillText('TCP',tx(tcp)+9,ty(tcp)+4);
      ctx.fillStyle='#f22';
      ctx.beginPath(); ctx.arc(sx(tcp),sy(tcp),7,0,Math.PI*2); ctx.fill(); ctx.stroke();
      ctx.fillStyle='#fff'; ctx.fillText('TCP',sx(tcp)+9,sy(tcp)+4);
      ctx.fillStyle='#0fc'; ctx.font='10px Consolas';
      ctx.fillText('X:'+tcp.x.toFixed(3)+' Y:'+tcp.y.toFixed(3)+' Z:'+tcp.z.toFixed(3)+'m',6,h-6);
    }

    var joints=d.joints||{};
    var jns=['BaseRotate_0','UpperArmRotate_0','ForeArmRotate_0','WristRotate_0','PaddleRotate_0'];
    var jsh=['Base ','Shldr','Elbw ','Wrist','Paddl'];
    var jcl=['#00ff88','#00aaff','#ffaa00','#ff5555','#ff55ff'];
    ctx.font='10px Consolas';
    for(var i=0;i<jns.length;i++){
      var deg=((joints[jns[i]]||0)*180/Math.PI).toFixed(1);
      ctx.fillStyle=jcl[i]; ctx.fillText(jsh[i]+': '+deg+'\u00b0',w-82,12+i*13);
    }
  }
</script>
</body>
</html>
"""

# ── FLASK APP & API ─────────────────────────────────────────────────────────────
app = Flask(__name__)
logging.getLogger('werkzeug').setLevel(logging.WARNING)

@app.route('/')
def index():
    return render_template_string(_HTML)

@app.route('/api/mode')
def api_mode():
    return Response(json.dumps({'software': args.software}), mimetype='application/json')

@app.route('/api/stats')
def api_stats():
    return Response(json.dumps(state.to_dict()), mimetype='application/json')

@app.route('/api/node_log')
def api_node_log():
    lines = launcher.get_log() if launcher else []
    return Response(json.dumps(lines), mimetype='application/json')

@app.route('/api/test_arm', methods=['POST'])
def api_test_arm():
    data = request.get_json()
    msg = PointStamped()
    msg.header.frame_id = 'table'
    if worker.n: msg.header.stamp = worker.n.get_clock().now().to_msg()
    msg.point.x = float(data.get('x', 0.0))
    msg.point.y = float(data.get('y', 0.30))
    msg.point.z = float(data.get('z', -1.05))
    if worker.test_pub:
        worker.test_pub.publish(msg)
        return Response(json.dumps({'ok': True}), mimetype='application/json')
    return Response(json.dumps({'ok': False, 'msg': 'ROS not ready'}), mimetype='application/json')

@app.route('/api/arm_ready', methods=['POST'])
def api_arm_ready():
    if worker.arm_cmd_pub:
        msg = String(); msg.data = 'ready'; worker.arm_cmd_pub.publish(msg)
    return Response(json.dumps({'ok': True}), mimetype='application/json')

@app.route('/api/arm_home', methods=['POST'])
def api_arm_home():
    if worker.arm_cmd_pub:
        msg = String(); msg.data = 'home'; worker.arm_cmd_pub.publish(msg)
    return Response(json.dumps({'ok': True}), mimetype='application/json')

@app.route('/api/stm_home', methods=['POST'])
def api_stm_home():
    if worker.stm_cmd_pub:
        msg = String(); msg.data = 'home'; worker.stm_cmd_pub.publish(msg)
    return Response(json.dumps({'ok': True}), mimetype='application/json')

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    if launcher: launcher.start()
    worker.start()

    print('=' * 55)
    print(' IK DEBUG TOOL ONLINE')
    print(f' Mode: {"SOFTWARE (mock hardware)" if args.software else "HARDWARE (network)"}')
    print(f' Open: http://localhost:{args.port}')
    if args.software:
        print(f' Workspace: {args.ws}')
        print(' Launching: rsp → controllers → move_group → control_node → rviz2')
        print(' (MoveIt and RViz take ~10s to be ready after startup)')
    print(f' DDS iface: {args.iface or "(all)"}  domain: 42')
    print('=' * 55)

    try:
        app.run(host='0.0.0.0', port=args.port, threaded=True)
    except KeyboardInterrupt:
        if launcher: launcher.stop()