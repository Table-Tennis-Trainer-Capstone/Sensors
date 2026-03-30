#!/usr/bin/env python3
"""
Ball Detection Diagnostic Tool  —  port 5001
Runs its own frame-diff blob detection with NO thresholds so every motion blob
is reported. Use this to find the min radius / brightness / contrast of the ball
when it is at its smallest / farthest from the cameras.

Run:  python3 diag_tool.py
View: http://<jetson-ip>:5001
"""

import os, sys, subprocess, time, threading, json, collections
import cv2, numpy as np
from flask import Flask, Response, render_template_string, request

# ── ROS env setup (same as marty_gui.py) ────────────────────────────────────
for k in ['CYCLONEDDS_URI']:
    if k in os.environ: del os.environ[k]
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

_cyc = """<?xml version="1.0" encoding="UTF-8" ?><CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface address="192.168.1.20"/></Interfaces></General>
    <Discovery><Peers><Peer Address="192.168.1.10"/></Peers></Discovery>
  </Domain></CycloneDDS>"""
with open('/tmp/cyc_diag.xml', 'w') as f: f.write(_cyc)
os.environ['CYCLONEDDS_URI'] = 'file:///tmp/cyc_diag.xml'

for _s in ['/opt/ros/humble/setup.bash',
           '/home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash']:
    for _l in subprocess.run(['bash', '-c', f'source {_s} && env'],
                             capture_output=True, text=True).stdout.splitlines():
        if '=' in _l: os.environ.setdefault(*_l.split('=', 1))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# ── Tunable detection params (loose defaults) ────────────────────────────────
PARAMS = {
    'motion_thresh': 8,    # frame-diff threshold (lower = more blobs)
    'min_area':      4,    # minimum bounding-box area in pixels
    'dilate':        2,    # dilation iterations to fill blobs
}

# ── Per-camera state ──────────────────────────────────────────────────────────
class CamState:
    def __init__(self, side):
        self.side      = side
        self.prev      = None          # previous blurred frame
        self.jpeg      = None          # latest annotated JPEG bytes
        self.blobs     = []            # list of dicts from last frame
        self.fps       = 0.0
        self._cnt      = 0
        self._t0       = time.time()
        # running min/max across ALL frames since last reset
        self.mins = {}
        self.maxs = {}
        self.lock = threading.Lock()

    def reset_minmax(self):
        with self.lock:
            self.mins.clear()
            self.maxs.clear()

    def update_minmax(self, blobs):
        if not blobs: return
        keys = ['r', 'area', 'brightness', 'contrast']
        for k in keys:
            vals = [b[k] for b in blobs if k in b]
            if not vals: continue
            mn, mx = min(vals), max(vals)
            if k not in self.mins or mn < self.mins[k]: self.mins[k] = mn
            if k not in self.maxs or mx > self.maxs[k]: self.maxs[k] = mx


_cams = {'left': CamState('left'), 'right': CamState('right')}

# ── Blob detection (pure CPU, no ROS) ────────────────────────────────────────
def detect_blobs(gray, prev_gray, cs):
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    if prev_gray is None:
        return blurred, []

    diff = cv2.subtract(blurred, prev_gray)
    _, motion = cv2.threshold(diff, PARAMS['motion_thresh'], 255, cv2.THRESH_BINARY)

    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    for _ in range(PARAMS['dilate']):
        cv2.dilate(motion, motion, kern)

    cnts, _ = cv2.findContours(motion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in cnts:
        bx, by, bw, bh = cv2.boundingRect(c)
        area = bw * bh
        if area < PARAMS['min_area']: continue

        cx_b = bx + bw // 2
        cy_b = by + bh // 2
        r    = max(bw, bh) / 2.0

        # Brightness: mean of original pixels inside motion blob
        roi      = gray[by:by+bh, bx:bx+bw]
        mask_roi = motion[by:by+bh, bx:bx+bw]
        brightness = float(cv2.mean(roi, mask_roi)[0]) if mask_roi.any() else 0.0

        # Contrast: inner circle vs 2.5x annulus in original frame
        inner_r = max(1, int(r))
        outer_r = max(inner_r + 2, int(r * 2.5))
        im = np.zeros(gray.shape, np.uint8)
        om = np.zeros(gray.shape, np.uint8)
        cv2.circle(im, (cx_b, cy_b), inner_r, 255, -1)
        cv2.circle(om, (cx_b, cy_b), outer_r, 255, -1)
        cv2.subtract(om, im, om)
        mean_in  = float(cv2.mean(gray, im)[0])
        mean_out = float(cv2.mean(gray, om)[0])
        contrast = mean_in - mean_out

        blobs.append({
            'x': cx_b, 'y': cy_b,
            'r': round(r, 1), 'area': int(area),
            'brightness': round(brightness, 1),
            'contrast':   round(contrast, 1),
        })

    # Sort brightest first
    blobs.sort(key=lambda b: b['brightness'], reverse=True)
    return blurred, blobs


def annotate_frame(bgr, blobs):
    """Draw all blobs on a colour frame. Top blob = red, rest = yellow."""
    out = bgr.copy()
    for i, b in enumerate(blobs):
        color = (0, 0, 255) if i == 0 else (0, 200, 255)
        cx, cy, r = int(b['x']), int(b['y']), max(int(b['r']), 6)
        cv2.circle(out, (cx, cy), r, color, 2)
        label = f"r={b['r']:.0f} br={b['brightness']:.0f} ct={b['contrast']:.0f}"
        cv2.putText(out, label, (cx - 55, cy - r - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
    # param overlay
    cv2.putText(out, f"thr={PARAMS['motion_thresh']} min_area={PARAMS['min_area']}",
                (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 255, 180), 1)
    return out


# ── ROS worker ────────────────────────────────────────────────────────────────
class DiagNode(Node):
    def __init__(self):
        super().__init__('ball_diag')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(CompressedImage, '/camera/left/compressed',
                                 lambda m: self._cb(m, 'left'), q)
        self.create_subscription(CompressedImage, '/camera/right/compressed',
                                 lambda m: self._cb(m, 'right'), q)

    def _cb(self, msg, side):
        cs = _cams[side]

        # FPS
        cs._cnt += 1
        now = time.time()
        if now - cs._t0 >= 1.0:
            cs.fps = cs._cnt / (now - cs._t0)
            cs._cnt = 0; cs._t0 = now

        gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None: return

        new_prev, blobs = detect_blobs(gray, cs.prev, cs)
        cs.prev = new_prev

        with cs.lock:
            cs.blobs = blobs
            cs.update_minmax(blobs)

        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        ann = annotate_frame(bgr, blobs)
        cv2.putText(ann, f"{cs.fps:.1f} fps", (ann.shape[1]-80, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 255, 180), 1)
        _, j = cv2.imencode('.jpg', ann, [cv2.IMWRITE_JPEG_QUALITY, 70])
        cs.jpeg = j.tobytes()


# ── Launch cameras if not already running ────────────────────────────────────
def _launch_cameras():
    cmd_b = (
        "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;"
        "export CYCLONEDDS_URI=file:///tmp/cyc_diag.xml;"
        "source /opt/ros/humble/setup.bash &&"
        "source /home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash &&"
        "ros2 launch ttt_bringup camera_right.launch.py"
    )
    subprocess.Popen(f'bash -c "{cmd_b}"', shell=True,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd_a = (
        "echo \"<?xml version='1.0' encoding='UTF-8' ?>"
        "<CycloneDDS xmlns='https://cdds.io/config'><Domain id='any'>"
        "<General><Interfaces><NetworkInterface address='192.168.1.10'/></Interfaces></General>"
        "<Discovery><Peers><Peer Address='192.168.1.20'/></Peers></Discovery>"
        "</Domain></CycloneDDS>\" > /tmp/cyc_a.xml && "
        "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;"
        "export CYCLONEDDS_URI=file:///tmp/cyc_a.xml;"
        "source /opt/ros/humble/setup.bash &&"
        "source /home/capstone-nano1/Sensors/tabletennistrainer_ws/install/setup.bash &&"
        "ros2 launch ttt_bringup camera_left.launch.py"
    )
    subprocess.Popen(
        f"ssh capstone-nano1@192.168.1.10 'bash -c \"{cmd_a}\"'",
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print('[CAMERAS] Launched camera_right (local) and camera_left (nano1). '
          'Allow ~3s for topics to appear.')

_launch_cameras()

_ros_status = {'ok': False, 'msg': 'starting...'}

def _ros_thread():
    try:
        rclpy.init()
        _ros_status['msg'] = 'rclpy initialized, creating node...'
        node = DiagNode()
        _ros_status['ok']  = True
        _ros_status['msg'] = 'subscribed to /camera/left/compressed and /camera/right/compressed'
        print('[ROS] Node online — waiting for camera topics...')
        rclpy.spin(node)
    except Exception as e:
        _ros_status['ok']  = False
        _ros_status['msg'] = str(e)
        print(f'[ROS ERROR] {e}')

threading.Thread(target=_ros_thread, daemon=True).start()


def _placeholder(text):
    img = np.zeros((200, 400, 3), np.uint8)
    cv2.putText(img, text, (20, 95),  cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 1)
    cv2.putText(img, 'check terminal for errors', (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    _, j = cv2.imencode('.jpg', img)
    return j.tobytes()

# ── Flask web app ─────────────────────────────────────────────────────────────
app = Flask(__name__)

_PAGE = """<!DOCTYPE html><html>
<head>
  <title>Ball Detector Diagnostics</title>
  <style>
    body{background:#111;color:#ddd;font-family:Consolas,monospace;margin:0;padding:12px;}
    h2{color:#0f0;margin:0 0 8px;}
    .feeds{display:flex;gap:12px;flex-wrap:nowrap;}
    .feed{flex:1;text-align:center;}
    .feed h3{color:#aaa;margin:4px 0;font-size:13px;}
    .feed img{width:100%;border:2px solid #333;}
    .panel{background:#1a1a1a;border:1px solid #333;border-radius:6px;padding:10px;margin:10px 0;}
    table{width:100%;border-collapse:collapse;font-size:12px;}
    th{color:#666;text-align:left;padding:3px 10px;border-bottom:1px solid #333;}
    td{padding:4px 10px;border-bottom:1px solid #222;}
    .highlight{color:#0f0;font-weight:bold;}
    .minmax{color:#aaa;font-size:11px;}
    .ctrl{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin:6px 0;}
    label{color:#aaa;font-size:12px;}
    input[type=range]{width:120px;vertical-align:middle;}
    input[type=number]{width:60px;background:#222;color:#ddd;border:1px solid #444;padding:2px 4px;}
    button{background:#222;color:#0f0;border:1px solid #0f0;padding:4px 12px;cursor:pointer;font-family:Consolas,monospace;font-size:12px;}
    button.red{color:#f55;border-color:#f55;}
    .hint{color:#555;font-size:11px;margin-top:6px;}
  </style>
</head>
<body>
  <h2>&#128302; Ball Detector Diagnostics</h2>
  <div class="hint">All blobs shown — no size/brightness filter. Red = brightest blob. Yellow = rest.
  Place ball at farthest point to record minimum values.</div>

  <div class="panel">
    <div class="ctrl">
      <label>motion_thresh <input type="number" id="p_motion_thresh" value="8" min="1" max="60" style="width:50px"></label>
      <label>min_area <input type="number" id="p_min_area" value="4" min="1" max="500"></label>
      <label>dilate <input type="number" id="p_dilate" value="2" min="0" max="6"></label>
      <button onclick="applyParams()">&#9654; Apply</button>
      <button class="red" onclick="resetMinMax()">&#8635; Reset min/max</button>
    </div>
  </div>

  <div class="feeds">
    <div class="feed"><h3>LEFT CAMERA</h3><img src="/stream/left"></div>
    <div class="feed"><h3>RIGHT CAMERA</h3><img src="/stream/right"></div>
  </div>

  <div style="display:flex;gap:12px;margin-top:10px;">
    <div class="panel" style="flex:1">
      <div style="color:#aaa;font-size:12px;margin-bottom:6px;">LEFT — blobs this frame</div>
      <table><thead><tr><th>x</th><th>y</th><th>radius</th><th>area</th><th>brightness</th><th>contrast</th></tr></thead>
      <tbody id="left-blobs"></tbody></table>
      <div id="left-minmax" class="minmax" style="margin-top:6px;"></div>
    </div>
    <div class="panel" style="flex:1">
      <div style="color:#aaa;font-size:12px;margin-bottom:6px;">RIGHT — blobs this frame</div>
      <table><thead><tr><th>x</th><th>y</th><th>radius</th><th>area</th><th>brightness</th><th>contrast</th></tr></thead>
      <tbody id="right-blobs"></tbody></table>
      <div id="right-minmax" class="minmax" style="margin-top:6px;"></div>
    </div>
  </div>

  <script>
    function poll(){
      fetch('/api/blobs').then(r=>r.json()).then(d=>{
        ['left','right'].forEach(function(side){
          var s = d[side];
          var rows = '';
          (s.blobs||[]).forEach(function(b,i){
            var cls = i===0?'highlight':'';
            rows += '<tr class="'+cls+'"><td>'+b.x+'</td><td>'+b.y+'</td><td>'+b.r+'</td>'
                  + '<td>'+b.area+'</td><td>'+b.brightness+'</td><td>'+b.contrast+'</td></tr>';
          });
          if(!rows) rows='<tr><td colspan="6" style="color:#444">no motion detected</td></tr>';
          document.getElementById(side+'-blobs').innerHTML = rows;
          var mn = s.mins, mx = s.maxs;
          document.getElementById(side+'-minmax').innerHTML =
            'min/max seen &mdash; '
            +'radius: '+fmt(mn.r)+' / '+fmt(mx.r)+'  &nbsp;'
            +'area: '+fmt(mn.area)+' / '+fmt(mx.area)+'  &nbsp;'
            +'brightness: '+fmt(mn.brightness)+' / '+fmt(mx.brightness)+'  &nbsp;'
            +'contrast: '+fmt(mn.contrast)+' / '+fmt(mx.contrast);
        });
      }).catch(function(){});
      setTimeout(poll, 150);
    }
    function fmt(v){ return v!==undefined&&v!==null ? v.toFixed(1) : '--'; }
    poll();

    function applyParams(){
      fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},
        body: JSON.stringify({
          motion_thresh: parseInt(document.getElementById('p_motion_thresh').value),
          min_area:      parseInt(document.getElementById('p_min_area').value),
          dilate:        parseInt(document.getElementById('p_dilate').value),
        })
      });
    }

    function resetMinMax(){
      fetch('/api/reset_minmax', {method:'POST'});
    }
  </script>
</body></html>"""

@app.route('/')
def index(): return render_template_string(_PAGE)

@app.route('/api/status')
def api_status():
    return Response(json.dumps(_ros_status), mimetype='application/json')

@app.route('/stream/<side>')
def stream(side):
    def gen():
        while True:
            cs = _cams.get(side)
            j  = cs.jpeg if cs else None
            if not j:
                j = _placeholder(f'waiting for /camera/{side}/compressed ...')
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n'
            time.sleep(0.04)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/blobs')
def api_blobs():
    out = {}
    for side, cs in _cams.items():
        with cs.lock:
            out[side] = {
                'fps':   round(cs.fps, 1),
                'blobs': cs.blobs[:10],
                'mins':  dict(cs.mins),
                'maxs':  dict(cs.maxs),
            }
    return Response(json.dumps(out), mimetype='application/json')

@app.route('/api/params', methods=['POST'])
def api_params():
    data = request.get_json()
    for k in ('motion_thresh', 'min_area', 'dilate'):
        if k in data: PARAMS[k] = int(data[k])
    # Reset prev frames so new threshold takes effect immediately
    for cs in _cams.values(): cs.prev = None
    return Response('{}', mimetype='application/json')

@app.route('/api/reset_minmax', methods=['POST'])
def api_reset():
    for cs in _cams.values(): cs.reset_minmax()
    return Response('{}', mimetype='application/json')

if __name__ == '__main__':
    print("=" * 50)
    print("Ball Detector Diagnostics — http://192.168.55.1:5002")
    print("=" * 50)
    app.run(host='0.0.0.0', port=5002, threaded=True)
