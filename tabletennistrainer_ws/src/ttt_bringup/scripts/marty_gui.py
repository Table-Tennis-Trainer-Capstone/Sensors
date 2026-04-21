import sys, cv2, numpy as np, subprocess, os, signal, time, threading, logging, json, math
from collections import deque
from flask import Flask, Response, render_template_string, request
from rclpy.duration import Duration

# --- 1. DDS CONFIG ---
cyc_b = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface address="192.168.1.20"/></Interfaces></General>
  </Domain>
</CycloneDDS>"""
with open('/tmp/cyc_b.xml', 'w') as f: f.write(cyc_b)

for k in ['CYCLONEDDS_URI']:
    if k in os.environ: del os.environ[k]
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = 'file:///tmp/cyc_b.xml'

# --- 2. SOURCE WORKSPACE ---
_scripts = ['/opt/ros/humble/setup.bash', '/home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash']
for _s in _scripts:
    for _l in subprocess.run(['bash', '-c', f'source {_s} && env'], capture_output=True, text=True).stdout.splitlines():
        if '=' in _l: os.environ.setdefault(_l.split('=', 1)[0], _l.split('=', 1)[1])

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Int32
from ttt_msgs.msg import BallDetection
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import sys as _sys, re as _re
_sys.path.insert(0, '/home/capstone-nano2/Sensors/tabletennistrainer_ws/src/ttt_bringup/launch')
from calibration import PARAMS as CAL

_CAL_PATH = '/home/capstone-nano2/Sensors/tabletennistrainer_ws/src/ttt_bringup/launch/calibration.py'

# --- MERGED FROM DOC 6: unified save helper replaces _save_roi_to_cal + _save_angles_to_cal ---
def _save_all_to_cal(data):
    """Write any subset of PARAMS keys back to calibration.py and update the live CAL dict."""
    try:
        with open(_CAL_PATH, 'r') as f: src = f.read()
        for k, v in data.items():
            if f"'{k}'" in src:
                src = _re.sub(rf"('{k}':\s*)[0-9.\-]+", lambda m, val=v: m.group(1) + str(val), src)
        with open(_CAL_PATH, 'w') as f: f.write(src)
        CAL.update(data)
    except Exception as e: print(f"Save Error: {e}")

def _save_roi_to_cal(side, pts):
    """Save table ROI pixel coords for one side back to calibration.py."""
    key = f'table_roi_{side}'
    val = repr(list(pts))
    try:
        with open(_CAL_PATH, 'r') as f: src = f.read()
        new_src = _re.sub(rf"('{key}':\s*)\[[^\]]*\]", lambda m: m.group(1) + val, src)
        with open(_CAL_PATH, 'w') as f: f.write(new_src)
        CAL[key] = list(pts)
    except Exception as e: pass

# --- HTML DASHBOARD ---
_HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <title>M.A.R.T.Y. Control Center</title>
  <style>
    body{background:#121212;color:#e0e0e0;font-family:Consolas,monospace;margin:0;padding:20px;}
    h1{text-align:center;color:#00FF00;margin:0 0 12px;}
    .feeds{display:flex;gap:16px;justify-content:center;flex-wrap:nowrap;width:100%;}
    .feed{text-align:center;flex:1;}
    .feed h3{color:#aaa;margin:4px 0;}
    .feed img{border:2px solid #444;width:100%;max-width:48vw;height:auto;}
    .stats{background:#1e1e1e;border-radius:10px;border:1px solid #333;padding:16px;margin:16px auto;max-width:820px;}
    .hdr{color:#aaa;text-align:center;font-size:11px;margin:8px 0 4px;}
    .row{display:flex;justify-content:space-around;margin:6px 0;}
    .lbl{color:#aaa;font-size:11px;text-align:center;}
    .val{font-size:22px;font-weight:bold;text-align:center;}
    .sval{font-size:16px;text-align:center;}
    .x{color:#ff5555}.y{color:#55ff55}.z{color:#5555ff}
    .px{color:#ff9999}.py{color:#99ff99}.pz{color:#9999ff}
    .land{color:#ffcc00}
    .topics-panel{background:#1e1e1e;border-radius:10px;border:1px solid #333;padding:16px;margin:16px auto;max-width:1100px;overflow-x:auto;}
    .tp-table{width:100%;border-collapse:collapse;font-size:12px;}
    .tp-table th{color:#666;text-align:left;padding:4px 12px;border-bottom:1px solid #333;font-weight:normal;letter-spacing:1px;}
    .tp-table td{padding:5px 12px;border-bottom:1px solid #222;white-space:nowrap;}
    .tp-name{color:#ccc;font-family:Consolas,monospace;}
    .tp-type{color:#666;}
    .tp-age{font-weight:bold;min-width:60px;}
    .tp-vals{color:#aaa;font-family:Consolas,monospace;white-space:pre;}
    .fresh{color:#00ff00;}.warn{color:#ffaa00;}.stale{color:#ff4444;}
    .btn{background:#222;border:1px solid #0f0;color:#0f0;padding:6px 18px;font-family:Consolas,monospace;cursor:pointer;font-size:13px;margin:0 5px;border-radius:4px;}
    .btn:hover{background:#333;}
    .btn-align{border-color:#0ff;color:#0ff;}
    .btn-vision{border-color:#ff55ff;color:#ff55ff;}
    .btn-test{border-color:#00ffaa;color:#00ffaa;}
    .btn-roi{border-color:#ffaa00;color:#ffaa00;}
    .vis-container{display:flex;gap:16px;justify-content:center;flex-wrap:wrap;margin:16px auto;max-width:1100px;}
    .vis-box{background:#111;border:1px solid #333;border-radius:8px;padding:10px;position:relative;flex:1;min-width:400px;}
    .vis-title{color:#0ff;font-size:12px;font-weight:bold;position:absolute;top:15px;left:15px;background:#111;padding:2px 6px;border-radius:4px;border:1px solid #0ff;}
    canvas{background:#0a0a0a;border-radius:4px;width:100%;height:350px;}
    .tab-bar{display:flex;gap:8px;justify-content:center;margin-bottom:10px;}
    .tab-btn{background:#222;border:1px solid #444;color:#888;padding:7px 24px;font-family:Consolas,monospace;cursor:pointer;font-size:13px;border-radius:4px;}
    .tab-btn.active{background:#0a1a0a;border-color:#0f0;color:#0f0;}
    .det-table{width:100%;border-collapse:collapse;font-size:11px;font-family:Consolas,monospace;}
    .det-table th{color:#666;padding:4px 8px;border-bottom:1px solid #333;text-align:right;font-weight:normal;letter-spacing:1px;}
    .det-table th:first-child,.det-table td:first-child{text-align:left;}
    .det-table td{padding:4px 8px;border-bottom:1px solid #1a1a1a;text-align:right;white-space:nowrap;}
    .det-table tr:hover td{background:#1a1a1a;}
    .det-r{color:#ff9999;}.det-contrast{color:#ffcc55;}.det-bright{color:#aaaaff;}.det-xy{color:#888;}.det-3d{color:#55ff99;}
  </style>
</head>
<body>
  <h1>M.A.R.T.Y. Control Center</h1>
  <div style="text-align:center;margin-bottom:8px;">
    <button class="btn" onclick="toggleCal()" id="cal-btn">&#9654; SET ROI</button>
    <button class="btn btn-align" onclick="toggleAlign()" id="align-btn">TARGET OVERLAY: OFF</button>
    <button class="btn btn-vision" onclick="toggleAdvConfig()" id="adv-btn">&#9881; ADVANCED CONFIG</button>
    <button class="btn" onclick="toggleDebugPanel()" id="dbg-btn" style="border-color:#ff9999;color:#ff9999;">&#128027; TRAJECTORY LOGS</button>
    <button class="btn btn-roi" onclick="toggleRoiMask()" id="roi-toggle-btn">ROI: VISIBLE</button>
  </div>

  <div id="cal-panel" style="display:none;background:#1a1a1a;border:1px solid #0f0;border-radius:8px;padding:14px;margin:0 auto 10px;max-width:1200px;font-size:12px;">
    <div style="color:#0f0;font-weight:bold;margin-bottom:6px;">Click 4 table corners on each camera image below.</div>
    <div style="display:flex;gap:20px;flex-wrap:wrap;">
      <div style="flex:1;min-width:280px;">
        <div style="color:#aaa;margin-bottom:4px;">LEFT (Jetson A) <span id="lcnt" style="color:#ff5">(0/4)</span>
          <button onclick="resetCal('left')" style="background:#333;color:#f55;border:1px solid #f55;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:8px;">Reset</button>
          <span id="lapply-status" style="margin-left:10px;font-size:12px;"></span></div>
        <div id="lpts" style="color:#ccc;line-height:1.8;min-height:48px;font-size:11px;"></div>
      </div>
      <div style="flex:1;min-width:280px;">
        <div style="color:#aaa;margin-bottom:4px;">RIGHT (Jetson B) <span id="rcnt" style="color:#ff5">(0/4)</span>
          <button onclick="resetCal('right')" style="background:#333;color:#f55;border:1px solid #f55;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:8px;">Reset</button>
          <span id="rapply-status" style="margin-left:10px;font-size:12px;"></span></div>
        <div id="rpts" style="color:#ccc;line-height:1.8;min-height:48px;font-size:11px;"></div>
      </div>
    </div>
    <div style="margin-top:10px;border-top:1px solid #333;padding-top:8px;">
      <span style="color:#aaa;font-size:11px;">Net Z — place ball at net: </span>
      <button onclick="captureNetZ()" style="background:#222;color:#55f;border:1px solid #55f;padding:3px 10px;cursor:pointer;font-size:11px;font-family:Consolas,monospace;">Capture Net Z</button>
      <span id="net-z-val" style="color:#0ff;margin-left:10px;font-family:Consolas,monospace;font-size:11px;"></span>
    </div>
  </div>

  <div id="align-panel" style="display:none;background:#1a1a1a;border:1px solid #0ff;border-radius:8px;padding:16px;margin:12px auto 10px;max-width:1200px;text-align:center;">
    <div style="display:flex;gap:24px;justify-content:center;align-items:center;flex-wrap:wrap;margin-bottom:12px;">
      <div>
        <span style="color:#ff55ff;font-size:11px;font-weight:bold;">Dist to Net (Z): </span>
        <input type="range" id="net-slider" min="0.0" max="1.5" step="0.01" value="0.52" oninput="onAngleSlider()" style="width:120px;vertical-align:middle;">
        <span id="net-val" style="color:#ff55ff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.52m</span>
      </div>
      <div>
        <span style="color:#fff;font-size:11px;font-weight:bold;">Threshold: </span>
        <input type="range" id="contrast-slider" min="10" max="250" step="1" value="100" oninput="onAngleSlider()" style="width:120px;vertical-align:middle;">
        <span id="contrast-val" style="color:#fff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:35px;">100</span>
      </div>
      <div>
        <span style="color:#fff;font-size:11px;font-weight:bold;">Motion Thresh: </span>
        <input type="range" id="motion-slider" min="1" max="50" step="1" value="5" oninput="onAngleSlider()" style="width:90px;vertical-align:middle;">
        <span id="motion-val" style="color:#fff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:25px;">5</span>
      </div>
      <div>
        <span style="color:#fff;font-size:11px;font-weight:bold;">Area (Min/Max): </span>
        <input type="range" id="min-area-slider" min="1" max="100" step="1" value="4" oninput="onAngleSlider()" style="width:70px;vertical-align:middle;">
        <input type="range" id="max-area-slider" min="20" max="500" step="1" value="150" oninput="onAngleSlider()" style="width:70px;vertical-align:middle;">
        <span id="area-val" style="color:#fff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:60px;">1-150</span>
      </div>
      <button onclick="applyAngles()" style="background:#222;color:#0ff;border:1px solid #0ff;padding:6px 16px;cursor:pointer;font-family:Consolas,monospace;border-radius:4px;font-weight:bold;">Apply to C++</button>
      <span id="angle-status" style="font-size:12px;margin-left:8px;"></span>
    </div>

    <div style="display:flex;gap:40px;justify-content:center;flex-wrap:wrap;">
      <div style="border:1px solid #333; padding:10px; border-radius:6px; background:#111; text-align:right;">
        <div style="color:#00ffaa;font-size:12px;font-weight:bold;margin-bottom:8px;text-align:center;">LEFT CAM</div>
        <div>
          <span style="color:#aaa;font-size:10px;">Height (Y): </span>
          <input type="range" id="height-l-slider" min="0.0" max="1.5" step="0.01" value="0.889" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="height-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.89m</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Tilt (Pitch): </span>
          <input type="range" id="tilt-l-slider" min="0" max="60" step="0.1" value="45.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="tilt-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">45.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Pan: </span>
          <input type="range" id="pan-l-slider" min="0" max="45" step="0.1" value="15.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="pan-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">15.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Roll: </span>
          <input type="range" id="roll-l-slider" min="-30" max="30" step="0.1" value="0.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="roll-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.0</span>
        </div>
      </div>

      <div style="border:1px solid #333; padding:10px; border-radius:6px; background:#111; text-align:right;">
        <div style="color:#ffaa00;font-size:12px;font-weight:bold;margin-bottom:8px;text-align:center;">RIGHT CAM</div>
        <div>
          <span style="color:#aaa;font-size:10px;">Height (Y): </span>
          <input type="range" id="height-r-slider" min="0.0" max="1.5" step="0.01" value="0.889" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="height-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.89m</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Tilt (Pitch): </span>
          <input type="range" id="tilt-r-slider" min="0" max="60" step="0.1" value="45.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="tilt-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">45.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Pan: </span>
          <input type="range" id="pan-r-slider" min="0" max="45" step="0.1" value="15.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="pan-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">15.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Roll: </span>
          <input type="range" id="roll-r-slider" min="-30" max="30" step="0.1" value="0.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="roll-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.0</span>
        </div>
      </div>
    </div>
  </div>

  <div id="adv-panel" style="display:none;background:#1a1a1a;border:1px solid #ff55ff;border-radius:8px;padding:16px;margin:12px auto 10px;max-width:1200px;">
    <h3 style="color:#fff;margin-top:0;">Advanced Calibration Parameters</h3>
    <p style="color:#ffaa00;font-size:12px;">Changes here are saved directly to <code>calibration.py</code>. Most fundamental parameters (like camera FPS or solver buffers) require a system restart to fully apply.</p>
    <div id="adv-form" style="display:grid;grid-template-columns:repeat(auto-fit, minmax(280px, 1fr));gap:12px;margin-bottom:16px;">
      <!-- Populated via JS -->
    </div>
    <button onclick="saveAdvConfig()" class="btn btn-vision">Save to calibration.py</button>
    <span id="adv-status" style="margin-left:10px;font-size:12px;"></span>
  </div>

  <div id="dbg-panel" style="display:none;background:#1a1a1a;border:1px solid #ff9999;border-radius:8px;padding:16px;margin:12px auto 10px;max-width:1200px;">
    <h3 style="color:#fff;margin-top:0;">Trajectory Debug Logs</h3>
    <p style="color:#aaa;font-size:12px;">Automatically logs 3D samples and landing predictions when a trajectory ends.</p>
    <textarea id="dbg-textarea" style="width:100%;height:300px;background:#111;color:#0f0;font-family:monospace;border:1px solid #333;padding:10px;box-sizing:border-box;" readonly></textarea>
    <button onclick="clearDebugLogs()" class="btn" style="margin-top:10px;border-color:#f55;color:#f55;">Clear Logs</button>
  </div>

  <div class="feeds">
    <div class="feed"><h3>LEFT CAMERA (Jetson A)</h3><img id="img-left" src="/stream/left" onclick="imgClick(event,'left')" style="cursor:crosshair;"></div>
    <div class="feed"><h3>RIGHT CAMERA (Jetson B)</h3><img id="img-right" src="/stream/right" onclick="imgClick(event,'right')" style="cursor:crosshair;"></div>
  </div>


  <div class="vis-container">
    <div class="vis-box">
      <div class="vis-title">TOP VIEW (X-Z)</div>
      <canvas id="topCanvas" width="600" height="350"></canvas>
    </div>
    <div class="vis-box">
      <div class="vis-title">SIDE VIEW (Y-Z)</div>
      <canvas id="sideCanvas" width="600" height="400"></canvas>
    </div>
    <div class="vis-box" style="min-width:340px;flex:0 0 340px;">
      <div class="vis-title">ARM VIEW (IK)</div>
      <canvas id="armCanvas" width="340" height="400"></canvas>
    </div>
  </div>

  <div style="text-align:center;margin:12px auto;background:#111;padding:10px;border-radius:6px;border:1px solid #333;display:inline-block;position:relative;left:50%;transform:translateX(-50%);">
    <button class="btn" style="background:#1a3a1a;border:1px solid #0a0;color:#0f0;" onclick="fetch('/api/arm_ready',{method:'POST'})">READY</button>
    <button class="btn" style="background:#1a3a1a;border:1px solid #0a0;color:#0f0;margin-left:4px;" onclick="fetch('/api/arm_home',{method:'POST'})">HOME</button>
    <button class="btn" id="stm-home-btn" style="background:#1a1a3a;border:1px solid #55f;color:#88f;margin-left:12px;" onclick="sendStmHome()">STM HOME</button>
  </div>

  <div class="topics-panel">
    <div class="hdr">ROS TOPICS</div>
    <table class="tp-table">
      <thead><tr><th>TOPIC</th><th>TYPE</th><th>AGE</th><th>VALUES</th></tr></thead>
      <tbody id="topics-tbody">
        <tr><td colspan="4" style="color:#444;text-align:center;padding:12px;">waiting for data...</td></tr>
      </tbody>
    </table>
  </div>

  <script>
    var _panL = 15.0, _panR = 15.0, _tiltL = 45.0, _tiltR = 45.0, _hl = 0.889, _hr = 0.889, _rollL = 0.0, _rollR = 0.0, _net = 0.52;
    var _motion = 5;
    var _fx = 448.2, _fy = 448.2, _cx = 320.0, _cy = 200.0, _baseline = 1.525;
    var _SVG_Z_SCALE = 80, _SVG_X_SCALE = 60;
    var _CAM_L = {x:88, y:29}, _CAM_R = {x:88, y:121};
    var calOpen = false, alignOpen = false, advOpen = false, dbgOpen = false;
    var padTrailSide = [], padTrailTop = [];  // paddle arc history
    var PAD_TRAIL_LEN = 90;  // ~1.5s at 60fps
    var calPts = {left:[], right:[]};
    var CAM_W = 640, CAM_H = 400;
    var CORNER_LABELS = ['top-left','top-right','bottom-right','bottom-left'];

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

      function tx(p){ return cx - p.y*SC; }
      function ty(p){ return topCY - p.x*SC; }
      function sx(p){ return cx + p.x*SC; }
      function sy(p){ return sideCY - p.z*SC; }

      var basePt = links['Base'];
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
        ctx.beginPath(); ctx.arc(tx(tcp),ty(tcp),5,0,Math.PI*2); ctx.fill(); ctx.stroke();
        ctx.fillStyle='#fff'; ctx.font='bold 9px Consolas'; ctx.fillText('TCP',tx(tcp)+9,ty(tcp)+4);
        ctx.fillStyle='#f22';
        ctx.beginPath(); ctx.arc(sx(tcp),sy(tcp),5,0,Math.PI*2); ctx.fill(); ctx.stroke();
        ctx.fillStyle='#fff'; ctx.fillText('TCP',sx(tcp)+9,sy(tcp)+4);
        ctx.fillStyle='#0fc'; ctx.font='10px Consolas';
        ctx.fillText('X:'+tcp.x.toFixed(3)+' Y:'+tcp.y.toFixed(3)+' Z:'+tcp.z.toFixed(3)+'m',6,h-6);
      }
      
      var joints=d.joints||{};
      var jns=['BaseRotate_0','UpperArmRotate_0','ForeArmRotate_0','WristRotate_0','PaddleRotate_0'];
      var jsh=['Base ','Shldr','Elbw ','Wrist','Paddl'];
      var jcl=['#00ff88','#00aaff','#ffaa00','#ff5555','#ff55ff'];
      var offsets=[0.0, 15.0, 25.0, 90.0, 0.0];
      ctx.font='10px Consolas';
      for(var i=0;i<jns.length;i++){
        var deg=((joints[jns[i]]||0)*180/Math.PI + offsets[i]);
        if(jns[i] === 'WristRotate_0') deg *= 2.0;
        deg = deg.toFixed(1);
        ctx.fillStyle=jcl[i]; ctx.fillText(jsh[i]+': '+deg+'\u00b0',w-82,12+i*13);
      }
    }

    function drawVisualizations(d) {
      const topCv = document.getElementById('topCanvas');
      const sideCv = document.getElementById('sideCanvas');
      if(!topCv || !sideCv) return;
      const tCtx = topCv.getContext('2d');
      const sCtx = sideCv.getContext('2d');
      const w = topCv.width, h = topCv.height;

      tCtx.clearRect(0,0,w,h); sCtx.clearRect(0,0,w,h);

      // MAPPINGS:  TopView = X vs Z  |  SideView = Y vs Z
      // Z range: -2m to +2m fits in w=600px -> Scale = 130 px/m
      const SCALE = 130; 
      const czTop = w * 0.50;    // Net (Z=0) perfectly centered horizontally
      const cxTop = h * 0.50;    // Table center line (X=0) perfectly centered vertically
      
      const czSide = w * 0.65;   // Shift right so the robot at Z=-1.37 fits on the left
      const cySide = h - 40;     // Push the table surface down to the floor of the canvas

      // Helpers
      const T_X = (x) => cxTop + (x * SCALE);
      const T_Z = (z) => czTop + (z * SCALE);
      const S_Y = (y) => cySide - (y * SCALE);
      const S_Z = (z) => czSide + (z * SCALE);

      // 1. Draw Table and Net
      tCtx.strokeStyle = '#333'; tCtx.lineWidth = 2;
      tCtx.strokeRect(T_Z(-1.37), T_X(-0.7625), 2.74 * SCALE, 1.525 * SCALE); // Table Outline
      tCtx.strokeStyle = '#555'; tCtx.setLineDash([5,5]);
      tCtx.beginPath(); tCtx.moveTo(T_Z(0), T_X(-0.7625)); tCtx.lineTo(T_Z(0), T_X(0.7625)); tCtx.stroke(); // Net Top
      tCtx.setLineDash([]);

      sCtx.fillStyle = '#112211'; sCtx.fillRect(S_Z(-1.37), S_Y(0), 2.74 * SCALE, 5); // Table Top
      sCtx.strokeStyle = '#0f0'; sCtx.lineWidth = 2;
      sCtx.beginPath(); sCtx.moveTo(S_Z(0), S_Y(0)); sCtx.lineTo(S_Z(0), S_Y(0.1525)); sCtx.stroke(); // Net Side

      // 2. Draw Ball Trail
      if(d.trail && d.trail.length > 0) {
        d.trail.forEach((pt, i) => {
          let alpha = 0.2 + 0.8*(i/d.trail.length);
          tCtx.fillStyle = `rgba(255,255,0,${alpha})`;
          tCtx.beginPath(); tCtx.arc(T_Z(pt[2]), T_X(pt[0]), 3, 0, Math.PI*2); tCtx.fill();
          sCtx.fillStyle = `rgba(255,255,0,${alpha})`;
          sCtx.beginPath(); sCtx.arc(S_Z(pt[2]), S_Y(pt[1]), 3, 0, Math.PI*2); sCtx.fill();
        });
      }

      // 3. Draw Current & Predicted Lines
      let hasBall = (d.x !== null && d.z !== null && d.y !== null);
      let hasPred = (d.px !== null && d.pz !== null && d.py !== null);
      let hasLand = (d.land_x !== null && d.land_z !== null);

      if(hasBall) {
        tCtx.fillStyle = '#fff'; tCtx.beginPath(); tCtx.arc(T_Z(d.z), T_X(d.x), 5, 0, Math.PI*2); tCtx.fill();
        sCtx.fillStyle = '#fff'; sCtx.beginPath(); sCtx.arc(S_Z(d.z), S_Y(d.y), 5, 0, Math.PI*2); sCtx.fill();
      }

      // Draw all 3 stage landing markers independently
      var stageDefs = [
        {key:'s1', color:'#5588ff', r:12, label:'S1'},
        {key:'s2', color:'#ffcc00', r:8,  label:'S2'},
        {key:'s3', color:'#00ff80', r:5,  label:'S3'},
      ];
      stageDefs.forEach(function(sd) {
        var lx = d['land_'+sd.key+'_x'], lz = d['land_'+sd.key+'_z'];
        if(lx === null || lx === undefined || lz === null || lz === undefined) return;
        tCtx.strokeStyle = sd.color; tCtx.lineWidth = 2;
        tCtx.beginPath(); tCtx.arc(T_Z(lz), T_X(lx), sd.r, 0, Math.PI*2); tCtx.stroke();
        if(sd.key === 's3') {
          tCtx.beginPath(); tCtx.moveTo(T_Z(lz)-16, T_X(lx)); tCtx.lineTo(T_Z(lz)+16, T_X(lx)); tCtx.stroke();
          tCtx.beginPath(); tCtx.moveTo(T_Z(lz), T_X(lx)-16); tCtx.lineTo(T_Z(lz), T_X(lx)+16); tCtx.stroke();
        }
        tCtx.fillStyle = sd.color; tCtx.font = 'bold 10px Consolas';
        tCtx.fillText(sd.label, T_Z(lz)+sd.r+3, T_X(lx)+4);
        // Side view: land on table surface (Y=0)
        sCtx.strokeStyle = sd.color; sCtx.lineWidth = 2;
        sCtx.beginPath(); sCtx.arc(S_Z(lz), S_Y(0), sd.r, 0, Math.PI*2); sCtx.stroke();
        if(sd.key === 's3') {
          sCtx.beginPath(); sCtx.moveTo(S_Z(lz)-16, S_Y(0)); sCtx.lineTo(S_Z(lz)+16, S_Y(0)); sCtx.stroke();
          sCtx.beginPath(); sCtx.moveTo(S_Z(lz), S_Y(0)-16); sCtx.lineTo(S_Z(lz), S_Y(0)+16); sCtx.stroke();
        }
        sCtx.fillStyle = sd.color; sCtx.font = 'bold 10px Consolas';
        sCtx.fillText(sd.label, S_Z(lz)+sd.r+3, S_Y(0)-4);
      });

      // 3b. Draw Predicted Trajectory Arc Graph
      if(hasPred) {
        tCtx.fillStyle = '#ff5555'; tCtx.beginPath(); tCtx.arc(T_Z(d.pz), T_X(d.px), 5, 0, Math.PI*2); tCtx.fill();
        sCtx.fillStyle = '#ff5555'; sCtx.beginPath(); sCtx.arc(S_Z(d.pz), S_Y(d.py), 5, 0, Math.PI*2); sCtx.fill();

        if(hasBall) {
          tCtx.strokeStyle = 'rgba(255,85,85,0.8)'; tCtx.lineWidth = 2; tCtx.setLineDash([5,5]);
          sCtx.strokeStyle = 'rgba(255,85,85,0.8)'; sCtx.lineWidth = 2; sCtx.setLineDash([5,5]);

          tCtx.beginPath(); tCtx.moveTo(T_Z(d.z), T_X(d.x));
          sCtx.beginPath(); sCtx.moveTo(S_Z(d.z), S_Y(d.y));

          // If bounce is physically between the ball and the paddle
          if(hasLand && d.land_z < d.z && d.land_z > d.pz) {
            tCtx.lineTo(T_Z(d.land_z), T_X(d.land_x));
            
            // Draw parabolic arcs for Side View (simulated gravity curve using Bezier control points)
            let dz1 = Math.abs(d.z - d.land_z);
            sCtx.quadraticCurveTo(S_Z((d.z + d.land_z) / 2), S_Y(Math.max(d.y, 0) + (dz1 * 0.25)), S_Z(d.land_z), S_Y(0));
            
            let dz2 = Math.abs(d.land_z - d.pz);
            sCtx.quadraticCurveTo(S_Z((d.land_z + d.pz) / 2), S_Y(Math.max(0, d.py) + (dz2 * 0.25)), S_Z(d.pz), S_Y(d.py));
          } else {
            // Direct flight (no bounce expected)
            let dz = Math.abs(d.z - d.pz);
            sCtx.quadraticCurveTo(S_Z((d.z + d.pz)/2), S_Y(Math.max(d.y, d.py) + (dz * 0.15)), S_Z(d.pz), S_Y(d.py));
          }
          
          tCtx.lineTo(T_Z(d.pz), T_X(d.px));
          tCtx.stroke(); sCtx.stroke();
          tCtx.setLineDash([]); sCtx.setLineDash([]);
        }
      }

      // Stage indicator badge
      var phase = d.phase || 0;
      if(phase > 0) {
        var stageColors = ['','#5588ff','#ffcc00','#00ff80'];
        var stageNames  = ['','STAGE 1','STAGE 2','STAGE 3'];
        tCtx.fillStyle = stageColors[phase] || '#888';
        tCtx.font = 'bold 13px Consolas';
        tCtx.fillText(stageNames[phase] || '', 10, 18);
        sCtx.fillStyle = stageColors[phase] || '#888';
        sCtx.font = 'bold 13px Consolas';
        sCtx.fillText(stageNames[phase] || '', 10, 18);
      }
    }

    var _latestStats = {};
    function poll(){
      fetch('/api/stats').then(r=>r.json()).then(d=>{ _latestStats=d; }).catch(()=>{});
      setTimeout(poll, 8);
    }
    function renderLoop(){
      drawVisualizations(_latestStats);
      drawArmView(_latestStats);
      requestAnimationFrame(renderLoop);
    }
    poll();
    requestAnimationFrame(renderLoop);

    function pollTopics(){
      fetch('/api/topics').then(r=>r.json()).then(d=>{
        var topics = Object.keys(d).sort();
        var rows = '';
        if(topics.length === 0){
          rows = '<tr><td colspan="4" style="color:#444;text-align:center;padding:12px;">waiting for data...</td></tr>';
        } else {
          topics.forEach(function(name){
            var t = d[name];
            var age = t.age_ms;
            var ageStr = age === null ? 'never' : age < 1000 ? age+'ms' : (age/1000).toFixed(1)+'s';
            var ageCls = age === null ? 'stale' : age < 150 ? 'fresh' : age < 1000 ? 'warn' : 'stale';
            var vals = Object.entries(t.values).map(function(e){ return e[0]+': '+e[1]; }).join('    ');
            rows += '<tr><td class="tp-name">'+name+'</td><td class="tp-type">'+t.type+'</td>'
                  + '<td class="tp-age '+ageCls+'">'+ageStr+'</td><td class="tp-vals">'+vals+'</td></tr>';
          });
        }
        document.getElementById('topics-tbody').innerHTML = rows;
      }).catch(function(){});
      setTimeout(pollTopics, 250);
    }
    pollTopics();

    fetch('/api/config').then(r=>r.json()).then(cfg=>{
      _panL = cfg.pan_left_deg || 15.0;
      _panR = cfg.pan_right_deg || 15.0;
      _tiltL = cfg.tilt_left_deg || 45.0;
      _tiltR = cfg.tilt_right_deg || 45.0;
      _hl = cfg.height_left || 0.889;
      _hr = cfg.height_right || 0.889;
      _rollL = cfg.roll_left_deg || 0.0;
      _rollR = cfg.roll_right_deg || 0.0;
      _net = cfg.net_dist_z || 0.52;
      _contrast = cfg.min_contrast || 100;
      _motion = cfg.motion_threshold || 5;
      _minArea = cfg.min_area || 1;
      _maxArea = cfg.max_area || 150;
      _fx = cfg.fx || 448.2;
      _fy = cfg.fy || 448.2;
      _cx = cfg.cx || 320.0;
      _cy = cfg.cy || 200.0;

      document.getElementById('pan-l-slider').value = _panL;
      document.getElementById('pan-r-slider').value = _panR;
      document.getElementById('tilt-l-slider').value = _tiltL;
      document.getElementById('tilt-r-slider').value = _tiltR;
      document.getElementById('height-l-slider').value = _hl;
      document.getElementById('height-r-slider').value = _hr;
      document.getElementById('roll-l-slider').value = _rollL;
      document.getElementById('roll-r-slider').value = _rollR;
      document.getElementById('net-slider').value = _net;
      document.getElementById('contrast-slider').value = _contrast;
      document.getElementById('motion-slider').value = _motion;
      document.getElementById('min-area-slider').value = _minArea;
      document.getElementById('max-area-slider').value = _maxArea;
      onAngleSlider();
    }).catch(()=>{});

    function toggleCal(){
      calOpen = !calOpen;
      document.getElementById('cal-panel').style.display = calOpen ? 'block' : 'none';
      document.getElementById('cal-btn').innerHTML = calOpen ? '&#9660; SET ROI' : '&#9654; SET ROI';
    }

    function toggleAlign(){
      fetch('/api/toggle_align', {method:'POST'});
      alignOpen = !alignOpen;
      var btn = document.getElementById('align-btn');
      btn.textContent = alignOpen ? 'TARGET OVERLAY: ON' : 'TARGET OVERLAY: OFF';
      btn.style.background = alignOpen ? '#003333' : '#222';
      document.getElementById('align-panel').style.display = alignOpen ? 'block' : 'none';
    }

    function toggleAdvConfig(){
      advOpen = !advOpen;
      document.getElementById('adv-panel').style.display = advOpen ? 'block' : 'none';
      var btn = document.getElementById('adv-btn');
      btn.style.background = advOpen ? '#330033' : '#222';
      if(advOpen) {
        fetch('/api/config_all').then(r=>r.json()).then(cfg=>{
          var html = '';
          var skip = ['table_roi_left', 'table_roi_right'];
          var keys = Object.keys(cfg).sort();
          keys.forEach(k => {
            if(skip.includes(k)) return;
            html += '<div style="display:flex;justify-content:space-between;border-bottom:1px solid #333;padding:4px 0;">' +
                    '<label style="color:#ccc;font-size:12px;margin-top:4px;">' + k + '</label>' +
                    '<input type="text" id="adv_cfg_' + k + '" value="' + cfg[k] + '" style="width:100px;background:#222;color:#0f0;border:1px solid #555;text-align:right;font-family:monospace;padding:2px 4px;">' +
                    '</div>';
          });
          document.getElementById('adv-form').innerHTML = html;
        });
      }
    }

    function saveAdvConfig(){
      var inputs = document.querySelectorAll('input[id^="adv_cfg_"]');
      var data = {};
      inputs.forEach(inp => { data[inp.id.replace('adv_cfg_', '')] = inp.value; });
      var st = document.getElementById('adv-status');
      st.textContent = 'Saving...'; st.style.color = '#ff0';
      fetch('/api/save_config_all', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
      }).then(r=>r.json()).then(res=>{
         if(res.ok) {
           st.textContent = 'Saved! Restart system to fully apply.';
           st.style.color = '#0f0';
         } else {
           st.textContent = 'Error saving config.';
           st.style.color = '#f55';
         }
      }).catch(()=>{ st.textContent='Network error'; st.style.color='#f55'; });
    }

    function toggleRoiMask(){
      fetch('/api/toggle_roi', {method:'POST'}).then(r=>r.json()).then(d=>{
        var btn = document.getElementById('roi-toggle-btn');
        btn.textContent = d.state ? 'ROI: VISIBLE' : 'ROI: HIDDEN';
        btn.style.background = d.state ? '#222' : '#332200';
      });
    }

    function toggleDebugPanel(){
      dbgOpen = !dbgOpen;
      document.getElementById('dbg-panel').style.display = dbgOpen ? 'block' : 'none';
      var btn = document.getElementById('dbg-btn');
      btn.style.background = dbgOpen ? '#330000' : '#222';
      if(dbgOpen) pollDebug();
    }

    function pollDebug(){
      if(!dbgOpen) return;
      fetch('/api/debug_logs').then(r=>r.json()).then(logs=>{
        var ta = document.getElementById('dbg-textarea');
        var newText = logs.join('\\n');
        if(ta.value !== newText) { ta.value = newText; ta.scrollTop = ta.scrollHeight; }
      }).catch(()=>{});
      setTimeout(pollDebug, 1000);
    }

    function clearDebugLogs(){
      fetch('/api/debug_logs', {method: 'DELETE'}).then(()=>{ document.getElementById('dbg-textarea').value = ''; });
    }

    function imgClick(e, side){
      if(!calOpen) return;
      var pts = calPts[side];
      if(pts.length >= 4) return;
      var img = e.currentTarget;
      var rect = img.getBoundingClientRect();
      pts.push([Math.round((e.clientX - rect.left) * CAM_W / rect.width), Math.round((e.clientY - rect.top) * CAM_H / rect.height)]);
      updateCal(side);
    }
    function resetCal(side){ calPts[side] = []; updateCal(side); }
    function updateCal(side){
      var pts = calPts[side];
      var pre = side === 'left' ? 'l' : 'r';
      document.getElementById(pre+'cnt').textContent = '('+pts.length+'/4)';
      var phtml = '';
      pts.forEach(function(p,i){ phtml += CORNER_LABELS[i]+': ('+p[0]+', '+p[1]+')<br>'; });
      document.getElementById(pre+'pts').innerHTML = phtml;
      if(pts.length === 4){ applyRoi(side); }
    }

    function applyRoi(side){
      var pts = calPts[side];
      var flat = pts.reduce(function(a,p){return a.concat(p);}, []);
      var body = {}; body[side] = flat;
      var st = document.getElementById((side==='left'?'l':'r')+'apply-status');
      st.style.color='#ff0'; st.textContent='applying...';
      fetch('/api/set_roi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
        .then(function(r){return r.json();}).then(function(res){
          if(res[side] && res[side].ok){ st.style.color='#0f0'; st.textContent='\u2713 applied'; }
          else { st.style.color='#f55'; st.textContent='\u2717 '+(res[side]?res[side].msg:'failed'); }
        }).catch(function(){ st.style.color='#f55'; st.textContent='\u2717 error'; });
    }

    function captureNetZ(){
      var el = document.getElementById('net-z-val');
      el.innerHTML = 'setting...';
      fetch('/api/stats').then(r=>r.json()).then(d=>{
        var z = d.z;
        fetch('/api/set_net_z', {
          method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({z: z})
        }).then(r=>r.json()).then(res=>{
          if(res.ok){
            el.innerHTML = '&#10003; net_z set to <code>'+z.toFixed(3)+'</code>';
            el.style.color='#0f0';
          } else {
            el.innerHTML = '&#10007; Failed';
            el.style.color='#f55';
          }
        });
      });
    }

    function onAngleSlider(){
      _panL = parseFloat(document.getElementById('pan-l-slider').value);
      _panR = parseFloat(document.getElementById('pan-r-slider').value);
      _tiltL = parseFloat(document.getElementById('tilt-l-slider').value);
      _tiltR = parseFloat(document.getElementById('tilt-r-slider').value);
      _hl = parseFloat(document.getElementById('height-l-slider').value);
      _hr = parseFloat(document.getElementById('height-r-slider').value);
      _rollL = parseFloat(document.getElementById('roll-l-slider').value);
      _rollR = parseFloat(document.getElementById('roll-r-slider').value);
      _net = parseFloat(document.getElementById('net-slider').value);
      _contrast = parseInt(document.getElementById('contrast-slider').value);
      _motion = parseInt(document.getElementById('motion-slider').value);
      _minArea = parseInt(document.getElementById('min-area-slider').value);
      _maxArea = parseInt(document.getElementById('max-area-slider').value);

      document.getElementById('pan-l-val').textContent = _panL.toFixed(1);
      document.getElementById('pan-r-val').textContent = _panR.toFixed(1);
      document.getElementById('tilt-l-val').textContent = _tiltL.toFixed(1);
      document.getElementById('tilt-r-val').textContent = _tiltR.toFixed(1);
      document.getElementById('height-l-val').textContent = _hl.toFixed(2) + 'm';
      document.getElementById('height-r-val').textContent = _hr.toFixed(2) + 'm';
      document.getElementById('roll-l-val').textContent = _rollL.toFixed(1);
      document.getElementById('roll-r-val').textContent = _rollR.toFixed(1);
      document.getElementById('net-val').textContent = _net.toFixed(2) + 'm';
      document.getElementById('contrast-val').textContent = _contrast;
      document.getElementById('motion-val').textContent = _motion;
      document.getElementById('area-val').textContent = _minArea + '-' + _maxArea;

      fetch('/api/preview_angles', {method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({pl:_panL, pr:_panR, tl:_tiltL, tr:_tiltR, hl:_hl, hr:_hr, rl:_rollL, rr:_rollR, nd:_net})});
    }

    function applyAngles(){
      var st = document.getElementById('angle-status');
      st.style.color='#ff0'; st.textContent='applying to C++...';
      _contrast = parseInt(document.getElementById('contrast-slider').value);
      _motion = parseInt(document.getElementById('motion-slider').value);
      _minArea = parseInt(document.getElementById('min-area-slider').value);
      _maxArea = parseInt(document.getElementById('max-area-slider').value);
      fetch('/api/set_angles',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({pl:_panL, pr:_panR, tl:_tiltL, tr:_tiltR, hl:_hl, hr:_hr, rl:_rollL, rr:_rollR, nd:_net, contrast:_contrast, motion_thresh:_motion, min_area:_minArea, max_area:_maxArea})})
        .then(r=>r.json()).then(res=>{
          if(res.ok){ st.style.color='#0f0'; st.textContent='\u2713 C++ updated!'; }
          else { st.style.color='#f55'; st.textContent='\u2717 '+(res.msg||'error'); }
        }).catch(()=>{ st.style.color='#f55'; st.textContent='\u2717 error'; });
    }

function sendStmHome(){
  var btn = document.getElementById('stm-home-btn');
  btn.textContent = 'HOMING...'; btn.style.color = '#ff0';
  fetch('/api/stm_home', {method:'POST'}).then(r=>r.json()).then(res=>{
    if(res.ok){ btn.textContent = 'STM HOME'; btn.style.color = '#0f0'; setTimeout(()=>{ btn.textContent='STM HOME'; btn.style.color='#88f'; }, 1500); }
    else { btn.textContent = 'ERROR'; btn.style.color = '#f55'; setTimeout(()=>{ btn.textContent='STM HOME'; btn.style.color='#88f'; }, 2000); }
  }).catch(()=>{ btn.textContent = 'ERROR'; btn.style.color = '#f55'; setTimeout(()=>{ btn.textContent='STM HOME'; btn.style.color='#88f'; }, 2000); });
}
  </script>
</body>
</html>"""

# --- DRAWING FUNCTIONS ---
CAM_FX, CAM_FY, CAM_CX, CAM_CY = 448.2, 448.2, 320.0, 200.0  # fy=fx for OV9281 square pixels

def project_3d(x, y, z):
    if z <= 0.05: return None
    return (int(CAM_FX * x / z + CAM_CX), int(CAM_FY * y / z + CAM_CY))

def _dist3d(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

# MERGED FROM DOC 6: table_to_cam now takes (tx, ty, tz) so the relative vertical
# vector dy = ty - cam_y is correct. Previously dy was always set to ws.hl/ws.hr
# regardless of the 3D point's Y, which was wrong for anything not at camera height.
def draw_alignment_overlay(frame, side, ws):
    h, w = frame.shape[:2]

    def table_to_cam(tx, ty, tz):
        # World frame: tx=0 is table centre, ty=0 is table surface, tz=0 is net
        cam_x = -(ws.baseline / 2.0) if side == 'left' else (ws.baseline / 2.0)
        cam_y = ws.hl if side == 'left' else ws.hr   # camera height above table surface
        cam_z = -ws.net_dist                          # camera Z offset behind net

        # Vector from camera to world point
        dx = tx - cam_x
        dy = cam_y - ty   # Camera Y points DOWN, World Y points UP
        dz = tz - cam_z

        p = ws.pl if side == 'left' else -ws.pr
        t = ws.tl if side == 'left' else ws.tr
        r = ws.rl if side == 'left' else ws.rr

        # 1. Pan (Y-axis rotation)
        x1 = dx * math.cos(p) - dz * math.sin(p)
        z1 = dx * math.sin(p) + dz * math.cos(p)

        # 2. Tilt (X-axis rotation)
        y2 = dy * math.cos(t) - z1 * math.sin(t)
        z2 = dy * math.sin(t) + z1 * math.cos(t)

        # 3. Roll (Z-axis rotation)
        cx = x1 * math.cos(r) - y2 * math.sin(r)
        cy = x1 * math.sin(r) + y2 * math.cos(r)

        return cx, cy, z2

    half_w = 0.7625  # half table width (1.525m standard)
    depth  = 1.37    # half table length from net to baseline

    # Table surface points: ty=0 (table surface level)
    pts_3d = {
        'net_l':      table_to_cam(-half_w, 0, 0),
        'net_r':      table_to_cam( half_w, 0, 0),
        'base_l':     table_to_cam(-half_w, 0, depth),
        'base_r':     table_to_cam( half_w, 0, depth),
        'center_net': table_to_cam(0,       0, 0),
        'center_base':table_to_cam(0,       0, depth),
    }

    p = {k: project_3d(*v) for k, v in pts_3d.items()}

    def in_bounds(pt): return pt and -1000 <= pt[0] <= w+1000 and -1000 <= pt[1] <= h+1000
    def midpt(a, b):
        if a and b: return ((a[0]+b[0])//2, (a[1]+b[1])//2)

    # Net line (pink)
    if p['net_l'] and p['net_r'] and in_bounds(p['net_l']) and in_bounds(p['net_r']):
        cv2.line(frame, p['net_l'], p['net_r'], (255, 50, 255), 2)
        mid = midpt(p['net_l'], p['net_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_l'], pts_3d['net_r']):.2f}m",
                        (mid[0]-25, mid[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 50, 255), 1, cv2.LINE_AA)

    # Opponent baseline (yellow)
    if p['base_l'] and p['base_r'] and in_bounds(p['base_l']) and in_bounds(p['base_r']):
        cv2.line(frame, p['base_l'], p['base_r'], (255, 255, 50), 2)
        mid = midpt(p['base_l'], p['base_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['base_l'], pts_3d['base_r']):.2f}m",
                        (mid[0]-25, mid[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # Sidelines (yellow)
    if p['net_l'] and p['base_l'] and in_bounds(p['net_l']) and in_bounds(p['base_l']):
        cv2.line(frame, p['net_l'], p['base_l'], (255, 255, 50), 2)
        mid = midpt(p['net_l'], p['base_l'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_l'], pts_3d['base_l']):.2f}m",
                        (mid[0]+5, mid[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 100), 1, cv2.LINE_AA)
    if p['net_r'] and p['base_r'] and in_bounds(p['net_r']) and in_bounds(p['base_r']):
        cv2.line(frame, p['net_r'], p['base_r'], (255, 255, 50), 2)
        mid = midpt(p['net_r'], p['base_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_r'], pts_3d['base_r']):.2f}m",
                        (mid[0]+5, mid[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 100), 1, cv2.LINE_AA)

    # Centre line (cyan)
    if p['center_net'] and p['center_base'] and in_bounds(p['center_net']) and in_bounds(p['center_base']):
        cv2.line(frame, p['center_net'], p['center_base'], (0, 255, 255), 1, cv2.LINE_AA)

    # Net centre cross + label
    if p['center_net'] and in_bounds(p['center_net']):
        cv2.drawMarker(frame, p['center_net'], (255, 50, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, "NET CENTER", (p['center_net'][0]+10, p['center_net'][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 50, 255), 2, cv2.LINE_AA)

    # Baseline centre cross + label
    if p['center_base'] and in_bounds(p['center_base']):
        cv2.drawMarker(frame, p['center_base'], (255, 255, 50), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, "OPPONENT BASELINE", (p['center_base'][0]+10, p['center_base'][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 50), 2, cv2.LINE_AA)


def draw_trajectory_overlay(frame, trail, predicted, landing, side, ws):
    h, w = frame.shape[:2]
    def in_bounds(pt): return pt and 0 <= pt[0] < w and 0 <= pt[1] < h

    def table_to_cam(tx, ty, tz):
        cam_x = -(ws.baseline / 2.0) if side == 'left' else (ws.baseline / 2.0)
        cam_y = ws.hl if side == 'left' else ws.hr
        cam_z = -ws.net_dist
        dx = tx - cam_x
        dy = cam_y - ty
        dz = tz - cam_z
        p = ws.pl if side == 'left' else -ws.pr
        t = ws.tl if side == 'left' else ws.tr
        r = ws.rl if side == 'left' else ws.rr
        x1 = dx * math.cos(p) - dz * math.sin(p)
        z1 = dx * math.sin(p) + dz * math.cos(p)
        y2 = dy * math.cos(t) - z1 * math.sin(t)
        z2 = dy * math.sin(t) + z1 * math.cos(t)
        cx = x1 * math.cos(r) - y2 * math.sin(r)
        cy = x1 * math.sin(r) + y2 * math.cos(r)
        return cx, cy, z2

    proj = []
    for x, y, z in trail:
        cx, cy, cz = table_to_cam(x, y, z)
        proj.append(project_3d(cx, cy, cz))

    for i, pt in enumerate(proj):
        if not in_bounds(pt): continue
        alpha = (i + 1) / len(trail)
        cv2.circle(frame, pt, max(2, int(4 * alpha)), (int(255 * alpha), int(180 * alpha), 0), -1)
        if i > 0 and in_bounds(proj[i-1]):
            cv2.line(frame, proj[i-1], pt, (int(255 * alpha), int(180 * alpha), 0), 1)


def draw_axis_indicator(frame):
    h, w = frame.shape[:2]
    origin, length = (55, h - 55), 40
    cv2.rectangle(frame, (origin[0]-15, origin[1]-length-20), (origin[0]+length+20, origin[1]+15), (0,0,0), -1)
    for (dx, dy), color, lbl in [((length,0),(0,0,255),'X'), ((0,-length),(0,255,0),'Y'), ((length//2,-length//2),(255,0,0),'Z')]:
        tip = (origin[0] + dx, origin[1] + dy)
        cv2.arrowedLine(frame, origin, tip, color, 2, tipLength=0.3)
        cv2.putText(frame, lbl, (tip[0]+4, tip[1]+4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

# --- WEB SERVER ---
class WebStreamer:
    def __init__(self):
        self._frames = {'left': None, 'right': None}
        self._stats = {'x':None, 'y':None, 'z':None, 'px':None, 'py':None, 'pz':None, 'land_x':None, 'land_z':None,
                       'land_s1_x':None, 'land_s1_z':None, 'land_s2_x':None, 'land_s2_z':None, 'land_s3_x':None, 'land_s3_z':None,
                       'det_l_x':None, 'det_l_y':None, 'det_r_x':None, 'det_r_y':None, 'trail':[], 'joints':{}, 'links':{}, 'phase':0}
        self._topic_data = {}
        self.traj_logs = []

        self.show_align = False
        self.show_roi_mask = True

        self.baseline = CAL.get('baseline_m', 1.525)
        self.pl = math.radians(CAL.get('pan_left_deg', 15.0))
        self.pr = math.radians(CAL.get('pan_right_deg', 15.0))
        self.tl = math.radians(CAL.get('tilt_left_deg', 45.0))
        self.tr = math.radians(CAL.get('tilt_right_deg', 45.0))
        self.hl = CAL.get('height_left', 0.889)
        self.hr = CAL.get('height_right', 0.889)
        self.rl = math.radians(CAL.get('roll_left_deg', 0.0))
        self.rr = math.radians(CAL.get('roll_right_deg', 0.0))
        self.net_dist = CAL.get('net_dist_z', 0.52)

        self._app = Flask(__name__)
        logging.getLogger('werkzeug').setLevel(logging.ERROR)

        @self._app.route('/')
        def index(): return render_template_string(_HTML_PAGE)

        @self._app.route('/stream/<side>')
        def stream(side):
            def gen():
                while True:
                    if self._frames.get(side): yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + self._frames[side] + b'\r\n')
                    time.sleep(0.008)
            return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self._app.route('/api/stats')
        def stats(): return Response(json.dumps(self._stats), mimetype='application/json')

        @self._app.route('/api/topics')
        def topics():
            now = time.time()
            out = {}
            for name, d in list(self._topic_data.items()):
                age_ms = int((now - d['last_t']) * 1000) if d.get('last_t') else None
                out[name] = {'type': d['type'], 'age_ms': age_ms, 'values': d['values']}
            return Response(json.dumps(out), mimetype='application/json')

        @self._app.route('/api/toggle_align', methods=['POST'])
        def toggle_align():
            self.show_align = not self.show_align
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/toggle_roi', methods=['POST'])
        def toggle_roi():
            self.show_roi_mask = not self.show_roi_mask
            return Response(json.dumps({'ok': True, 'state': self.show_roi_mask}), mimetype='application/json')

        @self._app.route('/api/config_all')
        def config_all():
            return Response(json.dumps(CAL), mimetype='application/json')

        @self._app.route('/api/save_config_all', methods=['POST'])
        def save_config_all():
            data = request.get_json()
            new_data = {}
            for k, v in data.items():
                if k in CAL:
                    try:
                        if isinstance(CAL[k], int): new_data[k] = int(float(v))
                        elif isinstance(CAL[k], float): new_data[k] = float(v)
                        else: new_data[k] = v
                    except: pass
            
            _save_all_to_cal(new_data)
            
            # Apply hot-swappable params immediately
            for node in ['/ball_detector_left', '/ball_detector_right']:
                if 'min_contrast' in new_data: subprocess.run(['ros2', 'param', 'set', node, 'min_contrast', str(new_data['min_contrast'])], timeout=1)
                if 'motion_threshold' in new_data: subprocess.run(['ros2', 'param', 'set', node, 'motion_threshold', str(new_data['motion_threshold'])], timeout=1)
                if 'min_area' in new_data: subprocess.run(['ros2', 'param', 'set', node, 'min_area', str(new_data['min_area'])], timeout=1)
                if 'max_area' in new_data: subprocess.run(['ros2', 'param', 'set', node, 'max_area', str(new_data['max_area'])], timeout=1)
                if 'edge_margin' in new_data: subprocess.run(['ros2', 'param', 'set', node, 'edge_margin', str(new_data['edge_margin'])], timeout=1)
            
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/set_roi', methods=['POST'])
        def set_roi():
            data = request.get_json()
            nodes = {'left': '/ball_detector_left', 'right': '/ball_detector_right'}
            results = {}
            for side, node in nodes.items():
                pts = data.get(side)
                if not pts or len(pts) != 8: continue
                val = '[' + ','.join(str(v) for v in pts) + ']'
                try:
                    r = subprocess.run(['ros2', 'param', 'set', node, 'table_roi', val], capture_output=True, text=True, timeout=8)
                    results[side] = {'ok': r.returncode == 0, 'msg': r.stdout.strip() or r.stderr.strip() or 'ok'}
                except subprocess.TimeoutExpired:
                    results[side] = {'ok': False, 'msg': f'timeout — is {node} running?'}
                except Exception as e:
                    results[side] = {'ok': False, 'msg': str(e)}
                else:
                    if results.get(side, {}).get('ok'): _save_roi_to_cal(side, pts)
            return Response(json.dumps(results), mimetype='application/json')

        @self._app.route('/api/set_net_z', methods=['POST'])
        def set_net_z():
            z = float(request.get_json().get('z', 0.0))
            result = subprocess.run(['ros2', 'param', 'set', '/trajectory_node', 'net_z', str(z)], capture_output=True, text=True, timeout=5)
            ok = result.returncode == 0
            return Response(json.dumps({'ok': ok, 'z': z, 'msg': result.stdout.strip() or result.stderr.strip()}), mimetype='application/json')

        @self._app.route('/api/debug_logs', methods=['GET', 'DELETE'])
        def debug_logs():
            if request.method == 'DELETE':
                self.traj_logs.clear()
                return Response(json.dumps({'ok': True}), mimetype='application/json')
            return Response(json.dumps(self.traj_logs), mimetype='application/json')

        @self._app.route('/api/config')
        def config():
            # MERGED: also expose intrinsics so JS ray-drawing uses real values
            return Response(json.dumps({
                'pan_left_deg':   CAL.get('pan_left_deg',  15.0),
                'pan_right_deg':  CAL.get('pan_right_deg', 15.0),
                'tilt_left_deg':  CAL.get('tilt_left_deg', 45.0),
                'tilt_right_deg': CAL.get('tilt_right_deg', 45.0),
                'height_left':    CAL.get('height_left', 0.889),
                'height_right':   CAL.get('height_right', 0.889),
                'roll_left_deg':  CAL.get('roll_left_deg', 0.0),
                'roll_right_deg': CAL.get('roll_right_deg', 0.0),
                'net_dist_z':     CAL.get('net_dist_z', 0.52),
                'min_contrast':   CAL.get('min_contrast', 100),
                'motion_threshold': CAL.get('motion_threshold', 5),
                'min_area':       CAL.get('min_area', 1),
                'max_area':       CAL.get('max_area', 150),
                'baseline_m':     CAL.get('baseline_m', 1.525),
                'fx':             CAL.get('fx', 448.2),
                'fy':             CAL.get('fy', 448.2),
                'cx':             CAL.get('cx', 320.0),
                'cy':             CAL.get('cy', 200.0),
            }), mimetype='application/json')

        @self._app.route('/api/preview_angles', methods=['POST'])
        def preview_angles():
            data = request.get_json()
            self.pl = math.radians(float(data.get('pl', 15.0)))
            self.pr = math.radians(float(data.get('pr', 15.0)))
            self.tl = math.radians(float(data.get('tl', 45.0)))
            self.tr = math.radians(float(data.get('tr', 45.0)))
            self.hl = float(data.get('hl', 0.889))
            self.hr = float(data.get('hr', 0.889))
            self.rl = math.radians(float(data.get('rl', 0.0)))
            self.rr = math.radians(float(data.get('rr', 0.0)))
            self.net_dist = float(data.get('nd', 0.52))
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/set_angles', methods=['POST'])
        def set_angles():
            data = request.get_json()
            pl = float(data.get('pl', CAL.get('pan_left_deg', 15.0)))
            pr = float(data.get('pr', CAL.get('pan_right_deg', 15.0)))
            tl = float(data.get('tl', CAL.get('tilt_left_deg', 45.0)))
            tr = float(data.get('tr', CAL.get('tilt_right_deg', 45.0)))
            hl = float(data.get('hl', CAL.get('height_left', 0.889)))
            hr = float(data.get('hr', CAL.get('height_right', 0.889)))
            rl = float(data.get('rl', CAL.get('roll_left_deg', 0.0)))
            rr = float(data.get('rr', CAL.get('roll_right_deg', 0.0)))
            nd = float(data.get('nd', CAL.get('net_dist_z', 0.52)))
            contrast = int(data.get('contrast', CAL.get('min_contrast', 100)))
            motion = int(data.get('motion_thresh', CAL.get('motion_threshold', 5)))
            min_area = int(data.get('min_area', CAL.get('min_area', 1)))
            max_area = int(data.get('max_area', CAL.get('max_area', 150)))

            msgs = []
            ok = True

            # Angle params to stereo_node
            for param, val in [('pan_left_deg', pl), ('pan_right_deg', pr),
                               ('tilt_left_deg', tl), ('tilt_right_deg', tr),
                               ('roll_left_deg', rl), ('roll_right_deg', rr)]:
                try:
                    r = subprocess.run(['ros2', 'param', 'set', '/stereo_node', param, str(val)],
                                       capture_output=True, text=True, timeout=3)
                    if r.returncode != 0: ok = False; msgs.append(r.stderr.strip())
                except: ok = False; msgs.append(f'timeout {param}')

            # MERGED FROM DOC 6: also push height and net distance to stereo_node
            avg_height = (hl + hr) / 2.0
            for param, val in [('height_m', avg_height), ('net_dist_m', nd)]:
                try:
                    subprocess.run(['ros2', 'param', 'set', '/stereo_node', param, str(val)], timeout=3)
                except: pass

            # Contrast, Motion, and Area to vision nodes
            for node in ['/ball_detector_left', '/ball_detector_right']:
                try: subprocess.run(['ros2', 'param', 'set', node, 'min_contrast', str(contrast)], timeout=3)
                except: pass
                try: subprocess.run(['ros2', 'param', 'set', node, 'motion_threshold', str(motion)], timeout=3)
                except: pass
                try: subprocess.run(['ros2', 'param', 'set', node, 'min_area', str(min_area)], timeout=3)
                except: pass
                try: subprocess.run(['ros2', 'param', 'set', node, 'max_area', str(max_area)], timeout=3)
                except: pass


            # MERGED: use unified _save_all_to_cal
            if ok:
                _save_all_to_cal({
                    'pan_left_deg': pl, 'pan_right_deg': pr,
                    'tilt_left_deg': tl, 'tilt_right_deg': tr,
                    'height_left': hl, 'height_right': hr,
                    'roll_left_deg': rl, 'roll_right_deg': rr,
                    'net_dist_z': nd, 'min_contrast': contrast, 'motion_threshold': motion,
                    'min_area': min_area, 'max_area': max_area,
                })
            return Response(json.dumps({'ok': ok, 'msg': '; '.join(msgs) or 'ok'}), mimetype='application/json')

        @self._app.route('/api/arm_ready', methods=['POST'])
        def api_arm_ready():
            if hasattr(self, 'ros_worker'):
                msg = String(); msg.data = 'ready'
                self.ros_worker.arm_cmd_pub.publish(msg)
            return Response(json.dumps({'ok': True}), mimetype='application/json')

        @self._app.route('/api/arm_home', methods=['POST'])
        def api_arm_home():
            if hasattr(self, 'ros_worker'):
                msg = String(); msg.data = 'home'
                self.ros_worker.arm_cmd_pub.publish(msg)
            return Response(json.dumps({'ok': True}), mimetype='application/json')

        @self._app.route('/api/arm_test', methods=['POST'])
        def api_arm_test():
            if hasattr(self, 'ros_worker'):
                msg = String(); msg.data = 'test'
                self.ros_worker.arm_cmd_pub.publish(msg)
            return Response(json.dumps({'ok': True}), mimetype='application/json')

        @self._app.route('/api/stm_home', methods=['POST'])
        def api_stm_home():
            if hasattr(self, 'ros_worker'):
                msg = String(); msg.data = 'home'
                self.ros_worker.stm_cmd_pub.publish(msg)
            return Response(json.dumps({'ok': True}), mimetype='application/json')

    def push_frame(self, f, s):
        _, j = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self._frames[s] = j.tobytes()

    def start(self): threading.Thread(target=lambda: self._app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()

class SystemLauncher(threading.Thread):
    def run(self):
        print(" Syncing and building workspace on Jetson A... (this may take a moment)")
        subprocess.run("bash /home/capstone-nano2/Sensors/tabletennistrainer_ws/sync_and_build.sh", shell=True)
        print(" Workspace synced! Launching ROS 2 nodes...")

        # --- RIGHT CAMERA LAUNCH COMMAND (Jetson B - Local) ---
        _roi_b = (' -p table_roi:="[' + ','.join(str(v) for v in CAL['table_roi_right']) + ']"') if CAL.get('table_roi_right') else ''
        cmd_b = (
            "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_b.xml\n"
            "export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\"\n"
            "source /opt/ros/humble/setup.bash && source /home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash\n"
            "ros2 launch ttt_bringup camera_right.launch.py &\n"
            f"ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_right -p camera_id:=right -p min_area:={CAL['min_area']} -p max_area:={CAL['max_area']} -p motion_threshold:={CAL['motion_threshold']} -p min_contrast:={CAL.get('min_contrast', 100)} -p dilate_iters:={CAL['dilate_iters']} -p edge_margin:={CAL['edge_margin']} -p kf_gate_px:={CAL.get('kf_gate_px', 250.0)} -p kf_process_noise:={CAL.get('kf_process_noise', 0.05)} {_roi_b} &\n"
            "wait"
        )
        with open('/tmp/lb.sh', 'w') as f: f.write(cmd_b)
        subprocess.Popen("bash /tmp/lb.sh", shell=True)

        # --- LEFT CAMERA + STEREO + TRAJECTORY LAUNCH COMMAND (Jetson A - Remote) ---
        _roi_a = (' -p table_roi:="[' + ','.join(str(v) for v in CAL['table_roi_left']) + ']"') if CAL.get('table_roi_left') else ''
        
        # Calculate the averaged Origin Shifts
        avg_height = (CAL.get('height_left', 0.889) + CAL.get('height_right', 0.889)) / 2.0
        net_dist = CAL.get('net_dist_z', 0.52)

        cmd_a = (
            "echo \"<?xml version='1.0' encoding='UTF-8' ?><CycloneDDS xmlns='https://cdds.io/config'><Domain id='any'><General><Interfaces><NetworkInterface address='192.168.1.10'/></Interfaces></General></Domain></CycloneDDS>\" > /tmp/cyc_a.xml\n"
            "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_a.xml\n"
            "export RCUTILS_CONSOLE_OUTPUT_FORMAT=\"[{severity}] [{name}]: {message}\"\n"
            "source /opt/ros/humble/setup.bash && source /home/capstone-nano1/Sensors/tabletennistrainer_ws/install/setup.bash\n"
            "ros2 launch ttt_bringup camera_left.launch.py &\n"
            
            # Vision Node
            f"ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_left -p camera_id:=left -p min_area:={CAL['min_area']} -p max_area:={CAL['max_area']} -p motion_threshold:={CAL['motion_threshold']} -p min_contrast:={CAL.get('min_contrast', 100)} -p dilate_iters:={CAL['dilate_iters']} -p edge_margin:={CAL['edge_margin']} -p kf_gate_px:={CAL.get('kf_gate_px', 250.0)} -p kf_process_noise:={CAL.get('kf_process_noise', 0.05)} {_roi_a} &\n"
            
            # Stereo Node (With all alignment parameters)
            f"ros2 run ttt_stereo stereo_node --ros-args -p fx:={CAL['fx']} -p fy:={CAL['fy']} -p cx:={CAL['cx']} -p cy:={CAL['cy']} -p baseline_m:={CAL.get('baseline_m', 1.525)} -p max_sync_age_ms:={CAL['max_sync_age_ms']} "
            f"-p pan_left_deg:={CAL.get('pan_left_deg', 15.0)} -p pan_right_deg:={CAL.get('pan_right_deg', 15.0)} "
            f"-p tilt_left_deg:={CAL.get('tilt_left_deg', 45.0)} -p tilt_right_deg:={CAL.get('tilt_right_deg', 45.0)} "
            f"-p roll_left_deg:={CAL.get('roll_left_deg', 0.0)} -p roll_right_deg:={CAL.get('roll_right_deg', 0.0)} "
            f"-p height_m:={avg_height} -p net_dist_m:={net_dist} "
            f"-p limit_x_m:={CAL.get('limit_x_m', 1.5)} -p limit_y_top_m:={CAL.get('limit_y_top_m', 2.0)} -p limit_y_bottom_m:={CAL.get('limit_y_bottom_m', -0.2)} -p limit_z_m:={CAL.get('limit_z_m', 2.5)} &\n"
            
            # Robust Trajectory Node
            f"ros2 run ttt_trajectory trajectory_node --ros-args -p lookahead_ms:={CAL['lookahead_ms']} -p stage1_min_samples:={CAL.get('stage1_min_samples', 3)} -p stage2_min_samples:={CAL.get('stage2_min_samples', 8)} -p max_samples:={CAL['max_samples']} "
            f"-p gravity:={CAL['gravity']} -p restitution:={CAL['restitution']} "
            f"-p min_incoming_speed:={CAL.get('min_incoming_speed', 0.5)} "
            f"-p net_margin_z:={CAL.get('net_margin_z', -0.2)} "
            f"-p max_track_z:={CAL.get('max_track_z', 1.15)} "
            f"-p max_velocity:={CAL.get('max_velocity', 25.0)} &\n"
            
            # TF Tree, MoveIt, Control, and Hardware Stack
            "ros2 launch ttt_calibration calibration.launch.py &\n"
            "ros2 launch ttt_control rsp.launch.py &\n"
            "ros2 launch ttt_control move_group.launch.py &\n"
            "ros2 launch ttt_control controllers.launch.py &\n"
            # Anchor the URDF root link (table_center) to world. The URDF joint
            # center_to_robot already offsets root 1.37m from table_center.
            "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id table_center &\n"
            # Logical table frame used by control_node and stereo/trajectory nodes
            "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id table &\n"
            "sleep 5 && ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.0 && "
            "ros2 param set /move_group start_state_max_bounds_error 3.15 &\n"
            "ros2 run ttt_control control_node --ros-args --params-file /home/capstone-nano1/Sensors/tabletennistrainer_ws/src/ttt_control/config/control_kinematics_params.yaml &\n"
            "ros2 run ttt_hardware hardware_node --ros-args -p stm_ip:=192.168.1.100 -p stm_port:=7777 -p joint_topic:=/joint_states &\n"

            "wait\n"
        )
        
        with open('/tmp/la.sh', 'w') as f: f.write(cmd_a)
        subprocess.run("scp /tmp/la.sh capstone-nano1@192.168.1.10:/tmp/la.sh", shell=True, stderr=subprocess.DEVNULL)
        subprocess.Popen("ssh -tt capstone-nano1@192.168.1.10 'bash /tmp/la.sh'", shell=True)

class ROSWorker(threading.Thread):
    def __init__(self, ws):
        super().__init__(); self.daemon = True; self.ws = ws
        self.trail = deque(maxlen=25); self.pred = None; self.land = None; self.dets = {'left':None, 'right':None}
        self.last_3d_time = 0.0
        self._cam_count = {'left': 0, 'right': 0}; self._cam_t0 = {}; self._cam_fps = {'left': 0.0, 'right': 0.0}
        self._prev_phase = 0
        self.sample_log = []  # (x, y, z, stage) for debug logging
        self._stage_locked = {1: False, 2: False, 3: False}

    def _rec(self, topic, typ, values):
        self.ws._topic_data[topic] = {'type': typ, 'last_t': time.time(), 'values': values}

    def cb_joints(self, msg):
        # Update live dictionary with the latest joint angles
        for i, name in enumerate(msg.name):
            self.ws._stats['joints'][name] = msg.position[i]

    def run(self):
        rclpy.init()
        self.n = Node('marty_dashboard')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.n.create_subscription(CompressedImage, '/camera/left/compressed', lambda m: self.pf(m, 'left'), q)
        self.n.create_subscription(CompressedImage, '/camera/right/compressed', lambda m: self.pf(m, 'right'), q)
        self.n.create_subscription(PointStamped, '/ball_position_3d', self.cb_3d, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/predicted', self.cb_pred, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/landing', self.cb_land, q)
        self.n.create_subscription(Int32, '/ball_trajectory/phase', self.cb_phase, q)
        self.n.create_subscription(JointState, '/joint_states', self.cb_joints, q)
        self.n.create_subscription(BallDetection, '/ball_detection/left',
            lambda m: (self.dets.update({'left': m}),
                       self.ws._stats.update({'det_l_x': m.x if m.x >= 0 else None, 'det_l_y': m.y if m.x >= 0 else None}),
                       self._rec('/ball_detection/left', 'BallDetection', {'x': round(m.x, 1), 'y': round(m.y, 1), 'radius': round(getattr(m, 'radius', 0.0), 1), 'area': int((getattr(m, 'radius', 0.0)*2)**2), 'conf': round(getattr(m, 'confidence', 0.0), 3)})), q)
        self.n.create_subscription(BallDetection, '/ball_detection/right',
            lambda m: (self.dets.update({'right': m}),
                       self.ws._stats.update({'det_r_x': m.x if m.x >= 0 else None, 'det_r_y': m.y if m.x >= 0 else None}),
                       self._rec('/ball_detection/right', 'BallDetection', {'x': round(m.x, 1), 'y': round(m.y, 1), 'radius': round(getattr(m, 'radius', 0.0), 1), 'area': int((getattr(m, 'radius', 0.0)*2)**2), 'conf': round(getattr(m, 'confidence', 0.0), 3)})), q)
        self.arm_cmd_pub = self.n.create_publisher(String, '/arm_named_target', 10)
        self.stm_cmd_pub = self.n.create_publisher(String, '/stm_cmd', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.n)
        self.n.create_timer(0.05, self.timer_tf)  # 20Hz TF poll

        rclpy.spin(self.n)

    def timer_tf(self):
        links = {}
        errors = []
        for link in ['root', 'Base', 'Shoulder', 'UpperArm', 'Forearm', 'Wrist', 'Paddle', 'paddle_tcp']:
            try:
                t = self.tf_buffer.lookup_transform(
                    'root', link,
                    rclpy.time.Time()
                )
                links[link] = {
                    'x': t.transform.translation.x,
                    'y': t.transform.translation.y,
                    'z': t.transform.translation.z
                }
            except Exception as e:
                errors.append(f"{link}: {type(e).__name__}")

        if links:
            self.ws._stats['links'] = links
            self.ws._stats['tf_status'] = f"OK ({len(links)}/8 links)"
        else:
            # Keep stale links visible while we wait for TF to arrive
            self.ws._stats['tf_status'] = "WAITING | " + "; ".join(errors[:3])

    def cb_3d(self, m):
        now = time.time()
        
        # 1. Continuity clear (mimicking trajectory_node >200ms gap)
        if now - self.last_3d_time > 0.2:
            self._archive_trajectory()
            self.trail.clear()
            self.sample_log.clear()
            self.pred = None
            self.land = None
            self.ws._stats.update({'px': None, 'py': None, 'pz': None, 'land_x': None, 'land_z': None,
                                   'land_s1_x': None, 'land_s1_z': None, 'land_s2_x': None, 'land_s2_z': None,
                                   'land_s3_x': None, 'land_s3_z': None, 'phase': 0})
            self._stage_locked = {1: False, 2: False, 3: False}
            self._prev_phase = 0

        # 2. Bounce clear (mimicking trajectory_node bounce logic)
        if m.point.z > 0.0 and m.point.y < 0.05:
            self._archive_trajectory()
            self.trail.clear()
            self.sample_log.clear()

        self.last_3d_time = now
        self.trail.append((round(m.point.x, 3), round(m.point.y, 3), round(m.point.z, 3)))
        self.sample_log.append((round(m.point.x, 3), round(m.point.y, 3), round(m.point.z, 3),
                                self.ws._stats.get('phase', 0)))

        self.ws._stats.update({'x': m.point.x, 'y': m.point.y, 'z': m.point.z, 'trail': list(self.trail)})
        self._rec('/ball_position_3d', 'PointStamped', {'x': round(m.point.x, 3), 'y': round(m.point.y, 3), 'z': round(m.point.z, 3)})

    def _archive_trajectory(self):
        if len(self.sample_log) < 4: return
        st = self.ws._stats
        s1x, s1z = st.get('land_s1_x'), st.get('land_s1_z')
        s2x, s2z = st.get('land_s2_x'), st.get('land_s2_z')
        s3x, s3z = st.get('land_s3_x'), st.get('land_s3_z')
        has_any_pred = any(v is not None for v in [s1x, s2x, s3x])
        if not has_any_pred: return

        log_str = "─" * 42 + "\n"
        log_str += f"HIT  {len(self.sample_log)} samples\n"
        log_str += "─" * 42 + "\n"
        log_str += "SAMPLES:\n"
        for i, (x, y, z, s) in enumerate(self.sample_log):
            stage_lbl = f"S{s}" if s > 0 else "  "
            log_str += f"  [{i+1:02d}] {stage_lbl}  X:{x:+.3f}  Y:{y:.3f}  Z:{z:+.3f}\n"
        log_str += "PREDICTIONS:\n"
        if s1x is not None: log_str += f"  Stage 1  →  X:{s1x:+.3f}  Z:{s1z:+.3f}\n"
        if s2x is not None: log_str += f"  Stage 2  →  X:{s2x:+.3f}  Z:{s2z:+.3f}\n"
        if s3x is not None: log_str += f"  Stage 3  →  X:{s3x:+.3f}  Z:{s3z:+.3f}\n"
        self.ws.traj_logs.append(log_str)
        if len(self.ws.traj_logs) > 20: self.ws.traj_logs.pop(0)

    def cb_pred(self, m):
        self.pred = (m.point.x, m.point.y, m.point.z)
        self.ws._stats.update({'px': m.point.x, 'py': m.point.y, 'pz': m.point.z})
        self._rec('/ball_trajectory/predicted', 'PointStamped', {'x': round(m.point.x, 3), 'y': round(m.point.y, 3), 'z': round(m.point.z, 3)})

    def cb_land(self, m):
        self.land = (m.point.x, m.point.z)
        self.ws._stats.update({'land_x': m.point.x, 'land_z': m.point.z})
        s = self.ws._stats.get('phase', 0)
        if s in (1, 2, 3):
            # S3 always updates (live target). S1/S2 lock once the next stage fires.
            if s == 3 or not self._stage_locked.get(s, False):
                self.ws._stats[f'land_s{s}_x'] = m.point.x
                self.ws._stats[f'land_s{s}_z'] = m.point.z
        self._rec('/ball_trajectory/landing', 'PointStamped', {'x': round(m.point.x, 3), 'z': round(m.point.z, 3)})

    def cb_phase(self, m):
        new_phase = m.data
        old_phase = self._prev_phase
        # On transition to a new stage, lock the previous stage's marker
        if new_phase != old_phase:
            if old_phase in (1, 2):
                self._stage_locked[old_phase] = True
            if new_phase == 0:
                # Rally reset — unlock all stages
                self._stage_locked = {1: False, 2: False, 3: False}
        self._prev_phase = new_phase
        self.ws._stats['phase'] = new_phase
        self._rec('/ball_trajectory/phase', 'Int32', {'stage': new_phase})

    def pf(self, msg, side):
        now = time.time()
        self._cam_count[side] += 1
        if side not in self._cam_t0: self._cam_t0[side] = now
        elif now - self._cam_t0[side] >= 1.0:
            self._cam_fps[side] = self._cam_count[side] / (now - self._cam_t0[side])
            self._cam_count[side] = 0
            self._cam_t0[side] = now
        self._rec(f'/camera/{side}/compressed', 'CompressedImage', {'fps': round(self._cam_fps[side], 1)})

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
        if frame is not None:
            if self.ws.show_align: draw_alignment_overlay(frame, side, self.ws)

            if self.ws.show_roi_mask:
                roi_pts = CAL.get(f'table_roi_{side}', [])
                if roi_pts and len(roi_pts) == 8:
                    pts = np.array([(roi_pts[i*2], roi_pts[i*2+1]) for i in range(4)], dtype=np.int32)
                    cv2.polylines(frame, [pts], True, (0, 200, 0), 2)

            det = self.dets.get(side)
            if det and getattr(det, 'confidence', 1) > 0:
                cx, cy = int(det.x), int(det.y)
                draw_r = max(int(getattr(det, 'radius', 0.0)) * 2, 20)
                cv2.circle(frame, (cx, cy), draw_r, (0, 0, 255), 3)
                h, w = frame.shape[:2]
                bx, by = max(0, min(cx, w - 1)), max(0, min(cy, h - 1))
                brightness = int(frame[by, bx, 0])
                r = getattr(det, 'radius', 0.0)
                area_approx = int((r * 2) ** 2)
                label = f"r={r:.1f} area~{area_approx} contrast={getattr(det, 'confidence', 0.0):.0f} bright={brightness}"
                label_y = max(cy - draw_r - 8, 14)
                cv2.putText(frame, label, (cx - 80, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                sx, sy, sz = self.ws._stats.get('x'), self.ws._stats.get('y'), self.ws._stats.get('z')
                if sx is not None and sy is not None and sz is not None:
                    cv2.putText(frame, f"X:{sx:+.2f} Y:{sy:+.2f} Z:{sz:.2f}m", (cx - 80, label_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 1, cv2.LINE_AA)

            # ── Per-stage prediction markers ──────────────────────────────────
            phase = self.ws._stats.get('phase', 0)
            land = self.land
            if phase > 0 and land is not None:
                ws = self.ws
                cam_x = -(ws.baseline / 2.0) if side == 'left' else (ws.baseline / 2.0)
                cam_y = ws.hl if side == 'left' else ws.hr
                cam_z = -ws.net_dist
                p_ang = ws.pl if side == 'left' else -ws.pr
                t_ang = ws.tl if side == 'left' else ws.tr
                r_ang = ws.rl if side == 'left' else ws.rr

                # Project landing point (table y=0) into camera
                dx = land[0] - cam_x
                dy = cam_y - 0.0
                dz = land[1] - cam_z
                x1 = dx * math.cos(p_ang) - dz * math.sin(p_ang)
                z1 = dx * math.sin(p_ang) + dz * math.cos(p_ang)
                y2 = dy * math.cos(t_ang) - z1 * math.sin(t_ang)
                z2 = dy * math.sin(t_ang) + z1 * math.cos(t_ang)
                cx2 = x1 * math.cos(r_ang) - y2 * math.sin(r_ang)
                cy2 = x1 * math.sin(r_ang) + y2 * math.cos(r_ang)
                lpt = project_3d(cx2, cy2, z2)
                h_f, w_f = frame.shape[:2]
                if lpt and 0 <= lpt[0] < w_f and 0 <= lpt[1] < h_f:
                    if phase == 1:
                        # Stage 1: large blue circle (rough estimate)
                        cv2.circle(frame, lpt, 30, (255, 80, 0), 2)
                        cv2.putText(frame, "S1", (lpt[0]+32, lpt[1]+5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 80, 0), 2, cv2.LINE_AA)
                    elif phase == 2:
                        # Stage 2: medium yellow circle (coarse estimate)
                        cv2.circle(frame, lpt, 18, (0, 200, 255), 2)
                        cv2.putText(frame, "S2", (lpt[0]+20, lpt[1]+5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2, cv2.LINE_AA)
                    elif phase == 3:
                        # Stage 3: small green circle + crosshair (final precise)
                        cv2.circle(frame, lpt, 10, (0, 255, 80), 2)
                        cv2.line(frame, (lpt[0]-18, lpt[1]), (lpt[0]+18, lpt[1]), (0, 255, 80), 1)
                        cv2.line(frame, (lpt[0], lpt[1]-18), (lpt[0], lpt[1]+18), (0, 255, 80), 1)
                        cv2.putText(frame, "S3", (lpt[0]+12, lpt[1]-12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 80), 2, cv2.LINE_AA)

            draw_axis_indicator(frame)
            self.ws.push_frame(frame, side)

def _cleanup():
    subprocess.run("pkill -f 'ros2 run'", shell=True, stderr=subprocess.DEVNULL)
    subprocess.run("ssh capstone-nano1@192.168.1.10 \"pkill -f 'ros2 run'\"", shell=True, stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    import atexit; atexit.register(_cleanup)
    signal.signal(signal.SIGTERM, lambda *_: (_cleanup(), sys.exit(0)))
    _cleanup()

    streamer = WebStreamer()
    streamer.start()
    SystemLauncher().start()
    worker = ROSWorker(streamer)
    streamer.ros_worker = worker
    worker.start()

    print("=======================================\n FULL VISION & TELEMETRY ONLINE\nVIEW AT: http://192.168.55.1:5000\n=======================================")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        _cleanup()
        sys.exit(0)
