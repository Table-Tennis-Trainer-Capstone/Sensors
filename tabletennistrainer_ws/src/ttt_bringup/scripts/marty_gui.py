import sys, cv2, numpy as np, subprocess, os, signal, time, threading, logging, json
from collections import deque
from flask import Flask, Response, render_template_string

# --- 1. THE NETWORK SILVER BULLET (FIXED) ---
# Force Jetson B to use its active Wi-Fi IP (Removed invalid Watermark tag)
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
_scripts = ['/opt/ros/humble/setup.bash', '/home/capstone-nano2/TTT-Capstone-Sensors/tabletennistrainer_ws/install/setup.bash']
for _s in _scripts:
    for _l in subprocess.run(['bash', '-c', f'source {_s} && env'], capture_output=True, text=True).stdout.splitlines():
        if '=' in _l: os.environ.setdefault(_l.split('=', 1)[0], _l.split('=', 1)[1])

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from ttt_msgs.msg import BallDetection

# --- HTML DASHBOARD ---
_HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <title>M.A.R.T.Y. Control Center</title>
  <style>
    body{background:#121212;color:#e0e0e0;font-family:Consolas,monospace;margin:0;padding:20px;}
    h1{text-align:center;color:#00FF00;margin:0 0 12px;}
    .feeds{display:flex;gap:16px;justify-content:center;flex-wrap:wrap;}
    .feed{text-align:center;}
    .feed h3{color:#aaa;margin:4px 0;}
    .feed img{border:2px solid #444;max-width:100%;height:auto;}
    .stats{background:#1e1e1e;border-radius:10px;border:1px solid #333;padding:16px;margin:16px auto;max-width:820px;}
    .hdr{color:#aaa;text-align:center;font-size:11px;margin:8px 0 4px;}
    .row{display:flex;justify-content:space-around;margin:6px 0;}
    .lbl{color:#aaa;font-size:11px;text-align:center;}
    .val{font-size:22px;font-weight:bold;text-align:center;}
    .sval{font-size:16px;text-align:center;}
    .x{color:#ff5555}.y{color:#55ff55}.z{color:#5555ff}
    .px{color:#ff9999}.py{color:#99ff99}.pz{color:#9999ff}
    .land{color:#ffcc00}
  </style>
</head>
<body>
  <h1>M.A.R.T.Y. Control Center</h1>
  <div class="feeds">
    <div class="feed"><h3>LEFT CAMERA (A)</h3><img src="/stream/left"></div>
    <div class="feed"><h3>RIGHT CAMERA (B)</h3><img src="/stream/right"></div>
  </div>
  <div class="stats">
    <div class="hdr">CURRENT POSITION (m)</div>
    <div class="row">
      <div><div class="lbl">X</div><div class="val x" id="x">0.00</div></div>
      <div><div class="lbl">Y</div><div class="val y" id="y">0.00</div></div>
      <div><div class="lbl">Z</div><div class="val z" id="z">0.00</div></div>
    </div>
    <div class="hdr" style="margin-top:12px;">PREDICTED +250ms (m)</div>
    <div class="row">
      <div><div class="lbl">pX</div><div class="sval px" id="px">--</div></div>
      <div><div class="lbl">pY</div><div class="sval py" id="py">--</div></div>
      <div><div class="lbl">pZ</div><div class="sval pz" id="pz">--</div></div>
    </div>
    <div class="row" style="margin-top:12px;">
      <div class="sval land" id="land">Landing: --</div>
    </div>
  </div>
  <script>
    function poll(){
      fetch('/api/stats').then(r=>r.json()).then(d=>{
        document.getElementById('x').textContent=d.x.toFixed(2);
        document.getElementById('y').textContent=d.y.toFixed(2);
        document.getElementById('z').textContent=d.z.toFixed(2);
        document.getElementById('px').textContent=d.px!==null?d.px.toFixed(2):'--';
        document.getElementById('py').textContent=d.py!==null?d.py.toFixed(2):'--';
        document.getElementById('pz').textContent=d.pz!==null?d.pz.toFixed(2):'--';
        document.getElementById('land').textContent=(d.land_x!==null&&d.land_z!==null)?'Landing X: '+d.land_x.toFixed(2)+' Z: '+d.land_z.toFixed(2):'Landing: --';
      }).catch(()=>{});
      setTimeout(poll,100);
    }
    poll();
  </script>
</body>
</html>"""

# --- DRAWING FUNCTIONS ---
CAM_FX, CAM_FY, CAM_CX, CAM_CY = 224.1, 200.0, 320.0, 200.0

def project_3d(x, y, z):
    if z <= 0.05: return None
    return (int(CAM_FX * x / z + CAM_CX), int(CAM_FY * y / z + CAM_CY))

def draw_trajectory_overlay(frame, trail, predicted, landing):
    h, w = frame.shape[:2]
    def in_bounds(pt): return pt and 0 <= pt[0] < w and 0 <= pt[1] < h

    proj = [project_3d(x, y, z) for x, y, z in trail]
    for i, pt in enumerate(proj):
        if not in_bounds(pt): continue
        alpha = (i + 1) / len(trail)
        cv2.circle(frame, pt, max(2, int(4 * alpha)), (int(255 * alpha), int(180 * alpha), 0), -1)
        if i > 0 and in_bounds(proj[i-1]):
            cv2.line(frame, proj[i-1], pt, (int(255 * alpha), int(180 * alpha), 0), 1)

    if predicted:
        pt = project_3d(*predicted)
        if in_bounds(pt):
            cv2.circle(frame, pt, 10, (0, 140, 255), 2)
            cv2.line(frame, (pt[0]-14, pt[1]), (pt[0]+14, pt[1]), (0, 140, 255), 2)
            cv2.line(frame, (pt[0], pt[1]-14), (pt[0], pt[1]+14), (0, 140, 255), 2)

    if landing:
        lpt = project_3d(landing[0], predicted[1] if predicted else 0.0, landing[1])
        if in_bounds(lpt):
            cv2.line(frame, (lpt[0]-8, lpt[1]-8), (lpt[0]+8, lpt[1]+8), (0, 255, 255), 2)
            cv2.line(frame, (lpt[0]+8, lpt[1]-8), (lpt[0]-8, lpt[1]+8), (0, 255, 255), 2)

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
        self._stats = {'x':0.0, 'y':0.0, 'z':0.0, 'px':None, 'py':None, 'pz':None, 'land_x':None, 'land_z':None}
        self._app = Flask(__name__)
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        
        @self._app.route('/')
        def index(): return render_template_string(_HTML_PAGE)
        
        @self._app.route('/stream/<side>')
        def stream(side):
            def gen():
                while True:
                    if self._frames.get(side): yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + self._frames[side] + b'\r\n')
                    time.sleep(0.02)
            return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
            
        @self._app.route('/api/stats')
        def stats(): return Response(json.dumps(self._stats), mimetype='application/json')

    def push_frame(self, f, s):
        _, j = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self._frames[s] = j.tobytes()
        
    def start(self): threading.Thread(target=lambda: self._app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()

# --- SAFE SYSTEM LAUNCHER ---
class SystemLauncher(threading.Thread):
    def run(self):
        # Local Jetson B script
        cmd_b = """export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_b.xml
        v4l2-ctl -d /dev/video0 -c exposure=8000 -c analogue_gain=1200
        source /opt/ros/humble/setup.bash && source /home/capstone-nano2/TTT-Capstone-Sensors/tabletennistrainer_ws/install/setup.bash
        ros2 run ttt_camera camera_node --ros-args -r __node:=camera_right -p device:=/dev/video0 -p camera_id:=right -p show_window:=false -p width:=640 -p height:=400 -p fps:=240 &
        ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_right -p camera_id:=right -p show_window:=false &
        wait"""
        with open('/tmp/lb.sh', 'w') as f: f.write(cmd_b)
        subprocess.Popen("bash /tmp/lb.sh", shell=True)

        # Remote Jetson A script - Creates the network XML dynamically on Jetson A before launching (Removed invalid Watermark tag)
        cmd_a = """
        echo "<?xml version='1.0' encoding='UTF-8' ?><CycloneDDS xmlns='https://cdds.io/config'><Domain id='any'><General><Interfaces><NetworkInterface address='192.168.1.10'/></Interfaces></General></Domain></CycloneDDS>" > /tmp/cyc_a.xml
        export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_a.xml
        v4l2-ctl -d /dev/video0 -c exposure=8000 -c analogue_gain=1200
        source /opt/ros/humble/setup.bash && source /home/capstone-nano1/TTT-Capstone-Sensors/tabletennistrainer_ws/install/setup.bash
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot_base root --ros-args -r __node:=robot_base_to_root &
        ros2 run ttt_calibration tf_broadcaster_node --ros-args --params-file /home/capstone-nano1/TTT-Capstone-Sensors/tabletennistrainer_ws/src/ttt_calibration/config/stereo_extrinsic.yaml &
        ros2 run ttt_camera camera_node --ros-args -r __node:=camera_left -p device:=/dev/video0 -p camera_id:=left -p show_window:=false -p width:=640 -p height:=400 -p fps:=240 &
        ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_left -p camera_id:=left -p show_window:=false &
        ros2 run ttt_stereo stereo_node &
        ros2 run ttt_trajectory trajectory_node &
        wait
        """
        with open('/tmp/la.sh', 'w') as f: f.write(cmd_a)
        subprocess.run("scp /tmp/la.sh capstone-nano1@192.168.1.10:/tmp/la.sh", shell=True, stderr=subprocess.DEVNULL)
        subprocess.Popen("ssh -tt capstone-nano1@192.168.1.10 'bash /tmp/la.sh'", shell=True)

# --- ROS 2 WORKER ---
class ROSWorker(threading.Thread):
    def __init__(self, ws):
        super().__init__(); self.daemon = True; self.ws = ws
        self.trail = deque(maxlen=25); self.pred = None; self.land = None; self.dets = {'left':None, 'right':None}
        
    def run(self):
        rclpy.init()
        self.n = Node('marty_dashboard')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.n.create_subscription(CompressedImage, '/camera/left/compressed', lambda m: self.pf(m, 'left'), q)
        self.n.create_subscription(CompressedImage, '/camera/right/compressed', lambda m: self.pf(m, 'right'), q)
        self.n.create_subscription(PointStamped, '/ball_position_3d', self.cb_3d, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/predicted', self.cb_pred, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/landing', self.cb_land, q)
        self.n.create_subscription(BallDetection, '/ball_detection/left', lambda m: self.dets.update({'left':m}), q)
        self.n.create_subscription(BallDetection, '/ball_detection/right', lambda m: self.dets.update({'right':m}), q)
        rclpy.spin(self.n)

    def cb_3d(self, m): 
        self.trail.append((m.point.x, m.point.y, m.point.z))
        self.ws._stats.update({'x':m.point.x, 'y':m.point.y, 'z':m.point.z})
    def cb_pred(self, m): 
        self.pred = (m.point.x, m.point.y, m.point.z)
        self.ws._stats.update({'px':m.point.x, 'py':m.point.y, 'pz':m.point.z})
    def cb_land(self, m): 
        self.land = (m.point.x, m.point.z)
        self.ws._stats.update({'land_x':m.point.x, 'land_z':m.point.z})

    def pf(self, msg, side):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
        if frame is not None:
            det = self.dets.get(side)
            if det and getattr(det, 'confidence', 1) > 0:
                cv2.circle(frame, (int(det.x), int(det.y)), max(int(getattr(det, 'radius', 5)), 5), (0, 255, 0), 2)
            if side == 'left': draw_trajectory_overlay(frame, self.trail, self.pred, self.land)
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
    ROSWorker(streamer).start()
    
    print("=======================================\n✅ FULL VISION & TELEMETRY ONLINE\nVIEW AT: http://192.168.55.1:5000\n=======================================")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        _cleanup()
        sys.exit(0)
