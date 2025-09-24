import os, time, math
import numpy as np
import cv2
from flask import Flask, Response, render_template_string, request


# ====== import your globals if they live elsewhere ======
# from your_nav_module import env_map, CELL_CM, GOAL_ROW_MIN, GOAL_ROW_MAX, GOAL_COL_MIN, GOAL_COL_MAX
# For demo defaults:
CELL_CM = 0.5
GOAL_ROW_MIN, GOAL_ROW_MAX = 190, 199
GOAL_COL_MIN, GOAL_COL_MAX = 190, 199

# If you have Vilib camera running, you can keep mjpg routes; else remove these
try:
    from vilib import Vilib
    HAVE_VILIB = True
except Exception:
    HAVE_VILIB = False

# ---- Shared state (set these from your main loop) ----
ENV_MAP = None                 # np.uint8 2D, 0 free / 1 occ, shape (H, W), row0 bottom
WAYPOINTS = None               # list of (x_cm, y_cm, theta)
CAR_POSE = None               # (x_cm, y_cm)

app = Flask(__name__)

INDEX_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Robot UI</title>
  <style>
    body { font-family: sans-serif; margin: 16px; }
    img { max-width: 48vw; border: 1px solid #ccc; }
    .row { display: flex; gap: 16px; align-items: flex-start; }
  </style>
</head>
<body>
  <h2>Robot UI</h2>
  <div class="row">
    {% if have_cam %}
    <div>
      <h3>Camera</h3>
      <img src="/mjpg" />
    </div>
    {% endif %}
    <div>
      <h3>Map</h3>
      <img id="map" src="/map.png?ts={{ts}}" />
    </div>
  </div>
  <script>
    // refresh the map every second by cache-busting with a timestamp
    setInterval(() => {
      const img = document.getElementById('map');
      img.src = '/map.png?ts=' + Date.now();
    }, 1000);
  </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(INDEX_HTML, have_cam=HAVE_VILIB, ts=int(time.time()))

# ---------- Camera MJPEG (optional) ----------
def _frame_jpeg():
    # Requires Vilib.camera_start(...) and Vilib.display(web=True) in your main
    return cv2.imencode('.jpg', Vilib.flask_img)[1].tobytes()

def _gen_mjpeg():
    while True:
        try:
            frame = _frame_jpeg()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
        except Exception:
            time.sleep(0.1)

@app.route("/mjpg")
def mjpg():
    if not HAVE_VILIB or not getattr(Vilib, "web_display_flag", False):
        return Response("Start camera: Vilib.display(web=True)", mimetype="text/plain")
    resp = Response(_gen_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')
    resp.headers.add("Access-Control-Allow-Origin", "*")
    return resp

# ---------- Map PNG ----------
def _render_map_png(env_map, waypoints=None, car_pose=None):
    """
    env_map: 2D uint8, 0 free / 1 occ, row0 bottom
    waypoints: list of (x_cm, y_cm, theta)
    car_pose:  (x_cm, y_cm)
    Returns: PNG bytes
    """
    h, w = env_map.shape

    # Make a grayscale: white=free, black=occ
    img = (255 - (env_map.astype(np.uint8) * 255))
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Draw goal box (in grid coords)
    x0, x1 = GOAL_COL_MIN, GOAL_COL_MAX + 1
    y0, y1 = GOAL_ROW_MIN, GOAL_ROW_MAX + 1
    # Remember: row0 bottom -> for display with OpenCV we need row0 at top ⇒ flip later
    # We'll draw in "grid coords" then flip once at the end
    cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 1)

    # Draw path (convert world cm -> grid cells)
    if waypoints:
        pts = []
        for (x_cm, y_cm, *_rest) in waypoints:
            c = int(round(x_cm / CELL_CM))
            r = int(round(y_cm / CELL_CM))
            pts.append((c, r))
        for i in range(1, len(pts)):
            cv2.line(img, pts[i-1], pts[i], (255, 0, 0), 1)

    # Draw car
    if car_pose:
        cx = int(round(car_pose[0] / CELL_CM))
        cy = int(round(car_pose[1] / CELL_CM))
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)

    # Flip vertically so row 0 appears at bottom in the browser
    img = cv2.flip(img, 0)

    # Encode PNG
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return None
    return buf.tobytes()

@app.route("/map.png")
def map_png():
    global ENV_MAP, WAYPOINTS, CAR_POSE
    if ENV_MAP is None:
        return Response("Map not ready", mimetype="text/plain", status=503)
    png = _render_map_png(ENV_MAP, WAYPOINTS, CAR_POSE)
    if png is None:
        return Response("Encode failed", mimetype="text/plain", status=500)
    resp = Response(png, mimetype="image/png")
    resp.headers.add("Cache-Control", "no-store")
    resp.headers.add("Access-Control-Allow-Origin", "*")
    return resp

def start_server():
    """Start Flask in a background thread."""
    import threading
    t = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    t.start()
    print("[web_ui] Flask server started on http://0.0.0.0:5000")

if __name__ == "__main__":
    # Listen on all interfaces so you can hit it from your laptop
    app.run(host="0.0.0.0", port=5000, debug=False)


