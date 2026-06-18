#!/usr/bin/env python3
"""
Live GPS tracker on OpenStreetMap.

Runs a Flask web server alongside the ROS2 node.
Open http://localhost:8080 (or the configured port) in any browser.

Features:
  - OpenStreetMap background with satellite-zoom tiles
  - Color-coded position marker and accuracy circle (1σ)
  - Trajectory trail, persistent across page reloads via /history endpoint
  - HUD panel: lat / lon / alt / accuracy / fix type / point count
  - Controls: follow robot, fit trail, clear trail
  - Server-Sent Events for <1 s latency updates
  - Auto-reconnects on disconnect
"""

import json
import math
import os
import sys
import threading
from queue import Empty, Queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    from flask import Flask, Response, jsonify
except ImportError:
    sys.exit("Flask is required — run:  pip install flask")


# ── shared state ─────────────────────────────────────────────────────────────

_lock = threading.Lock()

_latest = {
    "lat": None, "lon": None, "alt": 0.0,
    "fix": "NO FIX", "fix_color": "#888888",
    "accuracy_m": None,
}

_history: list[dict] = []          # [{lat, lon, fix_color}, …]
_sse_queues: list[Queue] = []      # one Queue per connected browser tab
_MAX_HISTORY = 5000


# ── helpers ───────────────────────────────────────────────────────────────────

def _classify(status: int, acc):
    if status == NavSatStatus.STATUS_NO_FIX:
        return "NO FIX",    "#cc0000"
    if status == NavSatStatus.STATUS_GBAS_FIX and acc is not None and acc < 0.05:
        return "RTK FIXED", "#00e040"
    if status == NavSatStatus.STATUS_GBAS_FIX:
        return "RTK FLOAT", "#00bcd4"
    if status == NavSatStatus.STATUS_SBAS_FIX:
        return "DGPS/SBAS", "#ffd600"
    return "GPS FIX",       "#ff6d00"


def _push_to_sse(payload: str):
    dead = []
    for q in _sse_queues:
        try:
            q.put_nowait(payload)
        except Exception:
            dead.append(q)
    for q in dead:
        try:
            _sse_queues.remove(q)
        except ValueError:
            pass


# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.logger.disabled = True          # silence Flask request logs


@app.route("/")
def index():
    return _MAP_HTML


@app.route("/history")
def get_history():
    with _lock:
        return jsonify(_history)


@app.route("/stream")
def stream():
    q: Queue = Queue(maxsize=20)
    with _lock:
        _sse_queues.append(q)
        # immediately send latest if available
        initial = json.dumps(_latest) if _latest["lat"] is not None else None

    def generate():
        if initial:
            yield f"data: {initial}\n\n"
        while True:
            try:
                data = q.get(timeout=25)
                yield f"data: {data}\n\n"
            except Empty:
                yield ": keepalive\n\n"   # prevents proxy timeouts

    return Response(
        generate(),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class OsmVisualizerNode(Node):
    def __init__(self):
        super().__init__("osm_gps_visualizer")
        self.declare_parameter("port", 8080)
        self._port = self.get_parameter("port").value

        self.create_subscription(NavSatFix, "/fix", self._fix_cb, 10)

        flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        flask_thread.start()

        self.get_logger().info(
            f"\n"
            f"  ╔══════════════════════════════════════════╗\n"
            f"  ║  OSM GPS Visualizer running              ║\n"
            f"  ║  Open → http://localhost:{self._port}           ║\n"
            f"  ╚══════════════════════════════════════════╝"
        )

    def _fix_cb(self, msg: NavSatFix):
        cov = msg.position_covariance
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            acc = max(math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[4], 0.0)))
        else:
            acc = None

        label, color = _classify(msg.status.status, acc)

        state = {
            "lat":       msg.latitude,
            "lon":       msg.longitude,
            "alt":       round(msg.altitude, 3),
            "fix":       label,
            "fix_color": color,
            "accuracy_m": round(acc, 5) if acc is not None else None,
        }

        with _lock:
            _latest.update(state)
            _history.append({"lat": state["lat"], "lon": state["lon"],
                              "fix_color": state["fix_color"]})
            if len(_history) > _MAX_HISTORY:
                _history.pop(0)

        _push_to_sse(json.dumps(state))

    def _run_flask(self):
        import logging
        log = logging.getLogger("werkzeug")
        log.setLevel(logging.ERROR)
        app.run(host="0.0.0.0", port=self._port, debug=False, threaded=True)


# ── embedded HTML ─────────────────────────────────────────────────────────────

_MAP_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot GPS Tracker</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; font-family: 'Courier New', monospace; overflow: hidden; }
    #map { position: fixed; inset: 0; }

    /* ── HUD ──────────────────────────────────────────────────────────── */
    #hud {
      position: fixed; top: 14px; right: 14px; z-index: 1000;
      background: rgba(10, 10, 18, 0.93);
      border: 1px solid #2a2a3a;
      border-radius: 10px;
      padding: 16px 20px 14px;
      min-width: 270px;
      color: #ddd;
      backdrop-filter: blur(6px);
      box-shadow: 0 4px 24px rgba(0,0,0,0.6);
    }
    #hud-title {
      font-size: 10px; letter-spacing: 3px; color: #555;
      text-transform: uppercase; margin-bottom: 12px;
      padding-bottom: 10px; border-bottom: 1px solid #1e1e2e;
    }
    #fix-badge {
      display: block; text-align: center;
      padding: 5px 0; border-radius: 20px;
      font-size: 11px; font-weight: bold; letter-spacing: 2px;
      margin-bottom: 14px;
      transition: background 0.4s, color 0.4s, border-color 0.4s;
    }
    .row {
      display: flex; justify-content: space-between;
      align-items: baseline; margin: 5px 0;
    }
    .lbl { font-size: 10px; color: #555; letter-spacing: 1.5px; }
    .val { font-size: 13px; color: #e8e8e8; font-weight: bold; }
    .val.mono { font-size: 12px; }
    .sep { border-top: 1px solid #1e1e2e; margin: 10px 0; }

    /* ── Controls ─────────────────────────────────────────────────────── */
    #controls {
      position: fixed; bottom: 20px; right: 14px; z-index: 1000;
      display: flex; flex-direction: column; gap: 5px;
    }
    .btn {
      background: rgba(10,10,18,0.9);
      border: 1px solid #333; border-radius: 6px;
      color: #999; padding: 7px 16px;
      font-size: 10px; letter-spacing: 1.5px;
      text-transform: uppercase; cursor: pointer;
      transition: all 0.2s;
    }
    .btn:hover { background: rgba(30,30,50,0.95); color: #fff; border-color: #555; }
    .btn.on { border-color: #00e040; color: #00e040; }

    /* ── Status bar ───────────────────────────────────────────────────── */
    #statusbar {
      position: fixed; bottom: 20px; left: 14px; z-index: 1000;
      background: rgba(10,10,18,0.9);
      border: 1px solid #222; border-radius: 6px;
      padding: 6px 14px; font-size: 10px;
      color: #555; letter-spacing: 1px;
    }
  </style>
</head>
<body>
<div id="map"></div>

<div id="hud">
  <div id="hud-title">&#x25CF; Robot GPS Tracker</div>
  <div id="fix-badge" style="background:#1a1a1a;color:#555;border:1px solid #333">
    WAITING FOR GPS
  </div>
  <div class="row">
    <span class="lbl">LAT</span>
    <span class="val mono" id="v-lat">—</span>
  </div>
  <div class="row">
    <span class="lbl">LON</span>
    <span class="val mono" id="v-lon">—</span>
  </div>
  <div class="row">
    <span class="lbl">ALT</span>
    <span class="val" id="v-alt">—</span>
  </div>
  <div class="sep"></div>
  <div class="row">
    <span class="lbl">ACCURACY (1σ)</span>
    <span class="val" id="v-acc">—</span>
  </div>
  <div class="row">
    <span class="lbl">TRAIL</span>
    <span class="val" id="v-pts">0 points</span>
  </div>
</div>

<div id="controls">
  <button class="btn on"  id="btn-follow"  onclick="toggleFollow()">&#x25B6; Follow</button>
  <button class="btn"     id="btn-fit"     onclick="fitTrail()">&#x26F6; Fit trail</button>
  <button class="btn"     id="btn-clear"   onclick="clearTrail()">&#x2715; Clear trail</button>
</div>

<div id="statusbar" id="statusbar">
  <span id="conn-dot">&#x25CF;</span>&nbsp;<span id="conn-txt">Connecting…</span>
</div>

<script>
// ── Map ──────────────────────────────────────────────────────────────────
var map = L.map('map', { zoomControl: true, preferCanvas: true });

// OSM standard tiles
var osmLayer = L.tileLayer(
  'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  { attribution: '© OpenStreetMap contributors', maxZoom: 22, maxNativeZoom: 19 }
).addTo(map);

// Satellite (Esri) — user can switch layers
var satLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { attribution: 'Tiles © Esri', maxZoom: 22, maxNativeZoom: 19 }
);

L.control.layers(
  { 'OpenStreetMap': osmLayer, 'Satellite (Esri)': satLayer }
).addTo(map);

map.setView([48.0, 8.4], 15);   // default near Baden-Württemberg

// ── Leaflet objects ───────────────────────────────────────────────────────
var trail        = [];
var trailLine    = L.polyline([], { color: '#ff8c00', weight: 2.5, opacity: 0.75 }).addTo(map);
var accCircle    = L.circle([0, 0], { radius: 1, color: '#00e040', fillColor: '#00e040',
                                       fillOpacity: 0.12, weight: 1.5, opacity: 0.6 });
var posMarker    = null;
var following    = true;
var initialized  = false;

function makeIcon(color) {
  return L.divIcon({
    className: '',
    html: '<div style="width:16px;height:16px;border-radius:50%;'
        + 'background:' + color + ';border:2.5px solid #fff;'
        + 'box-shadow:0 0 0 2px ' + color + '55,0 2px 8px rgba(0,0,0,.5)"></div>',
    iconSize: [16, 16], iconAnchor: [8, 8]
  });
}

// ── Load trail history from server ────────────────────────────────────────
fetch('/history').then(r => r.json()).then(function(h) {
  if (!h.length) return;
  h.forEach(function(p) { trail.push([p.lat, p.lon]); });
  trailLine.setLatLngs(trail);
  document.getElementById('v-pts').textContent = trail.length + ' points';
  if (!initialized) {
    map.fitBounds(L.latLngBounds(trail), { padding: [50, 50] });
    initialized = true;
  }
}).catch(function() {});

// ── Update function ───────────────────────────────────────────────────────
function update(g) {
  if (g.lat === null) return;
  var ll = [g.lat, g.lon];

  if (!initialized) {
    map.setView(ll, 19);
    initialized = true;
  }

  // Position marker
  if (!posMarker) {
    posMarker = L.marker(ll, { icon: makeIcon(g.fix_color), zIndexOffset: 9999 }).addTo(map);
  } else {
    posMarker.setLatLng(ll);
    posMarker.setIcon(makeIcon(g.fix_color));
  }

  // Accuracy circle
  if (g.accuracy_m !== null) {
    if (!map.hasLayer(accCircle)) accCircle.addTo(map);
    accCircle.setLatLng(ll);
    accCircle.setRadius(g.accuracy_m);
    accCircle.setStyle({ color: g.fix_color, fillColor: g.fix_color });
  }

  // Trail
  trail.push(ll);
  trailLine.setLatLngs(trail);

  if (following) map.panTo(ll, { animate: true, duration: 0.4 });

  // HUD
  var badge = document.getElementById('fix-badge');
  badge.textContent = g.fix;
  badge.style.background    = g.fix_color + '28';
  badge.style.color         = g.fix_color;
  badge.style.border        = '1px solid ' + g.fix_color + '88';

  document.getElementById('v-lat').textContent = g.lat.toFixed(7) + '°';
  document.getElementById('v-lon').textContent = g.lon.toFixed(7) + '°';
  document.getElementById('v-alt').textContent = g.alt.toFixed(2) + ' m';
  document.getElementById('v-pts').textContent = trail.length + ' points';

  if (g.accuracy_m !== null) {
    var a = g.accuracy_m;
    var s = a < 0.01  ? (a * 1000).toFixed(1) + ' mm'
          : a < 1.0   ? (a * 100).toFixed(1)  + ' cm'
          :              a.toFixed(2)           + ' m';
    document.getElementById('v-acc').textContent = s;
    document.getElementById('v-acc').style.color = g.fix_color;
  }
}

// ── Controls ──────────────────────────────────────────────────────────────
function toggleFollow() {
  following = !following;
  document.getElementById('btn-follow').classList.toggle('on', following);
}
function fitTrail() {
  if (trail.length > 1) map.fitBounds(L.latLngBounds(trail), { padding: [50, 50] });
  else if (trail.length === 1) map.setView(trail[0], 19);
}
function clearTrail() {
  trail = []; trailLine.setLatLngs([]);
  document.getElementById('v-pts').textContent = '0 points';
}

// ── SSE connection ────────────────────────────────────────────────────────
function connect() {
  var src    = new EventSource('/stream');
  var dot    = document.getElementById('conn-dot');
  var txt    = document.getElementById('conn-txt');

  src.onopen = function() {
    dot.style.color = '#00e040'; txt.textContent = 'Connected';
  };
  src.onmessage = function(e) { update(JSON.parse(e.data)); };
  src.onerror = function() {
    dot.style.color = '#cc0000'; txt.textContent = 'Reconnecting…';
    src.close();
    setTimeout(connect, 3000);
  };
}
connect();
</script>
</body>
</html>"""


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OsmVisualizerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
