"""Generate and serve a self-contained interactive trajectory dashboard prototype."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import threading
import time
import webbrowser
from email.utils import parsedate_to_datetime
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import plotly
from plotly.offline import get_plotlyjs


HERE = Path(__file__).resolve().parent
DEFAULT_CSV = HERE.parents[1] / "output" / "ins_solution.csv"
HTML_PATH = HERE / "index.html"
DISPLAY_LIMIT = 8000
OSM_TILE_CACHE: dict[tuple[int, int, int], tuple[bytes, str, dict[str, str], float]] = {}
OSM_TILE_CACHE_LOCK = threading.Lock()
OSM_TILE_DIAGNOSTIC_LOCK = threading.Lock()
OSM_TILE_DIAGNOSTICS = {"requests": 0, "active": 0, "last_status": None, "last_tile": "", "detail": ""}
OSM_TILE_CACHE_DIR = HERE / "data" / "osm_tile_cache"
OSM_TILE_DISK_LIMIT = 2048


def fetch_osm_tile(z: int, x: int, y: int, referer: str = "") -> tuple[int, bytes, str, dict[str, str]]:
    """Fetch a viewed OSM tile with app identity and persistent HTTP-compliant caching."""
    key = (z, x, y)
    now = time.time()
    with OSM_TILE_CACHE_LOCK:
        cached = OSM_TILE_CACHE.get(key)
        if cached is None:
            cached = _read_disk_tile(key)
            if cached:
                OSM_TILE_CACHE[key] = cached
    if cached and cached[3] > now:
        return 200, cached[0], cached[1], cached[2]

    # The app owner explicitly supplied this contact for OSM request identification.
    # Allow an environment override for deployments that use another contact.
    contact = os.environ.get("OSM_CONTACT", "mailto:superderekgogo@gmail.com").strip()
    user_agent = "NavRegressionTool/0.1" + (f" ({contact})" if contact else "")
    request_headers = {"User-Agent": user_agent, "Accept": "image/png"}
    if referer:
        request_headers["Referer"] = referer
    request = Request(f"https://tile.openstreetmap.org/{z}/{x}/{y}.png", headers=request_headers)
    if cached and cached[2].get("ETag"):
        request.add_header("If-None-Match", cached[2]["ETag"])
    if cached and cached[2].get("Last-Modified"):
        request.add_header("If-Modified-Since", cached[2]["Last-Modified"])

    try:
        with urlopen(request, timeout=12) as response:
            body = response.read()
            content_type = response.headers.get("Content-Type", "image/png")
            cache_headers = {
                name: response.headers[name]
                for name in ("Cache-Control", "Expires", "ETag", "Last-Modified")
                if response.headers.get(name)
            }
            expiry = _tile_expiry(cache_headers, time.time())
            with OSM_TILE_CACHE_LOCK:
                cached = (body, content_type, cache_headers, expiry)
                OSM_TILE_CACHE[key] = cached
                _write_disk_tile(key, cached)
                if len(OSM_TILE_CACHE) > 512:
                    OSM_TILE_CACHE.pop(next(iter(OSM_TILE_CACHE)))
            return 200, body, content_type, cache_headers
    except HTTPError as error:
        if error.code == 304 and cached:
            cache_headers = dict(cached[2])
            for name in ("Cache-Control", "Expires", "ETag", "Last-Modified"):
                if error.headers.get(name):
                    cache_headers[name] = error.headers[name]
            cached = (cached[0], cached[1], cache_headers, _tile_expiry(cache_headers, time.time()))
            with OSM_TILE_CACHE_LOCK:
                OSM_TILE_CACHE[key] = cached
                _write_disk_tile(key, cached)
            return 200, cached[0], cached[1], cache_headers
        return error.code, error.read(), "text/plain; charset=utf-8", {}
    except (URLError, TimeoutError, OSError) as error:
        return 502, f"Unable to reach OpenStreetMap tile server: {error}".encode(), "text/plain; charset=utf-8", {}


def _tile_cache_paths(key: tuple[int, int, int]) -> tuple[Path, Path]:
    z, x, y = key
    image_path = OSM_TILE_CACHE_DIR / str(z) / str(x) / f"{y}.png"
    return image_path, image_path.with_suffix(".json")


def _read_disk_tile(key: tuple[int, int, int]) -> tuple[bytes, str, dict[str, str], float] | None:
    image_path, metadata_path = _tile_cache_paths(key)
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        body = image_path.read_bytes()
        return body, metadata["content_type"], metadata["headers"], float(metadata["expires_at"])
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _write_disk_tile(key: tuple[int, int, int], tile: tuple[bytes, str, dict[str, str], float]) -> None:
    image_path, metadata_path = _tile_cache_paths(key)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    image_tmp, metadata_tmp = image_path.with_suffix(".png.tmp"), metadata_path.with_suffix(".json.tmp")
    image_tmp.write_bytes(tile[0])
    metadata_tmp.write_text(json.dumps({"content_type": tile[1], "headers": tile[2], "expires_at": tile[3]}), encoding="utf-8")
    image_tmp.replace(image_path)
    metadata_tmp.replace(metadata_path)
    _prune_disk_tile_cache()


def _prune_disk_tile_cache() -> None:
    tiles = list(OSM_TILE_CACHE_DIR.rglob("*.png"))
    if len(tiles) <= OSM_TILE_DISK_LIMIT:
        return
    for stale_path in sorted(tiles, key=lambda path: path.stat().st_mtime)[:len(tiles) - OSM_TILE_DISK_LIMIT]:
        stale_path.unlink(missing_ok=True)
        stale_path.with_suffix(".json").unlink(missing_ok=True)


def _tile_expiry(headers: dict[str, str], now: float) -> float:
    cache_control = headers.get("Cache-Control", "")
    match = re.search(r"(?:^|,)\s*max-age=(\d+)", cache_control, re.IGNORECASE)
    if match:
        return now + int(match.group(1))
    expires = headers.get("Expires")
    if expires:
        try:
            return parsedate_to_datetime(expires).timestamp()
        except (TypeError, ValueError, OverflowError):
            pass
    return now + 7 * 24 * 3600


def q_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert the CSV body-to-NED quaternion to ZYX roll, pitch, yaw degrees."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def _optional_float(value: str | None) -> float | None:
    try:
        result = float(value) if value else None
    except (TypeError, ValueError):
        return None
    return result if result is not None and math.isfinite(result) else None


def load_trajectory(path: Path) -> tuple[list[dict[str, float]], int]:
    if not path.is_file():
        raise FileNotFoundError(f"Trajectory CSV not found: {path}")
    rows: list[dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        fields = set(reader.fieldnames or [])
        if {"x_ecef_m", "y_ecef_m", "z_ecef_m"}.issubset(fields):
            a, f = 6378137.0, 1.0 / 298.257223563
            e2 = f * (2.0 - f)
            for index, row in enumerate(reader):
                try:
                    t = float(row["gps_seconds"])
                    x, y, z = (float(row[k]) for k in ("x_ecef_m", "y_ecef_m", "z_ecef_m"))
                    mask = int(row.get("valid_components", "0"))
                except (TypeError, ValueError, KeyError):
                    continue
                if not all(math.isfinite(v) for v in (t, x, y, z)) or math.hypot(x, y) < 1:
                    continue
                lon = math.atan2(y, x)
                pxy = math.hypot(x, y)
                lat = math.atan2(z, pxy * (1.0 - e2))
                for _ in range(8):
                    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
                    h = pxy / max(math.cos(lat), 1e-15) - n
                    lat = math.atan2(z, pxy * (1.0 - e2 * n / (n + h)))
                n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
                h = pxy / math.cos(lat) - n
                vn = ve = vd = None
                if mask & 56 == 56:
                    try:
                        vx, vy, vz = (float(row[k]) for k in ("vx_ecef_mps", "vy_ecef_mps", "vz_ecef_mps"))
                        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
                        vn, ve, vd = (-sl*co*vx-sl*so*vy+cl*vz, -so*vx+co*vy,
                                      -cl*co*vx-cl*so*vy-sl*vz)
                    except (TypeError, ValueError, KeyError):
                        pass
                rows.append({"i": index, "t": t, "lat": math.degrees(lat), "lon": math.degrees(lon),
                             "h": h, "vn": vn, "ve": ve, "vd": vd, "roll": None, "pitch": None,
                             "yaw": None, "status": row.get("status", "UNKNOWN"), "satellites": row.get("satellites", "")})
            if not rows:
                raise ValueError("CSV contains no usable RTK trajectory rows")
            total_rows = len(rows)
            if len(rows) > DISPLAY_LIMIT:
                rows = [rows[round(i * (len(rows) - 1) / (DISPLAY_LIMIT - 1))] for i in range(DISPLAY_LIMIT)]
            return rows, total_rows
        if {"north_m", "east_m", "down_m", "latitude_deg", "longitude_deg", "height_m"}.issubset(fields):
            is_gmis_tc = {"gnss_nis", "gnss_update_accepted", "heading_rad"}.issubset(fields)
            for index, row in enumerate(reader):
                try:
                    t = float(row["gps_seconds"])
                    core_keys = (
                        "latitude_deg", "longitude_deg", "height_m", "north_m", "east_m", "down_m",
                        "vn_mps", "ve_mps", "vd_mps", "roll_rad", "pitch_rad", "heading_rad",
                        "rms_n_m", "rms_e_m", "rms_d_m", "rms_vn_mps", "rms_ve_mps", "rms_vd_mps",
                        "rms_roll_rad", "rms_pitch_rad", "rms_heading_rad", "imu_nis")
                    values = {key: float(row[key]) for key in core_keys}
                    gnss_nis_key = "gnss_nis" if is_gmis_tc else "rtk_nis"
                    values["gnss_nis"] = float(row[gnss_nis_key])
                except (TypeError, ValueError, KeyError):
                    continue
                if not math.isfinite(t) or not all(math.isfinite(value) for value in values.values()):
                    continue
                rad_to_deg = 180.0 / math.pi
                rows.append({
                    "i": index, "t": t, "lat": values["latitude_deg"], "lon": values["longitude_deg"],
                    "h": values["height_m"], "vn": values["vn_mps"], "ve": values["ve_mps"],
                    "vd": values["vd_mps"], "roll": values["roll_rad"] * rad_to_deg,
                    "pitch": values["pitch_rad"] * rad_to_deg, "yaw": values["heading_rad"] * rad_to_deg,
                    "posRms": math.sqrt(sum(values[k] ** 2 for k in ("rms_n_m", "rms_e_m", "rms_d_m"))),
                    "velRms": math.sqrt(sum(values[k] ** 2 for k in ("rms_vn_mps", "rms_ve_mps", "rms_vd_mps"))),
                    "attRms": math.sqrt(sum(values[k] ** 2 for k in ("rms_roll_rad", "rms_pitch_rad", "rms_heading_rad"))) * rad_to_deg,
                    "rollRms": values["rms_roll_rad"] * rad_to_deg,
                    "pitchRms": values["rms_pitch_rad"] * rad_to_deg,
                    "headingRms": values["rms_heading_rad"] * rad_to_deg,
                    "imuNis": values["imu_nis"], "rtkNis": values["gnss_nis"],
                    "imuAccepted": int(row.get("imu_update_accepted", "0")),
                    "rtkAccepted": int(row.get("gnss_update_accepted", row.get("rtk_update_accepted", "0"))),
                    "zuptNis": _optional_float(row.get("zupt_nis")),
                    "nhcNis": _optional_float(row.get("nhc_nis")),
                    "zpcNis": _optional_float(row.get("zpc_nis")),
                    "zhcNis": _optional_float(row.get("zhc_nis")),
                    "dynamicStatus": row.get("dynamic_status", "unknown"),
                })
            if not rows:
                raise ValueError("CSV contains no usable GMIS navigation rows")
            total_rows = len(rows)
            if len(rows) > DISPLAY_LIMIT:
                rows = [rows[round(i * (len(rows) - 1) / (DISPLAY_LIMIT - 1))] for i in range(DISPLAY_LIMIT)]
            return rows, total_rows
        required = {
            "gps_seconds", "latitude_deg", "longitude_deg", "height_m",
            "velocity_n_mps", "velocity_e_mps", "velocity_d_mps",
            "q_w", "q_x", "q_y", "q_z",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(sorted(missing))}")
        for index, row in enumerate(reader):
            try:
                values = {key: float(row[key]) for key in required}
            except (TypeError, ValueError, KeyError):
                continue
            if not all(math.isfinite(value) for value in values.values()):
                continue
            roll, pitch, yaw = q_to_euler(*(values[f"q_{axis}"] for axis in ("w", "x", "y", "z")))
            rows.append({
                "i": index,
                "t": values["gps_seconds"],
                "lat": values["latitude_deg"],
                "lon": values["longitude_deg"],
                "h": values["height_m"],
                "vn": values["velocity_n_mps"],
                "ve": values["velocity_e_mps"],
                "vd": values["velocity_d_mps"],
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "posRms": _optional_float(row.get("position_3d_rms_1sigma_m")),
                "velRms": _optional_float(row.get("velocity_3d_rms_1sigma_mps")),
                "attRms": _optional_float(row.get("attitude_3d_rms_1sigma_deg")),
                "rollRms": _optional_float(row.get("roll_rms_1sigma_deg")),
                "pitchRms": _optional_float(row.get("pitch_rms_1sigma_deg")),
                "headingRms": _optional_float(row.get("heading_rms_1sigma_deg")),
            })
    if not rows:
        raise ValueError("CSV contains no usable trajectory rows")
    total_rows = len(rows)
    if len(rows) > DISPLAY_LIMIT:
        # Uniform index sampling keeps rendering responsive while retaining both endpoints.
        count = DISPLAY_LIMIT
        rows = [rows[round(i * (len(rows) - 1) / (count - 1))] for i in range(count)]
    return rows, total_rows


def page_html(rows: list[dict[str, float]], source: Path, total_rows: int, google_maps_api_key: str,
              local_url: str = "http://127.0.0.1:8765/index.html") -> str:
    data_json = json.dumps(rows, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    plotly_js = get_plotlyjs()
    return r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Navigation Trajectory Explorer</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin="">
<style>
:root{font-family:Inter,Segoe UI,Arial,sans-serif;color:#172033;background:#f3f6fb;font-synthesis:none}
*{box-sizing:border-box}body{margin:0;min-width:340px}.topbar{height:64px;background:#101a2d;color:#fff;display:flex;align-items:center;justify-content:space-between;padding:0 24px;box-shadow:0 2px 12px #101a2d33}
.brand{font-size:17px;font-weight:650;letter-spacing:.15px}.run-label{font-size:12px;color:#c3cee0}.shell{height:calc(100vh - 64px);min-height:650px;padding:16px;display:grid;grid-template-columns:minmax(0,1.12fr) minmax(420px,.88fr);gap:16px}
.card{background:#fff;border:1px solid #e2e8f0;border-radius:12px;box-shadow:0 2px 8px #20304a0a;overflow:hidden}.left{display:flex;flex-direction:column;min-height:0}.card-title{font-size:14px;font-weight:650;padding:16px 18px 12px}.map-wrap{position:relative;flex:1;min-height:420px}#map,#google-map{position:absolute;inset:0}#google-map{display:none}.map-foot{padding:10px 16px;font-size:11px;color:#6b778c;border-top:1px solid #edf0f5;line-height:1.45}
.card-title.map-title{display:flex;align-items:center;justify-content:space-between;gap:10px}.map-actions{display:flex;align-items:center;gap:7px}.map-actions button{border:1px solid #d9e1eb;border-radius:6px;background:#fff;color:#34415a;font-size:11px;padding:5px 9px;cursor:pointer}.map-actions button.active{background:#eef4ff;border-color:#9db7f3;color:#214b9a}
.right{display:flex;flex-direction:column;min-height:0}.control-head{padding:15px 18px 12px;border-bottom:1px solid #edf0f5}.heading{font-size:14px;font-weight:650;margin-bottom:11px}.checks{display:flex;gap:8px;flex-wrap:wrap}.check{border:1px solid #dce3ee;border-radius:8px;padding:7px 10px;font-size:12px;display:flex;align-items:center;gap:6px;cursor:pointer;color:#34415a}.check:has(input:checked){background:#eef4ff;border-color:#9db7f3;color:#214b9a}.check input{accent-color:#3867d6}.toolbar{display:flex;align-items:center;justify-content:space-between;margin-top:12px;font-size:11px;color:#68758b}.toolbar select{border:1px solid #d9e1eb;border-radius:6px;padding:5px 7px;background:white;color:#34415a}
#chart{flex:1;min-height:280px;overflow:auto}.detail{border-top:1px solid #edf0f5;padding:12px 16px 14px}.detail-title{font-size:12px;font-weight:650;margin-bottom:9px}.detail-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.metric{background:#f7f9fc;border-radius:7px;padding:7px 9px;min-width:0}.metric span{display:block;color:#778399;font-size:10px;margin-bottom:4px}.metric b{font-size:11px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;display:block}.status{color:#60718b;font-size:11px;margin-top:8px}.legend-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#3867d6;margin-right:5px}.badge{font-size:10px;padding:4px 7px;border-radius:12px;background:#fff2d8;color:#845500}
@media(max-width:900px){.shell{height:auto;min-height:0;grid-template-columns:1fr}.left{height:60vh;min-height:420px}.right{height:70vh;min-height:560px}.detail-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.run-label{display:none}}
</style>
<script>__PLOTLY_JS__</script>
<script defer src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
</head>
<body>
<div id="file-open-warning" style="display:none;position:fixed;inset:0;z-index:99999;background:#101a2ddd;place-items:center;padding:24px">
  <section style="max-width:560px;background:#fff;border-radius:12px;padding:28px 32px;box-shadow:0 12px 40px #0005;font:15px/1.55 Inter,Segoe UI,Arial,sans-serif;color:#172033">
    <h2 style="margin:0 0 12px">Open the dashboard through its local server</h2>
    <p>This page was opened as a local file (<code>file://</code>). Map tiles use the dashboard server's <code>/tiles/</code> proxy, which is unavailable in file mode.</p>
    <p>Open the server version to load the map:</p>
    <p><a href="__LOCAL_URL__" style="display:inline-block;background:#3867d6;color:#fff;text-decoration:none;padding:10px 14px;border-radius:7px;font-weight:600">Open dashboard</a></p>
    <p style="font-size:12px;color:#60718b;margin-bottom:0">If the link does not open, start the dashboard server and use the HTTP address printed in PowerShell.</p>
  </section>
</div>
<script>if(location.protocol==='file:'){document.getElementById('file-open-warning').style.display='grid';}</script>
<header class="topbar"><div class="brand">Navigation Trajectory Explorer</div><div class="run-label">INS run · __SOURCE__</div></header>
<main class="shell">
  <section class="card left"><div class="card-title map-title"><span>Trajectory map <span class="badge" id="map-mode">loading OSM</span></span><span class="map-actions"><button id="view-2d" class="active" type="button">2D map</button><button id="retry-osm" type="button" style="display:none">Retry map tiles</button><button id="view-3d" type="button">3D trajectory</button></span></div><div class="map-wrap"><div id="map"></div><div id="osm-map" style="position:absolute;inset:0;display:none"></div><div id="trajectory-3d" style="position:absolute;inset:0;display:none"></div><div id="google-map"></div></div><div class="map-foot" id="map-note"><span class="legend-dot"></span>INS solution · Loading OpenStreetMap tiles. The trajectory remains available if the map service cannot be reached.</div></section>
  <section class="card right">
    <div class="control-head"><div class="heading">Plot layers</div><div class="checks">
      <label class="check"><input type="checkbox" value="position" checked> Position</label>
      <label class="check"><input type="checkbox" value="velocity"> Velocity</label>
      <label class="check"><input type="checkbox" value="attitude"> Attitude</label>
      <label class="check"><input type="checkbox" value="rms"> RMS (formal 1σ)</label>
      <label class="check"><input type="checkbox" value="innovations" checked> Innovations (NIS)</label>
    </div><div class="toolbar"><span>Click a trajectory sample to sync the map and plots</span><span>Latitude / Longitude · local view</span></div></div>
    <div id="chart"></div>
    <div class="detail"><div class="detail-title">Selected epoch</div><div class="detail-grid">
      <div class="metric"><span>GPS time</span><b id="d-time">—</b></div><div class="metric"><span>Latitude / Longitude</span><b id="d-coord">—</b></div>
      <div class="metric"><span>Height</span><b id="d-height">—</b></div><div class="metric"><span>N / E / D velocity</span><b id="d-vel">—</b></div>
      <div class="metric"><span id="att-label">Roll / Pitch / Yaw</span><b id="d-att">—</b></div><div class="metric"><span>Displayed samples</span><b>__SAMPLES__ / __TOTAL__</b></div>
      <div class="metric"><span>Position span</span><b id="d-span">—</b></div><div class="metric"><span>Data note</span><b>Unfused INS output</b></div>
    </div><div class="status" id="run-status">Raw standalone INS trajectory; no GNSS corrections are applied in this run.</div></div>
  </section>
</main>
<script>
const D=__DATA__;
const IS_RTK=__IS_RTK__;
const GOOGLE_MAPS_API_KEY=__GOOGLE_MAPS_API_KEY__;
const fmt=v=>Number.isFinite(v)?v.toFixed(3):'—';
const colors={blue:'#3867d6',green:'#20a37a',orange:'#e39b29',purple:'#855bd6',red:'#d14a61',teal:'#168b9a'};
const lats=D.map(p=>p.lat), lons=D.map(p=>p.lon), times=D.map(p=>p.t);
const center={lat:(Math.min(...lats)+Math.max(...lats))/2,lon:(Math.min(...lons)+Math.max(...lons))/2};
const span=Math.max(Math.max(...lats)-Math.min(...lats),Math.max(...lons)-Math.min(...lons),0.002);
const zoom=Math.max(1,Math.min(15,Math.log2(90/span)));
const mapDiv=document.getElementById('map'), chartDiv=document.getElementById('chart'), threeDDiv=document.getElementById('trajectory-3d');
let mapMode='coordinates',activeMapView='2d',googleMap=null,googleSelectedMarker=null,osmMap=null,osmSelectedMarker=null,osmTileLayer=null,osmTilesUnavailable=false,osmTileLoads=0,osmTileErrors=0,osmLoadTimer=null;
function coordinateMapData(){return [
 {type:'scatter',mode:'lines',x:lons,y:lats,line:{color:colors.blue,width:2.5},name:'INS trajectory',customdata:D.map((p,i)=>[i,p.t,p.h]),hovertemplate:'GPS %{customdata[1]:.3f}<br>Lat %{y:.6f}°<br>Lon %{x:.6f}°<br>h %{customdata[2]:.2f} m<extra></extra>'},
 {type:'scatter',mode:'markers',x:[D[0].lon],y:[D[0].lat],marker:{size:11,color:colors.orange,line:{color:'#fff',width:2}},name:'Selected epoch',hoverinfo:'skip'},
 {type:'scatter',mode:'markers',x:[D[0].lon,D[D.length-1].lon],y:[D[0].lat,D[D.length-1].lat],marker:{size:8,color:['#20a37a','#d14a61'],line:{color:'#fff',width:1}},name:'Start / end',customdata:[0,D.length-1],hovertemplate:'%{customdata}<extra>Start / end sample</extra>'}
];}
function drawMap(){
 mapMode='coordinates';
 const traces=coordinateMapData();
 const xspan=Math.max(...lons)-Math.min(...lons),yspan=Math.max(...lats)-Math.min(...lats);
 const xpad=Math.max(xspan*.08,.00005),ypad=Math.max(yspan*.08,.00005);
 const layout={autosize:true,margin:{l:58,r:18,t:18,b:48},paper_bgcolor:'#f8fafc',plot_bgcolor:'#f8fafc',showlegend:false,hovermode:'closest',xaxis:{title:'Longitude (°)',range:[Math.min(...lons)-xpad,Math.max(...lons)+xpad],gridcolor:'#dfe6ef',zeroline:false,tickformat:'.2f'},yaxis:{title:'Latitude (°)',range:[Math.min(...lats)-ypad,Math.max(...lats)+ypad],gridcolor:'#dfe6ef',zeroline:false,tickformat:'.2f',scaleanchor:'x',scaleratio:1/Math.cos(center.lat*Math.PI/180)}};
 if(!mapDiv._fullLayout) return Plotly.newPlot(mapDiv,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});
 return Plotly.react(mapDiv,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});
}
function draw3DTrajectory(){
 const lat0=D[0].lat*Math.PI/180,lon0=D[0].lon*Math.PI/180,earthRadius=6378137;
 const east=D.map(p=>earthRadius*Math.cos(lat0)*(p.lon*Math.PI/180-lon0));
 const north=D.map(p=>earthRadius*(p.lat*Math.PI/180-lat0));
 const up=D.map(p=>p.h-D[0].h);
 const trace=[
  {type:'scatter3d',mode:'lines',x:east,y:north,z:up,line:{color:colors.blue,width:5},name:'INS trajectory',customdata:D.map((p,i)=>[i,p.t,p.lat,p.lon,p.h]),hovertemplate:'GPS %{customdata[1]:.3f} s<br>E %{x:.1f} m<br>N %{y:.1f} m<br>Δh %{z:.1f} m<br>%{customdata[2]:.6f}°, %{customdata[3]:.6f}°<extra></extra>'},
  {type:'scatter3d',mode:'markers',x:[east[0]],y:[north[0]],z:[up[0]],marker:{size:6,color:colors.orange,line:{color:'#fff',width:1}},name:'Selected epoch',customdata:[0],hovertemplate:'Selected epoch<extra></extra>'},
  {type:'scatter3d',mode:'markers',x:[east[0],east[east.length-1]],y:[north[0],north[north.length-1]],z:[up[0],up[up.length-1]],marker:{size:4,color:[colors.green,colors.red]},name:'Start / end',customdata:[0,D.length-1],hovertemplate:'Sample %{customdata}<extra>Start / end</extra>'}
 ];
 const layout={autosize:true,margin:{l:0,r:0,t:12,b:0},paper_bgcolor:'#f8fafc',showlegend:false,hovermode:'closest',scene:{xaxis:{title:'East (m)',gridcolor:'#dfe6ef',backgroundcolor:'#f8fafc'},yaxis:{title:'North (m)',gridcolor:'#dfe6ef',backgroundcolor:'#f8fafc'},zaxis:{title:'Height change (m)',gridcolor:'#dfe6ef',backgroundcolor:'#f8fafc'},aspectmode:'data',camera:{eye:{x:1.55,y:1.55,z:0.9}}}};
 const rendered=threeDDiv._fullLayout?Plotly.react(threeDDiv,trace,layout,{responsive:true,displaylogo:false,scrollZoom:true}):Plotly.newPlot(threeDDiv,trace,layout,{responsive:true,displaylogo:false,scrollZoom:true});
 return rendered.then(()=>{if(!threeDDiv._trajectoryClickBound){threeDDiv.on('plotly_click',e=>{if(e.points&&e.points.length){const value=e.points[0].customdata;selectIndex(Array.isArray(value)?value[0]:value)}});threeDDiv._trajectoryClickBound=true;}});
}
const definitions={
 position:{title:'Position',rows:[['Latitude','lat','deg',colors.blue],['Longitude','lon','deg',colors.green],['Height','h','m',colors.orange]]},
 velocity:{title:'Velocity',rows:[['North velocity','vn','m/s',colors.blue],['East velocity','ve','m/s',colors.green],['Down velocity','vd','m/s',colors.orange]]},
 attitude:{title:'Attitude (body to NED)',rows:[['Roll','roll','deg',colors.blue],['Pitch','pitch','deg',colors.green],['Yaw','yaw','deg',colors.orange]]}
 ,rms:{title:'Formal 1σ RMS (sqrt trace of covariance)',rows:[['Position 3D RMS','posRms','m',colors.blue],['Velocity 3D RMS','velRms','m/s',colors.green],['Attitude 3D RMS','attRms','deg',colors.orange]]}
 ,innovations:{title:'Pre-update normalized innovation squared (NIS)',rows:[['IMU update NIS','imuNis','χ²',colors.blue],['RTK P/V update NIS','rtkNis','χ²',colors.orange]]}
};
let chartRows=0;
function drawChart(){
 const selected=[...document.querySelectorAll('.check input:checked')].map(x=>x.value);
 const groups=selected.map(key=>definitions[key]); chartRows=groups.reduce((sum,group)=>sum+group.rows.length,0);
 if(!chartRows){const emptyLayout={height:280,margin:{l:55,r:15,t:15,b:40},annotations:[{text:'Select Position, Velocity, or Attitude to display plots',showarrow:false,x:.5,y:.5,xref:'paper',yref:'paper'}]};return chartDiv._fullLayout?Plotly.react(chartDiv,[],emptyLayout,{responsive:true,displaylogo:false}):Plotly.newPlot(chartDiv,[],emptyLayout,{responsive:true,displaylogo:false});}
 const traces=[],layout={height:Math.max(360,chartRows*125+55),margin:{l:68,r:20,t:20,b:42},showlegend:false,hovermode:'x unified',font:{family:'Inter,Segoe UI,Arial,sans-serif',size:10,color:'#4a5870'},shapes:[]};
 let row=0;
 for(const group of groups){
  for(const [name,key,unit,color] of group.rows){
   row++; const axisSuffix=row===1?'':row; const xaxis='x'+axisSuffix,yaxis='y'+axisSuffix; const domainTop=1-(row-1)/chartRows,domainBottom=1-row/chartRows;
   traces.push({type:'scatter',mode:'lines',x:times,y:D.map(p=>p[key]),xaxis,yaxis,name,line:{color,width:1.35},hovertemplate:`%{x:.3f}<br>${name}: %{y:.5f} ${unit}<extra></extra>`});
   layout['yaxis'+axisSuffix]={domain:[domainBottom+0.035,domainTop-0.025],title:{text:`${name} (${unit})`,font:{size:10}},automargin:true,zerolinecolor:'#e8edf4',gridcolor:'#eef1f6'};
   layout['xaxis'+axisSuffix]={anchor:yaxis,showticklabels:row===chartRows,title:row===chartRows?{text:'GPS seconds',font:{size:10}}:undefined,showgrid:false};
   layout.shapes.push({type:'line',x0:D[0].t,x1:D[0].t,y0:0,y1:1,xref:xaxis,yref:'paper',line:{color:'#d14a61',width:1,dash:'dot'}});
  }
 }
 return chartDiv._fullLayout?Plotly.react(chartDiv,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true}):Plotly.newPlot(chartDiv,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});
}
function selectIndex(i){
 i=Math.max(0,Math.min(D.length-1,i));const p=D[i];
 document.getElementById('d-time').textContent=p.t.toFixed(3)+' s';
 document.getElementById('d-coord').textContent=p.lat.toFixed(6)+'°, '+p.lon.toFixed(6)+'°';
 document.getElementById('d-height').textContent=p.h.toFixed(2)+' m';
 document.getElementById('d-vel').textContent=`${fmt(p.vn)} / ${fmt(p.ve)} / ${fmt(p.vd)} m/s`;
 document.getElementById('d-att').textContent=IS_RTK?`${p.status} · ${p.satellites} satellites`:`${p.roll.toFixed(2)} / ${p.pitch.toFixed(2)} / ${p.yaw.toFixed(2)}°`;
 if(mapDiv.data&&mapDiv.data.length>1) Plotly.restyle(mapDiv,{x:[[p.lon]],y:[[p.lat]]},[1]);
 if(threeDdivReady()){
  const lat0=D[0].lat*Math.PI/180,lon0=D[0].lon*Math.PI/180,earthRadius=6378137;
  Plotly.restyle(threeDDiv,{x:[[earthRadius*Math.cos(lat0)*(p.lon*Math.PI/180-lon0)]],y:[[earthRadius*(p.lat*Math.PI/180-lat0)]],z:[[p.h-D[0].h]]},[1]);
 }
 if(googleSelectedMarker) googleSelectedMarker.setPosition({lat:p.lat,lng:p.lon});
 if(osmSelectedMarker) osmSelectedMarker.setLatLng([p.lat,p.lon]);
 const shapes=[]; for(let r=0;r<chartRows;r++){let suffix=r===0?'':r+1,axis='x'+suffix;shapes.push({type:'line',x0:p.t,x1:p.t,y0:0,y1:1,xref:axis,yref:'paper',line:{color:'#d14a61',width:1,dash:'dot'}})}
 if(chartRows) Plotly.relayout(chartDiv,{shapes});
}
document.querySelectorAll('.check input').forEach(el=>el.addEventListener('change',drawChart));
function threeDdivReady(){return Boolean(threeDDiv.data&&threeDDiv.data.length>1);}
function setMapView(view){
 activeMapView=view;
 const is3d=view==='3d';
 document.getElementById('view-2d').classList.toggle('active',!is3d);document.getElementById('view-3d').classList.toggle('active',is3d);
 const offline2d=!osmMap||osmTilesUnavailable;
 mapDiv.style.display=!is3d&&offline2d?'block':'none';document.getElementById('osm-map').style.display=!is3d&&osmMap&&!osmTilesUnavailable?'block':'none';threeDDiv.style.display=is3d?'block':'none';
 if(is3d){document.getElementById('map-mode').textContent='3D trajectory';document.getElementById('map-note').innerHTML='<span class="legend-dot"></span>Interactive 3D trajectory · Local east/north plane, height relative to the first sample.';if(!threeDdivReady())draw3DTrajectory();else Plotly.Plots.resize(threeDDiv);}
 else if(osmTilesUnavailable){document.getElementById('map-mode').textContent='Offline trajectory';document.getElementById('map-note').innerHTML='<span class="legend-dot"></span>Map tiles did not respond; showing the interactive latitude/longitude trajectory without a basemap.';if(mapDiv._fullLayout)Plotly.Plots.resize(mapDiv);}
 else{document.getElementById('map-mode').textContent=osmMap?'OpenStreetMap':'offline coordinate view';document.getElementById('map-note').innerHTML=osmMap?'<span class="legend-dot"></span>OpenStreetMap &copy; contributors · Click the route to select an epoch.':'<span class="legend-dot"></span>Interactive latitude/longitude trajectory.';if(osmMap){osmMap.invalidateSize({animate:false});osmMap.fitBounds(L.latLngBounds(D.map(p=>[p.lat,p.lon])).pad(.04));}else if(mapDiv._fullLayout)Plotly.Plots.resize(mapDiv);}
}
document.getElementById('view-2d').addEventListener('click',()=>setMapView('2d'));
document.getElementById('view-3d').addEventListener('click',()=>setMapView('3d'));
document.getElementById('retry-osm').addEventListener('click',()=>{if(!osmTileLayer)return;osmTileLoads=0;osmTileErrors=0;osmTilesUnavailable=false;document.getElementById('retry-osm').style.display='none';if(osmLoadTimer)clearTimeout(osmLoadTimer);osmTileLayer.redraw();setMapView('2d');osmLoadTimer=setTimeout(()=>{if(osmTileLoads===0)showOfflineTrajectoryMap('OpenStreetMap tiles did not respond.');},8000);});
document.getElementById('d-span').textContent=`${(Math.min(...lats)).toFixed(3)}° to ${(Math.max(...lats)).toFixed(3)}° lat`;
Promise.all([drawMap(),drawChart()]).then(()=>{
 mapDiv.on('plotly_click',e=>{if(e.points&&e.points.length){const p=e.points[0];const selected=p.customdata&&Number.isInteger(p.customdata[0])?p.customdata[0]:(Number.isInteger(p.customdata)?p.customdata:p.pointIndex);selectIndex(selected)}});
 chartDiv.on('plotly_click',e=>{if(e.points&&e.points.length)selectIndex(e.points[0].pointIndex)});
 selectIndex(0);
 loadOpenStreetMap();
}).catch(error=>{
 console.error('Unable to initialize trajectory plots.',error);
 mapDiv.innerHTML='<div style="padding:24px;color:#9b2432">Trajectory map failed to initialize. Open the browser console for details.</div>';
 chartDiv.innerHTML='<div style="padding:24px;color:#9b2432">Navigation plots failed to initialize. Open the browser console for details.</div>';
});
function setOfflineMapNote(message){osmTilesUnavailable=true;document.getElementById('retry-osm').style.display='inline-block';document.getElementById('map-mode').textContent='Offline trajectory';document.getElementById('map-note').innerHTML='<span class="legend-dot"></span>'+ (IS_RTK?'RTK solution':'INS solution') +' · '+message;if(activeMapView==='2d'){mapDiv.style.display='block';document.getElementById('osm-map').style.display='none';if(mapDiv._fullLayout)Plotly.Plots.resize(mapDiv);}}
function showOfflineTrajectoryMap(message){setOfflineMapNote(message+' The trajectory is shown without a basemap.');}
function nearestSample(lat,lng){let best=0,bestDistance=Infinity;const cosLat=Math.cos(lat*Math.PI/180);for(let i=0;i<D.length;i++){const dy=D[i].lat-lat,dx=(D[i].lon-lng)*cosLat,d=dx*dx+dy*dy;if(d<bestDistance){bestDistance=d;best=i;}}return best;}
function loadOpenStreetMap(attempt=0){
 const leaflet=document.getElementById('osm-map');
 if(!window.L){if(attempt<30){setTimeout(()=>loadOpenStreetMap(attempt+1),200);return;}setOfflineMapNote('OpenStreetMap library could not be reached; showing the local trajectory view.');return;}
 try{
  const bounds=L.latLngBounds(D.map(p=>[p.lat,p.lon]));
  // Leaflet measures its container when created. The OSM pane starts hidden
  // behind the coordinate fallback, so reveal it before constructing the map.
  mapDiv.style.display='none';leaflet.style.display='block';
  osmMap=L.map(leaflet,{preferCanvas:true,scrollWheelZoom:true,attributionControl:true});
  // Route viewport-only tile requests through this app so the request can carry
  // the stable application identity and contact required by the OSM tile policy.
  osmTileLayer=L.tileLayer('/tiles/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'});
  osmTileLayer.on('tileload',()=>{osmTileLoads++;if(osmTilesUnavailable){osmTilesUnavailable=false;document.getElementById('retry-osm').style.display='none';if(osmLoadTimer)clearTimeout(osmLoadTimer);if(activeMapView==='2d')setMapView('2d');}});
  osmTileLayer.on('tileerror',()=>{osmTileErrors++;if(osmTileErrors>=2&&osmTileLoads===0)showOfflineTrajectoryMap('Your browser could not load OpenStreetMap tiles. Check browser network access.');});
  osmTileLayer.addTo(osmMap);
  osmLoadTimer=setTimeout(()=>{if(osmTileLoads===0)showOfflineTrajectoryMap('OpenStreetMap tiles timed out.');},8000);
  const route=L.polyline(D.map(p=>[p.lat,p.lon]),{color:'#3867d6',weight:4,opacity:.95}).addTo(osmMap);
  L.circleMarker([D[0].lat,D[0].lon],{radius:6,color:'#20a37a',fillColor:'#20a37a',fillOpacity:1}).bindTooltip('Start').addTo(osmMap);
  L.circleMarker([D[D.length-1].lat,D[D.length-1].lon],{radius:6,color:'#d14a61',fillColor:'#d14a61',fillOpacity:1}).bindTooltip('End').addTo(osmMap);
  osmSelectedMarker=L.circleMarker([D[0].lat,D[0].lon],{radius:7,color:'#e39b29',fillColor:'#fff',fillOpacity:1,weight:3}).addTo(osmMap);
  osmMap.fitBounds(bounds.pad(.04));
  route.on('click',e=>selectIndex(nearestSample(e.latlng.lat,e.latlng.lng)));
  osmMap.on('click',e=>selectIndex(nearestSample(e.latlng.lat,e.latlng.lng)));
  mapDiv.style.display='none';leaflet.style.display=activeMapView==='2d'?'block':'none';document.getElementById('map-mode').textContent=activeMapView==='2d'?'OpenStreetMap':'3D trajectory';
  if(activeMapView==='2d')document.getElementById('map-note').innerHTML='<span class="legend-dot"></span>OpenStreetMap &copy; contributors · Click the route to select an epoch.';mapMode='osm';
 }catch(error){console.error('OpenStreetMap initialization failed.',error);leaflet.style.display='none';mapDiv.style.display='block';setOfflineMapNote('OpenStreetMap could not initialize; showing the local trajectory view.');}
}
function loadGoogleMaps(){
 if(!GOOGLE_MAPS_API_KEY){setOfflineMapNote('Offline latitude/longitude view. Set GOOGLE_MAPS_API_KEY to enable the Google street map.');return;}
 const script=document.createElement('script');script.async=true;script.src='https://maps.googleapis.com/maps/api/js?key='+encodeURIComponent(GOOGLE_MAPS_API_KEY)+'&loading=async';
 script.onload=()=>{
  try{
   const el=document.getElementById('google-map');el.style.display='block';
   const bounds=new google.maps.LatLngBounds();const path=D.map(p=>{const point={lat:p.lat,lng:p.lon};bounds.extend(point);return point;});
   googleMap=new google.maps.Map(el,{center,zoom:5,mapTypeId:'roadmap',streetViewControl:false,fullscreenControl:true,mapTypeControl:true,gestureHandling:'greedy'});
   googleMap.fitBounds(bounds,32);
   const route=new google.maps.Polyline({path,geodesic:false,strokeColor:'#3867d6',strokeOpacity:.95,strokeWeight:4,map:googleMap});
   new google.maps.Marker({position:path[0],map:googleMap,title:'Trajectory start',label:'S'});
   new google.maps.Marker({position:path[path.length-1],map:googleMap,title:'Trajectory end',label:'E'});
   googleSelectedMarker=new google.maps.Marker({position:path[0],map:googleMap,title:'Selected epoch',zIndex:1000});
   googleMap.addListener('click',e=>selectIndex(nearestSample(e.latLng.lat(),e.latLng.lng())));
   route.addListener('click',e=>selectIndex(nearestSample(e.latLng.lat(),e.latLng.lng())));
   mapDiv.style.display='none';document.getElementById('map-mode').textContent='Google Maps';
   document.getElementById('map-note').innerHTML='<span class="legend-dot"></span>Google Maps · Click the route to select an epoch. Tiles and the displayed geographic area are provided by Google.';
   mapMode='google';
  }catch(error){console.error('Google Maps initialization failed.',error);document.getElementById('google-map').style.display='none';mapDiv.style.display='block';setOfflineMapNote('Google Maps failed to initialize; showing the local trajectory view instead.');}
 };
 script.onerror=()=>{document.getElementById('google-map').style.display='none';mapDiv.style.display='block';setOfflineMapNote('Google Maps could not be reached; showing the local trajectory view instead.');};
 window.gm_authFailure=()=>{document.getElementById('google-map').style.display='none';mapDiv.style.display='block';setOfflineMapNote('Google Maps rejected the API key or project settings; showing the local trajectory view instead.');};
 document.head.appendChild(script);
}
</script></body></html>'''.replace("__PLOTLY_JS__", plotly_js).replace("__SOURCE__", source.name).replace("__DATA__", data_json).replace("__SAMPLES__", f"{len(rows):,}").replace("__TOTAL__", f"{total_rows:,}").replace("__GOOGLE_MAPS_API_KEY__", json.dumps(google_maps_api_key)).replace("__IS_RTK__", "true" if "status" in rows[0] else "false").replace("__LOCAL_URL__", local_url)


def page_html_comparison(datasets: dict[str, list[dict[str, float]]], local_url: str) -> str:
    """Build an interactive LC/TC/LTC overlay using the regression dashboard's map and Plotly stack."""
    data_json = json.dumps(datasets, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    plotly_js = get_plotlyjs()
    return r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LC / TC / LTC Regression Explorer</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin="">
<style>
:root{font-family:Inter,Segoe UI,Arial,sans-serif;color:#172033;background:#f3f6fb}*{box-sizing:border-box}body{margin:0;min-width:360px}.top{height:58px;background:#101a2d;color:#fff;padding:0 22px;display:flex;align-items:center;justify-content:space-between}.title{font-weight:650}.sub{font-size:11px;color:#cad4e5}.grid{height:calc(100vh - 58px);min-height:650px;padding:14px;display:grid;grid-template-columns:1fr 1fr;gap:14px}.card{background:#fff;border:1px solid #e2e8f0;border-radius:11px;overflow:hidden;display:flex;flex-direction:column;min-height:0}.head{padding:12px 16px;font-size:13px;font-weight:650;border-bottom:1px solid #edf0f5;display:flex;justify-content:space-between}.mapbox{position:relative;flex:1;min-height:370px}.map{position:absolute;inset:0}#fallback{display:none}.foot{padding:8px 12px;font-size:10px;color:#68758b;border-top:1px solid #edf0f5}.controls{padding:10px 14px;border-bottom:1px solid #edf0f5;display:flex;gap:8px;flex-wrap:wrap}.controls label{font-size:11px;border:1px solid #dce3ee;padding:6px 8px;border-radius:7px}.controls input{accent-color:#3867d6}.chart{flex:1;min-height:350px;overflow:auto}.stats{padding:10px 14px;border-top:1px solid #edf0f5;font-size:11px;color:#53627a;line-height:1.55}.stats b{color:#172033}.badge{font-size:10px;padding:4px 7px;background:#fff2d8;color:#845500;border-radius:12px}@media(max-width:900px){.grid{height:auto;display:flex;flex-direction:column}.card{height:60vh;min-height:500px}}
</style><script>__PLOTLY_JS__</script><script defer src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script></head>
<body><header class="top"><div class="title">LC / TC / LTC Navigation Regression Explorer</div><div class="sub">Current 18-state replay · covariance 1σ and pre-update innovations</div></header>
<main class="grid"><section class="card"><div class="head"><span>Trajectory overlay</span><span class="badge" id="mapstatus">Loading OpenStreetMap</span></div><div class="mapbox"><div class="map" id="osm"></div><div class="map" id="fallback"></div></div><div class="foot" id="mapfoot">OpenStreetMap © contributors · Click any route to synchronize the epoch selection.</div></section>
<section class="card"><div class="head"><span>Navigation estimates and covariance</span><span>GPS seconds</span></div><div class="controls">
<label><input type="checkbox" value="position"> Position</label><label><input type="checkbox" value="velocity"> Velocity</label><label><input type="checkbox" value="attitude"> Attitude</label><label><input type="checkbox" value="rms" checked> RMS 1σ</label><label><input type="checkbox" value="attitude_axes_rms" checked> Roll / pitch / heading 1σ</label><label><input type="checkbox" value="gnss_nis" checked> GNSS NIS</label><label><input type="checkbox" value="constraint_nis" checked> Constraint NIS</label>
</div><div id="chart" class="chart"></div><div class="stats" id="stats">Select a sample to inspect aligned GMIS_LC / GMIS_TC / GMIS_LTC values.</div></section></main>
<script>
const DATA=__DATA__, MODES=['LC','TC','LTC'], COLORS={LC:'#3867d6',TC:'#e39b29',LTC:'#20a37a'};
const N=Math.min(...MODES.map(m=>DATA[m].length));let selected=0, chartRows=12, map=null, markers={};
const fields={position:[['Latitude','lat','deg'],['Longitude','lon','deg'],['Height','h','m']],velocity:[['North velocity','vn','m/s'],['East velocity','ve','m/s'],['Down velocity','vd','m/s']],attitude:[['Roll','roll','deg'],['Pitch','pitch','deg'],['Heading','yaw','deg']],rms:[['Position 3D 1σ','posRms','m'],['Velocity 3D 1σ','velRms','m/s'],['Attitude 3D 1σ','attRms','deg']],attitude_axes_rms:[['Roll 1σ','rollRms','deg'],['Pitch 1σ','pitchRms','deg'],['Heading 1σ','headingRms','deg']],gnss_nis:[['GNSS update NIS','rtkNis','χ²; variable DOF']],constraint_nis:[['ZUPT NIS','zuptNis','χ²'],['NHC NIS','nhcNis','χ²'],['ZPC NIS','zpcNis','χ²'],['ZHC NIS','zhcNis','χ²']]};
const chart=document.getElementById('chart'), fallback=document.getElementById('fallback');
function mapTraces(){const traces=[];for(const m of MODES){const d=DATA[m].slice(0,N);traces.push({type:'scatter',mode:'lines',name:m,x:d.map(p=>p.lon),y:d.map(p=>p.lat),line:{color:COLORS[m],width:2},customdata:d.map((p,i)=>i),hovertemplate:`${m}<br>Lat %{y:.6f}°<br>Lon %{x:.6f}°<extra></extra>`});traces.push({type:'scatter',mode:'markers',name:m+' selected',showlegend:false,x:[d[0].lon],y:[d[0].lat],marker:{color:COLORS[m],size:9,line:{color:'#fff',width:2}},hoverinfo:'skip'});}return traces;}
function drawFallback(){const all=MODES.flatMap(m=>DATA[m].slice(0,N));const lat=all.map(p=>p.lat),lon=all.map(p=>p.lon);const traces=mapTraces();const layout={autosize:true,margin:{l:55,r:12,t:12,b:42},showlegend:true,legend:{orientation:'h',y:1.08},xaxis:{title:'Longitude (°)',range:[Math.min(...lon)-.0002,Math.max(...lon)+.0002]},yaxis:{title:'Latitude (°)',range:[Math.min(...lat)-.0002,Math.max(...lat)+.0002],scaleanchor:'x',scaleratio:1/Math.cos(lat[0]*Math.PI/180)}};return fallback._fullLayout?Plotly.react(fallback,traces,layout,{responsive:true,displaylogo:false}):Plotly.newPlot(fallback,traces,layout,{responsive:true,displaylogo:false});}
function drawChart(){const selectedGroups=[...document.querySelectorAll('.controls input:checked')].map(x=>x.value);chartRows=selectedGroups.reduce((n,g)=>n+fields[g].length,0);if(!chartRows){return Plotly.react(chart,[],{height:250,annotations:[{text:'Select a plot layer',showarrow:false}]});}const traces=[],layout={height:Math.max(500,chartRows*112+55),margin:{l:68,r:20,t:18,b:38},hovermode:'x unified',showlegend:true,legend:{orientation:'h',y:1.04},shapes:[]};let r=0;for(const group of selectedGroups){for(const [label,key,unit] of fields[group]){r++;const suffix=r===1?'':r,xaxis='x'+suffix,yaxis='y'+suffix,top=1-(r-1)/chartRows,bottom=1-r/chartRows;for(const m of MODES){const d=DATA[m].slice(0,N);traces.push({type:'scatter',mode:'lines',x:d.map(p=>p.t),y:d.map(p=>p[key]),name:m,legendgroup:m,showlegend:r===1,line:{color:COLORS[m],width:1.25},xaxis,yaxis,customdata:d.map((_,i)=>i),hovertemplate:`${m} %{x:.3f}<br>${label}: %{y:.5f} ${unit}<extra></extra>`});}layout['yaxis'+suffix]={domain:[bottom+.035,top-.02],title:{text:`${label} (${unit})`,font:{size:10}},automargin:true,gridcolor:'#edf0f5'};layout['xaxis'+suffix]={anchor:yaxis,showticklabels:r===chartRows,title:r===chartRows?'GPS seconds':undefined};}}
for(let k=1;k<=chartRows;k++){const s=k===1?'':k;layout.shapes.push({type:'line',x0:DATA.LC[selected].t,x1:DATA.LC[selected].t,y0:0,y1:1,xref:'x'+s,yref:'paper',line:{color:'#d14a61',dash:'dot',width:1}});}return chart._fullLayout?Plotly.react(chart,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true}):Plotly.newPlot(chart,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});}
function select(i){selected=Math.max(0,Math.min(N-1,i));const time=DATA.LC[selected].t;if(map){for(const m of MODES){const p=DATA[m][selected];markers[m].setLatLng([p.lat,p.lon]);}}if(fallback.data){for(let j=0;j<MODES.length;j++){const p=DATA[MODES[j]][selected];Plotly.restyle(fallback,{x:[[p.lon]],y:[[p.lat]]},[2*j+1]);}}if(chart._fullLayout){const shapes=[];for(let k=0;k<chartRows;k++){const s=k===0?'':'x'+(k+1);shapes.push({type:'line',x0:time,x1:time,y0:0,y1:1,xref:s||'x',yref:'paper',line:{color:'#d14a61',dash:'dot',width:1}});}Plotly.relayout(chart,{shapes});}const fmt=v=>Number.isFinite(v)?v.toFixed(4):'—';document.getElementById('stats').innerHTML=`Selected GPS ${time.toFixed(3)} s<br>`+MODES.map(m=>{const p=DATA[m][selected];return `<b>${m}</b>: ${p.lat.toFixed(7)}°, ${p.lon.toFixed(7)}°, h ${p.h.toFixed(3)} m · vN/E/D ${fmt(p.vn)} / ${fmt(p.ve)} / ${fmt(p.vd)} m/s · σp/v/att ${fmt(p.posRms)} m / ${fmt(p.velRms)} m/s / ${fmt(p.attRms)}° · σroll/pitch/heading ${fmt(p.rollRms)} / ${fmt(p.pitchRms)} / ${fmt(p.headingRms)}°`;}).join('<br>');}
document.querySelectorAll('.controls input').forEach(x=>x.addEventListener('change',()=>drawChart().then(bindChartEvents)));
function bindChartEvents(){if(chart._compareClickBound)return;chart.on('plotly_click',e=>{if(e.points?.length)select(e.points[0].pointIndex);});chart._compareClickBound=true;}
drawFallback().then(()=>{fallback.on('plotly_click',e=>{if(e.points?.length)select(e.points[0].pointIndex);});return drawChart();}).then(()=>{bindChartEvents();select(0);}).catch(err=>{console.error(err);document.getElementById('stats').textContent='Plot initialization failed: '+err.message;});
function nearest(lat,lon,rows){let best=0,dist=Infinity;for(let i=0;i<N;i++){const p=rows[i],d=(p.lat-lat)**2+((p.lon-lon)*Math.cos(lat*Math.PI/180))**2;if(d<dist){dist=d;best=i;}}return best;}
function initMap(attempt=0){if(!window.L){if(attempt<30){setTimeout(()=>initMap(attempt+1),200);return;}document.getElementById('mapstatus').textContent='Offline trajectory';document.getElementById('mapfoot').textContent='OpenStreetMap library unavailable; showing the three trajectories on an interactive coordinate plot.';document.getElementById('osm').style.display='none';fallback.style.display='block';return;}try{const all=MODES.flatMap(m=>DATA[m].slice(0,N)),bounds=L.latLngBounds(all.map(p=>[p.lat,p.lon]));map=L.map('osm',{preferCanvas:true,scrollWheelZoom:true});L.tileLayer('/tiles/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'}).addTo(map);for(const m of MODES){const rows=DATA[m].slice(0,N),route=L.polyline(rows.map(p=>[p.lat,p.lon]),{color:COLORS[m],weight:3,opacity:.9}).addTo(map);route.bindTooltip(m);route.on('click',e=>select(nearest(e.latlng.lat,e.latlng.lng,rows)));markers[m]=L.circleMarker([rows[0].lat,rows[0].lon],{radius:6,color:COLORS[m],fillColor:'#fff',fillOpacity:1,weight:3}).addTo(map);}map.fitBounds(bounds.pad(.06));map.on('click',e=>select(nearest(e.latlng.lat,e.latlng.lng,DATA.LC)));document.getElementById('mapstatus').textContent='OpenStreetMap';}catch(err){console.error(err);document.getElementById('mapstatus').textContent='Offline trajectory';document.getElementById('osm').style.display='none';fallback.style.display='block';}}
initMap();
</script></body></html>'''.replace("__PLOTLY_JS__", plotly_js).replace("__DATA__", data_json)


def page_html_gmis_comparison(datasets: dict[str, list[dict[str, float]]], local_url: str) -> str:
    """Reuse the established interactive comparison page for all three GMIS modes."""
    legacy = {"LC": datasets["GMIS_LC"], "TC": datasets["GMIS_TC"], "LTC": datasets["GMIS_LTC"]}
    html = page_html_comparison(legacy, local_url)
    old_json = json.dumps(legacy, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    new_json = json.dumps({"GMIS_LC": datasets["GMIS_LC"], "GMIS_TC": datasets["GMIS_TC"], "GMIS_LTC": datasets["GMIS_LTC"]},
                          separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    old_decl = f"const DATA={old_json}, MODES=['LC','TC','LTC'], COLORS={{LC:'#3867d6',TC:'#e39b29',LTC:'#20a37a'}};"
    new_decl = f"const DATA={new_json}, MODES=['GMIS_LC','GMIS_TC','GMIS_LTC'], COLORS={{GMIS_LC:'#3867d6',GMIS_TC:'#e39b29',GMIS_LTC:'#20a37a'}};"
    if old_decl not in html:
        raise RuntimeError("could not adapt the regression comparison template for GMIS")
    return (html.replace(old_decl, new_decl)
            .replace("DATA.LC", "DATA.GMIS_LC").replace("DATA.TC", "DATA.GMIS_TC").replace("DATA.LTC", "DATA.GMIS_LTC")
            .replace("LC / TC / LTC Navigation Regression Explorer", "GMIS_LC / GMIS_TC / GMIS_LTC Regression Comparison")
            .replace("LC / TC / LTC Regression Explorer", "GMIS_LC / GMIS_TC / GMIS_LTC Regression Explorer")
            .replace("Current 18-state replay · covariance 1σ and pre-update innovations",
                     "GMIS_LC / GMIS_TC / GMIS_LTC · covariance 1σ · synchronized GNSS epochs")
            .replace("aligned LC / TC / LTC values", "aligned GMIS_LC / GMIS_TC / GMIS_LTC values")
            .replace("RTK P/V update NIS", "GNSS update NIS"))


def page_html_gmis_ltc_constraint_comparison(
    datasets: dict[str, list[dict[str, float]]], local_url: str,
) -> str:
    """Show GMIS-LTC before and after ZUPT/NHC/ZPC/ZHC on the shared dashboard."""
    legacy = {"LC": datasets["Baseline"], "TC": datasets["Constrained"], "LTC": datasets["Baseline"]}
    html = page_html_comparison(legacy, local_url)
    old_json = json.dumps(legacy, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    baseline_name, constrained_name = "LTC baseline", "LTC + ZUPT/NHC + ZPC/ZHC"
    new_data = {baseline_name: datasets["Baseline"], constrained_name: datasets["Constrained"]}
    new_json = json.dumps(new_data, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    old_decl = f"const DATA={old_json}, MODES=['LC','TC','LTC'], COLORS={{LC:'#3867d6',TC:'#e39b29',LTC:'#20a37a'}};"
    new_decl = (f"const DATA={new_json}, MODES={[baseline_name, constrained_name]!r}, "
                f"COLORS={{{baseline_name!r}:'#3867d6',{constrained_name!r}:'#20a37a'}};")
    if old_decl not in html:
        raise RuntimeError("could not adapt the regression comparison template for GMIS-LTC constraints")
    return (html.replace(old_decl, new_decl)
            .replace("DATA.LC", f"DATA[{baseline_name!r}]")
            .replace("LC / TC / LTC Navigation Regression Explorer", "GMIS-LTC Constraints · Baseline vs Enabled")
            .replace("LC / TC / LTC Regression Explorer", "GMIS-LTC Constraint Comparison")
            .replace("Current 18-state replay · covariance 1σ and pre-update innovations",
                     "GMIS-LTC 42-state lag filter · RTK position + L1 TD-DDCP · constraint diagnostics")
            .replace("Select a sample to inspect aligned LC / TC / LTC values.",
                     "Select an epoch to compare GMIS-LTC before and after constraints."))


def page_html_constraint_comparison(datasets: dict[str, list[dict[str, float]]]) -> str:
    """Build a six-run dashboard for LC/TC/LTC before and after ZUPT/NHC."""
    data_json = json.dumps(datasets, separators=(",", ":"), ensure_ascii=False).replace("</", "<\\/")
    plotly_js = get_plotlyjs()
    return r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ZPC / ZHC Navigation Performance</title><link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin="">
<style>
:root{font-family:Inter,Segoe UI,Arial,sans-serif;color:#172033;background:#f3f6fb}*{box-sizing:border-box}body{margin:0;min-width:360px}.top{height:58px;background:#101a2d;color:#fff;padding:0 22px;display:flex;align-items:center;justify-content:space-between}.title{font-weight:650}.sub{font-size:11px;color:#cad4e5}.grid{height:calc(100vh - 58px);min-height:650px;padding:14px;display:grid;grid-template-columns:1fr 1.2fr;gap:14px}.card{background:#fff;border:1px solid #e2e8f0;border-radius:11px;overflow:hidden;display:flex;flex-direction:column;min-height:0}.head{padding:12px 16px;font-size:13px;font-weight:650;border-bottom:1px solid #edf0f5;display:flex;justify-content:space-between}.mapbox{position:relative;flex:1;min-height:370px}#osm,#fallback{position:absolute;inset:0}#fallback{display:none}.foot{padding:8px 12px;font-size:10px;color:#68758b;border-top:1px solid #edf0f5}.controls{padding:9px 14px;border-bottom:1px solid #edf0f5;display:flex;gap:7px;flex-wrap:wrap;align-items:center}.controls label{font-size:11px;border:1px solid #dce3ee;padding:5px 7px;border-radius:7px;white-space:nowrap}.controls input{accent-color:#3867d6}.group{font-size:10px;color:#758198;margin-right:2px}.chart{flex:1;min-height:350px;overflow:auto}.stats{padding:8px 14px;border-top:1px solid #edf0f5;font-size:10px;color:#53627a;line-height:1.5}.badge{font-size:10px;padding:4px 7px;background:#fff2d8;color:#845500;border-radius:12px}@media(max-width:900px){.grid{height:auto;display:flex;flex-direction:column}.card{height:65vh;min-height:500px}}
</style><script>__PLOTLY_JS__</script><script defer src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script></head>
<body><header class="top"><div class="title">Navigation Performance · ZPC / ZHC comparison</div><div class="sub">LC / TC / LTC · baseline → ZUPT/NHC → ZPC/ZHC · 1 Hz</div></header>
<main class="grid"><section class="card"><div class="head"><span>Trajectory overlay</span><span class="badge" id="mapstatus">Loading map</span></div><div class="mapbox"><div id="osm"></div><div id="fallback"></div></div><div class="foot" id="mapfoot">OSM © contributors · Select a route point to synchronize the epoch across all plots.</div></section>
<section class="card"><div class="head"><span>Solution performance</span><span>Formal covariance 1σ · not truth error</span></div><div class="controls"><span class="group">Modes</span><label><input type="checkbox" value="LC" checked> LC</label><label><input type="checkbox" value="TC" checked> TC</label><label><input type="checkbox" value="LTC" checked> LTC</label><span class="group">Runs</span><label><input type="checkbox" value="baseline" checked> Baseline</label><label><input type="checkbox" value="zupt_nhc" checked> ZUPT/NHC</label><label><input type="checkbox" value="zpc_zhc" checked> + ZPC/ZHC</label><span class="group">Plots</span><label><input type="checkbox" value="position" checked> Position</label><label><input type="checkbox" value="velocity" checked> Velocity</label><label><input type="checkbox" value="attitude" checked> Attitude</label><label><input type="checkbox" value="rms" checked> RMS 1σ</label><label><input type="checkbox" value="axes"> Roll/Pitch/Heading σ</label></div><div id="chart" class="chart"></div><div id="stats" class="stats">Click a trajectory or plot to inspect the synchronized epoch.</div></section></main>
<script>
const DATA=__DATA__,MODES=['LC','TC','LTC'],RUNS=['baseline','zupt_nhc','zpc_zhc'],COLORS={LC:'#3867d6',TC:'#e39b29',LTC:'#20a37a'},RUNNAME={baseline:'baseline',zupt_nhc:'ZUPT/NHC',zpc_zhc:'ZUPT/NHC + ZPC/ZHC'};
const KEYS=MODES.flatMap(m=>RUNS.map(r=>m+'_'+r));let n=Math.min(...KEYS.map(k=>DATA[k].length)),selected=0,map,markers={},routes={},chartRows=0;
const fields={position:[['Latitude','lat','deg'],['Longitude','lon','deg'],['Height','h','m']],velocity:[['North velocity','vn','m/s'],['East velocity','ve','m/s'],['Down velocity','vd','m/s']],attitude:[['Roll','roll','deg'],['Pitch','pitch','deg'],['Heading','yaw','deg']],rms:[['Position 3D 1σ','posRms','m'],['Velocity 3D 1σ','velRms','m/s'],['Attitude 3D 1σ','attRms','deg']],axes:[['Roll 1σ','rollRms','deg'],['Pitch 1σ','pitchRms','deg'],['Heading 1σ','headingRms','deg']]};
const chart=document.getElementById('chart'),fallback=document.getElementById('fallback');
const runOf=k=>k.slice(k.indexOf('_')+1),modeOf=k=>k.split('_')[0],labelOf=k=>`${modeOf(k)} · ${RUNNAME[runOf(k)]}`;
const active=()=>KEYS.filter(k=>document.querySelector(`.controls input[value="${modeOf(k)}"]`)?.checked&&document.querySelector(`.controls input[value="${runOf(k)}"]`)?.checked);
function style(k){const m=modeOf(k),r=runOf(k);return {color:COLORS[m],dash:r==='baseline'?'dash':(r==='zpc_zhc'?'dot':'solid')};}
function drawMapFallback(){const keys=active(),tr=[];for(const k of keys){const d=DATA[k].slice(0,n),s=style(k);tr.push({type:'scatter',mode:'lines',name:labelOf(k),x:d.map(p=>p.lon),y:d.map(p=>p.lat),line:{...s,width:2},hovertemplate:`${labelOf(k)}<br>%{y:.7f}, %{x:.7f}<extra></extra>`});}const all=keys.flatMap(k=>DATA[k].slice(0,n));if(!all.length)return;const lat=all.map(p=>p.lat),lon=all.map(p=>p.lon);return Plotly.react(fallback,tr,{autosize:true,margin:{l:55,r:10,t:10,b:38},showlegend:true,legend:{orientation:'h',y:1.08},xaxis:{title:'Longitude (°)',range:[Math.min(...lon)-.0002,Math.max(...lon)+.0002]},yaxis:{title:'Latitude (°)',range:[Math.min(...lat)-.0002,Math.max(...lat)+.0002],scaleanchor:'x',scaleratio:1/Math.cos(lat[0]*Math.PI/180)}},{responsive:true,displaylogo:false});}
function drawChart(){const selectedGroups=[...document.querySelectorAll('.controls input:checked')].map(x=>x.value).filter(x=>fields[x]);const keys=active();chartRows=selectedGroups.length*3;if(!chartRows)return Plotly.react(chart,[],{height:250,annotations:[{text:'Select a plot layer',showarrow:false}]});const traces=[],layout={width:chart.clientWidth||undefined,height:Math.max(480,chartRows*105+60),margin:{l:76,r:18,t:24,b:40},hovermode:'x unified',showlegend:true,legend:{orientation:'h',y:1.05},shapes:[]};let r=0;for(const g of selectedGroups)for(const [label,key,unit]of fields[g]){r++;const sx=r===1?'':'x'+r,sy=r===1?'':'y'+r,top=1-(r-1)/chartRows,bottom=1-r/chartRows;for(const k of keys){const d=DATA[k].slice(0,n),s=style(k);traces.push({type:'scatter',mode:'lines',x:d.map(p=>p.t),y:d.map(p=>p[key]),name:labelOf(k),legendgroup:k,showlegend:r===1,line:{...s,width:1.35},xaxis:sx||'x',yaxis:sy||'y',hovertemplate:`${labelOf(k)} %{x:.0f}<br>${label}: %{y:.5f} ${unit}<extra></extra>`});}layout['yaxis'+(r===1?'':r)]={domain:[bottom+.03,top-.015],title:{text:`${label} (${unit})`,font:{size:10}},automargin:true,gridcolor:'#edf0f5'};layout['xaxis'+(r===1?'':r)]={anchor:sy||'y',showticklabels:r===chartRows,title:r===chartRows?'GPS seconds':undefined};}return Plotly.react(chart,traces,layout,{responsive:true,displaylogo:false,scrollZoom:true});}
function nearest(lat,lon,rows){let b=0,d0=Infinity;for(let i=0;i<rows.length;i++){const p=rows[i],d=(p.lat-lat)**2+((p.lon-lon)*Math.cos(lat*Math.PI/180))**2;if(d<d0){b=i;d0=d;}}return b;}
function select(i){selected=Math.max(0,Math.min(n-1,i));const t=DATA.LC_zpc_zhc[selected].t;for(const k of KEYS){const p=DATA[k][selected];if(markers[k])markers[k].setLatLng([p.lat,p.lon]);}if(chart._fullLayout){const sh=[];for(let j=0;j<chartRows;j++){const ax=j?'x'+(j+1):'x';sh.push({type:'line',x0:t,x1:t,y0:0,y1:1,xref:ax,yref:'paper',line:{color:'#d14a61',dash:'dot',width:1}});}Plotly.relayout(chart,{shapes:sh});}const f=v=>Number.isFinite(v)?v.toFixed(4):'—';document.getElementById('stats').innerHTML=`Selected GPS ${t.toFixed(0)} s · ${selected+1}/${n} samples<br>`+MODES.map(m=>{const b=DATA[m+'_baseline'][selected],z=DATA[m+'_zupt_nhc'][selected],a=DATA[m+'_zpc_zhc'][selected];return `<b>${m}</b> σpos ${f(b.posRms)} → ${f(z.posRms)} → ${f(a.posRms)} m · σvel ${f(b.velRms)} → ${f(z.velRms)} → ${f(a.velRms)} m/s · σatt ${f(b.attRms)} → ${f(z.attRms)} → ${f(a.attRms)}° · anchored roll/pitch/heading ${f(a.roll)} / ${f(a.pitch)} / ${f(a.yaw)}°`;}).join('<br>')+'<br><span>Covariance 1σ is not truth error; RTK-relative diagnostics are not independent ground truth.</span>';}
document.querySelectorAll('.controls input').forEach(x=>x.addEventListener('change',()=>{drawChart();drawMapFallback();for(const k of KEYS){if(routes[k])routes[k].setStyle({opacity:active().includes(k)?.9:0});}}));
function mapInit(attempt=0){if(!window.L){if(attempt<25)return setTimeout(()=>mapInit(attempt+1),200);document.getElementById('mapstatus').textContent='Offline trajectory';document.getElementById('mapfoot').textContent='Leaflet unavailable; showing interactive coordinate plot.';document.getElementById('osm').style.display='none';fallback.style.display='block';drawMapFallback();return;}try{map=L.map('osm',{preferCanvas:true});L.tileLayer('/tiles/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);const all=KEYS.flatMap(k=>DATA[k].slice(0,n));map.fitBounds(L.latLngBounds(all.map(p=>[p.lat,p.lon])).pad(.06));for(const k of KEYS){const rows=DATA[k].slice(0,n),s=style(k),route=L.polyline(rows.map(p=>[p.lat,p.lon]),{color:s.color,dashArray:s.dash==='dash'?'7 5':(s.dash==='dot'?'2 5':null),weight:3,opacity:active().includes(k)?.9:0}).addTo(map);routes[k]=route;route.bindTooltip(labelOf(k));route.on('click',e=>select(nearest(e.latlng.lat,e.latlng.lng,rows)));markers[k]=L.circleMarker([rows[0].lat,rows[0].lon],{radius:5,color:s.color,fillColor:'#fff',fillOpacity:1,weight:2}).addTo(map);}document.getElementById('mapstatus').textContent='OpenStreetMap';}catch(e){console.error(e);document.getElementById('osm').style.display='none';fallback.style.display='block';document.getElementById('mapstatus').textContent='Offline trajectory';drawMapFallback();}}
drawChart().then(()=>{chart.on('plotly_click',e=>{if(e.points?.length)select(e.points[0].pointIndex);});select(0);});mapInit();
</script></body></html>'''.replace("__PLOTLY_JS__", plotly_js).replace("__DATA__", data_json)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="INS or RTK trajectory CSV")
    parser.add_argument("--compare", action="store_true", help="overlay the current LC, TC, and LTC navigation outputs")
    parser.add_argument("--gmis-compare", action="store_true", help="compare GMIS_LC, GMIS_LTC, and GMIS_TC solution CSV files")
    parser.add_argument("--gmis-lc-csv", type=Path, default=HERE.parents[1] / "output" / "gmis_lc_restored_rtk" / "gmis_lc_solution.csv")
    parser.add_argument("--gmis-ltc-csv", type=Path, default=HERE.parents[1] / "output" / "gmis_ltc" / "gmis_ltc_solution.csv")
    parser.add_argument("--gmis-tc-csv", type=Path, default=HERE.parents[1] / "output" / "gmis_tc" / "gmis_tc_solution.csv")
    parser.add_argument("--gmis-ltc-constraint-compare", action="store_true", help="compare GMIS-LTC baseline against ZUPT/NHC/ZPC/ZHC")
    parser.add_argument("--gmis-ltc-weight-compare", action="store_true", help="compare GMIS-LTC solutions with standard and increased RTK position weight")
    parser.add_argument("--gmis-ltc-baseline-csv", type=Path, default=HERE.parents[1] / "output" / "gmis_ltc" / "gmis_ltc_solution.csv")
    parser.add_argument("--gmis-ltc-constraints-csv", type=Path, default=HERE.parents[1] / "output" / "gmis_ltc_zupt_nhc" / "gmis_ltc_solution.csv")
    parser.add_argument("--zupt-nhc-compare", action="store_true", help="compare baseline and ZUPT/NHC LC, TC, and LTC solutions")
    parser.add_argument("--lc-baseline-csv", type=Path, default=HERE.parents[1] / "output" / "zupt_nhc_baseline" / "lc" / "ltc_solution.csv")
    parser.add_argument("--tc-baseline-csv", type=Path, default=HERE.parents[1] / "output" / "zupt_nhc_baseline" / "tc" / "ltc_solution.csv")
    parser.add_argument("--ltc-baseline-csv", type=Path, default=HERE.parents[1] / "output" / "zupt_nhc_baseline" / "ltc" / "ltc_solution.csv")
    parser.add_argument("--lc-constraints-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_lc_constraints" / "ltc_solution.csv")
    parser.add_argument("--tc-constraints-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_tc_constraints" / "ltc_solution.csv")
    parser.add_argument("--ltc-constraints-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_ltc_constraints" / "ltc_solution.csv")
    parser.add_argument("--lc-anchor-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_lc_anchor_constraints" / "ltc_solution.csv")
    parser.add_argument("--tc-anchor-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_tc_anchor_constraints" / "ltc_solution.csv")
    parser.add_argument("--ltc-anchor-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_ltc_anchor_constraints" / "ltc_solution.csv")
    parser.add_argument("--lc-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_lc" / "ltc_solution.csv")
    parser.add_argument("--tc-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_tc" / "ltc_solution.csv")
    parser.add_argument("--ltc-csv", type=Path, default=HERE.parents[1] / "ins_rtk_nav" / "build" / "mode_ltc" / "ltc_solution.csv")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--label", help="explicit run label, e.g. 'TMIS LC' (overrides filename-based labels)")
    parser.add_argument("--generate-only", action="store_true", help="write the standalone HTML and exit")
    args = parser.parse_args()

    google_maps_api_key = os.environ.get("GOOGLE_MAPS_API_KEY", "").strip()
    local_url = f"http://127.0.0.1:{args.port}/index.html"
    if args.gmis_ltc_constraint_compare or args.gmis_ltc_weight_compare:
        paths = {"Baseline": args.gmis_ltc_baseline_csv.resolve(), "Constrained": args.gmis_ltc_constraints_csv.resolve()}
        raw, totals = {}, {}
        for mode, path in paths.items():
            raw[mode], totals[mode] = load_trajectory(path)
        keyed = {mode: {round(row["t"], 6): row for row in rows} for mode, rows in raw.items()}
        epochs = sorted(set.intersection(*(set(table) for table in keyed.values())))
        if not epochs:
            raise ValueError("GMIS-LTC baseline and constrained runs have no common GPS epochs")
        datasets = {mode: [keyed[mode][t] for t in epochs] for mode in paths}
        html = page_html_gmis_ltc_constraint_comparison(datasets, local_url)
        if args.gmis_ltc_weight_compare:
            html = (html.replace("GMIS-LTC Constraints · Baseline vs Enabled", "GMIS-LTC · RTK position weight comparison")
                    .replace("GMIS-LTC Constraint Comparison", "GMIS-LTC RTK Weight Comparison")
                    .replace("LTC baseline", "LTC constraints · RTK σ 0.5/0.5/1.0 m")
                    .replace("LTC + ZUPT/NHC + ZPC/ZHC", "LTC constraints · RTK σ 0.1/0.1/0.1 m")
                    .replace("before and after constraints", "with standard and increased RTK measurement weight"))

        def pct(values: list[float], q: float) -> float:
            ordered = sorted(values); index=(len(ordered)-1)*q; low=math.floor(index); high=math.ceil(index)
            return ordered[low] if low==high else ordered[low]*(high-index)+ordered[high]*(index-low)
        def summarize(mode: str, key: str) -> tuple[float,float,float,float]:
            vals=[float(row[key]) for row in datasets[mode] if row.get(key) is not None and math.isfinite(float(row[key]))]
            return math.sqrt(sum(v*v for v in vals)/len(vals)),pct(vals,.5),pct(vals,.95),max(vals)
        report=["# GMIS-LTC ZUPT/NHC/ZPC/ZHC comparison","",f"Common epochs: {len(epochs):,}; GPS seconds {epochs[0]:.3f}–{epochs[-1]:.3f}.",
                "All RMS values below are filter covariance-derived formal 1σ statistics, not independent truth errors.","",
                "| Metric | Baseline RMS / median / P95 / max | Constraints RMS / median / P95 / max | Unit |","|---|---:|---:|---|"]
        for label,key,unit in (("Position 3D σ","posRms","m"),("Velocity 3D σ","velRms","m/s"),("Attitude 3D σ","attRms","deg"),("GNSS update NIS","rtkNis","χ²; variable DOF")):
            a,b=summarize("Baseline",key),summarize("Constrained",key)
            report.append(f"| {label} | " + " / ".join(f"{v:.6g}" for v in a) + " | " + " / ".join(f"{v:.6g}" for v in b) + f" | {unit} |")
        delta=[]
        for a,b in zip(datasets["Baseline"],datasets["Constrained"]):
            lat=math.radians((a["lat"]+b["lat"])*.5);dlon=math.radians(b["lon"]-a["lon"]);dlat=math.radians(b["lat"]-a["lat"])
            e2=6.69437999014e-3;den=math.sqrt(1-e2*math.sin(lat)**2);rn=6378137.0/den;rm=6378137.0*(1-e2)/(den**3)
            dn=dlat*(rm+(a["h"]+b["h"])*.5);de=dlon*(rn+(a["h"]+b["h"])*.5)*math.cos(lat);du=b["h"]-a["h"]
            delta.append(math.sqrt(dn*dn+de*de+du*du))
        report += ["","| Pairwise solution difference | Median / P95 / max | Unit |","|---|---:|---|",
                   f"| Baseline vs constrained position | {pct(delta,.5):.6g} / {pct(delta,.95):.6g} / {max(delta):.6g} | m |","",
                   "Constraint trigger/accept counts are available in the sibling `constraint_innovations.csv`; per-epoch NIS traces are selectable in the interactive dashboard.",""]
        report_path=paths["Constrained"].parent/"gmis_ltc_zupt_nhc_analysis.md"
        report_path.write_text("\n".join(report),encoding="utf-8")
        diagnostics=paths["Constrained"].parent/"constraint_innovations.csv"
        if diagnostics.exists():
            counts={}
            for row in csv.DictReader(diagnostics.open(encoding="utf-8",newline="")):
                item=counts.setdefault(row["measurement_type"],[0,0]);item[0]+=1;item[1]+=int(row["accepted"]=="1")
            with report_path.open("a",encoding="utf-8") as f:
                f.write("\n| Constraint | Triggered | Accepted |\n|---|---:|---:|\n")
                for name,(count,accepted) in sorted(counts.items()):f.write(f"| {name.upper()} | {count} | {accepted} |\n")
        print("GMIS-LTC baseline/constrained inputs: "+", ".join(f"{m}={totals[m]:,} rows ({paths[m]})" for m in paths),flush=True)
        print(f"Common epochs: {len(epochs):,}; report: {report_path}",flush=True)
    elif args.gmis_compare:
        paths = {"GMIS_LC": args.gmis_lc_csv.resolve(), "GMIS_TC": args.gmis_tc_csv.resolve(),
                 "GMIS_LTC": args.gmis_ltc_csv.resolve()}
        raw, totals = {}, {}
        for mode, path in paths.items():
            raw[mode], totals[mode] = load_trajectory(path)
        keyed = {mode: {round(row["t"], 6): row for row in rows} for mode, rows in raw.items()}
        epochs = sorted(set.intersection(*(set(table) for table in keyed.values())))
        if not epochs:
            raise ValueError("GMIS_LC, GMIS_TC, and GMIS_LTC have no common GPS epochs")
        datasets = {mode: [keyed[mode][t] for t in epochs] for mode in paths}
        html = page_html_gmis_comparison(datasets, local_url)

        def pct(values: list[float], q: float) -> float:
            ordered = sorted(values)
            index = (len(ordered) - 1) * q
            low, high = math.floor(index), math.ceil(index)
            return ordered[low] if low == high else ordered[low] * (high-index) + ordered[high] * (index-low)
        def metric(mode: str, key: str) -> list[float]:
            return [float(row[key]) for row in datasets[mode] if row.get(key) is not None and math.isfinite(float(row[key]))]
        report=["# GMIS_LC / GMIS_TC / GMIS_LTC comparison","",f"Common epochs: {len(epochs):,}; GPS seconds {epochs[0]:.3f}–{epochs[-1]:.3f}.",
                "This is a mode-to-mode comparison, not an error comparison against independent truth. Covariance RMS values are formal filter 1σ estimates.","",
                "| Metric | GMIS_LC median / P95 | GMIS_TC median / P95 | GMIS_LTC median / P95 | Unit |","|---|---:|---:|---:|---|"]
        for label,key,unit in (("Position 3D formal σ","posRms","m"),("Velocity 3D formal σ","velRms","m/s"),("Attitude 3D formal σ","attRms","deg"),("Heading formal σ","headingRms","deg"),("GNSS update NIS","rtkNis","χ²; variable DOF")):
            stats=[]
            for mode in ("GMIS_LC","GMIS_TC","GMIS_LTC"):
                vals=metric(mode,key);stats.append(f"{pct(vals,.5):.6g} / {pct(vals,.95):.6g}")
            report.append(f"| {label} | " + " | ".join(stats) + f" | {unit} |")
        report += ["","GNSS NIS has different observation dimensions across LC, TC, and LTC; read it together with the per-epoch row count/DOF and measurement model. A low formal covariance does not prove low actual error.",
                   "","| Pair | Position separation median / P95 / max (m) |","|---|---:|"]
        for a_mode,b_mode,label in (("GMIS_LC","GMIS_TC","LC vs TC"),("GMIS_LC","GMIS_LTC","LC vs LTC"),("GMIS_TC","GMIS_LTC","TC vs LTC")):
            sep=[]
            for a,b in zip(datasets[a_mode],datasets[b_mode]):
                lat=math.radians((a["lat"]+b["lat"])*.5);lon=math.radians(b["lon"]-a["lon"]);dlat=math.radians(b["lat"]-a["lat"])
                e2=6.69437999014e-3;den=math.sqrt(1-e2*math.sin(lat)**2);rn=6378137.0/den;rm=6378137.0*(1-e2)/(den**3)
                dn=dlat*(rm+(a["h"]+b["h"])*.5);de=lon*(rn+(a["h"]+b["h"])*.5)*math.cos(lat);du=b["h"]-a["h"]
                sep.append(math.sqrt(dn*dn+de*de+du*du))
            report.append(f"| {label} | {pct(sep,.5):.6g} / {pct(sep,.95):.6g} / {max(sep):.6g} |")
        report += ["","All separation values describe differences between solutions, not ground-truth error.",""]
        comparison_report=paths["GMIS_TC"].parent / "gmis_lc_tc_ltc_comparison.md"
        comparison_report.write_text("\n".join(report),encoding="utf-8")
        print("Comparison inputs: "+", ".join(f"{m}={totals[m]:,} rows ({paths[m]})" for m in paths),flush=True)
        print(f"Common epochs: {len(epochs):,}; report: {comparison_report}",flush=True)
    elif args.zupt_nhc_compare:
        paths = {
            "LC_baseline": args.lc_baseline_csv, "TC_baseline": args.tc_baseline_csv,
            "LTC_baseline": args.ltc_baseline_csv, "LC_zupt_nhc": args.lc_constraints_csv,
            "TC_zupt_nhc": args.tc_constraints_csv, "LTC_zupt_nhc": args.ltc_constraints_csv,
            "LC_zpc_zhc": args.lc_anchor_csv, "TC_zpc_zhc": args.tc_anchor_csv,
            "LTC_zpc_zhc": args.ltc_anchor_csv,
        }
        datasets, totals = {}, {}
        for key, path in paths.items():
            datasets[key], totals[key] = load_trajectory(path.resolve())
        common_times = set.intersection(*(set(p["t"] for p in rows) for rows in datasets.values()))
        if not common_times:
            raise ValueError("Baseline and constrained runs have no common GPS epochs")
        epochs = sorted(common_times)
        datasets = {key: [next(row for row in rows if row["t"] == t) for t in epochs]
                    for key, rows in datasets.items()}
        html = page_html_constraint_comparison(datasets)
        print("Performance inputs: " + ", ".join(f"{key}={totals[key]:,} rows ({path.resolve()})" for key, path in paths.items()), flush=True)
    elif args.compare:
        paths = {"LC": args.lc_csv.resolve(), "TC": args.tc_csv.resolve(), "LTC": args.ltc_csv.resolve()}
        datasets, totals = {}, {}
        for mode, path in paths.items():
            datasets[mode], totals[mode] = load_trajectory(path)
        if len({len(rows) for rows in datasets.values()}) != 1:
            raise ValueError("LC, TC, and LTC display samples must have the same count for epoch synchronization")
        html = page_html_comparison(datasets, local_url)
        print("Comparison inputs: " + ", ".join(f"{mode}={totals[mode]:,} rows ({paths[mode]})" for mode in paths), flush=True)
    else:
        rows, total_rows = load_trajectory(args.csv.resolve())
        html = page_html(rows, args.csv, total_rows, google_maps_api_key, local_url)
        if args.csv.stem.lower().startswith("ltc"):
            html = html.replace("INS run ·", "LTC solution ·")
            html = html.replace("INS solution ·", "LTC fused solution ·")
            html = html.replace("Unfused INS output", "RTK P/V fused LTC")
            html = html.replace("Raw standalone INS trajectory; no GNSS corrections are applied in this run.",
                                "LTC output with RTK position/velocity updates. RMS plots show formal covariance-derived 1σ, not error against truth.")
            html = html.replace('value="velocity"> Velocity', 'value="velocity" checked> Velocity')
            html = html.replace('value="attitude"> Attitude', 'value="attitude" checked> Attitude')
            html = html.replace('value="rms"> RMS (formal 1σ)', 'value="rms" checked> RMS (formal 1σ)')
        elif "gmis_tc" in args.csv.stem.lower():
            html = html.replace("INS run ·", "GMIS_TC run ·")
            html = html.replace("Unfused INS output", "GMIS_TC float solution")
            html = html.replace("RTK P/V update NIS", "GNSS DD update NIS")
            note = ("GMIS_TC float DD C1/P2 + L1/L2 carrier phase; RTK P/V is not used. "
                    "RMS curves are formal covariance-derived 1σ, not truth error. GNSS NIS is report-only; "
                    "the trajectory should be compared against GMIS_LC/LTC and independent truth before accuracy claims.")
            html = html.replace("Raw standalone INS trajectory; no GNSS corrections are applied in this run.", note)
            html = html.replace('value="velocity"> Velocity', 'value="velocity" checked> Velocity')
            html = html.replace('value="attitude"> Attitude', 'value="attitude" checked> Attitude')
            html = html.replace('value="rms"> RMS (formal 1σ)', 'value="rms" checked> RMS (formal 1σ)')
        elif "gmis_ltc" in args.csv.stem.lower() or "gmis_lc" in args.csv.stem.lower():
            mode = "GMIS_LTC" if "gmis_ltc" in args.csv.stem.lower() else "GMIS_LC"
            html = html.replace("INS run ·", f"{mode} run ·")
            html = html.replace("Unfused INS output", f"{mode} fused solution")
            note = ("GMIS_LTC 42-state joint lag solution: RTK antenna position + L1 TD-DDCP. "
                    "RTK velocity is disabled. RMS curves are formal covariance-derived 1σ estimates; GNSS NIS is for the combined update.") if mode == "GMIS_LTC" else ("GMIS_LC full-state solution. RMS curves are formal covariance-derived 1σ estimates, not truth errors. Innovation plots show pre-update NIS.")
            html = html.replace("Raw standalone INS trajectory; no GNSS corrections are applied in this run.", note)
            html = html.replace('value="position" checked', 'value="position" checked')
            html = html.replace('value="velocity"> Velocity', 'value="velocity" checked> Velocity')
            html = html.replace('value="attitude"> Attitude', 'value="attitude" checked> Attitude')
            html = html.replace('value="rms"> RMS (formal 1σ)', 'value="rms" checked> RMS (formal 1σ)')
        elif "status" in rows[0]:
            html = html.replace("INS run ·", "RTK run ·").replace("Roll / Pitch / Yaw", "RTK status / satellites")
            html = html.replace("Raw standalone INS trajectory; no GNSS corrections are applied in this run.",
                                "RTK solution · position is ECEF converted to geodetic; velocity is ECEF rotated to NED. No attitude is available in this solution file.")
            html = html.replace("Unfused INS output", "RTK solution")
            html = html.replace('<label class="check"><input type="checkbox" value="attitude"> Attitude</label>', "")
        if args.label:
            label = args.label.strip()
            html = html.replace("LTC fused solution ·", f"{label} ·")
            html = html.replace("LTC solution ·", f"{label} ·")
            html = html.replace("INS run ·", f"{label} ·")
            html = html.replace("RTK P/V fused LTC", f"{label} with RTK position/velocity updates")
            html = html.replace("LTC output with RTK position/velocity updates.",
                                f"{label} output with RTK position/velocity updates.")
            html = html.replace("Unfused INS output", f"{label} output")
            html = html.replace("Raw standalone INS trajectory; no GNSS corrections are applied in this run.",
                                f"{label} output with GNSS corrections; RMS values are formal covariance-derived 1σ, not truth errors.")
    HTML_PATH.write_text(html, encoding="utf-8")
    if args.gmis_compare:
        print(f"Generated {HTML_PATH} using {len(epochs):,} aligned GMIS epochs", flush=True)
    elif args.gmis_ltc_constraint_compare or args.gmis_ltc_weight_compare:
        print(f"Generated {HTML_PATH} using {len(epochs):,} aligned GMIS-LTC epochs", flush=True)
    elif args.zupt_nhc_compare:
        print(f"Generated {HTML_PATH} using {len(epochs):,} common GPS epochs across nine solutions", flush=True)
    elif args.compare:
        print(f"Generated {HTML_PATH} using {len(datasets['LC']):,} aligned display points per mode", flush=True)
    else:
        print(f"Generated {HTML_PATH} using {len(rows):,} display points from {total_rows:,} usable samples", flush=True)
    if args.generate_only:
        return

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *handler_args, **handler_kwargs):
            super().__init__(*handler_args, directory=str(HERE), **handler_kwargs)

        def do_GET(self):
            request_path = self.path.split("?", 1)[0]
            if request_path == "/diagnostics/osm":
                with OSM_TILE_DIAGNOSTIC_LOCK:
                    payload = json.dumps(OSM_TILE_DIAGNOSTICS).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
                return
            match = re.fullmatch(r"/tiles/(\d+)/(\d+)/(\d+)\.png", request_path)
            if not match:
                return super().do_GET()
            z, x, y = map(int, match.groups())
            limit = 1 << z
            if z > 19 or x >= limit or y >= limit:
                self.send_error(400, "Invalid tile coordinates")
                return
            tile_name = f"{z}/{x}/{y}"
            with OSM_TILE_DIAGNOSTIC_LOCK:
                OSM_TILE_DIAGNOSTICS["requests"] += 1
                OSM_TILE_DIAGNOSTICS["active"] += 1
                OSM_TILE_DIAGNOSTICS["last_tile"] = tile_name
            status, body, content_type, headers = fetch_osm_tile(z, x, y, self.headers.get("Referer", ""))
            with OSM_TILE_DIAGNOSTIC_LOCK:
                OSM_TILE_DIAGNOSTICS["active"] -= 1
                OSM_TILE_DIAGNOSTICS["last_status"] = status
                OSM_TILE_DIAGNOSTICS["detail"] = "tile received" if status == 200 else body.decode("utf-8", errors="replace")[:500]
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            for name, value in headers.items():
                self.send_header(name, value)
            if status != 200:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *log_args):
            print(fmt % log_args, flush=True)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}/index.html"
    print(f"Serving interactive dashboard at {url}", flush=True)
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
