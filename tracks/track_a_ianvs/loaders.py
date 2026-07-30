import glob
import os
from datetime import datetime
import numpy as np
from pyproj import Transformer
def load_oxts(path):
    d = np.loadtxt(path)
    return {
        "lat": d[0],
        "lon": d[1],
        "alt": d[2],
        "roll": d[3],
        "pitch": d[4],
        "yaw": d[5],
        "vn": d[6],
        "ve": d[7],
        "vf": d[8],
        "af": d[14],
        "wu": d[23],
    }
def load_timestamps(ts_path):
    if not os.path.exists(ts_path):
        return None
    with open(ts_path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    times = []
    for ln in lines:
        try:
            dt = datetime.strptime(ln, "%Y-%m-%d %H:%M:%S.%f")
            times.append(dt.timestamp())
        except ValueError:
            try:
                times.append(float(ln))
            except ValueError:
                return None
    if not times:
        return None
    t0 = times[0]
    return np.array([t - t0 for t in times])
def gps_to_xy(records):
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    x0, y0 = transformer.transform(records[0]["lon"], records[0]["lat"])
    xy = []
    for r in records:
        x, y = transformer.transform(r["lon"], r["lat"])
        xy.append([x - x0, y - y0])
    return np.array(xy)
def load_velodyne(path):
    pts = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    return pts[:, :3]
def list_drives(data_root):
    return sorted(
        d
        for d in os.listdir(data_root)
        if os.path.isdir(os.path.join(data_root, d)) and "drive" in d
    )
def load_drive_files(drive_path):
    oxts_dir = os.path.join(drive_path, "oxts/data")
    velo_dir = os.path.join(drive_path, "velodyne_points/data")
    oxts_files = sorted(glob.glob(os.path.join(oxts_dir, "*.txt")))
    velo_files = sorted(glob.glob(os.path.join(velo_dir, "*.bin")))
    ts_path = os.path.join(drive_path, "oxts/timestamps.txt")
    return oxts_files, velo_files, ts_path