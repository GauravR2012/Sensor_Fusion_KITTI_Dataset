import glob, os
import numpy as np
from pyproj import Transformer
from . import config

def load_sync_lists(max_frames=None):
    left = sorted(glob.glob(f"{config.DATA_PATH}/image_02/data/*.png"))
    right = sorted(glob.glob(f"{config.DATA_PATH}/image_03/data/*.png"))
    bins = sorted(glob.glob(f"{config.DATA_PATH}/velodyne_points/data/*.bin"))
    oxts = sorted(glob.glob(f"{config.DATA_PATH}/oxts/data/*.txt"))
    n = min(len(left), len(right), len(bins), len(oxts))
    if max_frames is not None:
        n = min(n, max_frames)
    return left[:n], right[:n], bins[:n], oxts[:n]

def load_oxts_line(path):
    with open(path, encoding="utf-8") as f:
        return np.array(f.readline().strip().split(), dtype=float)

def load_all_oxts(paths):
    return np.vstack([load_oxts_line(p) for p in paths])

def gps_to_local_xy(lats, lons):
    lon0, lat0 = lons[0], lats[0]
    zone = int((lon0 + 180.0) / 6.0) + 1
    epsg = (32600 if lat0 >= 0 else 32700) + zone
    t = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    east, north = t.transform(lons, lats)
    east = np.asarray(east) - east[0]
    north = np.asarray(north) - north[0]
    return np.column_stack([east, north])

def gps_to_local_xyz(lats, lons, alts):
    lon0, lat0 = lons[0], lats[0]
    zone = int((lon0 + 180.0) / 6.0) + 1
    epsg = (32600 if lat0 >= 0 else 32700) + zone
    t = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    east, north = t.transform(lons, lats)
    east = np.asarray(east) - east[0]
    north = np.asarray(north) - north[0]
    up = np.asarray(alts) - alts[0]
    return np.column_stack([east, north, up])

def parse_calib_file(path):
    data = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            try:
                data[k.strip()] = np.array([float(x) for x in v.split()])
            except ValueError:
                continue
    return data

def read_raw_calib(calib_root=None):
    root = calib_root or config.CALIB_ROOT
    cam = parse_calib_file(os.path.join(root, "calib_cam_to_cam.txt"))
    velo = parse_calib_file(os.path.join(root, "calib_velo_to_cam.txt"))
    p2 = cam["P_rect_02"].reshape(3, 4)
    p3 = cam["P_rect_03"].reshape(3, 4)
    k = p2[:3, :3]
    r_rect = np.eye(4); r_rect[:3, :3] = cam["R_rect_02"].reshape(3, 3)
    t_cam0_cam2 = np.eye(4)
    t_cam0_cam2[:3, :3] = cam["R_02"].reshape(3, 3)
    t_cam0_cam2[:3, 3] = cam["T_02"].reshape(3, 1).ravel()
    t_velo_cam0 = np.eye(4)
    t_velo_cam0[:3, :3] = velo["R"].reshape(3, 3)
    t_velo_cam0[:3, 3] = velo["T"].reshape(3, 1).ravel()
    tr = r_rect @ t_cam0_cam2 @ t_velo_cam0
    return k, p2, p3, tr, np.linalg.inv(tr)

def load_velodyne(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]

def oxts_fields(oxts):
    return {
        "lat": oxts[:, 0], "lon": oxts[:, 1], "alt": oxts[:, 2], "yaw": oxts[:, 5],
        "vn": oxts[:, 6], "ve": oxts[:, 7], "vf": oxts[:, 8],
        "af": oxts[:, 14], "wu": oxts[:, 23],
    }
