import glob, os
import numpy as np
from . import config

def find_odometry_root():
    if os.path.isdir(config.DATA_ROOT):
        if os.path.isdir(os.path.join(config.DATA_ROOT, "poses")):
            return config.DATA_ROOT
    for root, dirs, _ in os.walk("/kaggle/input"):
        if "poses" in dirs and "sequences" in dirs:
            return root
    raise FileNotFoundError("KITTI Odometry root not found. Set ODOMETRY_DATA_ROOT.")

def sequence_paths(data_root, seq):
    seq_dir = os.path.join(data_root, "sequences", seq)
    return {
        "img_l": os.path.join(seq_dir, "image_2"),
        "img_r": os.path.join(seq_dir, "image_3"),
        "calib": os.path.join(seq_dir, "calib.txt"),
        "poses": os.path.join(data_root, "poses", f"{seq}.txt"),
    }

def load_poses(path):
    poses = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            v = np.fromstring(line, sep=" ")
            if v.size != 12:
                continue
            t = np.eye(4)
            t[:3, :4] = v.reshape(3, 4)
            poses.append(t)
    return poses

def read_calib(path):
    data = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            data[k.strip()] = np.array([float(x) for x in v.split()])
    p2 = data["P2"].reshape(3, 4)
    p3 = data["P3"].reshape(3, 4)
    k = p2[:3, :3]
    baseline = -p2[0, 3] / p2[0, 0]
    return k, p2, p3, baseline

def load_frame_lists(paths, max_frames):
    img_l = sorted(glob.glob(os.path.join(paths["img_l"], "*.png")))
    img_r = sorted(glob.glob(os.path.join(paths["img_r"], "*.png")))
    gt_poses = load_poses(paths["poses"])
    n = min(len(img_l), len(img_r), len(gt_poses))
    if max_frames is not None:
        n = min(n, max_frames)
    return img_l[:n], img_r[:n], gt_poses[:n]
