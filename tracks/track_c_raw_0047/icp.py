import numpy as np
from scipy.spatial import cKDTree
from . import config
from .loaders import load_velodyne

def voxel_downsample(pts, voxel):
    if len(pts) == 0:
        return pts
    q = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[idx]

def preprocess_lidar(points):
    r = np.linalg.norm(points[:, :2], axis=1)
    points = points[r < config.RANGE_LIMIT]
    points = points[points[:, 2] > -1.5]
    return voxel_downsample(points, config.VOXEL_SIZE)

def best_fit_transform(a, b):
    mu_a, mu_b = a.mean(0), b.mean(0)
    aa, bb = a - mu_a, b - mu_b
    u, _, vt = np.linalg.svd(bb.T @ aa)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = mu_a - r @ mu_b
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = t
    return out

def run_icp(src_pts, tgt_pts, init_t=np.eye(4)):
    src = preprocess_lidar(src_pts)
    tgt = preprocess_lidar(tgt_pts)
    if len(src) < 100 or len(tgt) < 100:
        return np.eye(4), 0.0, False
    t = init_t.copy()
    tree = cKDTree(tgt)
    fitness = 0.0
    for _ in range(config.ICP_ITERS):
        src_tf = (t[:3, :3] @ src.T + t[:3, 3:4]).T
        dist, j = tree.query(src_tf, k=1, distance_upper_bound=config.ICP_MAX_DIST)
        mask = np.isfinite(dist) & (j < len(tgt))
        if mask.sum() < config.ICP_MIN_PAIRS:
            break
        t = best_fit_transform(tgt[j[mask]], src_tf[mask]) @ t
        fitness = float(mask.sum()) / len(src)
    ok = fitness >= config.ICP_MIN_FITNESS
    return t, fitness, ok

def lidar_rel_to_camera(t_lidar_velo, tr, tr_inv):
    return tr @ t_lidar_velo @ tr_inv

def precompute_lidar_rel(bin_paths, tr, tr_inv):
    rel, fitness_hist = [], []
    for i in range(1, len(bin_paths)):
        t_rel, fit, ok = run_icp(load_velodyne(bin_paths[i-1]), load_velodyne(bin_paths[i]), np.eye(4))
        if not ok:
            t_rel, fit = np.eye(4), 0.0
        rel.append(lidar_rel_to_camera(t_rel, tr, tr_inv))
        fitness_hist.append(fit)
    return rel, fitness_hist
