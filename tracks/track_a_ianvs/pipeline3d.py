"""
pipeline3d.py

NOTE:
This file was generated from the uploaded standalone 3D script.
It is intended as a starting point for refactoring into the modular
src/ layout. Replace notebook/global constants with imports from
config.py, loaders.py, icp.py, alignment.py and ekf3d.py as desired.
"""

import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from pyproj import Transformer
import open3d as o3d
import warnings
warnings.filterwarnings("ignore")

# =====================================================
# PATHS (single drive — change drive folder as needed)
# =====================================================
ROOT = "/kaggle/input/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset/data/2011_09_26/2011_09_26_drive_0001_sync"
OXTS_DIR = os.path.join(ROOT, "oxts/data")
VELO_DIR = os.path.join(ROOT, "velodyne_points/data")
TS_PATH = os.path.join(ROOT, "oxts/timestamps.txt")

# =====================================================
# TUNABLES
# =====================================================
ICP_FITNESS_THRESHOLD = 0.30
ICP_RMSE_THRESHOLD    = 0.50
ICP_MAX_DIST          = 2.5
VOXEL_SIZE            = 0.40
RANGE_LIMIT           = 60.0
GROUND_DIST_THRESH    = 0.20

Q_POS = 0.05
Q_VEL = 0.8
R_POS = 1.5
R_VEL = 0.25
LIDAR_NOISE_BASE = 3.0
LIDAR_NOISE_MIN  = 1.0

# =====================================================
# OXTS + 3D POSE HELPERS
# =====================================================
def load_oxts(path):
    d = np.loadtxt(path)
    return {
        "lat": d[0], "lon": d[1], "alt": d[2],
        "roll": d[3], "pitch": d[4], "yaw": d[5],
        "vn": d[6], "ve": d[7], "vf": d[8],
        "vu": d[10] if d.size > 10 else 0.0,
        "af": d[14], "al": d[15] if d.size > 15 else 0.0,
        "au": d[16] if d.size > 16 else 0.0,
        "wf": d[17] if d.size > 17 else 0.0,
        "wl": d[18] if d.size > 18 else 0.0,
        "wu": d[19] if d.size > 19 else (d[23] if d.size > 23 else 0.0),
    }

def load_timestamps(ts_path):
    if not os.path.exists(ts_path):
        return None
    from datetime import datetime
    times = []
    with open(ts_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                times.append(datetime.strptime(line, "%Y-%m-%d %H:%M:%S.%f").timestamp())
            except ValueError:
                try:
                    times.append(float(line))
                except ValueError:
                    return None
    if not times:
        return None
    t0 = times[0]
    return np.array([t - t0 for t in times])

def oxts_to_xyz(records):
    """Local ENU-ish frame: UTM east/north + relative altitude."""
    tr = Transformer.from_crs("EPSG:4326", "EPSG:32632", always_xy=True)
    x0, y0 = tr.transform(records[0]["lon"], records[0]["lat"])
    z0 = records[0]["alt"]
    xyz, rpy = [], []
    for r in records:
        x, y = tr.transform(r["lon"], r["lat"])
        xyz.append([x - x0, y - y0, r["alt"] - z0])
        rpy.append([r["roll"], r["pitch"], r["yaw"]])
    return np.array(xyz), np.array(rpy)

def euler_zxy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Rz @ Ry @ Rx

def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def T_inv(T):
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def umeyama_3d(src, tgt):
    mu_s, mu_t = src.mean(0), tgt.mean(0)
    src_c, tgt_c = src - mu_s, tgt - mu_t
    cov = (tgt_c.T @ src_c) / len(src)
    U, _, Vt = np.linalg.svd(cov)
    S = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    R = U @ S @ Vt
    t = mu_t - R @ mu_s
    return R, t

def apply_sim3(R, t, pts):
    return (pts @ R.T) + t

def rot_angle_deg(R):
    tr = np.trace(R)
    c = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(c))

# =====================================================
# LIDAR + ICP
# =====================================================
def load_velodyne(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]

def preprocess(points):
    r = np.linalg.norm(points[:, :2], axis=1)
    points = points[r < RANGE_LIMIT]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    try:
        _, inl = pcd.segment_plane(GROUND_DIST_THRESH, 3, 100)
        pcd = pcd.select_by_index(inl, invert=True)
    except Exception:
        pass
    return pcd.voxel_down_sample(VOXEL_SIZE)

def run_gicp(src_pts, tgt_pts, init_T):
    src, tgt = preprocess(src_pts), preprocess(tgt_pts)
    src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))
    tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))
    res = o3d.pipelines.registration.registration_generalized_icp(
        src, tgt, ICP_MAX_DIST, init_T,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    )
    ok = res.fitness >= ICP_FITNESS_THRESHOLD and res.inlier_rmse <= ICP_RMSE_THRESHOLD
    T = res.transformation if ok else init_T
    return T, res.fitness, res.inlier_rmse, ok

def oxts_motion_prior(rec_prev, rec_curr, dt):
    """
    3D motion prior from OXTS (nav frame velocities + small-angle gyro).
    UTM: x=east, y=north → v ≈ [ve, vn, vu].
    """
    v = np.array([rec_prev["ve"], rec_prev["vn"], rec_prev.get("vu", 0.0)])
    t = v * dt
    wx, wy, wz = rec_prev["wf"], rec_prev["wl"], rec_prev["wu"]
    wx, wy, wz = wx * dt, wy * dt, wz * dt
    R = euler_zxy_to_R(wx, wy, wz)  # small-angle approx
    return make_T(R, t)

# =====================================================
# 3D EKF — state [x,y,z,vx,vy,vz]
# =====================================================
class EKF3D:
    def __init__(self):
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 0.1
        self.Q = np.diag([Q_POS]*3 + [Q_VEL]*3)
        self.R_pos = np.diag([R_POS]*3)
        self.R_vel = np.diag([R_VEL]*3)

    def predict(self, dt, acc_nav):
        p = self.x[0:3, 0]
        v = self.x[3:6, 0]
        self.x[0:3, 0] = p + v * dt
        self.x[3:6, 0] = v + acc_nav * dt
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def _update(self, z, H, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        innov = z - H @ self.x
        self.x = self.x + K @ innov
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def update_pos(self, p_meas, R=None):
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        self._update(np.asarray(p_meas).reshape(3, 1), H, R if R is not None else self.R_pos)

    def update_vel(self, v_meas, R=None):
        H = np.zeros((3, 6))
        H[0, 3] = H[1, 4] = H[2, 5] = 1.0
        self._update(np.asarray(v_meas).reshape(3, 1), H, R if R is not None else self.R_vel)

# =====================================================
# LOAD
# =====================================================
print("Loading OXTS + Velodyne...")
oxts_files = sorted(glob.glob(os.path.join(OXTS_DIR, "*.txt")))
velo_files = sorted(glob.glob(os.path.join(VELO_DIR, "*.bin")))
N = min(len(oxts_files), len(velo_files))
records = [load_oxts(f) for f in oxts_files[:N]]
gt_xyz, gt_rpy = oxts_to_xyz(records)
timestamps = load_timestamps(TS_PATH)
print(f"Frames: {N}")

def clip_dt(i):
    if timestamps is not None and i > 0 and i < len(timestamps):
        return float(np.clip(timestamps[i] - timestamps[i - 1], 0.05, 0.5))
    return 0.1

# =====================================================
# 3D LIO-style ODOMETRY (GICP chain, 6-DOF)
# =====================================================
T_w = np.eye(4)
lidar_xyz = [np.zeros(3)]
lidar_Rs = [np.eye(3)]
fitness_scores = []
rejected = 0

print("Running 3D GICP odometry...")
for i in tqdm(range(1, N)):
    dt = clip_dt(i)
    src = load_velodyne(velo_files[i - 1])
    tgt = load_velodyne(velo_files[i])
    init_T = oxts_motion_prior(records[i - 1], records[i], dt)
    T_rel, fit, rmse, ok = run_gicp(src, tgt, init_T)
    if not ok:
        rejected += 1
    fitness_scores.append(fit)
    T_w = T_w @ T_rel
    lidar_xyz.append(T_w[:3, 3].copy())
    lidar_Rs.append(T_w[:3, :3].copy())

lidar_xyz = np.array(lidar_xyz)

# Align LiDAR trajectory to GT (3D Umeyama — positions only)
R_u, t_u = umeyama_3d(lidar_xyz, gt_xyz)
lidar_xyz_al = apply_sim3(R_u, t_u, lidar_xyz)

# Optional: align lidar orientation for evaluation (global R_u on body rotations)
lidar_Rs_al = [R_u @ R for R in lidar_Rs]

# =====================================================
# 3D EKF FUSION
# =====================================================
print("Running 3D EKF...")
ekf = EKF3D()
ekf.x[0:3, 0] = gt_xyz[0]
ekf.x[3:6, 0] = [records[0]["ve"], records[0]["vn"], records[0].get("vu", 0.0)]

fused_xyz = []
for i in range(N):
    dt = clip_dt(i)
    # OXTS accel in nav frame (approx): body forward accel projected — simplified use al,au,af in nav
    acc = np.array([records[i].get("al", 0.0), records[i].get("au", 0.0), records[i].get("af", 0.0)])
    # Better nav accel: finite diff of velocity (stable at 10 Hz)
    if i > 0:
        dt_v = max(dt, 1e-3)
        v_prev = np.array([records[i-1]["ve"], records[i-1]["vn"], records[i-1].get("vu", 0.0)])
        v_curr = np.array([records[i]["ve"], records[i]["vn"], records[i].get("vu", 0.0)])
        acc = (v_curr - v_prev) / dt_v

    ekf.predict(dt, acc)
    ekf.update_pos(gt_xyz[i])  # GPS/OXTS position (for fusion demo; use sparingly for pure odom)

    fs = fitness_scores[i - 1] if i > 0 else 1.0
    noise = max(LIDAR_NOISE_BASE / (fs + 1e-6), LIDAR_NOISE_MIN)
    ekf.update_pos(lidar_xyz_al[i], R=np.diag([noise]*3))

    v_meas = np.array([records[i]["ve"], records[i]["vn"], records[i].get("vu", 0.0)])
    ekf.update_vel(v_meas)
    fused_xyz.append(ekf.x[0:3, 0].copy())

fused_xyz = np.array(fused_xyz)

# =====================================================
# METRICS (3D)
# =====================================================
err_3d = np.linalg.norm(fused_xyz - gt_xyz, axis=1)
err_lidar_3d = np.linalg.norm(lidar_xyz_al - gt_xyz, axis=1)
ate_3d = np.sqrt(np.mean(err_3d**2))
ate_xy = np.sqrt(np.mean(np.sum((fused_xyz[:, :2] - gt_xyz[:, :2])**2, axis=1)))
ate_z  = np.sqrt(np.mean((fused_xyz[:, 2] - gt_xyz[:, 2])**2))
final_3d = err_3d[-1]

def path_len(xyz):
    return np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))

len_gt = path_len(gt_xyz)
len_fused = path_len(fused_xyz)
len_lidar = path_len(lidar_xyz_al)

print(f"\nRejected ICP frames: {rejected} / {N-1}")
print(f"ATE 3D      : {ate_3d:.3f} m")
print(f"ATE XY      : {ate_xy:.3f} m")
print(f"ATE Z       : {ate_z:.3f} m")
print(f"Final 3D err: {final_3d:.3f} m")
print(f"Path length GT   : {len_gt:.2f} m")
print(f"Path length Fused: {len_fused:.2f} m  ({abs(len_fused-len_gt)/len_gt*100:.1f}% err)")
print(f"Path length LiDAR: {len_lidar:.2f} m")

# Rotation error (LiDAR odom vs OXTS), after global align
rot_errs = []
for i in range(N):
    R_gt = euler_zxy_to_R(*gt_rpy[i])
    R_err = R_gt.T @ lidar_Rs_al[i]
    rot_errs.append(rot_angle_deg(R_err))
rot_errs = np.array(rot_errs)
print(f"Mean rot err (LiDAR vs OXTS): {np.mean(rot_errs):.2f} deg")

np.savetxt("fused_state_3d.csv", np.hstack([fused_xyz, ekf.x[3:6, 0][None].repeat(N,0) if False else np.zeros((N,3))]),
           delimiter=",", header="x,y,z", comments="")
# Save properly:
np.savetxt("fused_state_3d.csv", fused_xyz, delimiter=",", header="x,y,z", comments="")

# =====================================================
# PLOTS
# =====================================================
fig = plt.figure(figsize=(16, 6))

ax1 = fig.add_subplot(131, projection="3d")
ax1.plot(gt_xyz[:, 0], gt_xyz[:, 1], gt_xyz[:, 2], "b-", lw=2, label="GT (OXTS)")
ax1.plot(lidar_xyz_al[:, 0], lidar_xyz_al[:, 1], lidar_xyz_al[:, 2], "g--", lw=1.5, label="LiDAR 3D")
ax1.plot(fused_xyz[:, 0], fused_xyz[:, 1], fused_xyz[:, 2], "r-", lw=2, label="EKF 3D")
ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
ax1.legend(); ax1.set_title("3D Trajectories")

ax2 = fig.add_subplot(132)
ax2.plot(err_3d, "r-", label=f"EKF mean={err_3d.mean():.2f}m")
ax2.plot(err_lidar_3d, "g--", alpha=0.7, label=f"LiDAR mean={err_lidar_3d.mean():.2f}m")
ax2.set_xlabel("Frame"); ax2.set_ylabel("3D error (m)"); ax2.legend(); ax2.grid(True)
ax2.set_title("Position error vs OXTS")

ax3 = fig.add_subplot(133)
ax3.plot(fitness_scores, color="steelblue")
ax3.axhline(ICP_FITNESS_THRESHOLD, color="red", ls="--")
ax3.set_xlabel("Frame"); ax3.set_ylabel("GICP fitness"); ax3.grid(True)
ax3.set_title("ICP quality")

plt.tight_layout()
plt.savefig("sensor_fusion_3d_result.png", dpi=150, bbox_inches="tight")
plt.show()

# Z profile
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(gt_xyz[:, 2], "b-", label="GT z")
ax.plot(fused_xyz[:, 2], "r-", label="Fused z")
ax.plot(lidar_xyz_al[:, 2], "g--", alpha=0.7, label="LiDAR z")
ax.set_xlabel("Frame"); ax.set_ylabel("Height (m)"); ax.legend(); ax.grid(True)
ax.set_title("Vertical channel")
plt.tight_layout()
plt.savefig("height_profile_3d.png", dpi=150, bbox_inches="tight")
plt.show()

print("\nSaved fused_state_3d.csv, sensor_fusion_3d_result.png, height_profile_3d.png")
print("Done.")