import os
import matplotlib.pyplot as plt
import numpy as np
from shared.metrics import ate_2d, ate_3d, path_length
from . import config
from .ekf import EKF2D
from .icp import precompute_lidar_rel
from .loaders import gps_to_local_xy, gps_to_local_xyz, load_all_oxts, load_sync_lists, oxts_fields, read_raw_calib
from .odom_pipeline import run_imu_lidar_vo_odom
from .stereo_vo import precompute_vo_rel

def run_aided(max_frames=None):
    left, right, bins, oxts_paths = load_sync_lists(max_frames)
    oxts = load_all_oxts(oxts_paths)
    fields = oxts_fields(oxts)
    gt_xy = gps_to_local_xy(fields["lat"], fields["lon"])
    gt_xyz = gps_to_local_xyz(fields["lat"], fields["lon"], fields["alt"])
    ekf = EKF2D()
    ekf.set_initial(gt_xy[0, 0], gt_xy[0, 1], fields["yaw"][0], fields["vf"][0])
    fused_xy = [gt_xy[0].copy()]
    for i in range(1, len(oxts_paths)):
        dt = 0.1
        ekf.predict(dt, fields["af"][i], fields["wu"][i])
        ekf.update_gps(gt_xy[i, 0], gt_xy[i, 1])
        ekf.update_speed(float(np.hypot(fields["vn"][i], fields["ve"][i])))
        ekf.update_yaw(float(fields["yaw"][i]))
        fused_xy.append(np.array(ekf.xy()))
    fused_xy = np.array(fused_xy)
    metrics = {"mode": "aided", "frames": len(fused_xy), "ate_2d": ate_2d(fused_xy, gt_xy),
               "ate_3d": ate_3d(np.column_stack([fused_xy, gt_xyz[:, 2]]), gt_xyz),
               "gt_len": path_length(gt_xy), "est_len": path_length(fused_xy)}
    metrics["len_err_pct"] = 100.0 * abs(metrics["est_len"] - metrics["gt_len"]) / metrics["gt_len"]
    _save_and_plot(fused_xy, gt_xy, metrics, "EKF GPS+IMU")
    return metrics

def run_odom(max_frames=None):
    left, right, bins, oxts_paths = load_sync_lists(max_frames)
    oxts = load_all_oxts(oxts_paths)
    fields = oxts_fields(oxts)
    gt_xy = gps_to_local_xy(fields["lat"], fields["lon"])
    gt_xyz = gps_to_local_xyz(fields["lat"], fields["lon"], fields["alt"])
    k, p2, p3, tr, tr_inv = read_raw_calib()
    print("Precomputing LiDAR ICP...")
    rel_lidar, fitness_hist = precompute_lidar_rel(bins, tr, tr_inv)
    print("Precomputing Stereo VO...")
    rel_vo, inl_hist = precompute_vo_rel(left, right, k, p2, p3)
    est_xyz = run_imu_lidar_vo_odom(rel_lidar, fitness_hist, rel_vo, inl_hist, gt_xyz)
    est_xy = est_xyz[:, :2]
    metrics = {"mode": "odom", "frames": len(est_xy), "ate_2d": ate_2d(est_xy, gt_xy),
               "ate_3d": ate_3d(est_xyz, gt_xyz), "gt_len": path_length(gt_xy),
               "est_len": path_length(est_xy), "mean_icp_fitness": float(np.mean(fitness_hist)),
               "mean_vo_inliers": float(np.mean(inl_hist))}
    metrics["len_err_pct"] = 100.0 * abs(metrics["est_len"] - metrics["gt_len"]) / metrics["gt_len"]
    _save_and_plot(est_xy, gt_xy, metrics, "LiDAR+VO (no GPS in filter)")
    return metrics

def _save_and_plot(est_xy, gt_xy, metrics, label):
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    np.savetxt(os.path.join(config.OUTPUT_DIR, f"{metrics['mode']}_trajectory.csv"),
               est_xy, delimiter=",", header="x,y", comments="")
    print(f"\n===== TRACK C — {metrics['mode'].upper()} =====")
    print(f"Frames       : {metrics['frames']}")
    print(f"ATE 2D       : {metrics['ate_2d']:.3f} m")
    print(f"ATE 3D       : {metrics['ate_3d']:.3f} m")
    print(f"Length error : {metrics['len_err_pct']:.1f}%")
    if config.SHOW_PLOTS:
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(gt_xy[:, 0], gt_xy[:, 1], "b-", lw=2, label="OXTS reference")
        ax.plot(est_xy[:, 0], est_xy[:, 1], "r--", lw=2, label=label)
        ax.set_title(f"Track C {metrics['mode']} | ATE={metrics['ate_2d']:.2f} m")
        ax.axis("equal"); ax.grid(True); ax.legend()
        out_png = os.path.join(config.OUTPUT_DIR, f"{metrics['mode']}_trajectory.png")
        plt.savefig(out_png, dpi=150, bbox_inches="tight")
        plt.close()
        print("Saved plot:", out_png)
