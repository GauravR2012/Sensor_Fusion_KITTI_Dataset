import os
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from src import config as cfg
from src.alignment import umeyama_align
from src.ekf import EKF
from src.icp import imu_init_transform, run_icp
from src.loaders import (
    gps_to_xy,
    list_drives,
    load_drive_files,
    load_oxts,
    load_timestamps,
    load_velodyne,
)
def clip_dt(timestamps, i):
    if timestamps is None or i <= 0 or i >= len(timestamps):
        return cfg.DT_DEFAULT
    return float(np.clip(timestamps[i] - timestamps[i - 1], cfg.DT_MIN, cfg.DT_MAX))
def compute_metrics(fused_xy, gps_xy):
    ate = np.sqrt(np.mean(np.sum((fused_xy - gps_xy) ** 2, axis=1)))
    gt_len = np.sum(np.linalg.norm(np.diff(gps_xy, axis=0), axis=1))
    est_len = np.sum(np.linalg.norm(np.diff(fused_xy, axis=0), axis=1))
    len_err = abs(est_len - gt_len)
    len_pct = (len_err / gt_len * 100.0) if gt_len > 0 else 0.0
    final_err = float(np.linalg.norm(fused_xy[-1] - gps_xy[-1]))
    return {
        "ate": ate,
        "final_err": final_err,
        "gt_len": gt_len,
        "est_len": est_len,
        "len_err": len_err,
        "len_pct": len_pct,
    }
def process_drive(drive_name, data_root=None, show_icp_progress=False):
    data_root = data_root or cfg.DATA_ROOT
    drive_path = os.path.join(data_root, drive_name)
    oxts_files, velo_files, ts_path = load_drive_files(drive_path)
    if not oxts_files or not velo_files:
        return None
    N = min(len(oxts_files), len(velo_files))
    if N < 3:
        return None
    records = [load_oxts(f) for f in oxts_files[:N]]
    gps_xy = gps_to_xy(records)
    timestamps = load_timestamps(ts_path)
    lx, ly = [0.0], [0.0]
    global_pose = np.eye(4)
    fitness_scores = []
    rejected = 0
    iterator = range(1, N)
    if show_icp_progress:
        iterator = tqdm(iterator, desc=f"ICP {drive_name}", leave=False)
    for i in iterator:
        src = load_velodyne(velo_files[i - 1])
        tgt = load_velodyne(velo_files[i])
        dt = clip_dt(timestamps, i)
        dyaw = records[i]["yaw"] - records[i - 1]["yaw"]
        init = imu_init_transform(records[i - 1]["vf"], dyaw, dt)
        T, fit, _, ok = run_icp(src, tgt, init)
        if not ok:
            rejected += 1
        fitness_scores.append(fit)
        global_pose = global_pose @ T
        lx.append(global_pose[0, 3])
        ly.append(global_pose[1, 3])
    raw_lidar = np.column_stack([lx, ly])
    lidar_xy = umeyama_align(raw_lidar, gps_xy)
    ekf = EKF()
    ekf.x = np.array(
        [[gps_xy[0, 0]], [gps_xy[0, 1]], [records[0]["yaw"]], [records[0]["vf"]]]
    )
    fused = []
    for i in range(N):
        dt = clip_dt(timestamps, i)
        ekf.predict(dt, records[i]["af"], records[i]["wu"])
        ekf.update_gps(gps_xy[i, 0], gps_xy[i, 1])
        fs = fitness_scores[i - 1] if i > 0 else 1.0
        ekf.update_lidar(lidar_xy[i, 0], lidar_xy[i, 1], fs)
        ekf.update_speed(records[i]["vn"], records[i]["ve"])
        ekf.update_yaw(records[i]["yaw"])
        fused.append(ekf.x.flatten().copy())
    fused = np.array(fused)
    metrics = compute_metrics(fused[:, :2], gps_xy)
    return {
        "drive": drive_name,
        "N": N,
        "gps_xy": gps_xy,
        "lidar_xy": lidar_xy,
        "fused": fused,
        "fitness_scores": fitness_scores,
        "rejected": rejected,
        "records": records,
        "metrics": metrics,
    }
def print_summary(results):
    print("\n" + "=" * 95)
    print(
        f"{'Drive':<35} {'Frames':>6} {'GPS(m)':>8} {'Fused(m)':>9} "
        f"{'LenErr%':>8} {'ATE(m)':>8} {'FinalE':>8}"
    )
    print("=" * 95)
    for r in results:
        m = r["metrics"]
        print(
            f"{r['drive']:<35} {r['N']:>6} {m['gt_len']:>8.2f} "
            f"{m['est_len']:>9.2f} {m['len_pct']:>7.1f}% "
            f"{m['ate']:>8.3f} {m['final_err']:>7.3f}m"
        )
    print("=" * 95)
    tot_gt = sum(r["metrics"]["gt_len"] for r in results)
    tot_est = sum(r["metrics"]["est_len"] for r in results)
    mean_ate = np.mean([r["metrics"]["ate"] for r in results])
    print(
        f"\nTotal GPS: {tot_gt:.2f}m | Total Fused: {tot_est:.2f}m | "
        f"Overall LenErr: {abs(tot_gt - tot_est) / tot_gt * 100:.1f}% | "
        f"Mean ATE: {mean_ate:.3f}m"
    )
def save_csv(result, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"fused_{result['drive']}.csv")
    np.savetxt(path, result["fused"], delimiter=",", header="x,y,yaw,v", comments="")
    return path
def plot_single_drive(result, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    gps = result["gps_xy"]
    lidar = result["lidar_xy"]
    fused = result["fused"]
    fs = result["fitness_scores"]
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    ax = axes[0, 0]
    ax.plot(gps[:, 0], gps[:, 1], "b-", lw=2.5, label="GPS")
    ax.plot(lidar[:, 0], lidar[:, 1], "g--", lw=1.5, label="LiDAR (Umeyama)")
    ax.plot(fused[:, 0], fused[:, 1], "r-", lw=2, label="EKF fused")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Trajectory")
    ax = axes[0, 1]
    ax.plot(fs, color="steelblue")
    ax.axhline(cfg.ICP_FITNESS_THRESHOLD, color="red", ls="--")
    ax.set_title("ICP fitness")
    ax.grid(True, alpha=0.3)
    fe = np.sqrt(np.sum((fused[:, :2] - gps) ** 2, axis=1))
    le = np.sqrt(np.sum((lidar - gps) ** 2, axis=1))
    ax = axes[1, 0]
    ax.plot(fe, "r-", label=f"EKF mean={fe.mean():.2f}m")
    ax.plot(le, "g--", alpha=0.7, label=f"LiDAR mean={le.mean():.2f}m")
    ax.legend()
    ax.set_title("Position error vs GPS")
    ax.grid(True, alpha=0.3)
    gps_spd = np.array([np.hypot(r["vn"], r["ve"]) for r in result["records"]])
    ax = axes[1, 1]
    ax.plot(gps_spd, "b-", label="GPS speed")
    ax.plot(fused[:, 3], "r-", label="EKF speed")
    ax.legend()
    ax.set_title("Speed")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(out_dir, "sensor_fusion_result.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    return path
def plot_all_drives(results, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    n = len(results)
    ncols = min(3, n)
    nrows = (n + ncols - 1) // ncols
    def _short(name):
        return name.replace("2011_09_26_drive_", "").replace("_sync", "")
    # Trajectories
    fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 6 * nrows))
    axes = np.atleast_1d(axes).flatten()
    for i, r in enumerate(results):
        ax = axes[i]
        ax.plot(r["gps_xy"][:, 0], r["gps_xy"][:, 1], "b-", lw=2, label="GPS")
        ax.plot(r["lidar_xy"][:, 0], r["lidar_xy"][:, 1], "g--", lw=1, label="LiDAR")
        ax.plot(r["fused"][:, 0], r["fused"][:, 1], "r-", lw=2, label="Fused")
        m = r["metrics"]
        ax.set_title(f"{_short(r['drive'])}\nATE={m['ate']:.2f}m LenErr={m['len_pct']:.1f}%")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
    for j in range(len(results), len(axes)):
        axes[j].set_visible(False)
    plt.suptitle("All drives — trajectories")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "all_drives_trajectories.png"), dpi=150, bbox_inches="tight")
    plt.close()
    # Errors
    fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 4 * nrows))
    axes = np.atleast_1d(axes).flatten()
    for i, r in enumerate(results):
        ax = axes[i]
        gps = r["gps_xy"]
        fe = np.sqrt(np.sum((r["fused"][:, :2] - gps) ** 2, axis=1))
        le = np.sqrt(np.sum((r["lidar_xy"] - gps) ** 2, axis=1))
        ax.plot(fe, "r-", label="EKF")
        ax.plot(le, "g--", alpha=0.7, label="LiDAR")
        ax.set_title(_short(r["drive"]))
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
    for j in range(len(results), len(axes)):
        axes[j].set_visible(False)
    plt.suptitle("Per-frame error vs GPS")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "all_drives_errors.png"), dpi=150, bbox_inches="tight")
    plt.close()
    # ICP fitness
    fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 4 * nrows))
    axes = np.atleast_1d(axes).flatten()
    for i, r in enumerate(results):
        ax = axes[i]
        ax.plot(r["fitness_scores"], color="steelblue")
        ax.axhline(cfg.ICP_FITNESS_THRESHOLD, color="red", ls="--")
        ax.set_title(f"{_short(r['drive'])} ({r['rejected']} rej)")
        ax.grid(True, alpha=0.3)
    for j in range(len(results), len(axes)):
        axes[j].set_visible(False)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "all_drives_icp.png"), dpi=150, bbox_inches="tight")
    plt.close()
    # Bar summary
    labels = [_short(r["drive"]) for r in results]
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    axes[0].bar(labels, [r["metrics"]["ate"] for r in results])
    axes[0].set_title("ATE (m)")
    axes[1].bar(labels, [r["metrics"]["len_pct"] for r in results])
    axes[1].set_title("Length error (%)")
    axes[2].bar(labels, [r["metrics"]["final_err"] for r in results])
    axes[2].set_title("Final error (m)")
    for ax in axes:
        ax.tick_params(axis="x", rotation=45)
        ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "performance_summary.png"), dpi=150, bbox_inches="tight")
    plt.close()
    if results:
        r = results[0]
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        gps_spd = np.array([np.hypot(rec["vn"], rec["ve"]) for rec in r["records"]])
        axes[0].plot(gps_spd, "b-", label="GPS speed")
        axes[0].plot(r["fused"][:, 3], "r-", label="EKF speed")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        yaw_gt = np.array([rec["yaw"] for rec in r["records"]])
        axes[1].plot(np.degrees(yaw_gt), "b-", label="GPS yaw")
        axes[1].plot(np.degrees(r["fused"][:, 2]), "r-", label="EKF yaw")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "velocity_yaw_profile.png"), dpi=150, bbox_inches="tight")
        plt.close()
def run_all(data_root=None, out_dir=None):
    data_root = data_root or cfg.DATA_ROOT
    out_dir = out_dir or cfg.OUTPUT_DIR
    drives = list_drives(data_root)
    results = []
    for drive in drives:
        print(f"Processing {drive}...")
        r = process_drive(drive, data_root=data_root, show_icp_progress=True)
        if r is None:
            print(f"  Skipped {drive}")
            continue
        results.append(r)
        save_csv(r, out_dir)
        m = r["metrics"]
        print(
            f"  ATE={m['ate']:.3f}m LenErr={m['len_pct']:.1f}% "
            f"Final={m['final_err']:.3f}m RejectedICP={r['rejected']}"
        )
    print_summary(results)
    plot_all_drives(results, out_dir)
    return results
def run_one(drive_name, data_root=None, out_dir=None):
    data_root = data_root or cfg.DATA_ROOT
    out_dir = out_dir or cfg.OUTPUT_DIR
    r = process_drive(drive_name, data_root=data_root, show_icp_progress=True)
    if r is None:
        raise RuntimeError(f"Failed to process drive: {drive_name}")
    save_csv(r, out_dir)
    # Also write generic name for single-run convenience
    np.savetxt(
        os.path.join(out_dir, "fused_state.csv"),
        r["fused"],
        delimiter=",",
        header="x,y,yaw,v",
        comments="",
    )
    plot_single_drive(r, out_dir)
    m = r["metrics"]
    print(
        f"ATE={m['ate']:.3f}m LenErr={m['len_pct']:.1f}% "
        f"Final={m['final_err']:.3f}m RejectedICP={r['rejected']}"
    )
    return r