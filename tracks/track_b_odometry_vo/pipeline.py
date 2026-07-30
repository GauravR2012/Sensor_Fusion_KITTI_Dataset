import os
import matplotlib.pyplot as plt
import numpy as np
from shared.metrics import ate_3d, path_length, umeyama_3d, xyz_from_poses
from . import config
from .loaders import find_odometry_root, load_frame_lists, read_calib, sequence_paths
from .stereo_vo import run_stereo_vo

def run_pipeline(seq=None, max_frames=None):
    seq = seq or config.SEQUENCE
    max_frames = config.MAX_FRAMES if max_frames is None else max_frames
    data_root = find_odometry_root()
    paths = sequence_paths(data_root, seq)
    img_l, img_r, gt_poses = load_frame_lists(paths, max_frames)
    k, p2, p3, baseline = read_calib(paths["calib"])
    gt_xyz = xyz_from_poses(gt_poses)
    print(f"Track B | seq {seq} | {len(img_l)} frames | baseline={baseline:.3f} m")

    vo_poses, inliers, failed = run_stereo_vo(img_l, img_r, k, p2, p3)
    vo_aligned = umeyama_3d(xyz_from_poses(vo_poses), gt_xyz)
    gt_len = path_length(gt_xyz)
    vo_len = path_length(vo_aligned)

    metrics = {
        "ate_3d": ate_3d(vo_aligned, gt_xyz),
        "final_err": float(np.linalg.norm(vo_aligned[-1] - gt_xyz[-1])),
        "gt_len": gt_len, "vo_len": vo_len,
        "len_err_pct": 100.0 * abs(vo_len - gt_len) / gt_len,
        "failed_frames": failed,
        "mean_inliers": float(np.mean(inliers)) if inliers else 0.0,
    }

    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    np.savetxt(os.path.join(config.OUTPUT_DIR, f"vo_seq{seq}.csv"), vo_aligned,
               delimiter=",", header="x,y,z", comments="")

    if config.SHOW_PLOTS:
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        axes[0].plot(gt_xyz[:, 0], gt_xyz[:, 1], "b-", lw=2, label="GT")
        axes[0].plot(vo_aligned[:, 0], vo_aligned[:, 1], "m--", lw=1.5, label="Stereo VO")
        axes[0].set_title(f"Seq {seq} | ATE={metrics['ate_3d']:.2f} m")
        axes[0].axis("equal"); axes[0].grid(True); axes[0].legend()
        err = np.linalg.norm(vo_aligned - gt_xyz, axis=1)
        axes[1].plot(err, "m-", label=f"mean={err.mean():.2f} m")
        axes[1].plot(inliers, color="gray", alpha=0.5, label="PnP inliers")
        axes[1].grid(True); axes[1].legend()
        plt.tight_layout()
        out_png = os.path.join(config.OUTPUT_DIR, f"stereo_vo_seq{seq}.png")
        plt.savefig(out_png, dpi=150, bbox_inches="tight")
        plt.close()
        print("Saved plot:", out_png)

    print("\n===== TRACK B RESULTS =====")
    print(f"Failed frames    : {metrics['failed_frames']} / {len(img_l)-1}")
    print(f"Mean PnP inliers : {metrics['mean_inliers']:.1f}")
    print(f"ATE 3D (aligned) : {metrics['ate_3d']:.3f} m")
    print(f"Final 3D error   : {metrics['final_err']:.3f} m")
    print(f"Length error     : {metrics['len_err_pct']:.1f}%")
    return metrics
