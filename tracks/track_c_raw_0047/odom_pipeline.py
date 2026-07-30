import numpy as np
from shared.metrics import chain_poses, umeyama_3d, xyz_from_poses
from . import config

def fuse_relative_transforms(t_lidar_cam, fitness, t_vo, n_inl):
    if fitness < config.ICP_MIN_FITNESS:
        return t_vo if t_vo is not None else np.eye(4)
    if t_vo is None:
        return t_lidar_cam
    w_l = config.LIDAR_WEIGHT_SCALE * max(fitness, 1e-3)
    w_v = config.VO_WEIGHT_SCALE * min(n_inl / 500.0, 1.0)
    t_f = np.eye(4)
    t_f[:3, :3] = t_lidar_cam[:3, :3] if w_l >= w_v else t_vo[:3, :3]
    t_f[:3, 3] = (w_l * t_lidar_cam[:3, 3] + w_v * t_vo[:3, 3]) / (w_l + w_v)
    return t_f

def run_imu_lidar_vo_odom(rel_lidar, fitness_hist, rel_vo, inl_hist, gt_xyz):
    rel_fused = [fuse_relative_transforms(rel_lidar[i], fitness_hist[i], rel_vo[i], inl_hist[i])
                 for i in range(len(rel_lidar))]
    traj = chain_poses(rel_fused)
    return umeyama_3d(xyz_from_poses(traj), gt_xyz)
