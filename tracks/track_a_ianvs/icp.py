import numpy as np
import open3d as o3d
from src import config as cfg
def preprocess(points):
    r = np.linalg.norm(points[:, :2], axis=1)
    points = points[r < cfg.RANGE_LIMIT]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    try:
        _, inliers = pcd.segment_plane(
            distance_threshold=cfg.GROUND_DIST_THRESH,
            ransac_n=3,
            num_iterations=100,
        )
        pcd = pcd.select_by_index(inliers, invert=True)
    except Exception:
        pass
    return pcd.voxel_down_sample(voxel_size=cfg.VOXEL_SIZE)
def run_icp(src_pts, tgt_pts, init_transform):
    src = preprocess(src_pts)
    tgt = preprocess(tgt_pts)
    src.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
    )
    tgt.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
    )
    result = o3d.pipelines.registration.registration_generalized_icp(
        src,
        tgt,
        max_correspondence_distance=cfg.ICP_MAX_DIST,
        init=init_transform,
        estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    )
    reliable = (
        result.fitness >= cfg.ICP_FITNESS_THRESHOLD
        and result.inlier_rmse <= cfg.ICP_RMSE_THRESHOLD
    )
    T = result.transformation if reliable else init_transform
    return T, result.fitness, result.inlier_rmse, reliable
def imu_init_transform(vf_prev, dyaw, dt):
    dx = vf_prev * dt
    c, s = np.cos(dyaw), np.sin(dyaw)
    return np.array(
        [
            [c, -s, 0, dx],
            [s, c, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0],
        ],
        dtype=float,
    )