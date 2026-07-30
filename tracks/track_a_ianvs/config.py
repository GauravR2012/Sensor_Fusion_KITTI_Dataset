import os
# Dataset root (override with env DATA_ROOT)
DATA_ROOT = os.environ.get(
    "DATA_ROOT",
    "/kaggle/input/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset/data/2011_09_26",
)
OUTPUT_DIR = os.environ.get("OUTPUT_DIR", "outputs")
DEFAULT_DRIVE = "2011_09_26_drive_0001_sync"
# ICP / point cloud
ICP_FITNESS_THRESHOLD = 0.30
ICP_RMSE_THRESHOLD = 0.50
ICP_MAX_DIST = 2.5
VOXEL_SIZE = 0.40
RANGE_LIMIT = 60.0
GROUND_DIST_THRESH = 0.20
# EKF
Q_POS_NOISE = 0.05
Q_YAW_NOISE = 0.005
Q_VEL_NOISE = 0.8
R_GPS_NOISE = 1.5
R_SPEED_NOISE = 0.3
LIDAR_NOISE_BASE = 2.5
LIDAR_NOISE_MIN = 0.8
R_YAW_MEAS = 0.05
DT_DEFAULT = 0.1
DT_MIN = 0.05
DT_MAX = 0.5