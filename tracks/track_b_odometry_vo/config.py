import os

DATA_ROOT = os.environ.get("ODOMETRY_DATA_ROOT", "/kaggle/input/datasets/hocop1/kitti-odometry")
SEQUENCE = os.environ.get("SEQUENCE", "00")
MAX_FRAMES = None
MIN_PNP_INLIERS = 40
MAX_LANDMARKS = 3000
NN_THRESH_PX = 3.0
PNP_REPROJ_ERR = 2.0
OUTPUT_DIR = os.environ.get("OUTPUT_DIR", "outputs/track_b")
SHOW_PLOTS = True
