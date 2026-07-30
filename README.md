# KITTI Multi-Track Sensor Fusion
Modular pose-estimation benchmarks on KITTI-family datasets: **GPS-aided fusion**, **stereo visual odometry**, and **4-sensor ablation** on raw synced logs.
> **Portfolio entry point:** pick a track below — each uses a different dataset and evaluation reference.
---
## Choose a Track
| | **Track A — Ianvs** | **Track B — Stereo VO** | **Track C — Raw 0047** |
|:---:|:---:|:---:|:---:|
| **Folder** | `tracks/track_a_ianvs/` | `tracks/track_b_odometry_vo/` | `tracks/track_c_raw_0047/` |
| **Sensors** | GPS + IMU + LiDAR | Camera (stereo) | GPS + IMU + LiDAR + Camera |
| **GPS in filter?** | Yes | No | Aided: Yes · Odom: No |
| **Ground truth** | OXTS INS/GNSS | `poses/XX.txt` | OXTS (eval / aided updates) |
| **Core algorithms** | GICP + Umeyama + 2D EKF | ORB + triangulation + PnP | EKF (aided) · ICP + VO fusion (odom) |
| **Highlight result** | ~1–6 m ATE (multi-drive) | ~15–17 m ATE (seq 00) | 0.17 m aided · ~28 m odom |
---
## Results at a Glance
| Track | Mode | Frames | ATE | Length err | Notes |
|-------|------|-------:|----:|-----------:|-------|
| **A** | GPS+IMU+LiDAR EKF | 108 (drive 0001) | ~1.1 m | ~2.5% | Short urban segment |
| **A** | Multi-drive mean | 9 drives | ~6 m | ~3–18% | Drive 0014 worst (~38 m ATE) |
| **B** | Stereo VO only | 4540 (seq 00) | ~15–17 m | ~1.5% | Official pose GT; camera only |
| **C** | GPS+IMU EKF (aided) | 837 | ~0.17 m | ~0% | GPS-aided navigation |
| **C** | LiDAR+VO odom (no GPS) | 200 | ~28 m | varies | Open-loop odometry |
> **Honest labeling:** Tracks A and C (aided) use OXTS as both sensor input and reference. Track B uses independent camera pose GT. Track C (odom) uses OXTS for evaluation only.
---
# Track A — Ianvs GPS + IMU + LiDAR (Stable 2D + Experimental 3D)
2D pose estimation by fusing **Velodyne scan-matching odometry**, **OXTS GPS**, and **IMU-related OXTS signals** with an **Extended Kalman Filter (EKF)**, evaluated on the **[KubeEdge-Ianvs KITTI Pose Estimation Dataset](https://www.kaggle.com/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset)**.
## Repository Versions (Track A)
This track contains two implementations:
### Stable 2D Pipeline
- 2D vehicle pose estimation
- State: `[x, y, yaw, velocity]`
- Generalized ICP (GICP)
- GPS + IMU + LiDAR EKF
- Multi-drive evaluation
### Experimental 3D Pipeline
An extension that estimates full 3D trajectories using LiDAR, GPS, and IMU:
- 3D LiDAR odometry
- 3D GPS alignment
- 3D Extended Kalman Filter
- Height profile evaluation
- 3D trajectory visualization
The 3D implementation is intended for experimentation and comparison with the stable 2D pipeline.
## Track A — Overview
This pipeline combines:
- **LiDAR odometry** from Generalized ICP (GICP)
- **GPS position** from KITTI OXTS packets
- **IMU motion** (forward acceleration and yaw rate)
- **Extended Kalman Filter** state estimation
Final estimated state:
[x, y, yaw, velocity]

## Track A — Features
### LiDAR Odometry
- Generalized ICP (GICP)
- Range filtering
- RANSAC ground removal
- Voxel grid downsampling
- IMU-aided ICP initialization
- ICP fitness and RMSE rejection
### Alignment
- Global Umeyama alignment
- LiDAR trajectory aligned into GPS frame
### Sensor Fusion (EKF)
**State:** `[x, y, yaw, v]`
**Measurements:**
- GPS position
- Adaptive LiDAR position
- GPS-derived speed
- OXTS yaw
**Prediction:**
- IMU forward acceleration
- IMU yaw rate
**Execution modes:**
- Single drive
- Multi-drive benchmark (`--all`)
## Track A — Pipeline
```text
             OXTS
      (GPS + IMU signals)
              │
              ▼
      Local UTM Coordinates
              │
              ▼
        GPS / Speed / Yaw
              │
              ▼
        Extended Kalman Filter
              ▲
              │
Velodyne scans (.bin)
        │
        ▼
 Preprocessing
  • Range filter
  • Ground removal
  • Downsampling
        │
        ▼
       GICP
        │
        ▼
 Umeyama Alignment
        │
        ▼
  LiDAR Position
```

Track A — Dataset
Download from Kaggle:

https://www.kaggle.com/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset

Directory layout:

data/
└── 2011_09_26/
    ├── 2011_09_26_drive_0001_sync/
    │   ├── oxts/
    │   │   └── data/
    │   └── velodyne_points/
    │       └── data/
    ├── 2011_09_26_drive_0002_sync/
    └── ...
Important: Do not commit dataset files to the repository.

Track A — Usage
Set dataset path:

# Linux/macOS
export DATA_ROOT=/path/to/data/2011_09_26
# Windows
set DATA_ROOT=C:\path\to\data\2011_09_26
Run a single drive (default: 2011_09_26_drive_0001_sync):

python -m src.run --drive 2011_09_26_drive_0001_sync
Run all drives:

python -m src.run --all
Outputs:

outputs/
├── fused_state.csv
├── sensor_fusion_result.png
├── all_drives_trajectories.png
├── all_drives_errors.png
├── all_drives_icp.png
├── performance_summary.png
└── velocity_yaw_profile.png
Running Track A on Kaggle
Add the KubeEdge-Ianvs KITTI Pose Estimation Dataset
Set:
DATA_ROOT = "/kaggle/input/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset/data/2011_09_26"
Run the notebook or python -m src.run --all
Track A — Configuration
Main parameters in src/config.py or src/pipeline.py:

Parameter	Typical Value	Description
ICP_FITNESS_THRESHOLD
0.30
Reject poor GICP registrations
ICP_RMSE_THRESHOLD
0.50
Reject high registration error
VOXEL_SIZE
0.40 m
Point cloud downsampling
RANGE_LIMIT
60 m
LiDAR range filter
Q_*
tuned
EKF process noise
R_*
tuned
EKF measurement noise
LIDAR_NOISE_*
tuned
Adaptive LiDAR covariance
Track A — Example Results
Evaluation uses OXTS GPS XY as the reference trajectory.

Drive	Frames	Length Error	Notes
0001
108
~2–3%
Short urban segment
0011
233
~3.3%
Good performance
0014
314
~15–18%
Long loop; ICP drift dominates
0017
114
N/A
Very short trajectory (~0.1 m); use ATE instead
Results vary slightly depending on Open3D version, ICP tuning, and EKF noise parameters.

Track A — Visualizations
Performance Summary Across All Drives
Track A — Performance Summary

Trajectory Error Across All Drives
Track A — Trajectory Error

ICP Fitness per Drive
Track A — ICP Fitness

Per-frame Position Error vs GPS
Track A — Position Error

Velocity Profile
Track A — Velocity Profile

Experimental 3D — Position Error vs OXTS
Track A — 3D Position Error

Experimental 3D — Vertical Channel Comparison
Track A — 3D Vertical Channel

Track B — KITTI Odometry Stereo Visual Odometry
Camera-only visual odometry on the KITTI Odometry dataset (image_2 + image_3 + calib.txt).

Ground truth from poses/XX.txt is used for evaluation only — not fused into the estimator.

Track B — Algorithms
ORB feature detection and descriptor matching
Stereo triangulation (cv2.triangulatePoints)
PnP RANSAC for frame-to-frame motion
Umeyama 3D alignment for ATE vs official poses
Track B — Dataset Layout
kitti-odometry/
├── poses/
│   └── 00.txt
└── sequences/
    └── 00/
        ├── image_2/
        ├── image_3/
        ├── velodyne/     # present but not used in Track B baseline
        └── calib.txt
Sequences 00–10 have public pose ground truth.

Track B — Usage
# Linux/macOS
export ODOMETRY_DATA_ROOT=/path/to/kitti-odometry
# Windows
set ODOMETRY_DATA_ROOT=C:\path\to\kitti-odometry
Run stereo VO on sequence 00:

python -m tracks.track_b_odometry_vo.run --seq 00
Quick test (500 frames):

python -m tracks.track_b_odometry_vo.run --seq 00 --max-frames 500
Outputs:

outputs/track_b/
├── vo_seq00.csv
└── stereo_vo_seq00.png
Track B — Example Results (seq 00)
Metric	Value
ATE 3D (aligned)
~15–17 m
Final 3D error
~32 m
Path length error
~1.5%
Failed frames
0 / 4540
Mean PnP inliers
~580
Track B — Visualization
Track B — Stereo VO seq 00

Track C — Raw KITTI 2011_10_03_drive_0047 (4-Sensor)
Fully synchronized 837-frame raw KITTI log with OXTS, Velodyne, stereo camera, and calibration.

Track C — Modes
Mode	Command	Sensors in estimator	Purpose
aided
--mode aided
GPS + IMU (OXTS)
GPS-aided navigation
odom
--mode odom
LiDAR + Camera
GPS-denied odometry
Track C — Algorithms
aided mode

WGS84 → local coordinates (pyproj)
2D EKF [x, y, yaw, v]
IMU predict + GPS / speed / yaw updates
odom mode

SciPy ICP LiDAR odometry
Stereo VO (ORB + triangulation + PnP)
Weighted relative-pose fusion (LiDAR + VO)
Umeyama 3D alignment for evaluation vs OXTS only
Track C — Dataset Layout
2011_10_03/
├── calib_cam_to_cam.txt
├── calib_imu_to_velo.txt
├── calib_velo_to_cam.txt
└── 2011_10_03_drive_0047_sync/
    ├── oxts/data/
    ├── velodyne_points/data/
    ├── image_02/data/
    └── image_03/data/
Download from KITTI raw data.

Important: Do not commit dataset files to the repository.

Track C — Usage
# Linux/macOS
export RAW_DATA_PATH=/path/to/2011_10_03/2011_10_03_drive_0047_sync
export RAW_CALIB_ROOT=/path/to/2011_10_03
# Windows
set RAW_DATA_PATH=C:\path\to\2011_10_03\2011_10_03_drive_0047_sync
set RAW_CALIB_ROOT=C:\path\to\2011_10_03
GPS-aided EKF:

python -m tracks.track_c_raw_0047.run --mode aided
python -m tracks.track_c_raw_0047.run --mode aided --max-frames 837
GPS-denied LiDAR + VO odometry:

python -m tracks.track_c_raw_0047.run --mode odom --max-frames 200
python -m tracks.track_c_raw_0047.run --mode odom
Outputs:

outputs/track_c/
├── aided_trajectory.csv
├── aided_trajectory.png
├── odom_trajectory.csv
└── odom_trajectory.png
Running Track C on Kaggle
RAW_DATA_PATH = "/kaggle/working/2011_10_03/2011_10_03_drive_0047_sync"
RAW_CALIB_ROOT = "/kaggle/working/2011_10_03"
Track C — Example Results
Mode	Frames	ATE 2D	Length err	Notes
aided
837
~0.17 m
~0%
GPS+IMU EKF
aided
200
~0.10 m
~0.1%
Quick test
odom
200
~28 m
varies
No GPS in filter
Track C — Visualizations
GPS-aided EKF trajectory
Track C — GPS+IMU aided

GPS-denied LiDAR + VO odometry
Track C — Odom mode

Installation
Create a virtual environment:

python -m venv .venv
Activate it:

# Linux/macOS
source .venv/bin/activate
# Windows
.venv\Scripts\activate
Install dependencies:

pip install -r requirements.txt
Main dependencies: NumPy, Matplotlib, pyproj, Open3D, OpenCV, SciPy, tqdm

Project Structure
.
├── README.md
├── CHANGELOG.md
├── LICENSE
├── requirements.txt
├── .gitignore
│
├── src/                              # Track A (Ianvs)
│   ├── __init__.py
│   ├── config.py
│   ├── loaders.py
│   ├── icp.py
│   ├── alignment.py
│   ├── ekf.py
│   ├── pipeline.py
│   └── run.py
│
├── shared/                           # Shared utilities
│   ├── __init__.py
│   └── metrics.py
│
├── tracks/
│   ├── __init__.py
│   │
│   ├── track_b_odometry_vo/          # Track B
│   │   ├── __init__.py
│   │   ├── config.py
│   │   ├── loaders.py
│   │   ├── stereo_vo.py
│   │   ├── pipeline.py
│   │   └── run.py
│   │
│   └── track_c_raw_0047/             # Track C
│       ├── __init__.py
│       ├── config.py
│       ├── loaders.py
│       ├── icp.py
│       ├── stereo_vo.py
│       ├── ekf.py
│       ├── odom_pipeline.py
│       ├── pipeline.py
│       └── run.py
│
├── notebooks/
│   └── kitti_sensor_fusion.ipynb
│
└── outputs/
    ├── ...                           # Track A plots
    ├── track_b/
    │   └── stereo_vo_seq00.png
    └── track_c/
        ├── aided_trajectory.png
        └── odom_trajectory.png
Limitations
Track A: 2D EKF only; global Umeyama may degrade on long trajectories (e.g. drive 0014)
Track B: Basic ORB+PnP VO; no loop closure; drift accumulates over long sequences
Track C aided: OXTS used as both measurement and evaluation reference
Track C odom: Open-loop drift; Umeyama used for evaluation only
No full 6-DOF LiDAR–Inertial Odometry (e.g. FAST-LIO2)
No explicit LiDAR–IMU extrinsic calibration in Track A
No ORB-SLAM2, graph SLAM, or learned VO models
Future improvements are listed in CHANGELOG.md.

Dataset Licenses
Dataset	Link	License
Ianvs KITTI Pose
Kaggle
CC BY-NC-SA 3.0 IGO / KITTI
KITTI Odometry
Kaggle
KITTI
Raw KITTI 0047
KITTI raw
KITTI
Please cite both KITTI and KubeEdge-Ianvs when using Track A.

Do not redistribute raw dataset files in this repository.

Citation
@article{Geiger2013IJRR,
  author  = {Andreas Geiger and Philip Lenz and Christoph Stiller and Raquel Urtasun},
  title   = {Vision Meets Robotics: The KITTI Dataset},
  journal = {The International Journal of Robotics Research},
  year    = {2013}
}
Dataset:

KubeEdge-Ianvs KITTI Pose Estimation Dataset (Kaggle)

License
Code
MIT License (see LICENSE)

Dataset
The datasets are distributed under their own licenses. Follow Kaggle/KITTI/CC BY-NC-SA terms and do not redistribute raw data.

Acknowledgements
KITTI Vision Benchmark Suite
KubeEdge-Ianvs
Open3D
OpenCV
NumPy
SciPy