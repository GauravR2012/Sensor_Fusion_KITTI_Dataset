# KITTI LiDAR–GPS–IMU Sensor Fusion (EKF)

2D pose estimation by fusing **Velodyne scan-matching odometry**, **OXTS GPS**, and **IMU-related OXTS signals** with an **Extended Kalman Filter (EKF)**, evaluated on the **KubeEdge-Ianvs KITTI Pose Estimation Dataset**.

---

## Overview

This repository implements a complete sensor fusion pipeline that combines:

- **LiDAR odometry** from Generalized ICP (GICP)
- **GPS position** from KITTI OXTS packets
- **IMU motion** (forward acceleration and yaw rate)
- **Extended Kalman Filter** state estimation

The final system estimates a robust **2D vehicle pose**:

```
[x, y, yaw, velocity]
```

---

## Features

This repository contains the **final upgraded pipeline** only.

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

State:

```
[x, y, yaw, v]
```

Measurements:

- GPS position
- Adaptive LiDAR position
- GPS-derived speed
- OXTS yaw

Prediction:

- IMU forward acceleration
- IMU yaw rate

### Execution Modes

- Single drive
- Multi-drive benchmark (`--all`)

---

## Pipeline

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

---

# Dataset

Download the dataset from Kaggle:

> https://www.kaggle.com/datasets/kubeedgeianvs/the-kitti-pose-estimation-dataset

Directory layout:

```text
data/
└── 2011_09_26/
    ├── 2011_09_26_drive_0001_sync/
    │   ├── oxts/
    │   │   └── data/
    │   └── velodyne_points/
    │       └── data/
    ├── 2011_09_26_drive_0002_sync/
    └── ...
```

> **Important:** Do **not** commit dataset files to the repository.

---

## Dataset License

The dataset follows:

- **CC BY-NC-SA 3.0 IGO**
- KITTI Dataset License

Please cite both:

- KITTI
- KubeEdge-Ianvs curated dataset

---

# Installation

Create a virtual environment.

```bash
python -m venv .venv
```

Activate it.

Linux/macOS

```bash
source .venv/bin/activate
```

Windows

```bat
.venv\Scripts\activate
```

Install dependencies.

```bash
pip install -r requirements.txt
```

---

# Usage

## Set dataset path

Linux/macOS

```bash
export DATA_ROOT=/path/to/data/2011_09_26
```

Windows

```bat
set DATA_ROOT=C:\path\to\data\2011_09_26
```

---

## Run a single drive

(Default: `2011_09_26_drive_0001_sync`)

```bash
python -m src.run --drive 2011_09_26_drive_0001_sync
```

Outputs:

```
outputs/
├── fused_state.csv
└── sensor_fusion_result.png
```

---

## Run all drives

```bash
python -m src.run --all
```

Generated outputs:

```
outputs/
├── all_drives_trajectories.png
├── all_drives_errors.png
├── all_drives_icp.png
├── performance_summary.png
└── velocity_yaw_profile.png
```

A performance summary is also printed to the console.

---

# Running on Kaggle

1. Add the **KubeEdge-Ianvs KITTI Pose Estimation Dataset**.
2. Set

```python
DATA_ROOT = "/kaggle/input/the-kitti-pose-estimation-dataset/data/2011_09_26"
```

3. Run the notebook.

The notebook mirrors the implementation inside `src/`.

---

# Configuration

Main parameters are located in:

```
src/config.py
```

or

```
src/pipeline.py
```

| Parameter | Typical Value | Description |
|-----------|--------------:|------------|
| `ICP_FITNESS_THRESHOLD` | 0.30 | Reject poor GICP registrations |
| `ICP_RMSE_THRESHOLD` | 0.50 | Reject high registration error |
| `VOXEL_SIZE` | 0.40 m | Point cloud downsampling |
| `RANGE_LIMIT` | 60 m | LiDAR range filter |
| `Q_*` | tuned | EKF process noise |
| `R_*` | tuned | EKF measurement noise |
| `LIDAR_NOISE_*` | tuned | Adaptive LiDAR covariance |

---

# Example Results

Evaluation uses OXTS GPS XY as the reference trajectory.

| Drive | Frames | Length Error | Notes |
|-------|-------:|-------------:|------|
| 0001 | 108 | ~2–3% | Short urban segment |
| 0014 | 314 | ~15–18% | Long loop; ICP drift dominates |
| 0017 | 114 | N/A | Very short trajectory (~0.1 m); use ATE instead |

Results vary slightly depending on:

- Open3D version
- ICP tuning
- EKF noise parameters

Additional plots are generated under `outputs/`.

---

# Project Structure

```text
.
├── README.md
├── CHANGELOG.md
├── LICENSE
├── requirements.txt
├── .gitignore
├── src/
│   ├── __init__.py
│   ├── config.py
│   ├── loaders.py
│   ├── icp.py
│   ├── alignment.py
│   ├── ekf.py
│   ├── pipeline.py
│   └── run.py
├── notebooks/
│   └── kitti_sensor_fusion.ipynb
└── outputs/
```

---

# Limitations

- 2D EKF only
- No full 6-DOF LiDAR–Inertial Odometry
- No explicit LiDAR–IMU extrinsic calibration
- Global Umeyama alignment may degrade on long trajectories
- GPS/OXTS is used as the evaluation reference

Future improvements are listed in `CHANGELOG.md`.

---

# Citation

If you use this project, please cite KITTI:

```bibtex
@article{Geiger2013IJRR,
  author  = {Andreas Geiger and Philip Lenz and Christoph Stiller and Raquel Urtasun},
  title   = {Vision Meets Robotics: The KITTI Dataset},
  journal = {The International Journal of Robotics Research},
  year    = {2013}
}
```

Dataset:

> KubeEdge-Ianvs KITTI Pose Estimation Dataset (Kaggle)

---

# License

### Code

Choose one:

- MIT License
- Apache-2.0 License

### Dataset

The dataset is distributed under its own license.

Please follow the Kaggle/KITTI/CC BY-NC-SA terms and **do not redistribute the raw data**.

---

## Acknowledgements

- KITTI Vision Benchmark Suite
- KubeEdge-Ianvs
- Open3D
- NumPy
- SciPy