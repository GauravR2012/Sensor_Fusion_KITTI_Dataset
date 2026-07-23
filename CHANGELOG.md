## v1.0.0
- Final pipeline: GICP, ground removal, ICP gates, IMU init, global Umeyama, EKF (GPS, adaptive LiDAR, speed, yaw).
- CLI: single drive (`--drive`) and all drives (`--all`).
- Modular `src/` layout.
## Prior work (Kaggle notebook, not in repo)
- Origin-only LiDAR alignment, point-to-point ICP, GPS-only EKF updates, iterative tuning documented in development notebook.