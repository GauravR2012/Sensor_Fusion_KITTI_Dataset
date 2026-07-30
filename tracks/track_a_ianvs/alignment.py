import numpy as np
def umeyama_align(src, tgt):
    """Rigid 2D/ND align: maps rows of src to tgt (same length)."""
    mu_s = src.mean(axis=0)
    mu_t = tgt.mean(axis=0)
    src_c = src - mu_s
    tgt_c = tgt - mu_t
    cov = (tgt_c.T @ src_c) / len(src)
    U, _, Vt = np.linalg.svd(cov)
    S = np.diag([1.0, np.linalg.det(U @ Vt)])
    R = U @ S @ Vt
    t = mu_t - R @ mu_s
    return (src @ R.T) + t
src/ekf.py
import numpy as np
from src import config as cfg
class EKF:
    def __init__(self):
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 0.1
        self.Q = np.diag(
            [cfg.Q_POS_NOISE, cfg.Q_POS_NOISE, cfg.Q_YAW_NOISE, cfg.Q_VEL_NOISE]
        )
        self.R_gps = np.diag([cfg.R_GPS_NOISE, cfg.R_GPS_NOISE])
        self.R_speed = np.array([[cfg.R_SPEED_NOISE]])
    def predict(self, dt, accel, yaw_rate):
        x, y, yaw, v = self.x.flatten()
        self.x = np.array(
            [
                [x + v * np.cos(yaw) * dt],
                [y + v * np.sin(yaw) * dt],
                [yaw + yaw_rate * dt],
                [v + accel * dt],
            ]
        )
        F = np.array(
            [
                [1, 0, -v * np.sin(yaw) * dt, np.cos(yaw) * dt],
                [0, 1, v * np.cos(yaw) * dt, np.sin(yaw) * dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        self.P = F @ self.P @ F.T + self.Q * dt
    def _update(self, z, H, R):
        innov = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    def update_gps(self, gx, gy):
        z = np.array([[gx], [gy]])
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self._update(z, H, self.R_gps)
    def update_lidar(self, lx, ly, fitness):
        noise = max(cfg.LIDAR_NOISE_BASE / (fitness + 1e-6), cfg.LIDAR_NOISE_MIN)
        R_lidar = np.diag([noise, noise])
        z = np.array([[lx], [ly]])
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self._update(z, H, R_lidar)
    def update_speed(self, vn, ve):
        speed = np.sqrt(vn**2 + ve**2)
        z = np.array([[speed]])
        H = np.array([[0, 0, 0, 1]])
        self._update(z, H, self.R_speed)
    def update_yaw(self, yaw_meas):
        H = np.array([[0, 0, 1, 0]])
        innov = np.array([[yaw_meas]]) - H @ self.x
        innov[0, 0] = (innov[0, 0] + np.pi) % (2 * np.pi) - np.pi
        R_yaw = np.array([[cfg.R_YAW_MEAS]])
        S = H @ self.P @ H.T + R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_yaw @ K.T