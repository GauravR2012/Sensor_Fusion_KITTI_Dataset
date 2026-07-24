"""
3D Extended Kalman Filter

State:
    x = [px, py, pz, vx, vy, vz]
"""

import numpy as np


class EKF3D:
    def __init__(
        self,
        q_pos=0.05,
        q_vel=0.8,
        r_pos=1.5,
        r_vel=0.25,
    ):
        self.x = np.zeros((6, 1))

        self.P = np.eye(6) * 0.1

        self.Q = np.diag([
            q_pos,
            q_pos,
            q_pos,
            q_vel,
            q_vel,
            q_vel,
        ])

        self.R_pos = np.diag([
            r_pos,
            r_pos,
            r_pos,
        ])

        self.R_vel = np.diag([
            r_vel,
            r_vel,
            r_vel,
        ])

    # --------------------------------------------------
    # Prediction
    # --------------------------------------------------

    def predict(self, dt, accel):

        accel = np.asarray(accel).reshape(3)

        p = self.x[:3, 0]
        v = self.x[3:, 0]

        p = p + v * dt
        v = v + accel * dt

        self.x[:3, 0] = p
        self.x[3:, 0] = v

        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        self.P = F @ self.P @ F.T + self.Q * dt

    # --------------------------------------------------
    # Generic update
    # --------------------------------------------------

    def _update(self, z, H, R):

        z = np.asarray(z).reshape((-1, 1))

        innovation = z - H @ self.x

        S = H @ self.P @ H.T + R

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation

        I = np.eye(6)

        IKH = I - K @ H

        # Joseph stabilized covariance update
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

    # --------------------------------------------------
    # Position update
    # --------------------------------------------------

    def update_position(self, xyz, R=None):

        if R is None:
            R = self.R_pos

        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        self._update(xyz, H, R)

    # --------------------------------------------------
    # Velocity update
    # --------------------------------------------------

    def update_velocity(self, vel, R=None):

        if R is None:
            R = self.R_vel

        H = np.zeros((3, 6))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 5] = 1

        self._update(vel, H, R)

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    @property
    def position(self):
        return self.x[:3, 0].copy()

    @property
    def velocity(self):
        return self.x[3:, 0].copy()

    def reset(self):

        self.x[:] = 0
        self.P = np.eye(6) * 0.1