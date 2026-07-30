import numpy as np

class EKF2D:
    def __init__(self):
        self.x = np.zeros((4, 1))
        self.p = np.diag([1.0, 1.0, 0.1, 1.0])
        self.q = np.diag([0.05, 0.05, 0.01, 0.2])
        self.r_gps = np.diag([0.5, 0.5])
        self.r_speed = np.array([[0.5]])
        self.r_yaw = np.array([[0.05]])

    def set_initial(self, x, y, yaw, v):
        self.x[:] = np.array([[x], [y], [yaw], [v]])

    def predict(self, dt, af, wu):
        x, y, yaw, v = self.x.ravel()
        v = max(v + af * dt, 0.0)
        yaw = yaw + wu * dt
        x = x + v * np.cos(yaw) * dt
        y = y + v * np.sin(yaw) * dt
        self.x[:] = np.array([[x], [y], [yaw], [v]])
        f = np.eye(4)
        f[0, 2] = -v * np.sin(yaw) * dt
        f[0, 3] = np.cos(yaw) * dt
        f[1, 2] = v * np.cos(yaw) * dt
        f[1, 3] = np.sin(yaw) * dt
        self.p = f @ self.p @ f.T + self.q

    def _update(self, z, h, r):
        y = z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.p = (np.eye(4) - k @ h) @ self.p

    def update_gps(self, gx, gy):
        h = np.zeros((2, 4)); h[0, 0] = 1.0; h[1, 1] = 1.0
        self._update(np.array([[gx], [gy]]), h, self.r_gps)

    def update_speed(self, speed):
        h = np.zeros((1, 4)); h[0, 3] = 1.0
        self._update(np.array([[speed]]), h, self.r_speed)

    def update_yaw(self, yaw):
        h = np.zeros((1, 4)); h[0, 2] = 1.0
        self._update(np.array([[yaw]]), h, self.r_yaw)

    def xy(self):
        return float(self.x[0, 0]), float(self.x[1, 0])
