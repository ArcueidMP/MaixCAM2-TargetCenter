"""Constant-velocity center filter. Raw observations remain the control input."""
import numpy as np


class ConstantVelocityKalman2D:
    def __init__(self, process_px=40., measure_px=.9, reset_px=25.,
                 max_predict_s=.25, velocity_limit_px_s=6000.):
        self.q, self.r = float(process_px), float(measure_px)
        self.reset_px, self.max_gap = float(reset_px), float(max_predict_s)
        self.velocity_limit = float(velocity_limit_px_s)
        if not np.isfinite([self.q, self.r, self.reset_px, self.max_gap, self.velocity_limit]).all() or min(self.r, self.max_gap, self.velocity_limit) <= 0 or self.q < 0:
            raise ValueError("invalid Kalman parameters")
        self.reset()

    def reset(self):
        self.x = self.p = self.timestamp = self.last_measurement = None

    def seed(self, position, timestamp):
        z = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(z).all() or not np.isfinite(timestamp):
            raise ValueError("non-finite observation")
        self.x = np.r_[z, 0., 0.]
        self.p = np.diag([self.r**2]*2 + [(self.r*30)**2]*2)
        self.timestamp, self.last_measurement = float(timestamp), z.copy()
        return z.copy()

    def update(self, position, timestamp):
        z = np.asarray(position, dtype=float).reshape(2)
        if not np.isfinite(z).all() or not np.isfinite(timestamp):
            raise ValueError("non-finite observation")
        dt = 0 if self.timestamp is None else timestamp - self.timestamp
        if self.x is None or not 0 < dt <= self.max_gap or np.linalg.norm(z-self.last_measurement) > self.reset_px:
            return self.seed(z, timestamp)
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        a = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
        x = f @ self.x
        p = f @ self.p @ f.T + self.q**2 * (a @ a.T)
        k = np.linalg.solve(p[:2, :2] + np.eye(2)*self.r**2, p[:, :2].T).T
        self.x = x + k @ (z-x[:2])
        # Joseph form keeps the covariance positive even after long runs.
        ikh = np.eye(4)
        ikh[:, :2] -= k
        p = ikh @ p @ ikh.T + self.r**2*(k @ k.T)
        self.p = .5*(p+p.T)
        speed = np.linalg.norm(self.x[2:])
        if speed > self.velocity_limit:
            self.x[2:] *= self.velocity_limit/speed
        self.timestamp, self.last_measurement = float(timestamp), z.copy()
        return self.x[:2].copy()

    def predict(self, timestamp):
        if self.x is None or not np.isfinite(timestamp):
            return None
        dt = timestamp-self.timestamp
        if not 0 <= dt <= self.max_gap:
            return None
        return self.x[:2] + self.x[2:]*dt
