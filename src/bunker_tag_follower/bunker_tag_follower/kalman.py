"""Linear constant-velocity KF for measured planar relative target positions."""
import math
import numpy as np


class PlanarKalmanFilter:
    """State: [forward, left, relative_forward_velocity, relative_left_velocity].

    Predict only to a new measurement's timestamp, then correct. The controller
    never receives extrapolated positions after measurements stop arriving.
    """
    def __init__(self, position_std=.04, acceleration_std=2.,
                 initial_velocity_std=1., max_gap=.25):
        if not all(math.isfinite(v) and v > 0 for v in (
                position_std, acceleration_std, initial_velocity_std, max_gap)):
            raise ValueError('KF noise standard deviations and max_gap must be positive and finite')
        self.position_variance = position_std**2
        self.acceleration_variance = acceleration_std**2
        self.velocity_variance = initial_velocity_std**2
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.state = None
        self.covariance = None
        self.stamp = None

    def update(self, forward, left, stamp):
        if not all(math.isfinite(v) for v in (forward, left, stamp)):
            raise ValueError('KF measurement and timestamp must be finite')
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError('KF timestamps must increase')
        if self.state is None or stamp-self.stamp > self.max_gap:
            self.state = np.array([forward, left, 0., 0.], dtype=float)
            self.covariance = np.diag([self.position_variance]*2+[self.velocity_variance]*2)
        else:
            dt = stamp-self.stamp
            transition = np.eye(4)
            transition[0, 2] = transition[1, 3] = dt
            acceleration = np.array([[dt*dt/2, 0], [0, dt*dt/2], [dt, 0], [0, dt]])
            process_noise = self.acceleration_variance * acceleration @ acceleration.T
            predicted = transition @ self.state
            covariance = transition @ self.covariance @ transition.T + process_noise
            innovation_covariance = covariance[:2, :2] + np.eye(2)*self.position_variance
            gain = np.linalg.solve(innovation_covariance, covariance[:2, :]).T
            self.state = predicted + gain @ (np.array([forward, left])-predicted[:2])
            # Joseph covariance update avoids loss of positive semidefiniteness.
            correction = np.eye(4)
            correction[:, :2] -= gain
            self.covariance = (correction @ covariance @ correction.T
                               + self.position_variance * gain @ gain.T)
            self.covariance = (self.covariance+self.covariance.T)/2
        self.stamp = stamp
        return tuple(float(v) for v in self.state)
