"""Discrete PI controllers ported from simulation/speed.slx."""
from dataclasses import dataclass
from math import atan2, hypot, isfinite

from .policy import Command


@dataclass(frozen=True)
class PIConfig:
    controller_mode: str = 'proportional'
    distance_kp: float = 0.6
    distance_ki: float = 0.12
    heading_kp: float = 2.0
    heading_ki: float = 0.02
    linear_speed_kp: float = 0.5
    linear_speed_ki: float = 3.0
    angular_speed_kp: float = 0.4
    angular_speed_ki: float = 4.0
    speed_feedback_timeout: float = 0.2

    def __post_init__(self):
        if self.controller_mode not in ('proportional', 'outer_pi', 'cascade_pi'):
            raise ValueError('controller_mode must be proportional, outer_pi or cascade_pi')
        for name, value in vars(self).items():
            if name == 'controller_mode':
                continue
            if isinstance(value, bool) or not isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.speed_feedback_timeout == 0:
            raise ValueError('speed_feedback_timeout must be positive')


class PI:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, error, dt, lower, upper):
        proportional = self.kp * error
        output = proportional + self.integral
        increment = self.ki * error * dt
        # Allow the integral to unwind, while preventing growth into saturation.
        if increment > 0.0:
            self.integral += min(increment, max(0.0, upper - output))
        else:
            self.integral += max(increment, min(0.0, lower - output))
        return max(lower, min(upper, proportional + self.integral))


class FollowPI:
    def __init__(self, limits, config):
        self.limits = limits
        self.cascade = config.controller_mode == 'cascade_pi'
        self.distance = PI(config.distance_kp, config.distance_ki)
        self.heading = PI(config.heading_kp, config.heading_ki)
        self.linear_speed = PI(config.linear_speed_kp, config.linear_speed_ki)
        self.angular_speed = PI(config.angular_speed_kp, config.angular_speed_ki)

    def reset(self):
        self.distance.reset()
        self.heading.reset()
        self.linear_speed.reset()
        self.angular_speed.reset()

    def step(self, forward, left, reason, linear_dt, angular_dt, feedback=None):
        """Return the Bunker command, outer-loop reference and tracking status."""
        if reason not in ('following', 'aligning'):
            self.reset()
            return Command(), Command(), reason
        if self.cascade and (feedback is None or not all(isfinite(v) for v in (
                feedback.linear, feedback.angular))):
            self.reset()
            return Command(), Command(), 'waiting_for_speed_feedback'

        limits = self.limits
        if reason == 'aligning':
            self.distance.reset()
            self.linear_speed.reset()
            linear = 0.0
        else:
            error = hypot(forward, left) - limits.desired_distance - limits.distance_deadband
            linear = self.distance.step(error, linear_dt, 0.0, limits.max_linear)

        angle = atan2(left, forward)
        if abs(angle) <= limits.heading_deadband:
            self.heading.reset()
            self.angular_speed.reset()
            angular = 0.0
        else:
            if angle * self.heading.integral < 0.0:
                self.heading.reset()
                self.angular_speed.reset()
            angular = self.heading.step(angle, angular_dt, -limits.max_angular, limits.max_angular)

        reference = Command(linear, angular)
        if not self.cascade:
            return reference, reference, reason

        if linear != 0.0:
            linear = self.linear_speed.step(
                linear - feedback.linear, linear_dt, 0.0, limits.max_linear)
        else:
            self.linear_speed.reset()
        if angular != 0.0:
            angular = self.angular_speed.step(
                angular - feedback.angular, angular_dt, -limits.max_angular, limits.max_angular)
        return Command(linear, angular), reference, reason
