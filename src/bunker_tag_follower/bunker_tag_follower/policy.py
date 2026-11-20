"""Deterministic control and watchdog rules, independent of ROS and hardware."""
from dataclasses import dataclass
from math import atan2, hypot, isfinite


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class FollowConfig:
    desired_distance: float = 0.5
    distance_deadband: float = 0.08
    minimum_distance: float = 0.35
    maximum_distance: float = 4.0
    heading_deadband: float = 0.06
    turn_only_angle: float = 0.25
    linear_gain: float = 0.2
    angular_gain: float = 0.6
    max_linear: float = 0.05
    max_angular: float = 0.1

    def __post_init__(self):
        if not all(isfinite(x) and x > 0 for x in vars(self).values()):
            raise ValueError('Control parameters must be finite and positive')
        if not self.minimum_distance < self.desired_distance < self.maximum_distance:
            raise ValueError(
                '距离参数冲突：必须满足 minimum_distance < desired_distance < maximum_distance；'
                f'当前 minimum_distance={self.minimum_distance}, '
                f'desired_distance={self.desired_distance}, maximum_distance={self.maximum_distance}')
        if self.distance_deadband >= self.desired_distance - self.minimum_distance:
            raise ValueError(
                '距离死区冲突：必须满足 desired_distance > minimum_distance + distance_deadband；'
                f'当前 {self.desired_distance} <= {self.minimum_distance} + {self.distance_deadband}')
        if not self.heading_deadband < self.turn_only_angle < 1.57:
            raise ValueError('Require heading deadband < turn-only angle < pi/2')


def follow_target(forward, left, config):
    """Positive angular speed turns left. Never reverse or pursue behind the robot."""
    if not all(isfinite(x) for x in (forward, left)) or forward <= 0:
        return Command(), 'invalid_or_behind'
    distance = hypot(forward, left)
    if distance < config.minimum_distance:
        return Command(), 'too_close'
    if distance > config.maximum_distance:
        return Command(), 'out_of_range'
    angle = atan2(left, forward)
    # Once inside the distance band, hold both axes instead of shuffling in place.
    if distance <= config.desired_distance + config.distance_deadband:
        return Command(), 'holding_distance'
    angular = 0.0 if abs(angle) <= config.heading_deadband else max(
        -config.max_angular, min(config.max_angular, config.angular_gain * angle))
    if abs(angle) >= config.turn_only_angle:
        return Command(0.0, angular), 'aligning'
    linear = min(config.max_linear, config.linear_gain * (
        distance - config.desired_distance - config.distance_deadband))
    return Command(linear, angular), 'following'


def optical_to_planar(x, y, z):
    """Handheld simulation only: camera right/down/forward -> forward/left."""
    if not all(isfinite(v) for v in (x, y, z)):
        return float('nan'), float('nan')
    return z, -x


class VelocityGuard:
    """Manual enable, dual-clock expiry, limits and independent heartbeat watchdog."""
    def __init__(self, timeout=0.3, max_linear=0.05, max_angular=0.1,
                 linear_accel=0.03, angular_accel=0.2, require_clearance=False):
        if not all(isfinite(v) and v > 0 for v in (
                timeout, max_linear, max_angular, linear_accel, angular_accel)):
            raise ValueError('Guard limits must be finite and positive')
        self.timeout = timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self.linear_accel, self.angular_accel = linear_accel, angular_accel
        self.require_clearance = require_clearance
        self.enabled = False
        self.sample = None
        self.clearance = None
        self.output = Command()
        self.last_tick = None

    def receive(self, command, stamp, monotonic_now):
        if not all(isfinite(v) for v in (command.linear, command.angular, stamp)):
            self.sample = None
            self.enabled = False
            return
        self.sample = (command, stamp, monotonic_now)

    def path_clear(self, clear, monotonic_now):
        self.clearance = (clear, monotonic_now)

    def readiness(self, ros_now, monotonic_now):
        if self.sample is None:
            return 'no_command'
        _, stamp, received = self.sample
        if not (-0.05 <= ros_now - stamp <= self.timeout):
            return 'stale_command_stamp'
        if not (0 <= monotonic_now - received <= self.timeout):
            return 'command_watchdog'
        if self.require_clearance:
            if (self.clearance is None or not self.clearance[0]
                    or not 0 <= monotonic_now - self.clearance[1] <= self.timeout):
                return 'path_not_confirmed_clear'
        return None

    def arm(self, enable, ros_now, monotonic_now):
        reason = self.readiness(ros_now, monotonic_now) if enable else None
        self.enabled = enable and reason is None
        self.output = Command()
        self.last_tick = monotonic_now
        return self.enabled if enable else True, reason or ('enabled' if enable else 'disabled')

    def tick(self, ros_now, monotonic_now):
        dt = 0.0 if self.last_tick is None else min(0.1, max(0.0, monotonic_now-self.last_tick))
        self.last_tick = monotonic_now
        reason = self.readiness(ros_now, monotonic_now)
        if not self.enabled or reason:
            if reason:
                self.enabled = False
            self.output = Command()
            return self.output, reason or 'disabled'
        desired = self.sample[0]
        linear = max(0.0, min(self.max_linear, desired.linear))
        angular = max(-self.max_angular, min(self.max_angular, desired.angular))
        # Brake immediately on either axis; acceleration is bounded on other changes.
        def limit(current, target, accel):
            if target == 0:
                return 0.0
            return current + max(-accel*dt, min(accel*dt, target-current))
        self.output = Command(limit(self.output.linear, linear, self.linear_accel),
                              limit(self.output.angular, angular, self.angular_accel))
        return self.output, 'enabled'
