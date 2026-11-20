"""Regression checks for steering signs, no reverse, dropout and watchdog braking."""
import math
import pytest
from bunker_tag_follower.policy import (
    Command, FollowConfig, VelocityGuard, follow_target, optical_to_planar)


@pytest.mark.parametrize('x,expected', [(0.3, -0.3), (-0.3, 0.3)])
def test_handheld_optical_axes(x, expected):
    assert optical_to_planar(x, 0.2, 2.0) == (2.0, expected)


@pytest.mark.parametrize('forward,left', [(0, 0), (-1, 0), (math.nan, 0), (1, math.inf)])
def test_invalid_or_behind_stops(forward, left):
    assert follow_target(forward, left, FollowConfig())[0] == Command()


@pytest.mark.parametrize('distance', [0.2, 0.4, 0.5, 0.57, 5.0])
def test_close_hold_or_far_stops(distance):
    assert follow_target(distance, 0, FollowConfig())[0] == Command()


def test_centered_target_forward_with_limit():
    command, state = follow_target(2, 0, FollowConfig())
    assert command == Command(0.05, 0)
    assert state == 'following'


@pytest.mark.parametrize('left,sign', [(1.0, 1), (-1.0, -1)])
def test_large_heading_turns_only(left, sign):
    command, state = follow_target(2, left, FollowConfig())
    assert command.linear == 0
    assert command.angular * sign > 0
    assert abs(command.angular) <= 0.1
    assert state == 'aligning'


def test_configuration_validation():
    with pytest.raises(ValueError):
        FollowConfig(minimum_distance=2)
    with pytest.raises(ValueError):
        FollowConfig(max_linear=math.nan)


def ready_guard(clearance=False):
    guard = VelocityGuard(require_clearance=clearance)
    guard.receive(Command(1, 1), 10, 20)
    return guard


def test_disabled_until_explicit_arm():
    guard = ready_guard()
    assert guard.tick(10, 20)[0] == Command()


def test_acceleration_limit_and_immediate_braking():
    guard = ready_guard()
    assert guard.arm(True, 10, 20)[0]
    command, _ = guard.tick(10.1, 20.1)
    assert command.linear == pytest.approx(0.003)
    assert command.angular == pytest.approx(0.02)
    guard.receive(Command(), 10.1, 20.1)
    assert guard.tick(10.11, 20.11)[0] == Command()


def test_watchdog_stops_with_frozen_ros_clock_and_disarms():
    guard = ready_guard()
    guard.arm(True, 10, 20)
    assert guard.tick(10, 20.31)[0] == Command()
    assert not guard.enabled
    guard.receive(Command(0.05, 0), 10, 20.32)
    assert guard.tick(10, 20.32)[0] == Command()


@pytest.mark.parametrize('stamp', [9.0, 11.0])
def test_delayed_or_future_commands_cannot_arm(stamp):
    guard = ready_guard()
    guard.receive(Command(.05, 0), stamp, 20)
    assert not guard.arm(True, 10, 20)[0]


def test_hardware_clearance_must_be_fresh_and_true():
    guard = ready_guard(True)
    assert not guard.arm(True, 10, 20)[0]
    guard.path_clear(True, 20)
    assert guard.arm(True, 10, 20)[0]
    guard.path_clear(False, 20.1)
    assert guard.tick(10.1, 20.1)[0] == Command()
    assert not guard.enabled


def test_nonfinite_input_disarms():
    guard = ready_guard()
    guard.arm(True, 10, 20)
    guard.receive(Command(math.nan, 0), 10, 20)
    assert guard.tick(10, 20)[0] == Command()
    assert not guard.enabled


def test_guard_never_reverses():
    guard = VelocityGuard()
    guard.receive(Command(-1, 0), 10, 20)
    guard.arm(True, 10, 20)
    assert guard.tick(10.1, 20.1)[0].linear == 0
