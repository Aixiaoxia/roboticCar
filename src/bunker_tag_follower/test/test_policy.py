"""Regression checks for steering signs, no reverse, dropout and watchdog braking."""
import math
import pytest
from bunker_tag_follower.follow_pi import FollowPI, PI, PIConfig
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


def test_pi_integral_stops_at_limit_and_can_unwind():
    pi = PI(kp=0.0, ki=1.0)
    assert pi.step(2.0, 1.0, -0.5, 0.5) == 0.5
    assert pi.step(2.0, 1.0, -0.5, 0.5) == 0.5
    assert pi.step(-0.1, 1.0, -0.5, 0.5) == pytest.approx(0.4)
    assert pi.step(-2.0, 1.0, -0.5, 0.5) == -0.5
    assert pi.step(2.0, 0.0, -0.5, 0.5) == -0.5


def test_outer_pi_accumulates_and_resets_after_loss():
    limits = FollowConfig(max_linear=1.0)
    controller = FollowPI(limits, PIConfig(controller_mode='outer_pi'))
    forward = 0.7
    _, reason = follow_target(forward, 0.0, limits)
    first, reference, state = controller.step(forward, 0.0, reason, 0.05, 0.05)
    assert first == reference
    assert state == 'following'
    assert first.linear == pytest.approx((0.6 + 0.12 * 0.05) * (0.7 - 0.5 - 0.08))
    second, _, _ = controller.step(forward, 0.0, reason, 0.05, 0.05)
    assert second.linear > first.linear > 0.0 and second.angular == 0.0
    assert controller.step(None, None, 'target_timeout', 0.05, 0.05)[0] == Command()
    assert controller.step(forward, 0.0, reason, 0.05, 0.05)[0] == first


@pytest.mark.parametrize('mode', ['outer_pi', 'cascade_pi'])
def test_pi_alignment_keeps_linear_zero_and_matches_ros_turn_sign(mode):
    limits = FollowConfig(max_angular=0.5)
    controller = FollowPI(limits, PIConfig(controller_mode=mode))
    for left in (1.0, -1.0):
        _, reason = follow_target(2.0, left, limits)
        command, _, _ = controller.step(2.0, left, reason, 0.05, 0.05, Command(0.1, 0.0))
        assert reason == 'aligning'
        assert command.linear == 0.0
        assert command.angular * left > 0.0


def test_cascade_uses_bunker_feedback_and_resets_when_it_expires():
    limits = FollowConfig(max_linear=1.0, max_angular=0.5)
    controller = FollowPI(limits, PIConfig(controller_mode='cascade_pi'))
    first, reference, _ = controller.step(0.7, 0.0, 'following', 0.05, 0.05, Command())
    assert first.linear == pytest.approx(reference.linear * (0.5 + 3.0 * 0.05))
    faster, _, _ = controller.step(0.7, 0.0, 'following', 0.05, 0.05, Command(0.2, 0.0))
    assert faster == Command()
    assert controller.step(0.7, 0.0, 'following', 0.05, 0.05) == (
        Command(), Command(), 'waiting_for_speed_feedback')
    assert controller.step(0.7, 0.0, 'following', 0.05, 0.05, Command())[0] == first
