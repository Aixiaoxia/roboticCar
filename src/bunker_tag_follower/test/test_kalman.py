import numpy as np
import pytest
from bunker_tag_follower.kalman import PlanarKalmanFilter


def test_first_frame_has_no_startup_delay():
    assert PlanarKalmanFilter().update(2., -.3, 10.) == (2., -.3, 0., 0.)


def test_reduces_stationary_measurement_jitter():
    rng = np.random.default_rng(23)
    noise = rng.normal(0, .04, size=(400, 2))
    measurements = noise + [2., .1]
    kf = PlanarKalmanFilter()
    output = np.array([kf.update(*p, i*.05)[:2] for i, p in enumerate(measurements)])
    assert np.mean((output[40:]-[2., .1])**2) < np.mean(noise[40:]**2)*.75


def test_tracks_velocity_with_variable_frame_intervals():
    kf = PlanarKalmanFilter()
    stamp = 0.
    for i in range(100):
        stamp += .035 if i % 2 else .08
        state = kf.update(2+.2*stamp, -.3+.1*stamp, stamp)
    assert state == pytest.approx((2+.2*stamp, -.3+.1*stamp, .2, .1), abs=.01)
    assert np.linalg.eigvalsh(kf.covariance).min() >= -1e-10


def test_reset_or_gap_reinitializes_without_old_velocity():
    kf = PlanarKalmanFilter()
    kf.update(2, 0, 1.)
    kf.update(2.1, .1, 1.05)
    assert kf.update(1., -.5, 2.) == (1., -.5, 0., 0.)
    kf.reset()
    assert kf.update(3., .2, .5) == (3., .2, 0., 0.)


@pytest.mark.parametrize('position', [(float('nan'), 0), (0, float('inf'))])
def test_invalid_measurement_does_not_change_filter(position):
    kf = PlanarKalmanFilter()
    kf.update(2, 0, 1.)
    with pytest.raises(ValueError):
        kf.update(*position, 1.05)
    assert kf.stamp == 1.


@pytest.mark.parametrize('stamp', [1., .5])
def test_replayed_timestamp_does_not_advance_filter(stamp):
    kf = PlanarKalmanFilter()
    kf.update(2, 0, 1.)
    with pytest.raises(ValueError):
        kf.update(3, 0, stamp)
    assert tuple(kf.state) == (2., 0., 0., 0.)


def test_noise_settings_must_be_positive_and_finite():
    with pytest.raises(ValueError):
        PlanarKalmanFilter(position_std=0)
    with pytest.raises(ValueError):
        PlanarKalmanFilter(acceleration_std=float('nan'))
