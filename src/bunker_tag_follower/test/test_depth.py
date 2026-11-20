import numpy as np
from bunker_tag_follower.depth_policy import assess_depth


def test_clear_scene():
    assert assess_depth(np.full((120, 160), 2.0))[0]


def test_near_object_blocks_even_with_good_coverage():
    depth = np.full((120, 160), 2.0)
    depth[60, 80] = .48
    assert assess_depth(depth)[1] == 'near_obstacle'


def test_invalid_depth_cannot_authorize_motion():
    for value in (0., np.nan, np.inf, 9.):
        assert not assess_depth(np.full((120, 160), value))[0]


def test_blind_sector_blocks():
    depth = np.full((120, 160), 2.0)
    depth[12:42, 0:40] = 0.
    assert assess_depth(depth)[1] == 'blind_sector'


def test_twenty_percent_sector_uses_configured_threshold():
    depth = np.full((120, 160), 2.0)
    depth[12:42, :40] = 0.
    depth[12:18, :40] = 2.
    assert assess_depth(depth, minimum_tile_valid_fraction=.15)[0]
    assert assess_depth(depth, minimum_tile_valid_fraction=.25)[1] == 'blind_sector'


def test_disable_only_sector_check():
    depth = np.full((120, 160), 2.)
    depth[12:42, :40] = 0.
    assert assess_depth(depth)[1] == 'blind_sector'
    assert assess_depth(depth, check_blind_sectors=False)[0]
    depth[60, 80] = .4
    assert assess_depth(depth, check_blind_sectors=False)[1] == 'near_obstacle'
    assert assess_depth(np.zeros((120, 160)), check_blind_sectors=False)[1] == 'insufficient_depth'


def test_current_site_twenty_percent_and_point_two_meter_limits():
    depth = np.zeros((120, 160))
    # 25% of every row is valid, while the other regions contain missing data.
    depth[:, :40] = 2.
    limits = dict(stop_distance=.2, minimum_valid_fraction=.2, check_blind_sectors=False)
    assert assess_depth(depth, **limits)[0]
    depth[:, 30:40] = 0.
    assert assess_depth(depth, **limits)[1] == 'insufficient_depth'
    depth[:, :40] = 2.
    depth[60, 20] = .19
    assert assess_depth(depth, **limits)[1] == 'near_obstacle'
    depth[60, 20] = .21
    assert assess_depth(depth, **limits)[0]
