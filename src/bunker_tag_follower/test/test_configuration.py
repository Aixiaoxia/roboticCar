import pytest
from bunker_tag_follower.configuration import validate_follow_configuration


def write_config(tmp_path, body):
    path = tmp_path/'follow.yaml'
    path.write_text(body, encoding='utf-8')
    return path


def test_valid_config_uses_shared_limits(tmp_path):
    path = write_config(tmp_path, '''/**:
  # 跟随限速
  ros__parameters: {max_linear: 1.0, max_angular: 0.5}
tag_follower:
  ros__parameters: {desired_distance: 0.5, minimum_distance: 0.35}
''')
    config = validate_follow_configuration(path)
    assert config.max_linear == 1. and config.max_angular == .5


def test_conflicting_distance_reports_names_and_values(tmp_path):
    path = write_config(tmp_path, '''tag_follower:
  ros__parameters: {desired_distance: 0.3, minimum_distance: 0.35}
''')
    with pytest.raises(ValueError, match='minimum_distance=0.35.*desired_distance=0.3'):
        validate_follow_configuration(path)


def test_deadband_overlap_is_rejected(tmp_path):
    path = write_config(tmp_path, '''tag_follower:
  ros__parameters: {desired_distance: 0.4, minimum_distance: 0.35, distance_deadband: 0.08}
''')
    with pytest.raises(ValueError, match='distance_deadband'):
        validate_follow_configuration(path)


def test_bad_yaml_has_file_context(tmp_path):
    path = write_config(tmp_path, 'tag_follower: [broken')
    with pytest.raises(ValueError, match='follow.yaml'):
        validate_follow_configuration(path)


@pytest.mark.parametrize('bad_parameter, reason', [
    ('controller_mode: unknown', 'controller_mode'),
    ('distance_ki: -0.1', 'distance_ki'),
    ('speed_feedback_timeout: 0.0', 'speed_feedback_timeout'),
])
def test_bad_pi_configuration_is_rejected_before_launch(tmp_path, bad_parameter, reason):
    path = write_config(tmp_path, f'tag_follower:\n  ros__parameters:\n    {bad_parameter}\n')
    with pytest.raises(ValueError, match=reason):
        validate_follow_configuration(path)
