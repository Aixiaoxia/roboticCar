"""Validate distance/control configuration before starting hardware processes."""
from dataclasses import fields
from pathlib import Path
import argparse

import yaml

from .policy import FollowConfig


def validate_follow_configuration(filename):
    path = Path(filename).expanduser()
    try:
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('YAML root must be a mapping')
        shared = data.get('/**', {}).get('ros__parameters', {})
        specific = data.get('tag_follower', {}).get('ros__parameters', {})
        merged = {**shared, **specific}
        controls = {field.name: merged[field.name] for field in fields(FollowConfig)
                    if field.name in merged}
        return FollowConfig(**controls)
    except (OSError, yaml.YAMLError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(f'配置文件错误：{path}\n{error}') from error


def main():
    parser = argparse.ArgumentParser(description='检查跟随参数，不启动相机或底盘')
    parser.add_argument('config')
    args = parser.parse_args()
    try:
        config = validate_follow_configuration(args.config)
    except ValueError as error:
        parser.exit(2, str(error)+'\n')
    print(f'配置检查通过：保持距离 {config.desired_distance} m，最小距离 '
          f'{config.minimum_distance} m，最大距离 {config.maximum_distance} m，'
          f'前进上限 {config.max_linear} m/s')


if __name__ == '__main__':
    main()
