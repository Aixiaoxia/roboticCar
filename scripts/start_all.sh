#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/setup_humble.bash"
exec ros2 launch bunker_tag_follower bringup.launch.py "config:=$workspace_dir/follow_config.yaml" "$@"
