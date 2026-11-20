#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/setup_humble.bash"
exec /usr/bin/python3 -m bunker_tag_follower.configuration "${1:-$workspace_dir/follow_config.yaml}"
