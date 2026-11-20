#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/setup_humble.bash"
exec /usr/bin/python3 "$workspace_dir/scripts/apriltag_overlay.py" "$@"
