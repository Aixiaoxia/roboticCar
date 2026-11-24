#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/setup_humble.bash"

# Initial mount estimate: 90 mm forward, 539 mm above base_link.
# The camera is assumed level and facing forward; refine after validation.
exec ros2 run tf2_ros static_transform_publisher \
  --x 0.090 --y 0.0 --z 0.539 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id camera_link
