#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <measured_tag_size_in_meters> [launch arguments...]" >&2
  echo "Example: $0 0.15 (ONLY if the printed black square measures 15 cm)" >&2
  exit 2
fi
if ! /usr/bin/python3 -c 'import math,sys; x=float(sys.argv[1]); sys.exit(0 if math.isfinite(x) and x>0 else 1)' "$1"; then
  echo 'Tag size must be a positive finite number in meters.' >&2
  exit 2
fi
tag_size="$1"
shift
source "$workspace_dir/setup_humble.bash"
exec ros2 launch "$workspace_dir/launch/apriltag_astra.launch.py" "tag_size:=$tag_size" "$@"
