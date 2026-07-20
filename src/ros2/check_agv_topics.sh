#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

ros2 topic list | sort | grep -E '^/(ugv/)?airsim_node/(AGV_1|origin_geo_point|instance_segmentation_labels|object_transforms)'
