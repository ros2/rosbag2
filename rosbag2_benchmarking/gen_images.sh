#!/usr/bin/env bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

TMP_DIR=$SCRIPTPATH/tmp
mkdir -p $TMP_DIR
echo $1 >> "$TMP_DIR/count"

ros2 run rosbag2_performance_workers image_worker --ros-args -p max_count:=$1 -p dt:=$2 -p dimensions:=$3