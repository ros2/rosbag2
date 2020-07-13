#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# REPORT_DIR=~/.ros/reports

REPORT_DIR=$1

if [ -z "$REPORT_DIR" ]; then
    echo "Usage: ./voyager REPORT_DIR"
    exit 1
fi

if [ ! -d "$REPORT_DIR" ]; then
    echo "$REPORT_DIR does not exist"
    exit 1
fi

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_1.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_10.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_100.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_1000.yaml report_dir:="$REPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_1.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_10.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_100.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_1000.yaml report_dir:="$REPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_1.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_10.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_100.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_1000.yaml report_dir:="$REPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_1.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_10.yaml report_dir:="$REPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_100.yaml report_dir:="$REPORT_DIR"
#ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_1000.yaml report_dir:="$REPORT_DIR"

######################################################################################################################

ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_1.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_10.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_100.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_1000.yaml -p report_dir:="$REPORT_DIR"

ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_1.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_10.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_100.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_1000.yaml -p report_dir:="$REPORT_DIR"

ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_1.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_10.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_100.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_1000.yaml -p report_dir:="$REPORT_DIR"

ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_1.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_10.yaml -p report_dir:="$REPORT_DIR"
ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_100.yaml -p report_dir:="$REPORT_DIR"
#ros2 run rosbag2_benchmarking report_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_1000.yaml -p report_dir:="$REPORT_DIR"

######################################################################################################################

ros2 run rosbag2_benchmarking voyager --ros-args -p description:=${SCRIPTPATH}/voyager/description.yaml -p report_dir:="$REPORT_DIR" -p output_dir:="$REPORT_DIR"/voyager_case_report
