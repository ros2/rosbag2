#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# RAPORT_DIR=~/.ros/raports

RAPORT_DIR=$1

if [ -z "$RAPORT_DIR" ]; then
    echo "Usage: ./voyager RAPORT_DIR"
    exit 1
fi

if [ ! -d "$RAPORT_DIR" ]; then
    echo "$RAPORT_DIR does not exist"
    exit 1
fi

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_1.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_10.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_100.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1k_1000.yaml raport_dir:="$RAPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_1.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_10.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_100.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/10k_1000.yaml raport_dir:="$RAPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_1.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_10.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_100.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/100k_1000.yaml raport_dir:="$RAPORT_DIR"

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_1.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_10.yaml raport_dir:="$RAPORT_DIR"
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_100.yaml raport_dir:="$RAPORT_DIR"
#ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/voyager/1000k_1000.yaml raport_dir:="$RAPORT_DIR"

######################################################################################################################

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_1.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_10.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_100.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1k_1000.yaml -p raport_dir:="$RAPORT_DIR"

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_1.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_10.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_100.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/10k_1000.yaml -p raport_dir:="$RAPORT_DIR"

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_1.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_10.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_100.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/100k_1000.yaml -p raport_dir:="$RAPORT_DIR"

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_1.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_10.yaml -p raport_dir:="$RAPORT_DIR"
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_100.yaml -p raport_dir:="$RAPORT_DIR"
#ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/voyager/1000k_1000.yaml -p raport_dir:="$RAPORT_DIR"

######################################################################################################################

ros2 run rosbag2_benchmarking voyager --ros-args -p description:=${SCRIPTPATH}/voyager/description.yaml -p raport_dir:="$RAPORT_DIR" -p output_dir:="$RAPORT_DIR"/voyager_case_raport
