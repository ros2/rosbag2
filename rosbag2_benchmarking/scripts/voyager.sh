#!/usr/bin/env bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1k_1.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1k_10.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1k_100.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1k_1000.yaml

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/10k_1.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/10k_10.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/10k_100.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/10k_1000.yaml

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/100k_1.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/100k_10.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/100k_100.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/100k_1000.yaml

ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1000k_1.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1000k_10.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1000k_100.yaml
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=${SCRIPTPATH}/../config/voyager/1000k_1000.yaml

######################################################################################################################

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1k_1.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1k_10.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1k_100.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1k_1000.yaml

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/10k_1.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/10k_10.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/10k_100.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/10k_1000.yaml

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/100k_1.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/100k_10.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/100k_100.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/100k_1000.yaml

ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1000k_1.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1000k_10.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1000k_100.yaml
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=${SCRIPTPATH}/../config/voyager/1000k_1000.yaml

######################################################################################################################

ros2 run rosbag2_benchmarking voyager --ros-args -p description:=${SCRIPTPATH}/../config/voyager/description.yaml