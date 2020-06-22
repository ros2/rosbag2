# Rosbag2 benchmarking

A python package using launch file and scripting to launch performance workers and a suite of monitor tools (recording cpu, memory and i/o use). Benchmarks the entire pipeline from publish to writing on disk (including the transport layer). Uses binaries from `rosbag2_performance_workers`.

## Dependencies

*  ROS2 Foxy
*  psutil: `python3 -m pip install psutil`
*  iotop: `sudo apt install iotop`

> **NOTE**
>
> `iotop` requires sudo privileges to run. There is a `pkexec` `iotop` policy which will mute the password prompt before each benchmark. This script deploying this policy is located in `scripts/install_pkexec_iotop.sh`.

## How it works

Use `rosbag2_benchmarking/config/*.yaml` to set up benchmarks recipes (see provided example file `bench1.yaml`). Currently three workers are available: `image`, `bytearray` and `pointcloud2` which are producing `sensor_msgs/msg/Image`, `std_msgs/msg/ByteMultiArray` and `sensor_msgs/msg/PointCloud2` messages respectively.

Run benchmarks with:

```bash
ros2 launch rosbag2_benchmarking benchmarking.launch.py description:=[CONFIG_PATH] raport_dir:=[RAPORT_DIR]
```

Once a benchmark finishes, you can run the report generator:

```bash
ros2 run rosbag2_benchmarking raport_gen --ros-args -p description:=[CONFIG_PATH] -p raport_dir:=[RAPORT_DIR]
```

Each benchmark produces `rosbag2` resources along with reports from workers and system monitor.

## Voyager bundle test case

> **WARNING**
>
> Last step of this test case requires a lot of RAM (it sends 1MB data on 1000 topics with frequency of 100Hz). This may fill up your working memory pretty quick. If it strugles too much, you can edit `voyager.sh` file and comment out last benchmark described as `1000k_1000.yaml`.

To run voyager test case:

```bash
./rosbag2_benchmarking/bundles/voyager.sh [RAPORT_DIR]
```

`voyager.sh` contains multiple benchmarks as well as various raport generator nodes.

It will generate `html` raport inside `[RAPORT_DIR]`.

## Scripts for step-by-step benchmarking

There are some bash scripts (in `rosbag2_benchmarking/scripts/manual` dir) for testing how many images `rosbag2` is capable of writing depending on `--max-cache-size` parameter.

**Scripts:**

* `rosbag_image_cache.sh X` - run rosbag record on only `/image` topic with `--max-cache-size` set to `X`,
* `gen_images.sh X Y Z` - generate and publish `X` random images every `Y` ms of size `ZxZ`,
* `dummy_raport.sh` - shows how many images have been generated and how many `rosbag2` succeeded to write.

**Procedure:**

1. Run `rosbag_image_cache.sh X` to set up `rosbag2`,
2. Run `gen_images.sh X Y Z` to feed images to `rosbag2`,
3. After image generation, kill `rosbag_image_cache.sh` and see raport with `dummy_raport.sh`.
