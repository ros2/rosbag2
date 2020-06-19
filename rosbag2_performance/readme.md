# TO BE UPDATED #

# Rosbag2 performance

Tools for performance tests of ros2 bags.

Workers random image and pc2 generation part was based on Martin Idel [code](https://github.com/Martin-Idel/rosbag2/tree/performance_testing).

## Dependencies

*  ROS2 Foxy
*  psutil: `python3 -m pip install psutil`
*  iotop: `sudo apt install iotop`

> **NOTE** 
> 
> `iotop` requires sudo privileges to run. There is a `pkexec` `iotop` policy which will mute the password prompt before each benchmark. This script deploying this policy is located in `scripts/install_pkexec_iotop.sh`.

#

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

## I/O benchmarking

#### Background: benchmarking disk writes on your system 

It might be useful to first understand what limitation your disk poses to the throughput of data recording. Performance of bag write can't be higher over extended period of time (you can only use as much memory).

**Using dd command**

`dd if=/dev/zero of=/tmp/output conv=fdatasync bs=384k count=1k; rm -f /tmp/output`

This method is not great for benchmarking the disk but an easy way to start since it requires no dependencies. 
This will write zeros to the /tmp/output file with block size 384k, 1000 blocks, ends when write finishes. Make sure to benchmark the disk which your bags will be written to (check your mount points and change “/tmp/output” to another path if needed). 
Note: this depends on parameters used and whatever else is running on your system but can give you a ballpark figure when ran several times. 

**Using fio**

For more sophisticated & accurate benchmarks, see the `fio` command. An example for big data blocks is: `fio --name TEST --eta-newline-5s --filename=fio-tempfile.dat --rw=write --size=500m --io_size=10g --blocksize=1024k --ioengine=libaio --fsync=10000 --iodepth=32 --direct=1 --numjobs=1 --runtime=60 --group_reporting`.

#### Profiling bags I/O with tools 

Tools that can help in I/O profiling: `sudo apt-get install iotop ioping sysstat` 
* `iotop` works similar as `top` command, but shows disk reads, writes, swaps and I/O %. Can be used at higher frequency in batch mode with specified process to deliver data that can be plotted.
  *  Example use: `sudo iotop -h -d 0.1 -t -b -o -p <PID>` after running the bag.  
* `ioping` can be used to check latency of requests to device
* `strace` can help determine syscalls associated with the bottleneck.
  *  Example use: `strace -c ros2 bag record /image --max-cache-size 10 -o ./tmp`. You will see a report after finishing recording with Ctrl-C.





