# Rosbag2 writer benchmarking

The primary package to test performance of the rosbag2.

## How it works

Use `benchmark_launch.py` launchfile to run an entire set of benchmarks.

Launchfile requires two arguments:

- `benchmark` - provides benchmark description (how many repetitions, cache sizes, database configurations etc.),
- `producers` - provides producers description (how many publisher instances, frequency, messages options etc.)

Templates for these configuration files are in `config` directory of this package.

To run test benchmark (with `test.yaml` and `mixed_110Mbs.yaml`):

```bash
ros2 launch rosbag2_performance_benchmarking benchmark_launch.py benchmark:=`ros2 pkg prefix rosbag2_performance_benchmarking`/share/rosbag2_performance_benchmarking/config/benchmarks/test.yaml producers:=`ros2 pkg prefix rosbag2_performance_benchmarking`/share/rosbag2_performance_benchmarking/config/producers/mixed_110Mbs.yaml
```

The summary of benchmarks goes into `results.csv` files, which includes rows of execution parameters and results. These files lie inside corresponding for each test optimization -> compression directories.

For human friendly output, a postprocess report generation tool can be used. Launch it with benchmark result directory as an `-i` argument (directory with `results.csv` file):

```bash
scripts/report_gen.py -i <BENCHMARK_RESULT_DIR>
```

## Compression

Note that while you can opt to select compression for benchmarking, the generated data is random so it is likely not representative for this specific case. To publish non-random data, you need to modify the ByteProducer.

## Building

To build the package in the rosbag2 build process, make sure to turn `BUILD_ROSBAG2_BENCHMARKS` flag on (e.g. `colcon build --cmake-args -DBUILD_ROSBAG2_BENCHMARKS=1`)

If you already built rosbag2, you can use `packages-select` option to build benchmarks.
Example: `colcon build --packages-select rosbag2_performance_benchmarking --cmake-args -DBUILD_ROSBAG2_BENCHMARKS=1`.

## General knowledge: I/O benchmarking

#### Background: benchmarking disk writes on your system

It might be useful to first understand what limitation your disk poses to the throughput of data recording.
Performance of bag write can't be higher over extended period of time (you can only use as much memory).

**Using dd command**

`dd if=/dev/zero of=/tmp/output conv=fdatasync bs=384k count=1k; rm -f /tmp/output`

This method is not great for benchmarking the disk but an easy way to start since it requires no dependencies.
This will write zeros to the /tmp/output file with block size 384k, 1000 blocks, ends when write finishes.
Make sure to benchmark the disk which your bags will be written to (check your mount points and change “/tmp/output” to another path if needed).
Note: this depends on parameters used and whatever else is running on your system but can give you a ballpark figure when ran several times.

**Using fio**

For more sophisticated & accurate benchmarks, see the `fio` command. An example for big data blocks is: `fio --name TEST --eta-newline=5s --filename=fio-tempfile.dat --rw=write --size=500m --io_size=10g --blocksize=1024k --ioengine=libaio --fsync=10000 --iodepth=32 --direct=1 --numjobs=1 --runtime=60 --group_reporting`.

#### Profiling bags I/O with tools

Tools that can help in I/O profiling: `sudo apt-get install iotop ioping sysstat`
* `iotop` works similar as `top` command, but shows disk reads, writes, swaps and I/O %. Can be used at higher frequency in batch mode with specified process to deliver data that can be plotted.
  *  Example use: `sudo iotop -h -d 0.1 -t -b -o -p <PID>` after running the bag.  
* `ioping` can be used to check latency of requests to device
* `strace` can help determine syscalls associated with the bottleneck.
  *  Example use: `strace -c ros2 bag record /image --max-cache-size 10 -o ./tmp`. You will see a report after finishing recording with Ctrl-C.
