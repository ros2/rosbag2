# Rosbag2 writer benchmarking

The primary package to test transport-less performance of the rosbag2 writer and storage. Enables parametrized batch execution of benchmarks and tries to replicate the flow to capture message loss in queues.

## How it works

Use `scripts/benchmark.sh` to run an entire set of benchmarks. These are currently aimed at several 100Mb/s scenarios. Parameters are easy to change
inside the script.

By default, result log files will be written to `/tmp/rosbag2_test/`. Database (bag) files are removed after recording to avoid filling up the disk.
