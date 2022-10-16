# Rosbag2 storage plugin comparison

This package contains a benchmark for comparing the write throughput and overall memory
usage of different storage plugins in different configurations. This can be used to determine
the storage options that best suit your use-case.

## How to use

### Building

```
colcon build --packages-up-to rosbag2_storage_plugin_comparison
```

### Running the benchmark
```
ros2 run rosbag2_storage_plugin_comparison sweep.py results.csv
```

### Interpreting the results

The resulting CSV contains the following columns:
| Name | Units | Description |
| ---- | ---- | ---- |
| name |      | A concatenated string of all parameter labels and values for this run. |
| messages |     | The name of the "messages" parameter variant used for this run. |
| batch_size |    |  The name of the "batch_size" parameter variant used for this run. |
| plugin_config |    |  The name of the "plugin_config" parameter variant used for this run. |
| plugin_config |    |  The name of the "plugin_config" parameter variant used for this run. |
| avg_byte_throughput | bytes/second | The average write throughput of the plugin for this run. |
| max_arena_size | bytes | The max additional bytes that were added to the allocator arena for creating the writer and writing all messages. |
| max_in_use_size | bytes | The max bytes that were allocated when writing messages for this run. |
| max_mmap_size | bytes | The max bytes that were mmapped by the allocator when writing for this run. |
| close_time | seconds | The time taken to finish and close the file, once all messages were written. |

#### Memory usage notes

All memory usage measurements are taken as a difference from a baseline measurement. The baseline
is measured just before allocating a Writer instance. all memory usage values include the memory allocated
on the heap for the Writer instance itself, as well as any that remain allocated after a write() call
finishes.

### Changing the benchmark sweep parameters

`sweep.py` runs the benchmark binary a number of times across a set of parameters. The parameters
and their values are defined in `CONFIG_DIMENSIONS` in sweep.py. Check `config.hpp` for the the set
of config parameters available to you.
