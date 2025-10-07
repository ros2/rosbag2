# rosbag2_storage_fifo_plugin

This package provides a storage plugin for `rosbag2` that writes bag data as a real-time stream to a named pipe (FIFO) instead of a conventional file. This allows other applications to consume ROS 2 topic data live as it's being recorded.

---

## Features

* **Real-time Streaming**: Data is available for consumption by other processes the moment it's recorded, without needing to wait for the bag file to be closed.
* **MCAP Format Stream**: The data within the FIFO is serialized using the standard MCAP format, allowing for compatibility with existing MCAP tools and readers.
* **Low Disk Overhead**: Since data is streamed directly to a consuming application, it minimizes the need for disk buffering and I/O on the recording machine.

---

## Usage

Using this plugin requires a two-step process involving two separate terminals: one for the **reader** (the application that will consume the data) and one for the **writer** (the `ros2 bag record` command).

The reader **must** be started before the writer, as the plugin will block until a reader connects to the FIFO.

### 1. Start the Reader

In your first terminal, start an application to read from the FIFO. The simplest way to test this is using `cat` to redirect the stream into a standard file. This command will wait until the recorder starts.

```bash
# Terminal 1: Start the reader
# This will block until the recorder starts writing data.
cat /Data/fifo/ros2_mcap.fifo > live_data.mcap
