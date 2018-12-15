# The plugin for rosbags from ROS 1

It is possible to read old bag files with `ros2`.
This requires translating ROS 1 messages into ROS 2 messages similar to the `ros1_bridge`.
If your rosbags contain custom message formats which can be translated into ROS 2 messages, the plugins need to be built from source.

## Building the ROS 1 plugin

You need to have the `ros1_bridge` built (see <https://index.ros.org/p/ros1_bridge/github-ros2-ros1_bridge/#bouncy>).
Secondly, this plugin is part of the `rosbag2` plugin architecture.
It is thus required that `rosbag2` is correctly installed, see <https://index.ros.org/r/rosbag2/github-ros2-rosbag2/#crystal>.

We assume that the ROS 2 as well as the rosbag2 environment is correctly built and sourced.
Then, in a fresh terminal:

* Source your ROS 1 installation
* Source your ROS 2 installation (including the `ros1_bridge`)
* Build the workspace using `colcon build --merge-install`

This will automatically match all ROS 1 messages to their ROS 2 counterpart using the same logic as the `ros1_bridge`.

**N.B:** The ROS 1 installation must be sourced first to avoid problems with the `class_loader`.

## Using the plugins

In order to use the plugins, again, the ROS 1 installation must be sourced **before** sourcing the ROS 2 installation.

You can then just use the plugin through the regular interface.
For instance, on the command line write:

```
ros2 bag info -s rosbag_v2 <path_to_bagfile>
```
Here, `-s rosbag_v2` tells rosbag2 to use the plugin to read rosbags (version 2) to query the bagfile.
For old rosbags, the storage format must be added to the info call as rosbag does not have the necessary information to read the plugin otherwise.

For playing, one can similarly write:
```
ros2 bag play -s rosbag_v2 <path_to_bagfile>
```

Currently, split bagfiles are unsupported.
