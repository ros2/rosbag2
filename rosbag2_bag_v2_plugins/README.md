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
It happens to occur that cyclic dependencies are detected when compiling this plugin.
The reason for this is that some packages which contain message definitions in ROS 1 also depend on class loader.
It is therefore very important to source the ROS 2 installation after ROS 1 in order to guarantee that the ROS 2 class loader is correctly found and linked.
There is an open issue for this stating that the ROS 1 and ROS 2 class loader API should be provide some basic interoperability.
See https://github.com/ros/class_loader/issues/109

## Using the plugins

In order to use the plugins, again, the ROS 1 installation must be sourced **before** sourcing the ROS 2 installation.

You can then just use the plugin through the regular interface.
For instance, on the command line write:

```
ros2 bag info -s rosbag_v2 <path_to_bagfile>
```
Here, `-s rosbag_v2` tells rosbag2 to use the plugin to read rosbags (version 2) to query the bagfile.
For old rosbags, the storage format must be added to the info call as rosbag does not have the necessary information to read the plugin otherwise.

The command above should print something like the following:

```
Files:             test_bag.bag
Bag size:          8.8 KiB
Storage id:        rosbag_v2
Duration:          0.268s
Start:             Nov 29 2018 16:43:33.298 (1543509813.298)
End                Nov 29 2018 16:43:33.567 (1543509813.567)
Messages:          5
Topic information: Topic: /rosout | Type: rosgraph_msgs/Log | Count: 3 | Serialization Format: rosbag_v2
                   Topic: /test_topic | Type: std_msgs/String | Count: 1 | Serialization Format: rosbag_v2
                   Topic: /test_topic2 | Type: std_msgs/String | Count: 1 | Serialization Format: rosbag_v2
```

For playing, one can similarly write:

```
ros2 bag play -s rosbag_v2 <path_to_bagfile>
```

If there is ROS 1 data where no topic matching exists to ROS 2 these topics are ignored when replying.
When calling `ros2 bag info`, one can see a list of mismatching topics:

```
[INFO] [rosbag2_bag_v2_plugins]: ROS 1 to ROS 2 type mapping is not available for topic '/rosout' which is of type 'rosgraph_msgs/Log'. Skipping messages of this topic when replaying
```

Currently, split bagfiles are unsupported.
