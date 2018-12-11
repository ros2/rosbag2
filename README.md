# rosbag2

THIS IS WORK IN PROGRESS AND NOT READY TO BE USED YET

Repository for implementing ROSBag2 as described in its corresponding [design article](https://github.com/ros2/design/blob/f69fbbd11848e3dd6866b71a158a1902e31e92f1/articles/rosbags.md)

## Build instructions

Create a new workspace:

```
$ mkdir -p ~/rosbag_ws/src
$ cd ~/rosbag_ws/src
```

Clone this repository into the source folder:

```
$ git clone https://github.com/ros2/rosbag2.git
```

Then build all the packages with this command:

```
$ colcon build --merge-install
```

The `--merge-install` flag is optional but ensures a cleaner environment which is helpful for development.

#### Executing tests

The tests can be run using the following commands:

```
$ colcon test --merge-install
$ colcon test-result --verbose
```

The first command executes the test and the second command displays the errors (if any).

## The plugins for rosbags from ROS 1

It is possible to read old bagfiles with `ros2`.
This requires translating ROS 1 messages into ROS 2 messages similar to the `ros1_bridge`.
If your rosbags contain custom message formats which can be translated into ROS 2 messages, the plugins need to be built from source.

#### Building the plugins

You need to have the `ros1_bridge` built (see <https://index.ros.org/p/ros1_bridge/github-ros2-ros1_bridge/#bouncy>).

Then, in a fresh terminal:

* Source your ROS 1 installation
* Source your ROS 2 installation (including the `ros1_bridge`)
* Build the workspace using `colcon build --merge-install`

This will automatically match all ROS 1 messages to their ROS 2 counterpart using the same logic as the `ros1_bridge`.

**N.B:** The ROS 1 installation must be sourced first to avoid problems with the `class_loader`.

#### Using the plugins

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
