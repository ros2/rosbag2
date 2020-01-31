# rosbag2
![License](https://img.shields.io/github/license/ros2/rosbag2)
[![GitHub Action Status](https://github.com/ros2/rosbag2/workflows/Test%20rosbag2/badge.svg)](https://github.com/ros2/rosbag2/actions)

Repository for implementing rosbag2 as described in its corresponding [design article](https://github.com/ros2/design/blob/f69fbbd11848e3dd6866b71a158a1902e31e92f1/articles/rosbags.md).

## Installation instructions

## Debian packages

rosbag2 packages are available via debian packages and thus can be installed via

```
$ export CHOOSE_ROS_DISTRO=crystal # rosbag2 is available starting from crystal
$ sudo apt-get install ros-$CHOOSE_ROS_DISTRO-ros2bag ros-$CHOOSE_ROS_DISTRO-rosbag2*
```

Note that the above command installs all packages related to rosbag2.
This also includes the plugin for [reading ROS1 bag files](https://github.com/ros2/rosbag2_bag_v2), which brings a hard dependency on the [ros1_bridge](https://github.com/ros2/ros1_bridge) with it and therefore ROS1 packages.
If you want to install only the ROS2 related packages for rosbag, please use the following command:

```
$ export CHOOSE_ROS_DISTRO=crystal # rosbag2 is available starting from crystal
$ sudo apt-get install ros-$CHOOSE_ROS_DISTRO-ros2bag ros-$CHOOSE_ROS_DISTRO-rosbag2-transport
```

## Build from source

It is recommended to create a new overlay workspace on top of your current ROS 2 installation.

```
$ mkdir -p ~/rosbag_ws/src
$ cd ~/rosbag_ws/src
```

Clone this repository into the source folder:

```
$ git clone https://github.com/ros2/rosbag2.git
```
**[Note]**: if you are only building rosbag2 on top of a Debian Installation of ROS2, please git clone the branch following your current ROS2 distribution. 

Then build all the packages with this command:

```
$ colcon build [--merge-install]
```

The `--merge-install` flag is optional and installs all packages into one folder rather than isolated folders for each package.

#### Executing tests

The tests can be run using the following commands:

```
$ colcon test [--merge-install]
$ colcon test-result --verbose
```

The first command executes the test and the second command displays the errors (if any).

## Using rosbag2

rosbag2 is part of the ROS 2 command line interfaces.
This repo introduces a new verb called `bag` and thus serves as the entry point of using rosbag2.
As of the time of writing, there are three commands available for `ros2 bag`:

* record
* play
* info

### Recording data

In order to record all topics currently available in the system:

```
$ ros2 bag record -a
```

The command above will record all available topics and discovers new topics as they appear while recording.
This auto-discovery of new topics can be disabled by given the command line argument `--no-discovery`.

To record a set of predefined topics, one can specify them on the command line explicitly.

```
$ ros2 bag record <topic1> <topic2> … <topicN>
```

The specified topics don't necessarily have to be present at start time.
The discovery function will automatically recognize if one of the specified topics appeared.
In the same fashion, this auto discovery can be disabled with `--no-discovery`.

If not further specified, `ros2 bag record` will create a new folder named to the current time stamp and stores all data within this folder.
A user defined name can be given with `-o, --output`.

### Replaying data

After recording data, the next logical step is to replay this data:

```
$ ros2 bag play <bag_file>
```

The bag file is by default set to the folder name where the data was previously recorded in.

### Analyzing data

The recorded data can be analyzed by displaying some meta information about it:

```
$ ros2 bag info <bag_file>
```

You should see something along these lines:

```
Files:             demo_strings.db3
Bag size:          44.5 KiB
Storage id:        sqlite3
Duration:          8.501s
Start:             Nov 28 2018 18:02:18.600 (1543456938.600)
End                Nov 28 2018 18:02:27.102 (1543456947.102)
Messages:          27
Topic information: Topic: /chatter | Type: std_msgs/String | Count: 9 | Serialization Format: cdr
                   Topic: /my_chatter | Type: std_msgs/String | Count: 18 | Serialization Format: cdr
```

## Storage format plugin architecture

Looking at the output of the `ros2 bag info` command, we can see a field called `storage id:`.
rosbag2 specifically was designed to support multiple storage formats.
This allows a flexible adaptation of various storage formats depending on individual use cases.
As of now, this repository comes with two storage plugins.
The first plugin, sqlite3 is chosen by default.
If not specified otherwise, rosbag2 will store and replay all recorded data in an SQLite3 database.

In order to use a specified (non-default) storage format plugin, rosbag2 has a command line argument for it:

```
$ ros2 bag <record> | <play> | <info> -s <sqlite3> | <rosbag2_v2> | <custom_plugin>
```

Have a look at each of the individual plugins for further information.

## Serialization format plugin architecture

Looking further at the output of `ros2 bag info`, we can see another field attached to each topic called `Serialization Format`.
By design, ROS 2 is middleware agnostic and thus can leverage multiple communication frameworks.
The default middleware for ROS 2 is DDS which has `cdr` as its default binary serialization format.
However, other middleware implementation might have different formats.
If not specified, `ros2 bag record -a` will record all data in the middleware specific format.
This however also means that such a bag file can't easily be replayed with another middleware format.

rosbag2 implements a serialization format plugin architecture which allows the user the specify a certain serialization format.
When specified, rosbag2 looks for a suitable converter to transform the native middleware protocol to the target format.
This also allows to record data in a native format to optimize for speed, but to convert or transform the recorded data into a middleware agnostic serialization format.

By default, rosbag2 can convert from and to CDR as it's the default serialization format for ROS 2.
