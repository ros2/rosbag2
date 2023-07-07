# Development guide for Rosbag2

## Build from source

Rosbag2 is built like any other ROS 2 package, and all necessary packages are contained under the metapackage `rosbag2`.
Therefore you can

```
colcon build --packages-up-to rosbag2
```

Note: building Rosbag2 from source, overlaid on a debian installation of `ros-$ROS_DISTRO-rosbag2` has undefined behavior (but should work in most cases, just beware the build may find headers from the binaries instead of the local workspace.)


## Executing tests

The tests can be run using the following commands:

```
$ colcon test [--merge-install]
$ colcon test-result --verbose
```

The first command executes the test and the second command displays the errors (if any).
