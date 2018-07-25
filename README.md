# rosbag2

THIS IS WORK IN PROGRESS AND NOT READY TO BE USED YET

Repository for implementing ROSBag2 as described in its corresponding [design article](https://github.com/ros2/design/blob/f69fbbd11848e3dd6866b71a158a1902e31e92f1/articles/rosbags.md)

## Build instructions

Create a new workspace:

```
$ mkdir -p ~/rosbag_ws/src
$ cd ~/rosbag_wsk/src
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
