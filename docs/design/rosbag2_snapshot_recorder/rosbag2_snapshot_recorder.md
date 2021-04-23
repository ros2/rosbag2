# Rosbag2 Snapshot

## Context

The following design handles functionality of keeping data from topics in buffer of specified size or from specific time duration and saving them after trigger.

Terms:
* "Snapshot" - save buffered data from topics to disk as rosbag
* "Trigger" - an invocation letting a node know when to save rosbag to disk
* "Recorder" - rosbag2 node which records data right after program start until killed

## Goal

Rosbag in ROS1 had functionality of creating a [snapshot](https://github.com/ros/rosbag_snapshot). Snapshotter was a program which subscribed to topics and buffered messages until a `trigger` for snapshot was sent. As in Rosbag2 we want to support the following features:
* Save data to rosbag on trigger
  * Enable triggering  buffer save to disk with service call
* Node and buffer configuration
  * Enable user to specify `cli` arguments which could be shared with `ros2 bag record` / `Recorder` node or similar to them (dependent on design). User should be able to configure buffer size (maximum memory per topic) and duration (maximum time difference between newest and oldest message), subscribed topics (list of topics or all currently available topics) and output rosbag filename.
* Pause and resume buffering process
  * Pausing refers to stopping buffering new messages until resumed or write is triggered.
  * Resuming continues the process of buffering after pause. Resuming would clear all buffers.


## Proposal

As thinking of this design in the past `rosbag_snapshot` was written as different package than `rosbag`. It was defined this way not to break the existent API of `rosbag`. Taken this into consideration it could be done so this time too. However it would require duplicating a large part of code which is already written and being optimized as a part of `rosbag2` repository. So as not being sure which way would be the most appropriate to implement I propose multiple approaches.

### Outside rosbag2

First approach implements `ros2 bag snapshot` as a different package. Mostly it requires rewriting `Recorder` node completly. Completly means in this case diving into implementation of `Writer`, `SequentialWriter` and `BaseWriterInterface` to copy and adjust writing to disk algorithms.

Pros:
* Design flexibility

Cons:
* Many new functions would be redefined or even redundant
* Breaking the initial desing of writers/readers in favor of putting more effort to custom mechanism
* Not being part of `rosbag2` and `ros2bag` (cli incompatibility)

### Writer Interfaces changes + `Recorder` modifications

Second approach would base on making changes to `Recorder`, writers and their interfaces. This approach bases on adding service to `Recorder` which handles triggering buffer save. Although current design of writers bases on `BaseWriterInterface` which would need to be changed to enable some kind of `save()/flush()` functionality. I believe this is not an elegant approach but I haven't find other way to trigger buffer save from `Recorder`, through `Writer` to `SequentialWriter`. For sure it requires implementing new `SnapshotWriter` since it needs to take care of buffering the messages from subscribers instead of passing them straight to `MessageCache` and `CacheConsumer`.

Pros:
* Maintaining snapshot inside `ros2 bag snapshot ...` (or even `ros2 bag record snapshot ...`) and `Recorder` functionality
* Reuse most of already written code (producer-consumer in save, abstract interfacing between writers)

Cons:
* `BaseWriterInterface` redefinition (adding at least one pure abstract function)
* May require many changes in current implementation of the above mentioned classes

### Custom `Recorder` implementation

Last but not least it is possible to write new alternative to `Recorder` node inside `rosbag2_transport` package. `Snapshoter` would implement functionalities of subscribing the messages, buffering them at high level node and most probably passing data straight to `MessageCache` and `CacheConsumer`.

Pros:
* Creating independent piece of code
* Adding new cli phrase and leaving package-wise compatibility inside `rosbag2` repository

Cons:
* Many new functions would be redefined or even redundant
* Mixing high and low level of abstraction in one piece of code
* Feels like putting much effort than it is needed