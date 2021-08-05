# Adding a Snapshot Service for rosbag2

## Context
In [ros2/rosbag2#663](https://github.com/ros2/rosbag2/issues/663), it was recommended that a "snapshot" feature be added to rosbag2.

## Motivation
The goal of this feature is to provide the user with a mechanism to maintain a transient cache of the recent messages sent on specified topics that is only written to permanent storage upon a user-invoked trigger.


For example, if a rare, anomalous behavior occurs, this feature will allow the user to record the most recent ROS topic messages when the behavior is observed, instead of having to record the entire execution.

## Example Usage

Starting `rosbag2` in snapshot mode with a specified cache size:
```
$ ros2 bag record --max-cache-size 100000 --snapshot-mode [topics [topics ...]]
```

Triggering a snapshot via CLI:
```
$ ros2 service call ~/snapshot rosbag2_interfaces/TakeSnapshot
```

Triggering a snapshot with a keyboard shortcut:
* For example, pressing the `s` key in the open recording process might be used to trigger a snapshot.


## Implementation Proposal

* Add a `--snapshot-mode` flag to the `record` verb in `ros2bag` that will be passed via `StorageOptions`.

* Create a new class `CircularBufferMessageCache` which maintains two circular buffers sized according to the existing `—max-cache-size` parameter. During snapshot mode, this class will keep adding messages to the same circular buffer until triggered to switch to the secondary buffer by the `Recorder` class.

* Create a service `~/snapshot` in the `rosbag2_recorder` node that will listen for snapshot requests. Upon receiving a service call, it will trigger a buffer flip in `CircularBufferMessageCache` and call the new `take_snapeshot` function in `SequentialWriter` (detailed below) to write the snapshot messages to storage.

* Modify `SequentialWriter` to use `CircularBufferMessageCache` when snapshot mode is enabled instead of `MessageCache` and `CacheConsumer`.

* Add a new `take_snapshot` function to `SequentialWriter` that will be invoked by the `rosbag2_recorder` node's snapshot service to write the snapshot data to the rosbag, close the rosbag, and open a new rosbag for a future snapshot.

* Implement the `~/snapshot` service interface as `TakeSnapshot.srv` in `rosbag2_interfaces`. It won’t require any arguments (similar to `Resume.srv`).


## Implementation Breakdown
The snapshot feature implementation could be broken down into the following PRs:

* Implement `CircularBufferMessageCache` and add corresponding tests.
* Integrate `CircularBufferMessageCache` into `SequentialWriter` and update its tests.
* Create the `~/snapshot` service in the `Recorder` class, add corresponding tests, and create the `TakeSnapshot.srv` service interface.
* Add `--snapshot-mode` to the `record` verb and update `StorageOptions`
* Add keyboard shortcut for triggering a snapshot in snapshot mode