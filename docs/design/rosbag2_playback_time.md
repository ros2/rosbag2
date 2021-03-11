# Rosbag2 Playback Time Handling

## Goal

We need rosbag2 playback to do more than just playing back in real time according to the system clock.

These situations must be handled
1. No ROS-time, rosbag2 plays back messages without considering external time
2. Rosbag2 drives ROStime by publishing to the /clock topic
3. Rosbag2 is driven by an external ROStime source - most commonly the Gazebo simulator

In the first two cases - rosbag2 needs to control the flow of time with the following operations
* pause and resume
* set a (forward) rate of playback - either faster or slower than real time
* Jump back or forward to an arbitrary point in time

In the final case, when rosbag2 is driven by an external source, it still must be able to handle the above operations, which may be performed by that external time controller. Therefore, a generic solution is required, so that  `rosbag2_transport::Player` is agnostic to the controller of the time.

## Current Implementation Notes

`rosbag2_transport::Player` currently uses `std::chrono::system_clock` to query time, and `std::this_thread::sleep_until` to wait between publishing messages, with explicit handling of playback rate.

## Proposal

Pass a `rclcpp::Clock` instance to `rosbag2_transport::Player`
* Use `Clock::now` to query starting time
* Use `Clock::sleep_until` between messages
  * `rclcpp::Clock` does not yet implement `sleep_until` - as noted in https://github.com/ros2/rcl/issues/898
  * If we are not able to get into `rclcpp` in time for the Galactic API freeze - it may need to temporarily go into a rosbag2-based subclass for the Galactic release.

When an external source drives time, a plain `rclcpp::Clock` can be used, configured to ROStime as the source. For situations where rosbag2 drives time, a `rosbag2_transport::PlayerClock` class is created - which returns the correct time based on requirements (see API declaration below).

```
class PlayerClock : public rclcpp::Clock
{
public:
  /**
   * playback_rate must be positive, 1.0 means real-time
   * starting_time provides an offset for all future times
   * clock_topic_publish_frequency:
   *   - if > 0 - publishes `now()` to /clock at the specified frequency in Hz, rate determined by a system steady clock
   *   - if <= 0 - does not publish the /clock topic
   */
  Clock(float playback_rate, rclcpp::Time starting_time, float clock_topic_publish_frequency);

  // rclcpp::Clock interface
  /**
   * based on an internal steady clock, calculates current "Player Time"
   */
  rclcpp::Time now() override;

  /**
   * Sleeps (non-busy-waiting) the current thread until the provided time is reached - according to this Clock!
   *
   * If time is paused, the requested time may never be reached
   * `real_time_timeout` uses the internal steady clock, if the timeout elapses, return false
   * If jump() is called, return false, allowing the caller to handle the new time
   * Return true if the time has been reached
   */
  bool sleep_until(rclcpp::Time until, rclcpp::Duration real_time_timeout) override;

  // Special playback interface

  /**
   * Pauses/resumes time.
   * While paused, `now()` will repeatedly return the same time, until resumed
   *
   * Note: this could have been implemented as `set_rate(0)`, but this interface allows this clock to maintain the
   * clock's rate internally, so that the caller does not have to save it in order to resume.
   */
  void set_paused(bool paused);

  /**
    * Changes the rate of playback. Must be positive (nonzero)
    */
  void set_rate(float rate);

  /**
    * Changes the current internally maintained offset so that next published time is different.
    * This should trigger any JumpHandler callbacks that are registered with the clock (see "Time Jumps" section)
    */
  void jump(rclcpp::Time time);

}
```

### Pause and Resume

No special allowance needs to be made in the `Player` - time will still be provided, but it will not move forward, so the next message will not be published until time is resumed.

### Setting the Rate

No special allowance needs to be made in the `Player` - time will be provided at a different rate, and `sleep_until` will handle this.

### Time Jumps

The `Player` must register a `JumpHandler` with the `Clock` - so that when a jump occurs, the current message playback queue can be invalidated and re-enqueued according to the new starting time.


## Open questions

* Should time interpolation in the ROS clock be used for external time driver? If the publish rate is too slow, playback cannot reach any close fidelity if there is none. E.g. a topic publishing at 200Hz would need at least a 200Hz `/clock`, unless the `Clock` can interpolate between samples.

## Implementation Staging

This should not be implemented monolithically. Implementation should focus on small incremental PRs with solid testing that are easy to review.
This is a proposed order of operations.

* Implement `rclcpp::Clock::sleep_until` - which will allow us to handle all external time drivers (e.g. Gazebo)
* Create `rosbag2_transport::PlayerClock` with unit tests, with rate setting - but without pause and jump
* Expose "rosbag2 publishes to `/clock`" (the most requested feature), passing the `PlayerClock` to `Player`
* Add pause/resume to the `PlayerClock`, controllable by Service on the Rosbag2 Node
* Add time jump to the `PlayerClock` (and handlers to the `Player`), controllable by a Service on the Rosbag2 Node

^ Note that the provided Services will allow for building GUI and CLI/keyboard controllers for this functionality, without having to build against rosbag2 directly
