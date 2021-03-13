# Rosbag2 Playback Time Handling

## Context

This design does not consider recording, it only handles playback, which has a more complex interaction with time.

Terms:
* "ROS Time" - the time expressed on a `/clock` topic
* "ROS Time Source" - the entity that publishes to `/clock`
For more context on time in ROS 2 see http://design.ros2.org/articles/clock_and_time.html#ros-time

## Goal

We need Rosbag2 playback to do more than just playing back in real time according to the system clock.
For playback we want to support the following "time-control" features:
* Pause and resume time
  * When time is resumed, rosbag2 will play the next message in sequence after the last one it published
  * There is an alternate concept of "pause and resume" that means "suppress recording/publishing while time continues, such that messages are skipped" - I use "pause and resume" in this document in the sense of "stopping and starting the flow of time". An alternative design handles that other feature, which may be more appropriately named something else such as "suppress", or "mute" for an analogy with an audio playback.
* Set a (forward) rate of playback - either faster or slower than real time
  * This is already implemented as of this writing, but this design proposes a change in the implementation
* Jump back or forward to an arbitrary point in time
  * This is called "jump" in the time interface, but will likely be exposed in the player interface as "seek"

These are the time situations to handle:
1. Steady Time - Rosbag2 keeps time internally with reference to a monotonic system clock, and neither publishes or subscribes to `/clock`
  * This is the default behavior
2. Rosbag2 acts as ROS Time Source by publishing to the `/clock` topic (a simple extension of case 1)
  * Chosen by `--clock` option to `ros2 bag play`
3. Rosbag2 is driven by an external ROS Time Source - most commonly the Gazebo simulator
  * In this case, Rosbag2 cannot use "time-control" features presented above, since it is a passive consumer of time
  * Chosen by `--use-sim-time` argument to `ros2 bag play`

NOTE: The user is responsible for ensuring that there is only one publisher on `/clock` - Rosbag2 will not try to de-conflict multiple ROS Time Sources, but it will print a warning if more than one publisher is detected.

## Notes on current implementation (as of writing this design)

`rosbag2_transport::Player` currently uses `std::chrono::system_clock` to query time, and `std::this_thread::sleep_until` to wait between publishing messages, with explicit handling of playback rate.

## Proposal

Pass a `PlayerClock` instance to `rosbag2_transport::Player`
* Use `PlayerClock::now` to query starting time
* Use `PlayerClock::sleep_until` between messages
  * `rclcpp::Clock` does not yet implement `sleep_until` - as noted in https://github.com/ros2/rcl/issues/898
  * If we are not able to contribute this feature into `rclcpp` in time for the Galactic API freeze - it may need to temporarily go into a rosbag2-based subclass for the Galactic release.

```
class PlayerClock : public rclcpp::Clock
{
public:
  /**
   * :param use_sim_time: if true, subscribe to /clock to provide time
   *   When use_sim_time is true, all time control is disabled.
   *   starting_time, playback_rate, pause/resume, jump will all be ignored - and the /clock publisher will be disabled
   * :param starting_time: provides a "first time"
   *    This should probably be the timestamp of the first message in the bag, but is a required argument because it cannot be guessed
   * :param playback_rate: must be positive, 1.0 means real-time
   * :param clock_topic_publish_frequency:
   *    - if > 0 - publishes `now()` to /clock at the specified frequency in Hz, rate determined by a system steady clock
   *    - if <= 0 - does not publish the /clock topic
   */
  Clock(
    bool use_sim_time,
    rclcpp::Time starting_time,
    float playback_rate = 1.0,
    float clock_topic_publish_frequency = 40.0);

  // rclcpp::Clock interface
  /**
   * Provides the current time according to the clock's internal model.
   *  if use_sim_time: provides current ROS Time (with optional extrapolation - see "Clock Rate and Time Extrapolation" section)
   *  if !use_sim_time: calculates current "Player Time" based on starting time, playback rate, pause state.
   *    this means that /clock time will match with the recorded messages time, as if we are fully reliving the recorded session
   */
  rclcpp::Time now() override;

  /**
   * Sleeps (non-busy wait) the current thread until the provided time is reached - according to this Clock
   *
   * If time is paused, the requested time may never be reached
   * `real_time_timeout` uses the internal steady clock, if the timeout elapses, return false
   * If jump() is called, return false, allowing the caller to handle the new time
   * Return true if the time has been reached
   */
  bool sleep_until(rclcpp::Time until, rclcpp::Duration real_time_timeout) override;

  // Time Control interface

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

### Clock Rate and Time Extrapolation

This section is only relevant for Case 3 (subscribing to an external ROS Time Source), and does not apply to other cases.

Common `/clock` publish rates are 10-100Hz - which is considerably slower than, for example, an IMU which may commonly publish at 200Hz or faster.
According to the [ROS Time Source Design](http://design.ros2.org/articles/clock_and_time.html#ros-time-source), "calls to the ROS time abstraction will return the latest time received from the `/clock` topic."
This means that faster topics will have a playback resolution only as fine as the `/clock` rate, as illustrated below.

```
# dots are arbitrary time tick, letters are when a message is published
# M is topic Message, C is /clock message

# original system
M..M..M..M..M..M..M..M..M..M..M..
C.......C........C........C......

# playback on use_sim_time with default behavior - when a new time sample is received a burst of backed up messages will be played
M.......MM.......MMM......MMM....
C.......C........C........C......
```

The next section "No Advanced Estimating Clock By Default" lays out that more advanced behavior is possible, though "these techniques will require making assumptions about the future behavior of the time abstraction. And in the case that playback or simulation is instantaneously paused, it will break any of these assumptions."

While the above is technically true, I propose that we implement a simple extrapolation anyways for the case of subscribing to an external ROS Time Source:
* Store internally the last 2 time samples (noting the rate that samples are received)
* When `now()` is called, determine the rate from the stored time samples and extrapolate based on time
* Put a watchdog on the clock to notice if new messages are not received in the expected period
  * Deadline QoS on the `/clock` publisher is preferred for this, but we may not be able to enforce this for all ROS Time Sources
  * Immediately "pause time" when a missing sample is noticed

Possible error:
  * ROS Time Source increases rate-of-time: playback may be slightly behind, and catches up by the next sample
  * ROS Time Source decreases rate-of-time: playback may be slightly ahead based on extrapolated messages being published early, and
  * ROS Time Source is instantaneously paused: Rosbag2 will play back "future" messages for no longer than one clock-sample period, stopping once the pause is noticed

This design deems the above error acceptable for most use cases. To let the user choose this, it is an optionally enabled behavior via `--extrapolate-ros-time`, which only has an effect when `--use-sim-time` is provided. Note: The faster the `/clock` rate, the smaller the possible error both with and without extrapolation.

### Pause and Resume Time

No special allowance needs to be made in the `Player` - time will still be provided, but it will not move forward, so the next message will not be published until time is resumed.

NOTE: this is different than "suppress/mute" where time continues but publishing stops - that feature is handled outside of time-control.

### Setting the Rate

No special allowance needs to be made in the `Player` - time will be provided at a different rate, and `sleep_until` will handle this.

### Time Jumps

The `Player` must register a `JumpHandler` with the `Clock` - so that when a jump occurs, the current message playback queue can be invalidated and re-enqueued according to the new starting time.

### Synchronizing rosbag2 playback

As a consequence of this design, it is possible to synchronize the playback of multiple rosbags, by setting one to publish with `--clock`, and the others to listen with `--use-sim-time`. This feature is useful if for example bags were recorded for different topic selections, in the same time range.

## Implementation Staging

This should not be implemented monolithically. Implementation should focus on small incremental PRs with solid testing that are easy to review.
This is a proposed order of operations.

* Implement `rclcpp::Clock::sleep_until` - which will allow us to handle all external time drivers (e.g. Gazebo)
* Create `rosbag2_transport::PlayerClock` with unit tests, with rate setting - but without pause and jump
* Expose "rosbag2 publishes to `/clock`" (the most requested feature), passing the `PlayerClock` to `Player`
* Add pause/resume to the `PlayerClock`, controllable by Service on the Rosbag2 Node
* Add time jump to the `PlayerClock` (and handlers to the `Player`), controllable by a Service on the Rosbag2 Node

^ Note that the provided Services will allow for building GUI and CLI/keyboard controllers for this functionality, without having to build against rosbag2 directly
