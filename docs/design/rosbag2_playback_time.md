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
/**
 * Used to control the timing of bag playback.
 * This clock should be used to query times and sleep between message playing,
 * so that the complexity involved around time control and time sources
 * is encapsulated in this one place.
 * Internally, it may own an rclcpp::Clock, but does not override the class in order to implement
 * a slightly different API.
 */
class PlayerClock
{
public:
  /**
   * Constructor.
   *
   * \param use_sim_time
   *   If false, starts paused.
   *     Use member functions to configure before calling set_paused(false)
   *   If true, subscribe to /clock to provide time
   *     All time control (rate, pause, jump) is disabled, and will not publish to /clock
   */
  explicit PlayerClock(bool use_sim_time);

  virtual ~PlayerClock();

  // rclcpp::Clock interface
  /**
   * Provides the current time according to the clock's internal model.
   *  if use_sim_time: provides current ROS Time (with optional extrapolation - see "Clock Rate and Time Extrapolation" section)
   *  if !use_sim_time: calculates current "Player Time" based on starting time, playback rate, pause state.
   *    this means that /clock time will match with the recorded messages time, as if we are fully reliving the recorded session
   */
  rclcpp::Time now();

  /**
   * Sleeps (non-busy wait) the current thread until the provided time is reached - according to this Clock
   *
   * If time is paused, the requested time may never be reached
   * `real_time_timeout` uses the internal steady clock, if the timeout elapses, return false
   * If jump() is called, return false, allowing the caller to handle the new time
   * Return true if the time has been reached
   */
  bool sleep_until(rclcpp::Time until, rclcpp::Duration real_time_timeout);

  // Time Control interface

  /**
   * Pauses/resumes time. Defaults to true (paused)
   * While paused, `now()` will repeatedly return the same time, until resumed
   *
   * Note: this could have been implemented as `set_rate(0)`, but this interface allows this clock to maintain the
   * clock's rate internally, so that the caller does not have to save it in order to resume.
   */
  void set_paused(bool paused);
  bool get_paused() const;

  /**
    * Changes the rate of playback. Defaults to 1.0
    * \param rate: must be nonzero positive. To pause playback, use \sa set_paused instead
    * \throws std::invalid_argument if rate is <= 0.0
    */
  void set_rate(float rate);
  float get_rate() const;

  /**
   * Set the rate that the clock will be published. Defaults to 0.
   * \param frequency If this is set to <= 0, then the clock will not be published
   */
  void set_clock_publish_frequency(float frequency);
  float get_clock_publish_frequency() const;

  /**
    * Change the current internally maintained offset so that next published time is different.
    *
    * This will trigger any registered JumpHandler callbacks.
    * Call this with the first message timestamp for a bag before starting playback (otherwise this will return current wall time)
    */
  void jump(rclcpp::Time time);

  rclcpp::JumpHandler::SharedPtr
  create_jump_callback(
    rclcpp::JumpHandler::pre_callback_t pre_callback,
    rclcpp::JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold);
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

* Create a mostly-unimplemented `PlayerClock` class, and have `Player` use it to maintain current functionality. This accomplishes the entire API change without taking very much implementation or review.
* Move playback rate handling out of `Player` to be handled by `PlayerClock`
* Implement `/clock` publisher
* Implement pause/resume to the `PlayerClock`
* Implement time jump to the `PlayerClock` (and handlers to the `Player`)
* Expose rate, pause/resume, and time jump as Services to enable CLI/keyboard/GUI controls
* Implement `use_sim_time` (`/clock` subscription) - without extrapolation
* Implement sim time extrapolation
