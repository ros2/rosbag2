# Design: Per-topic frequency throttling in the Rosbag2 recorder

## Problem
High-frequency topics (IMUs, cameras, point clouds) often dominate bag size even when the
recorded data is only needed at a much lower rate for analysis or regression testing. Today the
only options are:
1) Run a `topic_tools throttle` proxy node and record the throttled copy. This costs an extra
   node, an extra DDS publication of the (potentially large) payload, and renames the topic in
   the bag.
2) Record at full rate and downsample in post-processing. This preserves everything but pays the
   full serialization, caching, compression, and disk I/O cost during the live run.

Neither lets a user simply say "record `/imu` at 10 Hz" to the recorder itself.

## Goals
- Opt-in, per-topic maximum recording frequency, applied inside the recorder process.
- Drop messages before they enter the writer pipeline, so throttled data costs no cache,
  compression, or disk bandwidth.
- Zero overhead and unchanged behavior for topics without a throttle entry.
- Work identically from the `ros2 bag record` CLI, node parameters (composable recorder), and
  the C++/Python `RecordOptions` APIs.

## Non-goals
- Throttling in the storage layer. A storage plugin receives messages after serialization,
  caching, and compression, so filtering there would waste the exact work throttling is meant to
  save, and every plugin would need its own implementation.
- Throttling the direct `Recorder::write_message()` API. Programmatic writes are explicit;
  callers who want fewer of them can simply call less often.
- Bandwidth-based (bytes/sec) or count-based (every N-th message) policies. These can be added
  later as separate options if needed.

## User-facing interface

```shell
$ ros2 bag record -a --topic-throttle-frequency /imu=10.0 /camera/image_raw=2.0
```

Semantics:
- Each entry `<topic>=<frequency>` sets a maximum recording frequency in Hz for that topic.
- A message is written only if at least `1/frequency` seconds have elapsed since the last
  written message on that topic; otherwise it is dropped at the subscription edge.
- The first message on a throttled topic is always recorded.
- Frequencies must be finite and greater than 0; the frequency is mandatory (there is no
  sensible default, unlike `--repeat-transient-local` depths).
- Topics are keyed by fully qualified name, matching how `--topics` entries are matched.
- The same option is available as the `record.topic_throttle_frequency` node parameter (list of
  `<topic>=<frequency>` strings) and as `RecordOptions::topic_throttle_frequencies`
  (`std::unordered_map<std::string, double>`, exposed to Python via `rosbag2_py`).

## Design overview

### Where: the subscription callback in `rosbag2_transport`
The recorder's generic subscription callback (created in `RecorderImpl::create_subscription`)
is the earliest point the recorder owns the message. Dropping there skips
`rosbag2_cpp::Writer::write()` entirely — no cache insertion, no compression, no storage I/O.
The RMW layer has already delivered the serialized message, so the subscription cost itself is
unavoidable without QoS/DDS-level filtering, which is not portable across RMW implementations.

### Throttle state: owned by each subscription's closure
The throttle period is resolved once per topic when the subscription is created, and the
"last written receive timestamp" lives in the callback closure. This avoids a shared map lookup
and locking on the hot path, and costs literally nothing (one integer compare against 0) for
topics without a throttle entry. Recorder subscriptions use the node's default mutually
exclusive callback group, so per-subscription state needs no synchronization.

### Time base: the message receive timestamp
The guard compares the same `recv_timestamp` that is written into the bag (the RMW receive
timestamp, or the node clock when `use_sim_time` is set), rather than sampling
`std::chrono::steady_clock` or calling `node->now()` again:
- The message spacing in the recorded bag matches the requested frequency exactly, by
  construction.
- Under `use_sim_time` the throttle follows simulated time automatically, including paused and
  accelerated clocks.
- No extra clock query is added to the callback.

If the time base jumps backwards (e.g. a simulation reset), the message is accepted and the
throttle window restarts, so a throttled topic can never stall.

### Interaction with pause/resume and split bookkeeping
Dropped messages still update the recorder's last-seen timestamp bookkeeping, so
timestamp-based resume and split requests observe the full message stream. The throttle guard
runs inside the "not paused" branch, immediately before the writer call: pausing does not
consume the throttle window, and the first message after resume follows the normal spacing
rule.

### Validation
Frequencies are validated (finite, > 0) in three places: the `ros2 bag record` argument
validation, the `record.topic_throttle_frequency` node-parameter parsing, and
`Recorder::record()` itself (covering direct API users). Invalid values fail fast with
`std::invalid_argument` before any bag is created.

## Alternatives considered
- **`topic_tools throttle` proxy (status quo):** works everywhere, but re-publishes the payload
  over the middleware just to drop most of it, renames the topic in the bag, and adds an extra
  process to launch and monitor.
- **Storage plugin filter:** rejected; see Non-goals. Wrong layer — the savings happen before
  the storage interface, and the feature would need reimplementing per plugin.
- **QoS/DDS content or time-based filters:** `rmw` support is inconsistent across vendors, and
  time-based filters change subscription QoS in ways that can affect publisher matching.
- **Post-processing (`ros2 bag convert`-style downsampling):** complementary, not competing —
  it preserves transients for later inspection but saves nothing at record time. The README
  points users to it when they need every message during the live run.
