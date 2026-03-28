# Design: Support repeating transient-local ("latched") messages in Rosbag2 recorder
Author: Michael Orlov

## Problem
Some ROS 2 topics use Transient Local durability (ROS 1 “latched”) so late subscribers can
receive already-published data, but Rosbag2 users still hit two practical gaps:
1) Split bags: consumers often expect “latched” state (e.g., `/tf_static`, `/map`) at the start of
   each split, like ROS 1 `--repeat-latched`. Rosbag2 does not provide this today.
2) Snapshot mode: when the circular cache overwrites early transient-local messages, a snapshot
   can miss them (e.g., `/tf_static` lost after cache pressure).

## Background
ROS 2 “latched visibility” requires both publisher and subscriber to use Transient Local durability.
Rosbag2 supports QoS override files for recording/playback when defaults don’t match.

## Goals
- Provide an opt-in “repeat latched” experience for recorder splits and snapshots.
- Fix snapshot-mode eviction for selected transient-local topics under cache pressure.
- Keep default behavior deterministic: “repeat” must be explicitly requested. Meaning that
  transient local messages shall not be repeated by default, since it would introduce extra 
  messages that were not really published at the time of recording.

## Non-goals
- Auto-detect all transient-local topics and repeat them by default.

## User-facing interface
Add `ros2 bag record --repeat-transient-local` with per-topic message counts, e.g.:

```shell
$ ros2 bag record -a --repeat-transient-local /map=1 /tf_static=5
```

Semantics:
- Each entry `<topic>=N` means “capture the last N messages observed for <topic> and re-emit
  them at the start of each new bag file created by splitting or by snapshot writes.”
- If `=N` omitted, default N=1.
- Only affects recording output (does not change playback).
- QoS convenience:
  - If `--repeat-transient-local` is used and no explicit QoS override exists for that topic,
    recorder should request Transient Local durability for that subscription (to actually receive
    the cached sample), consistent with ROS docs.

## Design overview

### Architecture: Dual-cache approach

Proposed to store the transient-local messages in a separate `TransientLocalMessagesCache` 
alongside the existing `MessageCache` or `CircularMessageCache`. This separation ensures 
transient-local messages are preserved independently of regular message traffic.

**Key principles:**

1. **Separate storage:** `TransientLocalMessagesCache` maintains per-topic FIFO queues with
   user-specified depths from `--repeat-transient-local` option.

2. **Dual write path:** Messages from transient-local topics are written to both:
   - Main cache (`MessageCache` or `CircularMessageCache`) - preserves original sequences.
   - `TransientLocalMessagesCache` - guarantees retention regardless of the original cache size 
     constraints and old messages eviction.

3. **Timestamp adjustment:** When prepending transient-local messages during split or snapshot, 
   timestamps are overwritten to avoid playback gaps:
   - **Split mode**: set to last message timestamp (T_last) from previous bag file.
   - **Snapshot mode**: set to the earliest timestamp (T_earliest) from snapshot buffer range.

4. **Deduplication in snapshot mode:** Since messages may exist in both caches, deduplication
   removes duplicates before writing to storage.

### The `TransientLocalMessagesCache` design

**Purpose:** Store the most recent `N` messages per transient-local topic, independent of
main cache pressure and size constraints.

**Required API:**
- `add_topic(topic_name, queue_depth)` - register topic with specified queue depth.
- `remove_topic(topic_name)` - unregister topic from tracking.
- `push(topic_name, message)` - add message to topic queue (evicts oldest when full).
- `get_messages_sorted_by_timestamp()` - retrieve all cached messages ordered by recv_timestamp.
- `clear()` - remove all cached messages.
- `size()` - return total number of cached messages.

**Thread safety:** 
- Must support concurrent writes from `Recorder` and reads from `Writer` during
  splits/snapshots.
- Use mutexes or lock-free structures to ensure safe concurrent access without significant
  performance degradation.
- Ensure that retrieval of messages for writing is consistent and does not interfere with ongoing
  recording.
- Design for minimal contention, as writes are expected to be infrequent (only for 
  transient-local topics) and reads occur only during splits/snapshots.

### Split mode workflow

**Process:**
1. Close current storage file.
2. Open new storage file.
3. Retrieve transient-local messages from `TransientLocalMessagesCache`.
4. Adjust timestamps to T_last (last message timestamp from previous bag file).
5. Write adjusted messages to new storage file.
6. Resume normal recording.

**Timestamp adjustment:**
- `T_last` = timestamps (`receive_timestamp` and `send_timestamp`) of last message written to
  previous bag file.
- For each transient-local message: 
  - Overwrite both `receive_timestamp` and `send_timestamp` to the corresponding timestamps
    from the `T_last`. This ensures that during playback, the player sees these messages as if
    they were published at the time of the split, preventing large gaps in playback when original
    timestamps are far in the past.

**Messages deduplication:**
  - **No deduplication needed**, since transient-local messages are written upfront before new
    recording continues, so they naturally appears only once.

**Example:**
```
Split at T=1000.5s, /tf_static originally at T=0.1s
After prepend: /tf_static written with T=1000.5s
Next message at T=1000.6s
Playback: continuous timeline without hours-long gap
```

### Snapshot mode workflow with deduplication

**Challenge:** The same transient-local messages can exist in both caches:
- `CircularMessageCache`: preserves original message sequences if received during window
- `TransientLocalMessagesCache`: guarantees retention regardless of cache pressure and old 
  messages eviction. This can lead to duplicates in the snapshot output.

**Process:**

In the `CacheConsumer::exec_consuming()` method when snapshot mode is active and data is ready:
1. Swap `CircularMessageCache` buffers to obtain the cache consumer buffer.
2. Retrieve transient-local messages from `TransientLocalMessagesCache`.
3. Determine snapshot range [`T_earliest`, `T_latest`] from cache consumer buffer.
4. Adjust timestamps for early transient-local messages to `T_earliest`.
5. Deduplicate messages returned from the `TransientLocalMessagesCache` by timestamp range.\
   Deduplication ensures deterministic output: same message does not appear twice.
6. Merge pre-sorted messages from both caches (merge step can be optimized and combined with 
   the deduplication stage since both sets are already sorted by timestamp).
7. Write deduplicated sequence to storage.

**Timestamp adjustment:**
- `T_earliest` = timestamps (`receive_timestamp` and `send_timestamp`) from the first message in 
  the consumer buffer. i.e., the message with the earliest `recv_timestamp` that will be written to
  storage during this snapshot.
- For transient-local messages with `msg::recv_timestamp` < `T_earliest::recv_timestamp`
  - Overwrite `msg::recv_timestamp` to `T_earliest::recv_timestamp`
  - Overwrite `msg::send_timestamp` to `T_earliest::send_timestamp`

**Messages deduplication:**\
Deduplication criteria:
- Messages with `recv_timestamp` >= `T_earliest::recv_timestamp` are considered duplicates of
  messages in the consumer buffer and should be excluded from merge with the consumer buffer.
- Messages with `recv_timestamp` < `T_earliest::recv_timestamp` are considered unique and should be
  included in the merge, but with adjusted timestamps to ensure they appear at the start of the
  snapshot output.

### Integration points and required API changes

**rosbag2_storage (RecordOptions extension):**
- Add field: `std::unordered_map<std::string, size_t> repeat_transient_local_messages`
  - Maps topic name to queue depth for message retention.
  - Empty map disables feature (preserves current behavior).

**ros2bag CLI:**
- Add `add_repeat_transient_local_arg` function to parse `--repeat-transient-local` arguments.
- Populate `RecordOptions.repeat_transient_local_messages` map with topic names and depths.
- Pass configuration (`RecordOptions`) through the existing `rosbag2_transport::Recorder`
  constructors (no API changes required for constructors).

**rosbag2_transport (Recorder modifications):**
- For each new subscription if topic in `repeat_transient_local_messages`, call writer's
  `create_transient_local_topic()` instead of regular `create_topic()`.
- No direct interaction with `TransientLocalMessagesCache` - all caching is internal to `Writer`.
- Message callback remains unchanged: simply calls `writer_->write(message)`.
- Writer internally handles dual-write to both storage and transient-local cache.

**rosbag2_cpp (new TransientLocalMessagesCache class):**
- Location: `rosbag2_cpp/{include,src}/cache/transient_local_messages_cache.{hpp,cpp}`
- Implements per-topic FIFO queues with configurable depths.
- Thread-safe for concurrent producer (Writer write path) and `CacheConsumer`.
- Returns messages sorted by receive timestamp for deterministic output.
- Owned and managed internally by `SequentialWriter`.

**rosbag2_cpp (BaseWriterInterface API extension):**
- Add new virtual methods to writer interface:

  ```cpp
  virtual void create_transient_local_topic(
    const rosbag2_storage::TopicMetadata & topic_with_type,
    size_t num_last_messages) = 0;
  
  virtual void create_transient_local_topic(
    const rosbag2_storage::TopicMetadata & topic_with_type,
    size_t num_last_messages,
    const rosbag2_storage::MessageDefinition & message_definition) = 0;
  ```
  
- These methods register topics for transient-local message caching and create the topic
  in the underlying storage simultaneously.
- The `num_last_messages` parameter specifies the queue depth for that topic.

**rosbag2_cpp (SequentialWriter modifications):**
- Add private member: `std::shared_ptr<TransientLocalMessagesCache> transient_local_cache_`
- Implement `create_transient_local_topic()` methods:
  - Register topic in `transient_local_cache_` with specified queue depth.
  - Forward to regular `create_topic()` to register in storage.
- Modify `write()` method:
  - After writing to storage, check if topic is registered as transient-local. If yes, also push
    message to `transient_local_cache_`.
- Modify `split_bagfile()`:
  - If not in a snapshot mode. After opening new storage file, call internal
    `prepend_transient_local_messages()`.
  - The `prepend_transient_local_messages()` method retrieves messages from cache, adjusts 
    timestamps to `T_last`, writes to storage.

**rosbag2_cpp (CacheConsumer modifications):**
- Accept `TransientLocalMessagesCache` reference in constructor or via setter.
- In snapshot mode: 
  - Retrieve and merge transient-local messages before consuming. The snapshot
    mode can be determined by checking the underlying cache type (`CircularMessageCache`) or by an
    explicit flag passed from the Recorder.
  - Perform timestamps adjustments and deduplication when merging messages from both caches to
    ensure no duplicates in snapshot output.

**Workflow summary:**
1. User specifies `--repeat-transient-local /topic=N` on command line.
2. CLI parser populates `RecordOptions.repeat_transient_local_messages["/topic"] = N`.
3. Recorder calls `writer_->create_transient_local_topic("/topic", N)` during setup.
4. The `SequentialWriter` creates topic in storage and registers it in internal cache with depth N.
5. On each `writer_->write(msg)` call for that topic:
   - Message written to storage (normal path).
   - Message also pushed to internal `TransientLocalMessagesCache` (automatic dual-write).
6. On split or snapshot:
   - SequentialWriter retrieves messages from internal cache.
   - Adjusts timestamps, deduplicates if needed.
   - Writes to new storage file.
7. Recorder remains unaware of caching implementation details.

### Determinism

- Opt-in feature via `--repeat-transient-local` flag preserves default "record what happened"
  behavior.
- Timestamp adjustment based on observable bag file timeline.
- Deduplication by timespan of the snapshot buffer ensures deterministic output: same message does
  not appear twice.
- Sorting by receive timestamp before writing ensures that there are no out of order messages 
  during replay.

## Error handling

**Invalid configuration:**
- Topic in `--repeat-transient-local` not recorded: log warning, continue recording other topics.
- Queue depth <= 0: reject with error message.

**Memory pressure:**
- Bounded by user-specified depths per topic, no unbounded growth possible.

**QoS mismatch:**
- `--repeat-transient-local` specified but publisher does not offer Transient Local: log warning.

## Performance considerations

**CPU overhead:** Dual-write adds one shared pointer copy per message; snapshot
deduplication and merging adds `O(M log M)` sorting where `M` is buffer size (negligible for typical
sizes < 100k messages). Could be reduced to the `O(M)` in merge step since both caches are 
pre-sorted.

**Disk I/O:** Split prepending adds minimal writes (one per transient-local message per split).

## Implementation plan (rolling)

### Phase 1: Core infrastructure
- Implement `TransientLocalMessagesCache` class in `rosbag2_cpp/cache`
  - Per-topic FIFO queues with configurable depths.
  - Thread-safe push and retrieval operations.
  - Messages sorted by timestamp for deterministic output.
- Extend `RecordOptions` structure in `rosbag2_storage`.
  - Add `repeat_transient_local_messages` map field.
- Unit tests:
  - Verify per-topic queue behavior and eviction at capacity.
  - Verify thread safety under concurrent access.
  - Verify timestamp-based sorting of retrieved messages.

### Phase 2: Writer API extension
- Extend `BaseWriterInterface` with new virtual methods:
  - `create_transient_local_topic()` with and without message definition.
- Modify `SequentialWriter` implementation:
  - Add private `transient_local_cache_` member.
  - Implement `create_transient_local_topic()` to register topics and forward to storage.
  - Modify `write()` to dual-write messages to cache when topic is transient-local.
- Unit tests:
  - Verify `create_transient_local_topic()` registers topic correctly.
  - Verify dual-write behavior in `write()` method.
  - Verify cache is populated with correct message count per topic.

### Phase 3: Split mode integration
- Implement `prepend_transient_local_messages()` in `SequentialWriter`.
  - Retrieve messages from cache after opening new storage.
  - Adjust timestamps to T_last (last message from previous bag).
  - Write adjusted messages to new storage file.
- Modify `split_bagfile()` to call prepend logic when not in snapshot mode.
- Integration tests:
  - End-to-end split recording with transient-local topics.
  - Verify messages prepended at start of each split.
  - Verify timestamp adjustment to `T_last`.
  - Verify no duplicates in output.

### Phase 4: Snapshot mode integration
- Modify `CacheConsumer::exec_consuming()` for snapshot mode:
  - After swapping CircularMessageCache buffers, retrieve transient-local messages.
  - Determine snapshot range [`T_earliest`, `T_latest`] from consumer buffer.
  - Adjust timestamps for messages earlier than `T_earliest`.
  - Deduplicate messages by timestamp range.
  - Merge pre-sorted sequences from both caches.
  - Pass merged sequence to consume callback.
- Add logic to detect snapshot mode (check cache type or explicit flag).
- Integration tests:
  - End-to-end snapshot recording with transient-local topics.
  - Verify messages present despite cache pressure.
  - Verify timestamp adjustment to T_earliest for early messages.
  - Verify deduplication when message appears in both caches.

### Phase 5: Recorder integration
- Extend `Recorder` to parse `repeat_transient_local_messages` from options.
- For each transient-local topic during subscription setup:
  - Call `writer_->create_transient_local_topic()` instead of regular `create_topic()`.
- No changes needed in message callback (dual-write handled by Writer).
- Integration tests:
  - Verify correct API calls during recorder initialization.
  - Verify transient-local topics registered with correct depths.
  - End-to-end recording with multiple transient-local topics.

### Phase 6: CLI and documentation
- Add `--repeat-transient-local` parsing.
- Update user documentation README.md file with usage examples and best practices.

### Test strategy

**Unit tests (rosbag2_cpp):**
- `TransientLocalMessagesCache`:
  - Test per-topic queue with depth limits.
  - Test eviction of oldest messages when full.
  - Test thread-safe concurrent push and retrieval.
  - Test timestamp-based sorting of output.
  - Test empty cache behavior.
  - Test invalid configurations (depth <= 0).

- `SequentialWriter`:
  - Test `create_transient_local_topic()` registration.
  - Test dual-write to storage and cache.
  - Test prepend logic during split.
  - Test timestamp adjustment to `T_last`.
  - Test merge and deduplication in snapshot mode.
  - Test timestamp adjustment to `T_earliest`.

**Integration tests (rosbag2_transport):**
- Split mode scenarios:
  - Record with small `max_bagfile_size` to trigger splits.
  - Publish `/tf_static` at T=0.1s, trigger split at T=1000.5s.
  - Verify each split starts with `/tf_static` at `T_last` timestamp.
  - Verify no duplicates in output.
  - Republish `/tf_static` after first split, verify it does NOT appear in next split.
  - Test multiple transient-local topics with different depths.

- Snapshot mode scenarios:
  - Record with small `max_cache_size` to trigger cache pressure.
  - Publish `/tf_static` at T=0.1s, overflow cache with regular messages.
  - Trigger snapshot, verify `/tf_static` present despite eviction.
  - Verify timestamp adjusted to `T_earliest` for early messages.
  - Test deduplication:
    - Publish `/tf_static`, republish during snapshot window.
    - Verify no duplicates in output.
    - Verify `CircularMessageCache` timestamp preserved for duplicates.
  - Test `/tf_static` published only before snapshot window:
    - Verify adjusted timestamp to `T_earliest`.
    - Verify message appears exactly once.

- QoS negotiation:
  - Test recorder automatically requests Transient Local for transient-local topics.
  - Test explicit QoS override takes precedence.
  - Test warning logged when publisher does not offer Transient Local.

- Edge cases:
  - Empty cache (no messages received before split/snapshot).
  - Topic in `--repeat-transient-local` but never published.
  - Queue depth of 1 versus larger depths.
  - Multiple transient-local topics with mixed depths.
  - Transient-local message larger than cache capacity.

## Alternatives considered

**Single cache with priority queues:** Rejected, complicates eviction logic and risks
starving regular messages.

**Percentage reservation in CircularMessageCache:** Cannot guarantee preservation or
provide per-topic control.

**Special-case /tf_static only:** Lacks generality for custom transient-local topics.

**No timestamp adjustment:** Creates unacceptable playback gaps (player waits hours).

**Store transient-local messages only in the circular_message_cache:** Lack of support for the 
regular bag split mode.

**Store only in TransientLocalMessagesCache:** Breaks snapshot mode's ability to preserve
original message sequences.

## Open questions

1. Should we use `--repeat-tl` or `--repeat-transient-local` or `--repeat-latched` for the CLI 
   option name?
   - `--repeat-latched` is familiar to ROS 1 users but may be confusing for ROS 2 users since
     "latched" is not a term used in ROS 2 QoS. It also may not be clear that it applies to
     recording and not playback.
   - `--repeat-tl` is more concise and consistent with ROS 1 `--repeat-latched`, but less
     descriptive. `--repeat-transient-local` is more explicit but verbose.
   - Proposed: `--repeat-transient-local` as a more concise and explicit option. Users got 
     confused with the `--repeat-tl` and don't understand what does it mean.

2. Should we support regex patterns in `--repeat-transient-local` (e.g., `/tf_*`)?
   - Deferred to future work based on user feedback. Initial implementation focuses on explicit
     topic names for simplicity and clarity.

3. Should we provide option to disable timestamp adjustment?
   - **No**. It is necessary for usable playback in all practical scenarios.

4. Should we log deduplication statistics?
   - Yes, add DEBUG-level logging for transparency during testing.

5. Should we support different timestamp adjustment strategies?
   - Deferred. Current strategy covers all known use cases.
