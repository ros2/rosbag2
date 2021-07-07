# Rosbag2 Snapshot

## Context

The following design handles functionality of keeping data from topics in buffer of specified size or from specific time duration and saving them after trigger.

Terms:
* "Snapshot" - save buffered data from topics to disk as rosbag
* "Trigger" - an invocation letting a node know when to save rosbag to disk
* "Recorder" - rosbag2 node which records data right after program start until killed

## Feature description

Rosbag in ROS1 had functionality of creating a [snapshot](https://github.com/ros/rosbag_snapshot). Snapshotter was a program which subscribed to topics and buffered messages until a `trigger` for snapshot was sent. As in Rosbag2 we want to support the following features:

* Save data to rosbag on trigger
  * Enable triggering buffer save to disk with service call

* Pause and resume buffering process
  * Pausing refers to stopping buffering new messages until resumed or write is triggered.
  * Resuming continues the process of buffering after pause. Resuming would clear all buffers.

* Node and buffer configuration

  Enable user to specify `cli` arguments which could be shared with `ros2 bag record` command. User should be able to configure the following arguments:

    | parameter name | description   | behavior  |
    | -------------- | ------------- | --------- |
    | buffer size    | maximum memory per topic | <ul><li> defined by new `--max-buffer-size` argument, default value is 0 [bytes]  </li><li>if not set/set to non-positive value only duration parameter will be relevant (*note: either buffer size or duration must be specified*) </li></ul> |
    | duration       | maximum time difference between newest and oldest message | <ul><li> defined by new `--max-buffer-duration` argument, default value is 10.0 [s] </li><li> if not set/set to non-positive value only buffer size parameter will be relevant (*note: either buffer size or duration must be specified*) </li></ul> |
    | subscribed topics | list of topics to buffer | <ul><li> makes use of `topics`, `--regex`, `--exclude`, `--all` arguments from `ros2 bag record` </li><li> if a topic is not available it won't be recorded </li></ul> |
    | output rosbag filename | name of snapshot file saved to disk | <ul><li> defined by existing `--output` argument </li><li> rosbag name will be suffixed with timestamp data to prevent name collisions after triggering multiple snapshots </li><li> if not specified, default, timestamped format name will be used </li></ul> |


## User interface

This section shows how user can interact with the following feature.

```bash
# Running Recorder node in buffer mode for all topics with default configuration
ros2 bag record --snapshot --all

# Running Recorder node in buffer mode with all snapshot specific arguments
ros2 bag record --snapshot --max-buffer-size 5000000 --max-buffer-duration 30.0 --output snapshot /example_topic1 /example_topic2

# Triggering a snapshot with a service call
ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot "{ }"

# Reconfigure snapshot max memory configuration
ros2 service call /rosbag2_recorder/snapshot/memory rosbag2_interfaces/srv/SetMemory "{memory: 1000000}"

# Reconfigure snapshot max duration configuration
ros2 service call /rosbag2_recorder/snapshot/duration rosbag2_interfaces/srv/SetDuration "{duration: 50.0}"

# Clear snapshot buffer
ros2 service call /rosbag2_recorder/snapshot/clear_buffer rosbag2_interfaces/srv/ClearBuffer "{ }"

# Pause messages buffering
ros2 service call /rosbag2_recorder/snapshot/pause rosbag2_interfaces/srv/Pause "{ }"

# Resume messages buffering
ros2 service call /rosbag2_recorder/snapshot/resume rosbag2_interfaces/srv/Resume "{ }"
```

## Design

### Main assumptions

* Snapshot functionality makes use of producer-consumer cache patter which must be always turned on.
* Main idea is to use `split_bag()` functionality in `SequentialWriter` for creating snapshots. In current implementation storage `open()` must be called first before any other function. This design manipulates start_time and duration metadata in `SnapshotWriter::write` just to match the state of buffered mesages.
* Additionally this design forces that before `Recorder` node shut down, snapshot is getting saved to disk anyway.
* During arguments parsing, it should be checked if snapshot argument is true. If it's true one of arguments must be positive. In case both config arguments are invalid (non positive values) an exception should be thrown. In case config arguments are set but snapshot mode is not triggered, both config arguments should be set to invalid values (non positive) and adequate log should be displayed.
* This project blocks the possibility to split the bag during runtime because disk save happens once per some amount of time and it is already out of the SequentialWriter scope to interfere.
* For now implementation assumes implementing separate consumer-producer pattern. Idea is to create separate `SnapshotConsumer` and `SnapshotCache` classes (with different names but sharing same interfaces/deriving most functionality from `CacheConsumer` and `MessageCache`). They use similar mechanism as used in bag splitting. Implementation differs mostly in `SnapshotConsumer::close()` and `SnapshotConsumer::~SnapshotConsumer`. Producer-consumer pattern comes into flushing state only after destructor call. Calling bag change/split doesn't prevent `SnapshotCache` from receiving new messages. Additionally buffers swap is only trigerred after `SnapshotConsumer::close()`, which is expected.
* It should be considered that when buffer doesn't contain any messages to write, rosbag folder should be **deleted** but for now **it is not designed**.

### Pros

* Reuses vast part of `SequentialWriter` class code
* Accidental `Recorder` node (in snapshot mode) shutdown creates snapshot
* Producer-consumer patter reusage grants good performance


### Cons

* Prevents bag splitting
* Problematic configuration, pausing and resuming feature (no direct way to change behavior during runtime)
* Problematic messages counting and rosbag start timestamping
* (see `TODO` section)

## In-depth structure changes

```diff

namespace rosbag2_transport
{
struct RecordOptions
{
+ /// New variable indicating snapshot option
+ bool snapshot;
}
}

namespace rosbag2_storage
{

struct StorageOptions
{
+ /// New attributes for snapshot
+ double max_snapshot_duration;
+ uint64_t max_snapshot_memory;
}
}


namespace rosbag2_cpp
{
namespace cache
{
 /// Extract main functionality from MessageCacheBuffer and make it an interface
+ class CacheBufferInterface
+ {
+ public:
+   /// Push message into buffer
+   virtual bool push(buffer_element_t msg) = 0;
+
+   /// Clear buffer
+   virtual void clear() = 0;
+
+   /// Get number of elements in the buffer
+   virtual size_t size() = 0;
+
+   /// Get buffer data
+   virtual const std::vector<buffer_element_t> & data() = 0;
+ }

+ class MessageCacheBuffer : public CacheBufferInterface
{
  /// Implementation stays as it was, few overrides needs to be added
}

+ class MessageSnapshotBuffer : public CacheBufferInterface
{
public:
+ explicit MessageSnapshotBuffer(const uint64_t max_cache_size, const double max_duration) :
+ max_bytes_size_(max_cache_size),
+ max_duration_(max_duration) {};

  (...)
+ /**
+ * At first it is checked if last added message fits (now() - max_duration) period of time. If it doesn't it is being removed from buffer.
+ * This sequence is being repeated until any message satisfies the condition or we empty the buffer.
+ * Secondly new message is being tested with the same condition.
+ * After that it is checked if buffer size + message size exceeds defined max_buffer_size.
+ * If buffer size is exceeded new message won't be added to buffer.
+ */
+ bool push(buffer_element_t msg) override;
private:
  /// Buffer for all msgs
  std::vector<buffer_element_t> buffer_;
+  /// Since every max_bytes_size_ refers to maximum disk space per topic it is convenient to trace data size with key(topic name), value(data size) map
+  std::unordered_map<std::string, uint64_t> topic_names_to_data_size;
- std::atomic_bool drop_messages_ {false};
  /// Max buffer data size in bytes
  const uint64_t max_bytes_size_;
+  /// Max duration in past to accept new message
+  const double max_duration_;
-   /// This is irrelevant in this implementation.
- std::atomic_bool drop_messages_ {false};


/// abstract class
+ class Cache
+ {
+   virtual ~Cache() = default;
+
+   /// Puts msg into primary buffer. With full cache, msg is ignored and counted as lost
+   virtual void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) = 0;
+
+   /// Summarize dropped/remaining messages
+   virtual void log_dropped() = 0;
+
+   /// Producer API: notify consumer to wake-up (primary buffer has data)
+   virtual void notify_buffer_consumer() = 0;
+
+   /// Set the cache to consume-only mode for final buffer flush before closing
+   virtual void finalize() = 0;
+
+   /// Notify that flushing is complete
+   virtual void notify_flushing_done() = 0;
+
+   virtual void wait_for_buffer() = 0;
+
+   /// Consumer API: get current buffer to consume
+   virtual std::shared_ptr<CacheBufferInterface> consumer_buffer() = 0;
+
+   /// Exposes counts of messages dropped per topic
+   virtual std::unordered_map<std::string, uint32_t> messages_dropped() = 0;
+
+   /// Function returning earliest buffered message timestamp
+   /// Works for sequential write() function call which is assured by Writer and SequentialWriter classes
+   std::chrono::time_point<std::chrono::high_resolution_clock> get_buffer_start_timestamp()
+
+ protected:
+   /// Double buffers sync (following cpp core guidelines for condition variables)
+   bool primary_buffer_can_be_swapped_ {false};
+   std::condition_variable cache_condition_var_;
+   std::mutex cache_mutex_;
+
+   /// Cache is no longer accepting messages and is in the process of flushing
+   std::atomic_bool flushing_ {false};
+ }


+ class SnapshotCache : public Cache
+ {
+ public:
+  explicit SnapshotCache
+  (uint64_t max_snapshot_memory, double max_snapshot_duration)
+  {
+    primary_buffer_ = std::shared_ptr<cache::MessageSnapshotBuffer>(max_snapshot_memory, max_snapshot_duration)
+    secondary_buffer_ = std::shared_ptr<cache::MessageSnapshotBuffer>(max_snapshot_memory, max_snapshot_duration)
+  }

+ /// Puts msg into buffer. Doesn't end with notifying cache consumer, since it needs to be triggered with snapshot()
+ /// Doesn't need to log dropped messages
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
  {
    (...) /// The rest of implementation similar to MessageCache
-   notify_buffer_consumer();
  }

+ /// Function returning earliest buffered messages timestamp
+ /// Works for sequential write() function call which is assured by Writer and SequentialWriter classes
+ std::chrono::time_point<std::chrono::high_resolution_clock> get_buffer_start_timestamp()
+ {
+   std::lock_guard<std::mutex> cache_lock(cache_mutex_);
+   std::vector<buffer_element_t> primary_buffer_data = primary_buffer_->data();
+   if (primary_buffer_data.size() != 0)
+   {
+     return primary_buffer_data.front()->timestamp;
+   }
+   return std::chrono::time_point<std::chrono::high_resolution_clock>::min();
+ }

+ private:
+   /// Type change
+   /// Double buffers
+   std::shared_ptr<MessageSnapshotBuffer> primary_buffer_; TODO change to interface and push to Cache?
+   std::shared_ptr<MessageSnapshotBuffer> secondary_buffer_;

}

+ class MessageCache : public Cache
{
public:
+ /// Type change
+ std::shared_ptr<CacheBufferInterface> MessageCache::consumer_buffer() {(...)}

+ /// Dummy function, won't be called but should be implemented in similar way to SnapshotCache
+ std::chrono::time_point<std::chrono::high_resolution_clock> get_buffer_start_timestamp() {(...)};

private:
 /// Type change
+ /// Double buffers
+ std::shared_ptr<MessageCacheBuffer> primary_buffer_;
+ std::shared_ptr<MessageCacheBuffer> secondary_buffer_;

}


+ /// Add factory to create Cache derivative type instances
+ class CacheFactory
+ {
+ public:
+   CacheFactory() {};
+   virtual ~CacheFactory() = default;
+
+   explicit std::shared_ptr<cache::Cache>
+   create_cache(uint64_t max_buffer_size, uint64_t max_snapshot_memory, double max_snapshot_duration)
+   {
+     if (max_buffer_size > 0u)
+     {
+       return std::shared_ptr<cache::MessageCache>(max_buffer_size);
+     }
+     else
+     {
+       return std::shared_ptr<cache::SnapshotCache>(max_snapshot_memory, max_snapshot_duration);
+     }
+   }
+
+ }
+


+ /// abstract class for easier deriviation
+ class Consumer
+ {
+   /// shut down the consumer thread
+   virtual void close() = 0;
+
+   /// Set new consume callback, restart thread if necessary
+   virtul void change_consume_callback(consume_callback_function_t callback) = 0;
+
+ protected:
-   std::shared_ptr<MessageCache> message_cache_;
+   consume_callback_function_t consume_callback_;
+
+   /// Write buffer data to a storage
+   virtual void exec_consuming() = 0;
+
+   /// Consumer thread shutdown sync
+   std::atomic_bool is_stop_issued_ {false};
+   std::mutex consumer_mutex_;
+
+   std::thread consumer_thread_;
+ }


+ class CacheConsumer : public Consumer
+ {
+   /// No implementation changes, overrides needed
+ private:
+   std::shared_ptr<MessageCache> message_cache_;
+ }


+class SnapshotConsumer : public Consumer
{
+ /// Flushing state triggered only when closing node
~SnapshotConsumer() override
{
+ message_cache_->finalize();
  close();
+ message_cache_->notify_flushing_done();
}

+ /// Close operation triggered when doing snapshot
+ /// MessageCacheBuffer won't stop buffering messages while creating snapshot (no flushing)
void close()
{
- message_cache_->finalize();
  is_stop_issued_ = true;

  ROSBAG2_CPP_LOG_INFO_STREAM(
    "Writing remaining messages from cache to the bag. It may take a while");

  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
- message_cache_->notify_flushing_done();
}

+ /// Rest of implementation is similar to CacheConsumer

+ private:
+   std::shared_ptr<SnapshotCache> message_cache_;
}


+ /// Add factory to create Consumer derivative type instances
+ class ConsumerFactory
+ {
+ public:
+   ConsumerFactory() {};
+   virtual ~ConsumerFactory() = default;
+
+   explicit std::unique_ptr<cache::Consumer>
+   create_consumer(std::shared_ptr<MessageCache> message_cache,
+                            consume_callback_function_t consume_callback)
+   {
+     return std::make_unique<cache::CacheConsumer>(message_cache, consume_callback);
+   }
+
+   explicit std::unique_ptr<cache::Consumer>
+   create_consumer(std::shared_ptr<SnapshotCache> message_cache,
+                            consume_callback_function_t consume_callback)
+   {
+     return std::make_unique<cache::SnapshotConsumer>(message_cache, consume_callback);
+   }
+
+ }
+
+ }

}
}

namespace rosbag2_cpp
{
namespace writer_interfaces
{

class BaseWriterInterface
{
public:
  (...)

+  /// Add non-pure virtual function for snapshot to easily call it from rosbag2_cpp::Writer
+  virtual snapshot() { };
}
}


namespace writers
{
class SequentialWriter
{
public:
  /// Force to use cache in snapshot mode
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const ConverterOptions & converter_options)
  {
    (...)
+    snapshot_ = (storage_options.max_snapshot_duration > 0.0 || storage_options.max_snapshot_memory > 0u);
+    use_cache_ = (storage_options.max_cache_size > 0u) || snapshot_);
+    if (use_cache) {
+      CacheFactory cache_factory = CacheFactory();
+      ConsumerFactory consumer_factory = ConsumerFactory();
+      message_cache_ = cache_factory.create_cache(
+        storage.max_buffer_size,
+        storage_options.max_snapshot_memory,
+        storage_options.max_snapshot_duration);
+      cache_consumer_ = consumer_factory.create_consumer(
+        message_cache_,
+        make_callback(
+          storage_,
+          topics_names_to_info_,
+          topics_info_mutex_));
+    }
    (...)
  }

  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    (...)
    /// Prevent splitting bag in snapshot mode
+   if (!snapshot_ && should_split_bagfile())
    {
      split_bagfile();

      // Update bagfile starting time
      metadata_.starting_time = std::chrono::high_resolution_clock::now();
    }
+   if (snapshot_)
+   {
+     /// Update bagfile starting time (first received message from message_cache) - something like
+     auto buffer_start_timestamp = message_cache_->get_buffer_start_timestamp();
+     if (buffer_start_timestamp != std::chrono::time_point<std::chrono::high_resolution_clock>::min())
+     {
+       metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
+         std::chrono::nanoseconds(buffer_start_timestamp));
+     }
+   }
    (...)
    /// Force using message cache in snapshot mode
+   if (storage_options_.max_cache_size == 0u && !snapshot_) {
      // If cache size is set to zero, we write to storage directly
      storage_->write(converted_msg);
      ++topic_information->message_count;
    } else {
      // Otherwise, use cache buffer
      message_cache_->push(converted_msg);
    }
  }

  /// Add function implementation, it uses split bagfile functionality
+ void snapshot() override
+ {
    /// not sure if have to call message_cache_->notify_buffer_consumer() first
+   /// call split_bagfile() remembering that use_cache_ is set to true because of opening file with record_options
+   split_bagfile();
+ }

private:
+ /// Add snapshot_ field
+ bool snapshot_;
}
}
}


namespace rosbag2_cpp
{
class Writer
{
public:
+  /// Add function to create snapshot instantly. It should take care of saving messages to disk, closing bagfile and opening new one
+  void snapshot()
+  {
+    writer_impl_->snapshot();
+  }
}
}


namespace rosbag2_transport
{
namespace impl
{

class Recorder : public rclcpp::Node
{
public:
+  /// Add callback to handle snapshot functionality
+  snapshot(const std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Request> request,
+          std::shared_ptr<rosbag2_interfaces::srv::Snapshot::Response> response)
+  {
+    if (record_options_.snapshot)
+    {
+      writer_->snapshot();
+      return true;
+    }
+  return false;
+  }

private:
+  /// Add service to handle snapshot functionality
+  rclcpp::Service<rosbag2_interfaces::srv::Snapshot>::SharedPtr snapshot_srv;
}
}
}


```

<!-- ### Flushing fix proposal

New idea to improve snapshotting is to create separate `CacheConsumer` and `MessageCache` classes (with different names but sharing same interfaces/deriving most functionality). It all comes to the fact that flushing prevents `MessageCache` from pushing messages to `MessageCacheBuffer` while bag splitting. Moreover every push ends with `notify_buffer_consumer()` which triggers buffer swap, but when snapshotting we want to switch buffers only when saving bagfile to disk. That is why I would opt for changing `CacheConsumer::close()` and `CacheConsumer::~CacheConsumer` action to more suitable for snapshotting.

```diff

void MessageCache::push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  (...)
-  notify_buffer_consumer();
}
```

This change would distinguish calling `SequentialWriter::switch_to_next_storage()` (as snapshot) and `SequentialWriter::reset()` (which is called only in destructor).Additionally we would acquire data the whole time during closing one and opening another storage. It would require some more work to apply classes deriviation, unify interfaces and develop mechanism for objects creation.

Presented idea won't work in normal bag recording (and snapshotting) due to writing closed storage object inside consumer thread during bag split. In snapshot mode even program close (Ctrl-C) won't break the program behavior due to `MessageCache::flushing_` flag set when closing program and proper mutex locking during pushing and waiting for buffer swap. -->

### New Services

rosbag2_interfaces/srv/Snapshot

```
---
```

rosbag2_interfaces/srv/SetMemory

```
float64 memory
---
bool success  # true if memory value is positive or non-positive but duration is positive
```

rosbag2_interfaces/srv/SetDuration

```
float64 duration
---
bool success  # true if memory value is positive or non-positive but memory is positive
```

rosbag2_interfaces/srv/ClearBuffer

```
---
```

rosbag2_interfaces/srv/Pause and rosbag2_interfaces/srv/Resume are shared with `Player` node.

### Feature implementation tests

* `CacheBufferFactory` check if returns good `CacheBufferInterface` type instances, dependent on passed arguments
* `MessageSnapshotBuffer` check pushing behavior
* `MessageCache` check synchronization and start timestamp calculation
* `SequentialWriter` test if snapshot forces using cache, starting timestamp change in snapshot mode and bag split when calling snapshot
* `Recorder` service call check

#### TODO

* Deleting empty rosbag after node closure
* Resolve how to implement pause/resume functionality
* Think about configuration change during runtime

<!-- Other code ideas -->

<!-- ########################################################################################################################
namespace writers
{
class SequentialWriter
{
  /**
   * Opens a new bagfile and prepare it for writing messages. The bagfile must not exist.
   * This must be called before any other function is used.
+  * It should ommit splitting file sections and force use_cache variable to be true if storage_options_.snapshot is true
   **/
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    const ConverterOptions & converter_options)
  {
    /// TODO to zero
    storage_options_.uri = format_storage_uri(base_folder_, 0);

    (...)
    if (storage_options_.max_bagfile_size != 0 &&
      storage_options_.max_bagfile_size < storage_->get_minimum_split_file_size() &&
+      !storage_options_.snapshot)
    {
      std::stringstream error;
      error << "Invalid bag splitting size given. Please provide a value greater than " <<
        storage_->get_minimum_split_file_size() << ". Specified value of " <<
        storage_options.max_bagfile_size;
      throw std::runtime_error{error.str()};
    }

+    use_cache_ = (storage_options.max_cache_size > 0u || storage_options_.snapshot);
    (...)
  }

  /// Eliminate split bagfiles functionality when in snapshot mode
  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    (...)
+    if (!storage_options_.snapshot && should_split_bagfile()) {
    split_bagfile();

    // Update bagfile starting time
    metadata_.starting_time = std::chrono::high_resolution_clock::now();
  }
  (...)
+  if (storage_options_.max_cache_size == 0u && !storage_options_.snapshot) {
    // If cache size is set to zero, we write to storage directly
    storage_->write(converted_msg);
    ++topic_information->message_count;
  } else {
    // Otherwise, use cache buffer
    message_cache_->push(converted_msg);
  }
  }

+  /// Function that covers opening new storage file, saving current cached data, closing file and restarting message_cache and consumer.
+  /// It has to manipulate with `metadata_.relative_file_paths` to numerate every snapshot file during one program run.
+  void snapshot_bagfile();
    format_storage_uri()

}
}
}
######################################################################################################################
rosbag2_transport::RecordOptions new values: bool snapshot;
rosbag2_storage::StorageOptions new values: double max_snapshot_duration, uint64_t max_snapshot_memory;

rosbag2_transport::Recorder new values:
# +  set_snapshot_duration(const std::shared_ptr<rosbag2_interfaces::srv::SetDuration::Request> request,
# +          std::shared_ptr<rosbag2_interfaces::srv::SetDuration::Response> response);
# +  set_snapshot_memory(const std::shared_ptr<rosbag2_interfaces::srv::SetMemory::Request> request,
# +          std::shared_ptr<rosbag2_interfaces::srv::SetMemory::Response> response);
# +  pause_snapshot(const std::shared_ptr<rosbag2_interfaces::srv::Pause::Request> request,
# +          std::shared_ptr<rosbag2_interfaces::srv::Pause::Response> response);
# +  resume_snapshot(const std::shared_ptr<rosbag2_interfaces::srv::Resume::Request> request,
# +          std::shared_ptr<rosbag2_interfaces::srv::Resume::Response> response);
# +  clear_snapshot_buffer(const std::shared_ptr<rosbag2_interfaces::srv::ClearBuffer::Request> request,
# +          std::shared_ptr<rosbag2_interfaces::srv::ClearBuffer::Response> response);

Recorder::snapshot() as Snapshot service callback;
Recorder::set_snapshot_duration() SetDuration as service callback;
Recorder::set_snapshot_memory() as SetMemory service callback;
Recorder::pause_snapshot() as Pause service callback;
Recorder::resume_snapshot() as Resume service callback;
Recorder::clear_snapshot_buffer() as ClearBuffer service callback;
# +  rclcpp::Service<rosbag2_interfaces::srv::SetMemory>::SharedPtr snapshot_memory_srv;
# +  rclcpp::Service<rosbag2_interfaces::srv::SetDuration>::SharedPtr snapshot_duration_srv;
# +  rclcpp::Service<rosbag2_interfaces::srv::Pause>::SharedPtr snapshot_pause_srv;
# +  rclcpp::Service<rosbag2_interfaces::srv::Resume>::SharedPtr snapshot_resume_srv;
# +  rclcpp::Service<rosbag2_interfaces::srv::ClearBuffer>::SharedPtr snapshot_clear_srv;

# add RecordOptions to let know writer that it works in normal or snapshot mode
rosbag2_cpp::BaseWriterInterface::open(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const ConverterOptions & converter_options)

# add RecordOptions to let know writer that it works in normal or snapshot mode
rosbag2_cpp::writers::SequentialWriter::open(
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options,
    const ConverterOptions & converter_options)

# change write function logic wether it is in snapshot/record mode
rosbag2_cpp::writers::SequentialWriter::write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override; -->


<!-- Other designs -->
<!-- As thinking of this design in the past `rosbag_snapshot` was written as different package than `rosbag`. It was defined this way not to break the existent API of `rosbag`. Taken this into consideration it could be done so this time too. However it would require duplicating a large part of code which is already written and being optimized as a part of `rosbag2` repository. So as not being sure which way would be the most appropriate to implement I propose multiple approaches.

### Writer Interfaces changes + `Recorder` modifications

Second approach would base on making changes to `Recorder`, writers and their interfaces. This approach bases on adding service to `Recorder` which handles triggering buffer save. Although current design of writers bases on `BaseWriterInterface` which would need to be changed to enable some kind of `save()/flush()` functionality. I believe this is not an elegant approach but I haven't find other way to trigger buffer save from `Recorder`, through `Writer` to `SequentialWriter`. For sure it requires implementing new `SnapshotWriter` since it needs to take care of buffering the messages from subscribers instead of passing them straight to `MessageCache` and `CacheConsumer`.

Pros:
* Maintaining snapshot inside `ros2 bag snapshot ...` (or even `ros2 bag record snapshot ...`) and `Recorder` functionality
* Reuse most of already written code (producer-consumer in save, abstract interfacing between writers)

Cons:
* `BaseWriterInterface` redefinition (adding at least one pure abstract function)
* May require many changes in current implementation of the above mentioned classes -->


<!-- ### Outside rosbag2

First approach implements `ros2 bag snapshot` as a different package. Mostly it requires rewriting `Recorder` node completly. Completly means in this case diving into implementation of `Writer`, `SequentialWriter` and `BaseWriterInterface` to copy and adjust writing to disk algorithms.

Pros:
* Design flexibility

Cons:
* Many new functions would be redefined or even redundant
* Breaking the initial desing of writers/readers in favor of putting more effort to custom mechanism
* Not being part of `rosbag2` and `ros2bag` (cli incompatibility) -->

<!-- ### Custom `Recorder` implementation

Last but not least it is possible to write new alternative to `Recorder` node inside `rosbag2_transport` package. `Snapshoter` would implement functionalities of subscribing the messages, buffering them at high level node and most probably passing data straight to `MessageCache` and `CacheConsumer`.

Pros:
* Creating independent piece of code
* Adding new cli phrase and leaving package-wise compatibility inside `rosbag2` repository

Cons:
* Many new functions would be redefined or even redundant
* Mixing high and low level of abstraction in one piece of code
* Feels like putting much effort than it is needed -->