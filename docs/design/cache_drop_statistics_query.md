# Querying cache insertion failures

This design proposes a narrow C++ API for inspecting the counters already maintained by
`MessageCache`. It does not implement the recorder statistics service proposed in
`statistics_about_lost_and_recorded_messages.md`. In particular, it does not report transport or
storage failures, recorded messages, bytes written, or per-file statistics. The existing incremental
messages-lost event combines recorder-side sources and is not interchangeable with this counter.

## Contract

`Writer::get_cache_drop_statistics()` returns a point-in-time copy containing a status and a map
from topic name to the number of failed insertions counted by the normal `MessageCache`. A failed
insertion of a non-null message is counted whenever the producer buffer rejects it. This includes
size or duration limits, invalid serialized data, and out-of-order timestamps rejected by the buffer;
a null message has no topic and is not counted. Topics without failures are absent. Counts are
64-bit and match `log_dropped()` at the time of the read. The original protected 32-bit counter
remains in place for ABI compatibility, while an internal shared buffer state records its overflows.

* `available`: the map is meaningful and may be empty.
* `cache_disabled`: no cache is configured; this says nothing about storage failures.
* `unsupported`: the cache or writer implementation does not provide this statistic. The snapshot
  circular cache is currently unsupported: its normal eviction of old messages is not treated as a
  cache insertion failure, and this API does not count its rejected messages either.
* `writer_not_open`: the writer has not opened a recording, or no final statistics were retained.
* `busy`: the Writer lock is held, the query is reentrant, or the lock could not be acquired.
  No counts are returned; retry outside the current operation or callback.
* `counter_saturated`: one or more counts reached `UINT64_MAX`. Such values are lower bounds and
  remain at that limit rather than wrapping. The log marks them with `>=`.

The counter starts with a successful `open()` and is not reset on a bag split. The normal
`SequentialWriter` retains a final copy after `close()` with `is_final` set to true; its cache and
buffers are released as before. A new open attempt clears the final copy, even if opening fails.
The API does not persist counts to bag metadata. Disabled caches, snapshot caches, and custom
writers do not necessarily retain a final copy; a custom provider defines its own lifetime semantics.

## Concurrency and compatibility

`MessageCache` protects counter increments and snapshot copies with its existing producer mutex;
`log_dropped()` formats a copy of the same counters. `Writer` attempts to acquire its existing
mutex without waiting; on contention or reentrancy it returns `busy`. This makes callback queries
safe from the Writer lock deadlock, but a callback must defer and retry to obtain counts. An
independent provider must synchronize its own state and avoid acquiring the Writer lock while holding
its own locks. A caller using `SequentialWriter` directly must serialize the query with `open()` and
`close()`; concurrent cache updates and queries are protected by the cache mutex. Copying a large map
and frequent polling can delay writes.

The existing `BaseWriterInterface` and `MessageCacheInterface` are unchanged. No virtual methods,
base classes, or data members are added to existing classes. The new methods on `Writer`,
`SequentialWriter`, and `MessageCache` are non-virtual, and the existing mutexes are made mutable
without changing their types or placement. The query uses RTTI: it first checks the independent
`CacheDropStatisticsProvider` capability, then recognizes the normal sequential writer and normal
message cache by exact type. An implementation without either path returns `unsupported`, including
the built-in `SequentialCompressionWriter` until it explicitly supports the capability. In
particular, a `MessageCache` subclass must opt in even if it inherits the base getter: the protected
counter map could be modified without the base cache's private mutex. Existing custom writers
and caches need no source changes and continue to use their original virtual tables. Custom
implementations that are rebuilt can opt in by also implementing `CacheDropStatisticsProvider`;
the provider must satisfy the same counting and thread-safety contract. A custom cache provider's
statistics need not match the normal `MessageCache` log unless it explicitly uses those counters.
Calling the inherited `MessageCache` getter directly is likewise unsafe concurrently with a subclass
that modifies the protected map without the base cache's lock. Custom subclasses must supply their
own synchronization for any such mutations and queries.

Existing binaries should continue to work with the updated library on a compatible compiler and C++
runtime; applications using the new methods require the updated library. Source-level review alone
is not proof of ABI compatibility: compare baseline and modified libraries with debug information and
test old compiled implementations against the updated library. RTTI and exported type information
must work across the relevant shared-library boundaries. This remains a Rolling proposal; a Jazzy
backport, other platforms, and upstream acceptance need separate review.

## Verification performed

On Linux x86-64 using Ubuntu 26.04, ROS Rolling and GCC 15.2.0, the baseline at commit
`737e0fae1edb804a8c61499204384ac8ec7802de` and this implementation were built with the same
dependencies and debug information. `abidiff` with the public header directories and added symbols
filtered reported no removed or changed functions or variables; a separate dynamic-symbol comparison
found no removed exports. The added query and provider symbols require the updated library when used.

An executable and a shared plugin compiled against the baseline headers and library ran against
both baseline and updated libraries. The plugin exercised the existing writer and cache interfaces,
and subclasses of `SequentialWriter` and `MessageCache`. An executable built against the updated
headers also loaded the old plugin, received `unsupported` for its legacy writer, and loaded a new
plugin implementing the optional capability for both a writer and a cache. The `rosbag2_cpp` CTest
suite passed; its default cppcheck task skipped version 2.x, so a separate targeted cppcheck 2.13 run
was explicitly enabled and passed. The changed library and six focused cache and writer tests
also passed under GCC ThreadSanitizer, with address randomization disabled in WSL. The ROS
dependencies were not instrumented, so this is not a complete race check. These checks do not
establish ABI compatibility for Windows, other compiler or standard-library versions, or a
different ROS distribution.
