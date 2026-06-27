# Design: Asynchronous recording error handling

## Context

Rosbag2 recording can write messages to storage from background threads. In the current
architecture, some classes own their worker threads directly instead of delegating all asynchronous
execution to a single task owner. For example, `rosbag2_cpp::cache::CacheConsumer` owns the thread
that drains the writer cache and calls the storage write callback.

This means file I/O errors can be raised in a different thread from the `ros2 bag record` control
flow. A storage plugin can throw while the cache consumer thread is writing data, but the recorder
main loop, writer API caller, and CLI process cannot observe that exception automatically. The
failure must be captured in the worker thread and surfaced later from a thread that can safely
stop recording and report the error.

The purpose of this document is to define the policy used by the current architecture for
propagating asynchronous recording failures.

## Goals

- Surface file I/O failures from asynchronous writer/cache threads to the recorder control flow.
- Preserve the original exception type and message where possible.
- Stop recording promptly after a background write failure is detected.
- Keep bag finalization best-effort, so metadata and storage close handling can still run for data
  that was written successfully.
- Keep destructors exception-safe.

## Non-goals

- Redesign thread ownership in the recording pipeline.
- Define a generic asynchronous task framework for all rosbag2 components.
- Change storage plugin exception types or storage plugin transaction semantics.
- Guarantee that a bag is complete or recoverable after a fatal storage failure such as a full
  disk. The policy only defines how the failure is propagated and how cleanup is attempted.

## Scope

This policy applies to asynchronous failures that occur while recording, especially failures from
storage writes executed by the writer cache consumer. The main implementation points are:

- `rosbag2_cpp::cache::CacheConsumer`
- `rosbag2_cpp::writers::SequentialWriter`
- `rosbag2_cpp::Writer`
- `rosbag2_compression::SequentialCompressionWriter`
- `rosbag2_transport::Recorder`
- `rosbag2_py` transport bindings used by `ros2 bag record`

The policy does not cover playback, delayed action runners, event publisher threads, or other
background tasks unless they explicitly adopt the same API contract.

## Policy

### Capture exceptions at the worker-thread boundary

Exceptions must not escape worker thread entry points. A worker thread that calls storage or other
user-provided callbacks shall catch all exceptions at the thread boundary and store the first
failure as `std::exception_ptr`.

Only the first asynchronous failure is retained. Later failures may be ignored because the first
failure is the causal error that should stop recording and be reported to the caller.

### Preserve the original exception

The stored failure shall use `std::exception_ptr` rather than converting immediately to an error
code or string. This preserves the original exception type, message, and nested exception state for
the thread that eventually reports the failure.

### Surface failures by polling

Because worker threads are owned by individual classes in the current architecture, background file
I/O failures shall be surfaced by explicit polling APIs. Callers that drive recording progress are
responsible for polling often enough to stop promptly after a fatal background write error.

The API shape is:

- The lowest asynchronous owner stores the exception and exposes a `throw_if_failed()`-style API.
- The writer exposes a background-write failure polling API.
- Compression writers shall include both cache consumer failures and compression worker failures
  in the same writer-level polling API.
- The recorder polls the writer from its main control loop and rethrows the failure from the
  recorder thread.
- Language bindings shall expose the same failure path so the CLI can exit cleanly instead of
  aborting from an uncaught background-thread exception.

Polling APIs shall rethrow the stored exception on the calling thread. After the exception is
reported, the stored `exception_ptr` may be cleared so the same failure is not repeatedly thrown by
the same layer.

### Stop and close must also observe background failures

Polling during active recording is required for prompt failure detection, but it is not sufficient.
`stop()` and `close()` paths must also check background failures because an error can happen after
the last poll or while flushing the cache.

When stopping a cache consumer, the implementation shall join the worker thread, finish cache
flush bookkeeping, and then rethrow the stored worker exception from the caller thread.

When stopping compression workers, the implementation shall signal all compression workers, join
them, and then rethrow the first stored compression worker exception from the caller thread.

### Defer close-time rethrow until after finalization

If a background write failure is observed during `SequentialWriter::close()`, the writer shall
preserve the exception and rethrow it only after close/finalization cleanup has completed.

The close path shall attempt to:

1. Stop and destroy the cache consumer.
2. Finalize in-memory metadata.
3. Update storage metadata.
4. Reset storage so the storage backend can close its file handles.
5. Write `metadata.yaml` if metadata is available.
6. Reset per-topic message counts and converter state.
7. Rethrow the preserved background write exception.

This ordering prevents a caller that catches the close error from being left with skipped metadata
and close handling solely because the cache consumer reported a previous background write failure.

Exceptions raised by the cleanup steps themselves follow the normal synchronous exception path. This
policy does not require cleanup failures to be suppressed in order to rethrow an earlier background
failure.

### Destructors must not throw

Destructors for classes that may own asynchronous recording work must not allow exceptions to
escape. If a destructor needs to stop a worker, close a writer, or commit/close storage, it shall
catch and suppress or log exceptions.

Callers that need to observe failures must call the explicit polling, `stop()`, or `close()` APIs
before destruction. Destructors are only a last-resort cleanup path.

### Prefer explicit propagation over logging-only handling

Logging a background failure is not sufficient for fatal recording errors. File I/O errors such as
disk full must be propagated back to the recorder control flow so the process can stop recording and
return a failure status.

Logging may be used in destructors or best-effort cleanup paths where throwing is not possible, but
it must not be the only reporting mechanism for errors detected during normal recording operation.

## Current limitations

This policy can only propagate failures that are observable through the storage plugin API. At the
time of writing, this primarily covers failures reported by the SQLite3 storage backend.

The MCAP storage backend has an additional limitation: the MCAP file writer write API used by
rosbag2 does not currently expose all file I/O write failures back to the storage plugin in a way
that rosbag2 can catch and propagate. As a result, this policy does not guarantee that disk I/O
errors from the MCAP writer are surfaced through the same asynchronous error path. This should be
revisited after the MCAP-side behavior is changed or an upstream issue is resolved.

## Review and redesign triggers

This policy documents the error-handling contract for the current architecture. It should be
reviewed when any of the following changes happen:

- Thread ownership in the recording path is redesigned.
- `CacheConsumer` stops owning its own thread or is replaced by an external executor/task runner.
- A common asynchronous task abstraction is introduced for rosbag2.
- Writer cache flushing, storage writes, or recorder stop/close sequencing is substantially
  changed.
- New background recording workers are added that can fail due to file I/O.

In particular, the policy should be revisited if thread ownership is moved out of individual
classes and into a recorder-level supervisor or shared task runner. With centralized thread
ownership, it may be possible to reduce the amount of polling between layers and to represent
background task failures more directly. A future design can evaluate whether task completion
results, recorder-level supervision, or another shared mechanism would provide a clearer path from
fatal recording failures to recorder shutdown and CLI exit status.
