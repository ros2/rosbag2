# The bag directory whose filesystem was checked for free space
string path

# The free space in bytes available on the filesystem when the event was generated.
# If bag files were deleted to reclaim disk space, this is the value after deletion.
uint64 available_bytes

# The total capacity in bytes of the filesystem
uint64 capacity_bytes

# The effective minimum free space limit in bytes that was violated
uint64 min_free_space_bytes

# The full paths of the bag files deleted to reclaim disk space.
# Empty unless the "delete_oldest_files" low free space action is configured.
string[] deleted_files

# True if the recorder stopped recording because the free space is still below the limit
bool recording_stopped

# The fully qualified node name of the event sender
string node_name
