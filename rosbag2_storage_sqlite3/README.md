# rosbag2_storage_sqlite3

Storage implementation plugin for rosbag2 providing SQLite3 `.db3` files as the bag storage backend.


## Storage Configuration File

The `--storage-config-file` option for this plugin takes files in the following format:

```
read:
  pragmas: <list of SQLite pragma settings for read-only>
write:
  pragmas: <list of SQLite pragma settings for write modes>
```

By default, SQLite settings are significantly optimized for performance.
This might have consequences of bag data being corrupted after an application or system-level crash.
This consideration only applies to current bagfile in case bag splitting is on (through `--max-bag-*` parameters).
If increased crash-caused corruption resistance is necessary, use `resilient` option for `--storage-preset-profile` setting.

Settings are fully exposed to the user and should be applied with understanding.
Please refer to [documentation of pragmas](https://www.sqlite.org/pragma.html).

An example configuration file could look like this:

```
write:
  pragmas: ["journal_mode = MEMORY", "synchronous = OFF", "schema.cache_size = 1000", "schema.page_size = 4096"]

```
