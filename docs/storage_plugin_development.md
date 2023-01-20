# Writing storage plugins for Rosbag2

Storage plugins provide the actual underlying storage format for Rosbag2.

Plugins can implement the following APIs to provide a storage plugin, specified in `rosbag2_storage::storage_interfaces`:
* `ReadOnlyInterface` which covers only reading files
* `ReadWriteInterface` can both write new files and read existing ones. It is a superset of ReadOnly

## Creating a ReadWrite plugin

Goal: Create a plugin named `my_storage`, in package `rosbag2_storage_my_storage`, implemented by class `my_namespace::MyStorage`.

The following code snippets shows the necessary pieces to provide this plugin.


```
// my_storage.cpp
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

namespace my_namespace {

class MyStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  MyStorage();
  ~MyStorage() override;  // IMPORTANT: All cleanup must happen in the destructor, such as closing  file handles or database connections

  // ReadWriteInterface's virtual overrides here
};

// Implementations

}  // namespace my_namespace

// The following block exposes our class to pluginlib so that it can be discovered at runtime.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_namespace::MyStorage,
                       rosbag2_storage::storage_interfaces::ReadWriteInterface)
```

Next, our package must provide a file named `plugin_description.xml`.
Here, it contains

```
<library path="rosbag2_storage_my_storage">
  <class
    name="my_storage"
    type="MyStorage"
    base_class_type="rosbag2_storage::storage_interfaces::ReadWriteInterface"
  >
    <description>Rosbag2 storage plugin providing the MyStorage file format.</description>
  </class>
</library>
```

`rosbag2_storage_my_storage` is the name of the library from `package.xml` while `my_storage` is an identifier used by the pluginlib to refer to the plugin.

Finally, the `CMakeLists.txt` must add our `plugin_description.xml` file to the ament index to be found at runtime:

```
pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)
```

The first argument `rosbag2_storage` denotes the library we add our plugin to (this will always be `rosbag2_storage` for this plugin type), while the second argument is the path to the plugin description file.

## Creating a ReadOnly plugin

When writing plugins to only provide functionality for reading, derive your implementation class from `rosbag2_storage::storage_interfaces::ReadOnlyInterface` instead.
This is the only functional difference, it will require only a subset of the interface overrides.

```
// my_readonly_storage.cpp
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"

namespace my_namespace {

class MyReadOnlyStorage : public rosbag2_storage::storage_interfaces::ReadOnlyInterface
{
public:
  MyReadOnlyStorage();
  ~MyReadOnlyStorage() override;  // IMPORTANT: All cleanup must happen in the destructor, such as closing  file handles or database connections

  // ReadOnlyInterface's virtual overrides here
};

// Implementations

}  // namespace my_namespace

// The following block exposes our class to pluginlib so that it can be discovered at runtime.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_namespace::MyReadOnlyStorage,
                       rosbag2_storage::storage_interfaces::ReadOnlynterface)
```

```
<!-- plugin_description.xml -->
<library path="rosbag2_storage_my_storage">
  <class
    name="my_readonly_storage"
    type="my_namespace::MyReadOnlyStorage"
    base_class_type="rosbag2_storage::storage_interfaces::ReadOnlyInterface"
  >
    <description>Rosbag2 storage plugin providing read functionality for MyStorage file format.</description>
  </class>
</library>
```

and the usual pluginlib export in the CMakeLists:

```
# CMakeLists.txt
pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)
```

## Providing plugin-specific configuration

Some storage plugins may have configuration parameters unique to the format that you'd like to allow users to provide from the command line.
Rosbag2 provides a CLI argument `--storage-config-file` which allows users to pass the path to a file.
This file can contain anything, its format is specified by the storage implementation, it is passed as a path all the way to the plugin, where it may be used however desired.
Plugins are recommended to document the expected format of this file so that users can write well-formatted configurations.

### Extending CLI from a storage plugin

Commandline arguments can be a much more convenient way to expose configuration to users than writing out a file.
The `ros2bag` package, which creates the `ros2 bag` command, provides an entrypoint for plugins to extend the CLI.

All a package needs to do is expose a Python setuptools entrypoint to the group `ros2bag.storage_plugin_cli_extension`, with an entrypoint keyed by the name of the storage plugin. For example, here is `setup.cfg` from `rosbag2_storage_mcap`:

```
[options.entry_points]
ros2bag.storage_plugin_cli_extension =
  mcap = ros2bag_mcap_cli
```

This registers an entrypoint in group `ros2bag.storage_plugin_cli_extension`, for the plugin named `mcap`, that is implemented by a Python module called `rosbag2_mcap_cli`.

The exposed entrypoint can be installed as a Python module by any method, for example via `ament_cmake_python`'s `ament_python_install_package` macro, or by having a pure-python `ament_python` package with a `setup.py`.

The functions this entry point may provide:

* `get_preset_profiles(): List[str]` - provide a list of names of preset profiles for writing storage files. The first item will be used as default. Consider returning 'none' as the first element.

NOTE: For each of these lists, the string literal 'none' will be used to indicate the feature is disable/not used.

NOTE: Any entry point may exclude any of the extension functions, and a warning will be printed for each extention point omitted. When the function for a list of values is not provided, or returns `None`, by default `'none'` will be provided as the only option.
