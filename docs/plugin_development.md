# Writing storage plugins

There are different interfaces for storage plugins depending on your need: The general `ReadWriteStorage` and the more specific `ReadableStorage`.

## Writing a general plugin

Assume you write a plugin `MyStorage` which can both save messages and read messages.
Its header file could be `my_storage.hpp` and `MyStorage` will derive from `rosbag2_storage::storage_interfaces::ReadWriteInterface`.
**Important:** While implementing the interface provided by `rosbag2_storage::storage_interfaces::ReadWriteInterface`, make sure that all resources such as file handles or database connections are closed or destroyed in the destructor, no additional `close` call should be necessary.

In order to find the plugin at runtime, it needs to be exported to the pluginlib.
Add the following lines to `my_storage.cpp`:

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyStorage, rosbag2_storage::storage_interfaces::ReadWriteInterface)
```

Furthermore, we need some meta-information in the form of a `plugin_description.xml` file.
Here, it contains

```
<library path="my_storage_lib">
  <class name="my_storage" type="MyStorage" base_class_type="rosbag2_storage::storage_interfaces::ReadWriteInterface">
  </class>
</library>
```
`my_storage_lib` is the name of the library (ament package) while `my_storage` is an identifier used by the pluginlib to load it.

In addition, in the `CMakeLists.txt` the `plugin_description.xml` file needs to be added to the index to be found at runtime:

`pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)`

The first argument `rosbag2_storage` denotes the library we add our plugin to (this will always be `rosbag2_storage`), while the second argument is the path to the plugin description file.

## Writing a plugin for reading only

When writing plugins to only provide functionality for reading, derive from `rosbag2_storage::storage_interfaces::ReadOnlyInterface`.

If the read-only plugin is called `my_readonly_storage` in a library `my_storage_lib`, it will be registered using

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyReadonlyStorage, rosbag2_storage::storage_interfaces::ReadOnlyInterface)
```
with the plugin description
```
<library path="my_storage_lib">
  <class name="my_readonly_storage" type="MyReadonlyStorage" base_class_type="rosbag2_storage::storage_interfaces::ReadOnlyInterface">
  </class>
</library>
```
and the usual pluginlib export in the CMakeLists:

`pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)`
