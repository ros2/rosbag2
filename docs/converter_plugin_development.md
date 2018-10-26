# Converter plugins

Converter plugins are used to convert between different storage serialization formats.
A storage for rosbags in ROS 2 obtains and stores its data using a serialization format.
By default, it will store a bag in the serialization format of the middleware it was recorded with (two middlewares may share the same serialization format).
If messages should be read by a middleware with a different serialization format, the messages must be converted to that format.
To simplify conversion between all formats each plugin provides functions to convert to and from generic ROS 2 messages.

Rosbag2 is shipped with a default converter plugin to convert between ROS 2 messages and serialized messages in the CDR format (this is the general serialization format specified by DDS).

## Writing converter plugins

To write a plugin `MyConverter`, implement the interface `rosbag2::SerializationFormatConverterInterface`.

The plugin interface provides two functions: 

- The function `serialize` takes an initialized SerializedBagMessage and a ROS 2 message together with information about its topic and type and should fill the SerializedBagMessage with the ROS 2 message contents.
- The other function `deserialize` does the reverse, taking a full SerializedBagMessage to fill a preallocated ROS 2 message using the provided typesupport (which has to match the actual type of the SerializedBagMessage).

In order to find the plugin at runtime, it needs to be exported to the pluginlib. 
Add the following lines in the `my_converter.cpp`:

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyConverter, rosbag2::SerializationFormatConverterInterface)
```

Furthermore, we need some meta-information in the form of a `plugin_description.xml` file.
In the case of `MyConverter` this would look like:
```
<library path="my_converter_library">
  <class
    name="my_storage_format_converter"
    type="MyConverter"
    base_class_type="rosbag2::SerializationFormatConverterInterface"
  >
    <description>This is a converter plugin for my storage format.</description>
  </class>
</library>
```
where `my_converter_library` is the name of the library, while `my_storage_format_converter` is the identifier used by the pluginlib to load it.
It **must** have the format `<rmw storage format>_converter` as described in the section "How to choose conversion at runtime".

In addition, in the `CMakeLists.txt` the `plugin_description.xml` file needs to be added to the index to be found at runtime:
```
pluginlib_export_plugin_description_file(rosbag2 plugin_description.xml)
```

The first argument `rosbag2` denotes the ament index key we add our plugin to (this will always be `rosbag2` for converter plugins), while the second argument is the path to the plugin description file.

## How to choose conversion at runtime

The conversion will be chosen automatically according to the storage format specified in the bagfile or specified by the user.
To achieve this, rosbag2 searches for a converter plugin with name `<storage format>_converter`. 
For instance, the default plugin for conversion between CDR messages is named `cdr_converter` in its `pluginlib.txt` file.
There should be no need for different converters for the same storage format hence this is not supported.
