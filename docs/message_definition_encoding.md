# Message Definition Encoding

Message definitions can be stored in ROS 2 bags to facilitate decoding messages. This document
describes how message definitions can be encoded within a bag.

## Background

The message definitions in the bag are not used for bag playback. However, some
bag analysis tools are not aware of all message definitions ahead of time, and need to
generate a deserializer using the message definitions at run-time.

When deserializing a message at run-time, the deserializer needs the definition of that message's
type, along with the definitions for all fields of that type (and fields of field types, etc.).
This set of definitions with all field types recursively included can be called a
"complete message definition".

## `ros2msg` encoding

This encoding consists of definitions in [.msg](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html#message-description-specification) format, concatenated together in human-readable form with
a delimiter.

The top-level message definition is present first, with no delimiter. All dependent .msg definitions are preceded by a two-line delimiter:

* One line containing exactly 80 `=` characters
* One line containing `MSG: <package resource name>` for that type. The space between MSG: and the
  package resource name is mandatory. The package resource name does not include a file extension.

### `ros2msg` example

For example, the complete message definition for `my_msgs/msg/ExampleMsg` in `ros2msg` form is:

```
# defines a message that includes a field of a custom message type
my_msgs/BasicMsg my_basic_field
================================================================================
MSG: my_msgs/msg/BasicMsg
# defines a message with a primitive type field
float32 my_float
```

## `ros2idl` encoding

The IDL definition of the type specified by name along with all dependent types are stored together. The IDL definitions can be stored in any order. Every definition is preceded by a two-line delimiter:

* a line containing exactly 80 `=` characters, then
* A line containing only `IDL: <package resource name>` for that definition. The space between IDL: and the package resource name is mandatory. The package resource name does not include a file extension.

### `ros2idl` example

For example, the complete message definition for `my_msgs/msg/ComplexMsg` in `ros2idl` form is:

```
================================================================================
IDL: my_msgs/msg/ComplexMsg
// generated from rosidl_adapter/resource/msg.idl.em
// with input from my_msgs/msg/ComplexMsg.msg
// generated code does not contain a copyright notice

#include "my_msgs/msg/BasicMsg.idl"

module my_msgs {
  module msg {
    struct ComplexMsg {
      my_msgs::msg::BasicMsg my_basic_field;
    };
  };
};
================================================================================
IDL: my_msgs/msg/BasicMsg
// generated from rosidl_adapter/resource/msg.idl.em
// with input from my_msgs/msg/BasicMsg.msg
// generated code does not contain a copyright notice


module my_msgs {
  module msg {
    struct BasicMsg {
      float my_float;
    };
  };
};
```

## FAQ

### Why store message definitions in human-readable form?

These formats are designed to be updated as rarely as possible. Definitions in bags recorded now
should ideally be parseable years in the future with no format migrations. MSG and OMG IDL 4.2 are
both well-known formats for ROS 2 message definitions, and no additional knowledge besides their
specifications are required for readers to deserialize messages.

### Why do these examples include comments?

The `.msg` and `.idl` message definitions are encoded according to their specifications, which
include comments as valid syntax. Bag writers may strip comments out or leave them in.
