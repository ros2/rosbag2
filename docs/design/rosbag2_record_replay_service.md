# Rosbag2 record and replay service

## Context

In [ros2/rosbag2#773](https://github.com/ros2/rosbag2/issues/773), it was recommended that record service requests feature should be added to rosbag2.  

This design describe how to implement this feature based on current architecture and how to extend current command parameter.  

## How to save record service message

Service introspection ([ros2/ros2#1285](https://github.com/ros2/ros2/issues/1285)) had been implemented. All service messages (requests and responses) are published to service event topic which is hidden topic named as `_service_event` under service name (e.g. /add_two_ints/_service_event). So all messages in this topic can be recorded like general topic.

## How to distinguish service event topic in all recorded topics

Service event topic is saved like general topic. But they should use different process after reading from recording file. So they should be distinguished by topic name.

- General topic  
  
  [NameSpace/]TopicName

- Hidden topic  
  
  [NameSpace/]_TopicName

- Service event topic
  
  [NameSpace/]ServiceName/_service_event

The following criteria will be used to determine a service event topic:
1. The topic ends with `/_service_event`
2. The topic type name matches  `*/srv/*_Event` (e.g. `example_interfaces/srv/AddTwoInts_Event`)

### Expand the 'record' command

Added or changed parameters:

- `--services ServiceName1 [ServiceName2 ...]`
  
    Specify which services will be recorded. User must specify at least one service.

- `--all-topics`

    Record all topics. Hidden topics including service event topics are excluded.

- `--all-services`

    Record all service event topics.

- `--exclude-regex`

    Rename from `--exclude`.  Exclude topics and services containing provided regular expression.

- `--exclude-topics`

    List of topics not being recorded.

- `--exclude-services`

    List of services not being recorded.

The description of the relevant parameter behavior.

| Parameter | Description |
| :-- | :--|
|--services ServiceName1 [ServiceName2 ...] | Record services (service event topics) with specified service names. |
| --all | Record all topics and service event topics. Other hidden topics are excluded. |
| --all-topics | Record all topics. Hidden topics including service event topics are excluded. |
| --all-services | Only record all service event topics. |
| --include-hidden-topics | Record all hidden topics. Include service event topic. |
| --exclude-regex | Exclude topics and services containing provided regular expression. |
| --exclude-topics | List of topics not being recorded. |
| --exclude-services | List of services not being recorded. |
| -e REGEX, --exclude-regex REGEX | Record only topics and service containing provided regular expression. |

### Change output of 'Info' command

A new parameter is added.

- `-v` or `--verbose`

    If `-v` or `--verbose` is set, the info command shows two additional pieces of information: the size contribution of topics and services, and the number of requests and responses for each service based on the service event. Otherwise, only show the message count of the topics and the general number of service events. Note that parsing the messages need spent time. The duration of the parsing is related to the number of recorderd topic messages and service events.


Without `-v` or `--verbose` parameter, info command shows as below example.

```
Files:             123_0.mcap
Bag size:          28.2 KiB
Storage id:        mcap
Duration:          15.59s
Start:             May 19 2023 13:22:25.340 (1684473745.340)
End:               May 19 2023 13:22:40.400 (1684473760.400)
Messages:          60
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 16 | Serialization Format: cdr
                   Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 44 | Serialization Format: cdr

>>>> The below is new added items for service event topic <<<<<<<
Service : XX  <== The number of service
Service information: Service: /xxx/xxx | Type: xxx/xxx/xxx | Event Count: XX | Serialization Format: XX
                     Service: /xxx/xxx | Type: xxx/xxx/xxx | Event Count: XX | Serialization Format: XX
```

With `-v` or `--verbose` parameter, info command shows as below example.
```
Files:             123_0.mcap
Bag size:          28.2 KiB
Storage id:        mcap
Duration:          15.59s
Start:             May 19 2023 13:22:25.340 (1684473745.340)
End:               May 19 2023 13:22:40.400 (1684473760.400)
Messages:          60
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 16 | Size Contribution XX xxB | Serialization Format: cdr
                   Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Size Contribution 0 B | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Size Contribution 0 B | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 44 | Size Contribution XX xxB | Serialization Format: cdr

>>>> The below is new added items for service <<<<<<<
Service : XX  <== The number of service
Service information: Service: /xxx/xxx | Type: xxx/xxx/xxx | Request Count: XX | Response Count: XX | Size Contribution XX xxB | Serialization Format: XX
                     Service: /xxx/xxx | Type: xxx/xxx/xxx | Request Count: XX | Response Count: XX | Size Contribution XX xxB | Serialization Format: XX
```

### Expand the 'play' command

Added or changed parameters:
- `--publish-service-requests`

    Send service requests based on recorded service introspection messages. By default, recorded service introspection messages will be published by service event topic.

- `--service-requests-source {service_introspection, client_introspection}`

    Determine the source of the service request to be replayed. By default, the service request is from recorded service introspection message.
    This option only makes sense if the `--publish-service-requests` option is set.

- `--services ServiceName1 [ServiceName2 ...]`

    Determine which service's requests are played.

- `--exclude-topics topic1 [topic2 ...]`

    List of topics not being played.

- `--exclude-services ServiceName1 [ServiceName2 ...]`

    List of services not being played.

- `--exclude-regex REGEX`

    Rename from `--exclude`. Exclude topics and services containing provided regular expression.

`-e REGEX, --regex REGEX` affects both topics and services.

## Implementation Staging

Implementation is going to be split into two phases.  
On phase 1 is expected to be implemented all functionality which is related to the service events recording.  
On phase 2 the rest functionality related to the service requests playback.  
The service requests playback is not straightforward to implement since we don't have API from rclcpp layer to be able to publish service requests with dynamically created type support. We will need to create an analog of the `GenericPublisher` i.e. `GenericClient` on the `rclcpp` layer to be able to do this. To facilitate implementation of the `GenericClient` we likely will need to update get_typesupport_handle() to support service [ros2/rclcpp#2209](https://github.com/ros2/rclcpp/pull/2209).  
Therefore, phase 2 will be implemented when the required new rclcpp API will be ready.