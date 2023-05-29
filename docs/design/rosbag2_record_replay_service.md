# Rosbag2 record and replay service

## Context

In [ros2/rosbag2#773](https://github.com/ros2/rosbag2/issues/773), it was recommended that record service requests feature should be added to rosbag2.  

This design describe how to implement this feature based on current architecture and how to extend current command parameter.  

## How to save record service message

Service introspection ([ros2/ros2#1285](https://github.com/ros2/ros2/issues/1285)) had been implemented. All service messages (requests and responses) are published to service introspection topic which is hidden topic named as `_service_event` under service name (e.g. /add_two_ints/_service_event). So all messages in this topic can be recorded like general topic. 

## How to distinguish service introspection topic in all recorded topics 

Service introspection is saved like general topic. But they should use different process after reading from recording file. So they should be distinguished by topic name.  

- General topic  
  
  [NameSpace/]TopicName

- Hidden topic  
  
  [NameSpace/]_TopicName

- Service introspection topic
  
  [NameSpace/]ServiceName/_service_event

So if the string ends with `/_service_event` in topic name, it should be service introspection topic. Of course, it is possible that hidden topic has the same name. So message type is used for further check. The message type for service introspection must ends with "_Event" (e.g. example_interfaces/srv/AddTwoInts_Event). Therefore, service introspection topic can be identified.

### Expand the 'record' command

Add 1 parameter.

- `-S [SerivceName1 ServiceName2 ...]` or `--services [SerivceName1 ServiceName2 ...]`
  
    If the user doesn't specify service names, all service introspection topics are recorded.

### Change output of 'Info' command

Topic for service introspection should be listed distinctively, as below example.

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

>>>> The below is new added items for service introspection <<<<<<<
Service Events: XX  
Service information: Service: /xxx/xxx | Type: xxx/xxx/xxx | Event Count: XX | Serialization Format: XX
                     Service: /xxx/xxx | Type: xxx/xxx/xxx | Event Count: XX | Serialization Format: XX
```

### Expand the 'play' command

Add 2 parameters.  
- `-S [ServiceName1 ...]` or `--services [ServiceName1 ...]` (TBD: Not sure whether this is needed.)
    
    Replay service events. If there is no service name set, all recorded service events are played.  User can use `ros2 service echo ServiceName` to check different service events. 

- `--services-requests [ServiceName1 ...]`

    Replay request with recorded sequence and interval. If there is no service name set, all recorded service request are sent.

Other parameters need to be checked one by one. If it is unsuitable for playing service event, the description of parameter should be updated to mention this limitation.  