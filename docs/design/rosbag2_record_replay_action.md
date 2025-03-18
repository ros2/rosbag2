# Rosbag2 record and replay action

## Context

This design describes how to implement this feature based on current architecture and how to extend current command parameter.

## How to record action message

The implementation of the action uses three services and two topics. For the services, when service introspection is enabled, rosbag2 can record service data through the service event topic (refer to [rosbag2_record_replay_service.md](./rosbag2_record_replay_service.md)). So all data related to the action can be recorded.


## Topics to be recorded for an action

- goal service

  [NameSpace/]ActionName/_action/send_goal  
  Service Event Topic: [NameSpace/]ActionName/_action/send_goal/_service_event

- result service

  [NameSpace/]ActionName/_action/get_result  
  Service Event Topic: [NameSpace/]ActionName/_action/get_result/_service_event

- cancel service

  [NameSpace/]ActionName/_action/cancel_goal  
  Service Event Topic: [NameSpace/]ActionName/_action/cancel_goal/_service_event

- status topic

  [NameSpace/]ActionName/_action/status

- feedback topic

  [NameSpace/]ActionName/_action/feedback

For an action, five related topics will be recorded.

### Expand the 'record' command

Newly added arguments:

| Argument Name | Description |
| :-- | :-- |
| --actions ActionName [ActionName ...] | Space-delimited list of actions to record. |
| --all-actions | Record all actions via service event topics and hidden action topics. |
| --exclude-actions ActionName [ActionName ...] | Space-delimited list of actions not being recorded. Works on top of --all, --all-actions, --actions  or --regex. |

Updated arguments:

| Argument Name | Description |
| :-- | :-- |
| -a, --all | Record all topics, services and **actions** (Exclude hidden topic). |
| -e REGEX, --regex REGEX | Record only topics, services and **actions** containing provided regular expression. Note: --all, --all-topics, --all-services or **--all-actions** will override --regex. |
| --exclude-regex EXCLUDE_REGEX | Exclude topics, services and actions containing provided regular expression. Works on top of --all, --all-topics, --all-services, **--all-actions**, --topics, --services, **--actions** or --regex. |

### Change output of 'Info' command

Add action part.

Without `-v` or `--verbose` parameter, info command shows as below example.
```
    Files:             <bag_name>.mcap
    Bag size:          39.9 KiB
    Storage id:        mcap
    ROS Distro:        rolling
    Duration:          4.835s
    Start:             Mar 18 2024 16:42:14.323 (1710805334.323)
    End:               Mar 18 2024 16:42:19.159 (1710805339.159)
    Messages:          39
    Topic information: Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                       Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                       Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 39 | Serialization Format: cdr
    Services:          1
    Service information: Service: /add_two_ints | Type: example_interfaces/srv/AddTwoInts | Event Count: 42 | Serialization Format: cdr
    Actions:           2
    Action information:
      Action: /fibonacci | Type: action_tutorials_interfaces/action/Fibonacci | Topics: 2 | Services: 3 | Serialization Format: cdr
        Topic: feedback | Count: 1
        Topic: status | Count: 10
        Service: send_goal | Event Count: 20
        Service: cancel_goal | Event Count: 2
        Service: get_result | Event Count: 9
      Action: /other | Type: other_action_interfaces/action/Other | Topics: 2 | Services: 3 | Serialization Format: cdr
        Topic: feedback | Count: 2
        Topic: status | Count: 20
        Service: send_goal | Event Count: 40
        Service: cancel_goal | Event Count: 4
        Service: get_result | Event Count: 9
```

With `-v` or `--verbose` parameter, info command shows as below example.
```
    Files:             <bag_name>.mcap
    Bag size:          39.9 KiB
    Storage id:        mcap
    ROS Distro:        rolling
    Duration:          4.835s
    Start:             Mar 18 2024 16:42:14.323 (1710805334.323)
    End:               Mar 18 2024 16:42:19.159 (1710805339.159)
    Messages:          39
    Topic information: Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                       Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                       Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 39 | Serialization Format: cdr
    Services:          1
    Service information: Service: /add_two_ints | Type: example_interfaces/srv/AddTwoInts | Request Count: 20 | Response Count: 22 | Serialization Format: cdr
    Actions:           2
    Action information:
      Action: /fibonacci | Type: action_tutorials_interfaces/action/Fibonacci | Topics: 2 | Services: 3 | Serialization Format: cdr
        Topic: feedback | Count: 1
        Topic: status | Count: 10
        Service: send_goal | Request Count: 10 | Response Count: 10
        Service: cancel_goal | Request Count: 1 | Response Count: 1
        Service: get_result | Request Count: 5 | Response Count: 4
      Action: /other | Type: other_action_interfaces/action/Other | Topics: 2 | Services: 3 | Serialization Format: cdr
        Topic: feedback | Count: 2
        Topic: status | Count: 20
        Service: send_goal | Request Count: 20 | Response Count: 20
        Service: cancel_goal | Request Count: 2 | Response Count: 2
        Service: get_result | Request Count: 5 | Response Count: 4
```

### Expand the 'play' command

Newly added arguments:

| Argument Name | Description |
| :-- | :-- |
| --actions action [action ...] | Space-delimited list of actions to play. |
| --exclude-actions action [action ...] | Space-delimited list of actions not to play. |
| --send-actions-as-client | Send the send_goal request, cancel_goal request, and get_result request respectively based on the recorded send_goal, cancel_goal, and get_result event messages. Note that the messages from action's "status topic" [NameSpace/]ActionName/_action/status and "feedback topic" [NameSpace/]ActionName/_action/feedback will not be sent because they are expected to be sent from the action server side. |

Updated arguments:

| Argument Name | Description |
| :-- | :-- |
| -e REGEX, --regex REGEX | Play only topics, services and **actions** matches with regular expression. |
| -x EXCLUDE_REGEX, --exclude-regex EXCLUDE_REGEX | regular expressions to exclude topics, services  and **actions** from replay. |
| --service-requests-source {service_introspection,client_introspection} | Determine the source of the service requests to be replayed. This option only makes sense if the "--publish-service-requests" or **"--send-actions-as-client"** option is set. By default, the service requests replaying from recorded service introspection message. |
