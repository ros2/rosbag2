# Statistics about number of lost and recorded messages

## Context

The current implementation of Rosbag2 does not provide any statistics about the number of lost 
and recorded messages in real time or during post-processing. To be more specific, the current
implementation only print out to the log at the time when the current bag file closes the number of 
messages dropped by the Rosbag2 writer in cases if the writer is not able to keep up with the
incoming messages and messages are dropped due to the internal buffer being full.
However, it does not provide any information about the number of messages that were lost on the 
DDS transport layer. And this information is not available in the bag file itself or in the 
runtime during recording. 
This is a problem because it makes it difficult to assess the quality of the recorded data and to
identify potential issues with the recording process.

This design proposes to add a new feature to Rosbag2 that will provide statistics about the 
number of lost and recorded messages as well as the number of bytes recorded per topic in real time 
and during post-processing.

## Use cases

1. Real-time monitoring of the performance of recording.
    1. Overall statistics about the number of messages lost and bytes written on a per-topic 
       basis from recording start should be available in real-time by service request.
    2. Incremental statistics about the number of messages lost and bytes written on a per-topic 
      basis should be published on a specific topic when messages lost happened.
2. Statistics about the number of lost messages and bytes written to the current bag file on a 
   per-topic should be saved in metadata at the end of the recording.

## Design proposals and implementation details consideration

<a name="dds-lost-stat"></a>
1. To facilitate functionality for gathering statistics about number messages lost on the DDS 
   transport layer, we can specify `message_lost_callback` in the 
   `rclcpp::SubscriptionOptions::event_callbacks` when creating the subscription in the 
   `rosbag2_transport::recorder` by providing `rclcpp::SubscriptionOptions` parameter when 
   creating `GenericSubscription`. The `message_lost_callback` will provide a reference to the 
   `rclcpp::QOSMessageLostInfo` which is mapped to the `rmw_message_lost_status_s` struct.
      ```cpp
        typedef struct RMW_PUBLIC_TYPE rmw_message_lost_status_s
        {
          /// Total number of messages lost.
          size_t total_count;
          /// Number of messages lost since last callback.
          size_t total_count_change;
        } rmw_message_lost_status_t;
      ```
<a name="recorder-lost-stat"></a>
2. To facilitate functionality for gathering statistics about number of messages lost on the Rosbag2
  recorder side will need to add a new `messages_lost_callback` to the `rosbag2_cpp::writer` class.
  The `messages_lost_callback` will be called when the `rosbag2_cpp::writer` is not able to 
  keep up with the incoming messages and the internal buffer is full.
  The `messages_lost_callback` will have a parameter as a return value defined as a const 
  reference to the `std::vector` of the `rosbag2_cpp::MessageLostInfo` structure that will 
  contain the following fields:
     - `topic_name` (string): The name of the topic on which messages lost happened.
     - `num_messages_lost` (uint64_t): The number of messages lost.
  The `messages_lost_callback` shall be defined in the `rosbag2_cpp::bag_events` namespace and
  added to the `WriterEventCallbacks` struct similar to the `write_split_callback`.
<a name="cli-option-update-rate"></a>
3. Will need to add a CLI option to the Rosbag2 recorder to publish statistics on the predefined
  topic with the rate not higher than specified.
    - We can name this option as `--statistics-update-rate` and define it as integer . It 
      will be used to 
      specify the maximum update rate in times per second at which the statistics will be published.
      The default value will be 1Hz. The value of 0 will disable the publishing of statistics.
<a name="messages-lost-notifier"></a>
4. Will need to add a new `MessagesLostNotifier` class to the `rosbag2_transport::recorder` that 
   will publish on a predefined topic incremental statistics about the number of lost messages and 
   bytes written on a per-topic basis when messages lost happened.
   - To facilitate this, the Rosbag2 recorder shall satisfy the following requirements:
      1. Messages lost event shall be published from the Rosbag2 recorder on a dedicated topic.
      e.g., `/events/rosbag2_messages_lost`
      2. Messages lost event shall include per-topic statistics with a number of lost messages.
        The message type could be named as `rosbag2_interfaces::msg::MessagesLostEvent` and 
        defined in the `rosbag2_interfaces` package. The message type will be similar to the 
        `rosbag2_interfaces::msg::WriteSplitEvent` message type and shall contain the following 
         fields:
         - `node_name` (string): The name of the node that is recording.
         - Array of the following fields:
         - `topic_name` (string): The name of the topic on which messages lost happened.
         - `messages_lost_in_transport` (uint64_t): The number of messages lost since last event on
           a DDS transport layer.
         - `messages_lost_in_recorder` (uint64_t): The number of messages lost since last event in
           the Rosbag2 recorder.
         - `bytes_written` (uint64_t): The total number of bytes written on this topic.
      3. The messages lost event shall not be published more often than user specified event update 
         rate to avoid excessive resource usage.
      4. The Messages lost event shall **not** include topics with zero number of lost messages.
         To get a full statistics about the number of recorded and lost messages the user shall use
         the service request `GetRecorderStatistics` described in the
         [clause 5 "New service request"](#new-service-request).
      5. The message lost event shall be published in a separate thread to avoid blocking the 
         recording process.
      6. The `MessagesLostNotifier` will be getting information from the `message_lost_callback`
         specified in the `rclcpp::SubscriptionOptions::event_callbacks` described in the
         [clause 1 ""Messages lost in DDS transport"](#dds-lost-stat) 
         and `messages_lost_callback` described in the
         [clause 2 "Messages lost in recorder"](#recorder-lost-stat).
<a name="new-service-request"></a>
5. Will need to add a new service request to the `rosbag2_transport::recorder` that will provide
   statistics about the number of lost and recorded messages as well as the number of bytes
   recorded per topic since the recording started.
    - The service could be named as `rosbag2_interfaces::srv::GetRecorderStatistics` and defined
      in the `rosbag2_interfaces` package.
    - The `rosbag2_interfaces::srv::GetRecorderStatistics` service request shall return the 
      following fields:
      - `node_name` (string): The name of the node that is recording.
      - Array of the following fields:
          - `topic_name` (string): The names of the topics that are being recorded.
          - `messages_lost_in_transport` (uint64_t): The number of messages lost on the DDS
            transport layer for each topic.
          - `messages_lost_in_recorder` (uint64_t): The number of messages lost in the Rosbag2
             recorder for each topic.
          - `messages_recorded` (uint64_t): The number of messages recorded for the topic.
          - `bytes_written` (uint64_t): The number of bytes written for the topic.
          - `messages_rate` (float): The average messages rate on the topic in Hz.
    - To fulfill information for the `rosbag2_interfaces::srv::RecorderStatistics` message in the
      `GetRecorderStatistics` callback the `rosbag2_cpp::writer` class should be extended with the
      new getter method `get_total_topics_statistics` that will return information gathered in the
      internal `total_topics_stat_` variable that is a `std::unordered_map` with the keys as the
      topic name. Please reference to the
      [clause 6 "Store statistics in metadata"](#new-fields-in-metadata) for more details about the 
      `total_topics_stat_`.
<a name="new-fields-in-metadata"></a>
6. Will need to add a new metadata fields to the bag file that will store the statistics about the 
  number of lost messages and bytes written on a per-topic basis when closing the current bag file.
   - To facilitate this, the `rosbag2_cpp::writer` class should be extended to include a new
     `RecorderStatistics` class that will keep track of the number of lost messages on DDS
     transport layer and internally in the `rosbag2_cpp::writer` as well as number of bytes
     written on a per-topic basis. The `RecorderStatistics` class internally will have a two
     `std::unordered_map` with the keys as the topic names and the values as the `TopicStatistics`
     struct containing the following fields:
       - `messages_lost_in_transport` (uint64_t): The number of messages lost on the DDS
         transport layer for the topic.
       - `messages_lost_in_recorder` (uint64_t): The number of messages lost in the
         Rosbag2 recorder for the topic i.e. in the `rosbag2_cpp::writer` class.
       - `messages_recorded` (uint64_t): The number of messages recorded for the topic.
       - `bytes_written` (uint64_t): The number of bytes written to the bag file for the topic.
       - `messages_rate` (float): The average messages rate on the topic in Hz.
   - One `std::unordered_map` with possible name `total_topics_stat_` will be used to keep 
     track of the number of lost messages since the recording started and the other
     `std::unordered_map` with a possible name `bag_topics_stat_` will be used to keep track of 
     the number of lost messages since the recording in the current bag file started.
     Note that we can have multiple bag files in the same recording session.
   - To keep `messages_lost_in_transport` up to date will need to add a new
     `on_messages_lost_in_transport` method to the `rosbag2_cpp::writer` class that will be
     called when the `message_lost_callback` is triggered. The method will take a const reference
     to the `rclcpp::QOSMessageLostInfo` struct and `topic_name` and will update the
     `messages_lost_in_transport` field in `total_topics_stat_` and `bag_topics_stat_` maps in the 
     `RecorderStatistics` class.
<a name="ros2-bag-info-extention"></a>
7. Will need to extend the `ros2 bag info` command to include the statistics about the number of
  lost messages on a per-topic basis by reading this information from saved metadata.
  The statistics shall be available with the `--verbose` CLI option.
