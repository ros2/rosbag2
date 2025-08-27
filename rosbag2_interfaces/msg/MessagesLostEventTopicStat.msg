# The name of the topic on which the messages were lost
string topic_name

# The number of messages lost since the last event on a DDS transport layer
uint64 messages_lost_in_transport

# The number of messages lost since the last event in the Rosbag2 recorder
uint64 messages_lost_in_recorder
