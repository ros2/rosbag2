from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.serialization import serialize_message
from example_interfaces.msg import Int32

import rosbag2_py


def main(args=None):
    writer = rosbag2_py.SequentialWriter()

    storage_options = rosbag2_py._storage.StorageOptions(
        uri='big_synthetic_bag',
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py._storage.TopicMetadata(
        name='synthetic',
        type='example_interfaces/msg/Int32',
        serialization_format='cdr')
    writer.create_topic(topic_info)

    time_stamp = Clock().now()
    for ii in range(0, 100):
        data = Int32()
        data.data = ii
        writer.write(
            'synthetic',
            serialize_message(data),
            time_stamp.nanoseconds)
        time_stamp += Duration(seconds=1)

if __name__ == '__main__':
    main()
