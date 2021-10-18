import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from example_interfaces.msg import Int32

import rosbag2_py

class DataGeneratorNode(Node):
    def __init__(self):
        super().__init__('data_generator_node')
        self.data = Int32()
        self.data.data = 0
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='timed_synthetic_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='synthetic',
            type='example_interfaces/msg/Int32',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.writer.write(
            'synthetic',
            serialize_message(self.data),
            self.get_clock().now().nanoseconds)
        self.data.data += 1


def main(args=None):
    rclpy.init(args=args)
    dgn = DataGeneratorNode()
    rclpy.spin(dgn)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
