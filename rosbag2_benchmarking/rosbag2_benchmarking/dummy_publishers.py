import rclpy
from rclpy.node import Node

class DummyPublisherUtility(Node):

    __publishers = []

    def __init__(self):
        super().__init__('dummy_publisher')
        self.declare_parameter("topics")
        self.declare_parameter("types")

        topics = self.get_parameter("topics").get_parameter_value().string_array_value
        types = self.get_parameter("types").get_parameter_value().string_array_value

        if len(topics) != len(types):
            raise RuntimeError("Topics and types length mismatch.")

        for i in range(0,len(topics)):
            self.warm_up_topic(topics[i], types[i])

    def warm_up_topic(self, topic, type_):
        if type_ == "sensor_msgs/msg/Image":
            from sensor_msgs.msg import Image
            self.__publishers.append(self.create_publisher(Image, topic, 1))
        elif type_ == "sensor_msgs/msg/PointCloud2":
            from sensor_msgs.msg import PointCloud2
            self.__publishers.append(self.create_publisher(PointCloud2, topic, 1))
        else:
            # (piotr.jaroszek) TODO: fill out rest or make a dynamic import
            pass

def main():
    rclpy.init()
    node = DummyPublisherUtility()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()
    node.destroy_node()

if __name__ == '__main__':
    main()