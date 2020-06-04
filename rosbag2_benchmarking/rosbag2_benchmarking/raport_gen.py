import rclpy
from rclpy.node import Node

class RaportGen(Node):

    def __init__(self):
        super().__init__('raport_gen')
        self.logger = rclpy.logging.get_logger("RAPORT")
        self.logger.info("Raport gen dummy checking in!")

def main():
    rclpy.init()
    node = RaportGen()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()
    node.destroy_node()

if __name__ == '__main__':
    main()