import psutil, pathlib, signal

import rclpy
from rclpy.node import Node

class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')
        self.logger = rclpy.logging.get_logger("SYSTEM_MONITOR")
        self.declare_parameter("benchmark_path")
        self.declare_parameter("frequency", 10)

        self.__benchmark_path = self.get_parameter("benchmark_path").get_parameter_value().string_value
        self.__frequency = self.get_parameter("frequency").get_parameter_value().integer_value

        if self.__frequency == 0:
            self.logger.warn("Monitor frequency is set to 0. Invalid value. Rolling back to 10Hz.")
            self.__frequency = 10
        
        self.create_timer(1/self.__frequency, self.gather_data)

        self.__monitor_file = open(pathlib.Path(self.__benchmark_path).joinpath("system_monitor.yaml"), 'w')
        self.__monitor_file.write(str(self.get_clock().now()))

    def close_monitor_file(self):
        self.__monitor_file.close()

    def gather_data(self):
        cpu_percents = psutil.cpu_percent(interval=0, percpu=True)
        mem = psutil.virtual_memory()
        disk = psutil.disk_io_counters()
        self.__monitor_file.write(str(cpu_percents)+"\n")
        self.__monitor_file.write(str(mem) + "\n")
        self.__monitor_file.write(str(disk) + "\n")

def signal_handler(signal, frame):
    rclpy.shutdown()

def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal_handler)
    node = SystemMonitor()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.close_monitor_file()
    node.destroy_node()
    exit(0)

if __name__ == '__main__':
    main()