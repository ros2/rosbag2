import pathlib, yaml, csv
import numpy as np

import rclpy
from rclpy.node import Node

class VoyagerCase(Node):

    def __init__(self):
        super().__init__('voyager_case')
        self.logger = rclpy.logging.get_logger("VOY")

        self.declare_parameter("description", "")
        config_path = self.get_parameter("description").get_parameter_value().string_value
        if config_path == "":
            raise RuntimeError("You must specify a description file: \n --ros-args -p description:=[PATH]")

        # Manage config file
        self.config = None
        path = pathlib.Path(config_path)
        self.voyager_dir = path.parents[0]
        if path.is_file():
            with open(path) as config_file:
                self.config = yaml.load(config_file)
        else:
            raise RuntimeError("{} is not correct yaml config file.".format(path))
        
        self.generate_voyager_results()

    def generate_voyager_results(self):
        table = []
        index = 0
        for record in self.config["layout"]:
            with open(str(self.voyager_dir.joinpath(record)), 'r') as description_file:
                data = yaml.load(description_file)
                raport_file = pathlib.Path(data["raport_dir"]).expanduser().joinpath(str(data["benchmark"]["id"])+"-"+str(data["benchmark"]["tag"])).joinpath("raport.yaml")
                with open(str(raport_file), 'r') as data_file:
                    raport_data = yaml.load(data_file)
                    table.append("{:-6.2F}".format(raport_data["messages"]["percent"]))

        print("\t", ["     1", "    10", "   100", "  1000"])
        print("1KB \t", table[:4])
        print("10KB \t",table[4:8])
        print("100KB \t",table[8:12])
        print("1000KB \t",table[12:16])
        print()

def main():
    rclpy.init()
    node = VoyagerCase()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()
    node.destroy_node()

if __name__ == '__main__':
    main()
