# Copyright 2020, Robotec.ai sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Voyager case bundle handler and report generator."""

import pathlib
import shutil

import rclpy
from rclpy.node import Node

import yaml


class DefaultMapper(dict):
    """Helper for string formatting."""

    def __missing__(self, key):
        """When key is missing."""
        return '{'+key+'}'


class VoyagerCase(Node):
    """Report generator for voyager case bundle."""

    def __init__(self):
        """Init voyager case bundle."""
        super().__init__('voyager_case')
        self.logger = rclpy.logging.get_logger('VOY')

        self.declare_parameter('report_dir', str(pathlib.Path.cwd()))
        self.report_dir = pathlib.Path(
            self.get_parameter(
                'report_dir'
            ).get_parameter_value().string_value
        ).expanduser()

        self.declare_parameter(
            'output_dir',
            str(pathlib.Path.cwd().joinpath('voyager_case'))
        )
        self.output_dir = pathlib.Path(
            self.get_parameter(
                'output_dir'
                ).get_parameter_value().string_value
            ).expanduser()
        self.output_dir.mkdir(exist_ok=True, parents=True)

        self.declare_parameter(
            'report_template_dir',
            str(pathlib.Path(__file__).parent.absolute().joinpath(
                '..',
                'bundles',
                'voyager',
                'template.html'))
        )
        report_template_dir = pathlib.Path(
            self.get_parameter(
                'report_template_dir'
            ).get_parameter_value().string_value).expanduser()

        with open(pathlib.Path(report_template_dir), 'r') as report_template:
            self.report_template = report_template.read()

        self.declare_parameter('description', '')
        config_path = self.get_parameter(
            'description'
        ).get_parameter_value().string_value

        if config_path == '':
            raise RuntimeError('You must specify a description file: \n\
                 --ros-args -p description:=[PATH]')

        # Manage config file
        self.config = None
        path = pathlib.Path(config_path)
        self.voyager_dir = path.parents[0]
        if path.is_file():
            with open(path) as config_file:
                self.config = yaml.load(config_file)
        else:
            raise RuntimeError(
                '{} is not correct yaml config file.'.format(path)
            )

        self.__generate_voyager_results()

    def __generate_voyager_results(self):
        html_kwargs = {'su': ''}
        table = []

        for record in self.config['layout']:
            with open(str(self.voyager_dir.joinpath(record)), 'r') as dscf:
                data = yaml.load(dscf)
                benchmark_name = str(data['benchmark']['id']) + '-' \
                    + str(data['benchmark']['tag'])
                report_file = \
                    pathlib.Path(self.report_dir) \
                    .expanduser() \
                    .joinpath(benchmark_name) \
                    .joinpath('report.yaml')
                if report_file.exists():
                    with open(str(report_file), 'r') as data_file:
                        report_data = yaml.load(data_file)
                        table.append(
                            '{:-6.2F}'
                            .format(report_data['messages']['percent'])
                        )
                        images_dir = self.output_dir.joinpath('images')
                        images_dir.mkdir(exist_ok=True, parents=True)
                        shutil.copy(
                            str(report_file.parents[0].joinpath('plots.jpg')),
                            str(images_dir.joinpath(benchmark_name+'.jpg')))
                        html_kwargs['su'] += \
                            (
                                "<img src='images/" +
                                benchmark_name +
                                ".jpg'></img><br>\n"
                            )
                else:
                    table.append('  n/a')

        print('==================================')
        print('Voyager case results: \n')
        print('\t', ['     1', '    10', '   100', '  1000'])
        print('1KB \t', table[:4])
        print('10KB \t', table[4:8])
        print('100KB \t', table[8:12])
        print('1000KB \t', table[12:16])
        print()

        for row in range(0, 4):
            for col in range(0, 4):
                val = 'n/a' if row*4+col >= len(table) else table[row*4+col]
                html_kwargs.update({'t{}{}'.format(row, col): val})
        out_html = \
            self.report_template.format_map(DefaultMapper(**html_kwargs))
        voyager_report_path = self.output_dir.joinpath('voyager_report.html')
        with open(str(voyager_report_path), 'w') as outfile:
            outfile.write(out_html)


def main():
    """Ros2 once-spin run."""
    rclpy.init()
    node = VoyagerCase()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
    main()
