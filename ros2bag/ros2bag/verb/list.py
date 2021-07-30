# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import os

from xml.dom import minidom

from ament_index_python import get_resource
from ament_index_python import get_resources

from ros2bag.verb import VerbExtension


class ListVerb(VerbExtension):
    """Print information about available plugins to the screen."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'plugin_type',
            help='lists available plugins',
            choices=['storage', 'converter', 'compression', 'decompression'])
        parser.add_argument(
            '--verbose', help='output verbose information about the available plugin',
            action='store_true')

    def main(self, *, args):  # noqa: D102
        # the following is the resource index which is created when installing a pluginlib xml file
        if args.plugin_type == 'storage':
            pluginlib_resource_index = 'rosbag2_storage__pluginlib__plugin'
        elif args.plugin_type == 'compression' or args.plugin_type == 'decompression':
            pluginlib_resource_index = 'rosbag2_compression__pluginlib__plugin'
        else:
            pluginlib_resource_index = 'rosbag2_cpp__pluginlib__plugin'

        resources = get_resources(pluginlib_resource_index)
        if args.verbose:
            print('available %s plugins are:' % args.plugin_type)
        for resource in resources:
            plugin_xml_file_paths, base_folder = get_resource(pluginlib_resource_index, resource)
            for file_path in list(filter(None, plugin_xml_file_paths.split('\n'))):
                abs_path = os.path.join(base_folder, file_path)
                if not os.path.exists(abs_path):
                    return 'path does not exist: %s' % abs_path

                xmldoc = minidom.parse(abs_path)
                for class_item in xmldoc.getElementsByTagName('class'):
                    class_name = class_item.attributes['name']
                    type_name = class_item.attributes['type']
                    base_class_name = class_item.attributes['base_class_type']
                    description = class_item.getElementsByTagName('description')[0]

                    # Compression and decompression plugins share the same resource index
                    # so they must be filtered using their base class
                    if args.plugin_type == 'compression' and \
                            base_class_name.value != \
                            'rosbag2_compression::BaseCompressorInterface':
                        continue
                    elif args.plugin_type == 'decompression' and \
                            base_class_name.value != \
                            'rosbag2_compression::BaseDecompressorInterface':
                        continue

                    print('%s%s' % (('name: ' if args.verbose else ''), class_name.value))
                    if args.verbose:
                        print('\t%s' % description.childNodes[0].data)
                        print('\ttype: %s' % type_name.value)
                        print('\tbase_class: %s' % base_class_name.value)
