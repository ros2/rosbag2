# Copyright 2018, Bosch Software Innovations GmbH.
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

import argparse
import os
import sys

from ros1_bridge import generate_messages
from rosidl_cmake import expand_template


def generate_cpp(output_path, template_dir):
    data = generate_messages()

    template_file = os.path.join(template_dir, 'convert_rosbag_message.cpp.em')
    output_file = os.path.join(output_path, 'convert_rosbag_message.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Generate a C++ template converter specialization for rosbag_storage to ROS 2',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--output-path',
        required=True,
        help='The path to the generated C++ files')
    parser.add_argument(
        '--template-dir',
        required=True,
        help='The location of the template file')
    args = parser.parse_args(argv)

    try:
        return generate_cpp(args.output_path, args.template_dir)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
