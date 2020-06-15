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

import rosbag2_py._rosbag2_py as rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
  storage_options = rosbag2_py.StorageOptions()
  storage_options.uri = path
  storage_options.storage_id = 'sqlite3'

  converter_options = rosbag2_py.ConverterOptions()
  converter_options.input_serialization_format = serialization_format
  converter_options.output_serialization_format = serialization_format

  return storage_options, converter_options
