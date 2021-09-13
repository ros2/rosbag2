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

import time
from typing import Callable

from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
import rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def wait_for(
    condition: Callable[[], bool],
    timeout: Duration,
    sleep_time: float = 0.1,
):
    clock = Clock(clock_type=ClockType.STEADY_TIME)
    start = clock.now()
    while not condition():
        if clock.now() - start > timeout:
            return False
        time.sleep(sleep_time)
        return True
