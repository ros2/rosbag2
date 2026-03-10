# Copyright 2026 Open Source Robotics Foundation, Inc.
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
import rclpy
from rclpy.executors import ExternalShutdownException
import rosbag2_py


def main(args=None):
    rclpy.init(args=args)

    storage_options = rosbag2_py.StorageOptions(uri='my_bag')

    record_options = rosbag2_py.RecordOptions()
    record_options.all_topics = True
    record_options.is_discovery_disabled = False
    record_options.rmw_serialization_format = 'cdr'
    record_options.compression_format = 'zstd'
    record_options.compression_mode = 'file'

    recorder = rosbag2_py.Recorder(
        storage_options,
        record_options,
        'info',
        'compressed_recorder_demo')

    recorder.start_spin()
    recorder.record()

    try:
        while (rclpy.ok()):
            pass
    except (KeyboardInterrupt, ExternalShutdownException):
        recorder.stop_spin()


if __name__ == '__main__':
    main()
