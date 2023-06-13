# Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import datetime
import os
from pathlib import Path
import sys
import threading


from common import get_rosbag_options, wait_for
import rclpy
from rclpy.qos import QoSProfile
import rosbag2_py
from std_msgs.msg import String

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)


def test_options_qos_conversion():
    # Tests that the to-and-from C++ conversions are working properly in the pybind structs
    simple_overrides = {
        '/topic': QoSProfile(depth=10)
    }

    play_options = rosbag2_py.PlayOptions()
    play_options.topic_qos_profile_overrides = simple_overrides
    assert play_options.topic_qos_profile_overrides == simple_overrides

    record_options = rosbag2_py.RecordOptions()
    record_options.topic_qos_profile_overrides = simple_overrides
    assert record_options.topic_qos_profile_overrides == simple_overrides


def test_record_cancel(tmp_path):
    bag_path = str(tmp_path / 'test_record_cancel')
    storage_options, converter_options = get_rosbag_options(bag_path)

    recorder = rosbag2_py.Recorder()

    record_options = rosbag2_py.RecordOptions()
    record_options.all = True
    record_options.is_discovery_disabled = False
    record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)

    rclpy.init()
    record_thread = threading.Thread(
        target=recorder.record,
        args=(storage_options, record_options),
        daemon=True)
    record_thread.start()

    node = rclpy.create_node('test_record_cancel')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    pub = node.create_publisher(String, 'chatter', 10)

    i = 0
    msg = String()

    while rclpy.ok() and i < 10:
        msg.data = 'Hello World: {0}'.format(i)
        i += 1
        pub.publish(msg)

    recorder.cancel()

    metadata_path = Path(bag_path) / 'metadata.yaml'
    db3_path = Path(bag_path) / 'test_record_cancel_0.db3'
    assert wait_for(lambda: metadata_path.is_file() and db3_path.is_file(),
                    timeout=rclpy.duration.Duration(seconds=3))
    record_thread.join()
