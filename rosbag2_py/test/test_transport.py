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
from pathlib import Path
import re
import threading

from common import get_rosbag_options, wait_for

import pytest

import rclpy
from rclpy.qos import QoSProfile
import rosbag2_py
from rosbag2_test_common import TESTED_STORAGE_IDS
from std_msgs.msg import String


RESOURCES_PATH = Path(__file__).parent / 'resources'
PLAYBACK_UNTIL_TIMESTAMP_REGEX_STRING = r'\[rosbag2_player]: Playback until timestamp: -1'


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


def test_player_log_level():
    rosbag2_py.Player()  # Test for default constructor
    valid_log_level = 'debug'
    rosbag2_py.Player(valid_log_level)

    invalid_log_level = 'xxx'
    with pytest.raises(RuntimeError):
        rosbag2_py.Player(invalid_log_level)


def test_recoder_log_level():
    rosbag2_py.Recorder()  # Test for default constructor
    valid_log_level = 'debug'
    rosbag2_py.Recorder(valid_log_level)

    invalid_log_level = 'xxx'
    with pytest.raises(RuntimeError):
        rosbag2_py.Recorder(invalid_log_level)


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_record_cancel(tmp_path, storage_id):
    bag_path = tmp_path / 'test_record_cancel'
    storage_options, converter_options = get_rosbag_options(str(bag_path), storage_id)

    recorder = rosbag2_py.Recorder()

    record_options = rosbag2_py.RecordOptions()
    record_options.all_topics = True
    record_options.is_discovery_disabled = False
    record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)

    ctx = rclpy.Context()
    ctx.init()
    record_thread = threading.Thread(
        target=recorder.record,
        args=(storage_options, record_options),
        daemon=True)
    record_thread.start()

    node = rclpy.create_node('test_record_cancel', context=ctx)
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    pub = node.create_publisher(String, 'chatter', 10)

    i = 0
    msg = String()

    while rclpy.ok() and i < 10:
        msg.data = 'Hello World: {0}'.format(i)
        i += 1
        pub.publish(msg)

    recorder.cancel()

    metadata_io = rosbag2_py.MetadataIo()
    assert wait_for(lambda: metadata_io.metadata_file_exists(str(bag_path)),
                    timeout=rclpy.duration.Duration(seconds=3))
    record_thread.join()

    metadata = metadata_io.read_metadata(str(bag_path))
    assert len(metadata.relative_file_paths)
    storage_path = bag_path / metadata.relative_file_paths[0]
    assert wait_for(lambda: storage_path.is_file(),
                    timeout=rclpy.duration.Duration(seconds=3))


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_play_cancel(storage_id, capfd):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    player = rosbag2_py.Player()

    play_options = rosbag2_py.PlayOptions()
    play_options.loop = True
    play_options.start_paused = True

    player_thread = threading.Thread(
        target=player.play,
        args=(storage_options, play_options),
        daemon=True)
    player_thread.start()

    def check_playback_start_output(cumulative_out, cumulative_err):
        out, err = capfd.readouterr()
        cumulative_err += err
        cumulative_out += out
        expected_string_regex = re.compile(PLAYBACK_UNTIL_TIMESTAMP_REGEX_STRING)
        matches = expected_string_regex.search(cumulative_err)
        return matches is not None

    captured_out = ''
    captured_err = ''
    assert wait_for(lambda: check_playback_start_output(captured_out, captured_err),
                    timeout=rclpy.duration.Duration(seconds=3))

    player.cancel()
    player_thread.join(3)
    assert not player_thread.is_alive()
