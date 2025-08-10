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
import re
import signal
import threading

from common import get_rosbag_options, wait_for

import pytest

import rclpy
from rclpy.qos import QoSProfile
from rosbag2_interfaces.srv import Resume
import rosbag2_py
from rosbag2_test_common import TESTED_STORAGE_IDS
from std_msgs.msg import String


RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])
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


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_player_log_level(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)
    play_options = rosbag2_py.PlayOptions()

    rosbag2_py.Player(storage_options, play_options)  # Test for default value
    valid_log_level = 'debug'
    rosbag2_py.Player(storage_options, play_options, valid_log_level)

    invalid_log_level = 'xxx'
    with pytest.raises(RuntimeError):
        rosbag2_py.Player(storage_options, play_options, invalid_log_level)


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_recoder_log_level(tmp_path, storage_id):
    bag_path = tmp_path / 'test_recoder_log_level'
    storage_options, converter_options = get_rosbag_options(str(bag_path), storage_id)
    record_options = rosbag2_py.RecordOptions()

    rosbag2_py.Recorder(storage_options, record_options)  # Test for default value
    valid_log_level = 'debug'
    rosbag2_py.Recorder(storage_options, record_options, valid_log_level)

    invalid_log_level = 'xxx'
    with pytest.raises(RuntimeError):
        rosbag2_py.Recorder(storage_options, record_options, invalid_log_level)


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_player_api(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, _ = get_rosbag_options(bag_path, storage_id)

    play_options = rosbag2_py.PlayOptions()
    play_options.start_paused = True
    play_options.topics_to_filter = ['topic']
    play_options.disable_keyboard_controls = True

    player = rosbag2_py.Player(storage_options, play_options, 'debug', 'rosbag2_player_test')
    assert player.is_paused()

    ctx = rclpy.Context()
    ctx.init()
    node = rclpy.create_node('test_player_api_node', context=ctx)
    msgs = []
    sub = node.create_subscription(
        String, '/topic', lambda msg: msgs.append(msg.data), QoSProfile(depth=15))
    resume_client = node.create_client(Resume, '/rosbag2_player_test/resume')
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True)
    executor_thread.start()

    assert wait_for(
        lambda: sub.get_publisher_count() == 1,
        timeout=rclpy.duration.Duration(seconds=5),
    ), sub.get_publisher_count()

    player.start_spin()
    # Calling start_spin() a second time should do nothing
    player.start_spin()
    player.play()
    # assert player.wait_for_playback_to_start()
    assert player.play_next()
    assert player.play_next()
    assert 2 == player.burst(2)
    player.seek(0)
    # This requires that the player node be spun by an executor
    resume_client.call(Resume.Request(), 5.0)
    assert not player.is_paused()
    assert not player.play_next()
    assert player.wait_for_playback_to_finish(15.0)
    player.stop()

    # 2 msgs with play_next, 2 msgs with burst, then seek to the beginning and play all 10 msgs
    expected_number_of_messages = 14
    assert wait_for(
        lambda: len(msgs) == expected_number_of_messages,
        timeout=rclpy.duration.Duration(seconds=10),
    ), str(msgs)

    executor.shutdown()
    executor_thread.join(3)
    assert not executor_thread.is_alive()
    resume_client.destroy()
    sub.destroy()
    node.destroy_node()
    ctx.shutdown()


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_player_get_starting_time(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, _ = get_rosbag_options(bag_path, storage_id)

    play_options = rosbag2_py.PlayOptions()
    play_options.topics_to_filter = ['topic']

    player = rosbag2_py.Player(storage_options, play_options, 'info', 'rosbag2_player_test')
    starting_time = player.get_starting_time()
    assert starting_time is not None, 'Expected a valid starting time'


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_player_get_playback_duration(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, _ = get_rosbag_options(bag_path, storage_id)

    play_options = rosbag2_py.PlayOptions()
    play_options.topics_to_filter = ['topic']

    player = rosbag2_py.Player(storage_options, play_options, 'info', 'rosbag2_player_test')
    playback_duration = player.get_playback_duration()
    assert playback_duration is not None, 'Expected a valid starting time'


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_recorder_api(tmp_path, storage_id):
    bag_path = tmp_path / 'test_recorder_api'
    storage_options, _ = get_rosbag_options(str(bag_path), storage_id)

    record_options = rosbag2_py.RecordOptions()
    record_options.start_paused = False
    record_options.topics = ['/test_recorder_api_node/topic']
    record_options.is_discovery_disabled = False
    record_options.topic_polling_interval = datetime.timedelta(milliseconds=10)
    record_options.disable_keyboard_controls = True

    recorder = rosbag2_py.Recorder(
        storage_options, record_options, 'info', 'rosbag2_recorder_test')
    assert not recorder.is_paused()
    recorder.start_spin()
    # Calling start_spin() a second time should do nothing
    recorder.start_spin()
    recorder.record()

    ctx = rclpy.Context()
    ctx.init()
    node = rclpy.create_node('test_recorder_api_node', context=ctx)
    pub = node.create_publisher(String, '~/topic', QoSProfile(depth=10))
    # Wait for the recorder to discover the topic
    assert wait_for(
        lambda: pub.get_subscription_count() > 0,
        timeout=rclpy.duration.Duration(seconds=10))
    resume_client = node.create_client(Resume, '/rosbag2_recorder_test/resume')
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True)
    executor_thread.start()

    assert resume_client.wait_for_service(5.0), str(node.get_service_names_and_types())
    recorder.resume()
    assert not recorder.is_paused()
    recorder.pause()
    assert recorder.is_paused()
    # This requires that the recorder node be spun by an executor
    resume_client.call(Resume.Request(), 5.0)
    assert not recorder.is_paused()
    i = 0
    while rclpy.ok() and i < 10:
        pub.publish(String(data=str(i)))
        i += 1
    recorder.stop()

    executor.shutdown()
    executor_thread.join(3)
    assert not executor_thread.is_alive()
    resume_client.destroy()
    pub.destroy()
    node.destroy_node()
    ctx.shutdown()

    metadata_io = rosbag2_py.MetadataIo()
    assert wait_for(
        lambda: metadata_io.metadata_file_exists(str(bag_path)),
        timeout=rclpy.duration.Duration(seconds=10))
    metadata = metadata_io.read_metadata(str(bag_path))
    bag_topics = [
        topic_info.topic_metadata.name
        for topic_info in metadata.topics_with_message_count
    ]
    assert '/test_recorder_api_node/topic' in bag_topics, str(bag_topics)


def test_player_unconfigured():
    # Test that using a constructor without full configuration raises an error when trying to use
    # the player
    player = rosbag2_py.Player()
    with pytest.raises(RuntimeError):
        player.start_spin()
    with pytest.raises(RuntimeError):
        player.play()
    with pytest.raises(RuntimeError):
        player.wait_for_playback_to_start()
    with pytest.raises(RuntimeError):
        player.wait_for_playback_to_finish()
    with pytest.raises(RuntimeError):
        player.stop()
    with pytest.raises(RuntimeError):
        player.pause()
    with pytest.raises(RuntimeError):
        player.resume()
    with pytest.raises(RuntimeError):
        player.is_paused()
    with pytest.raises(RuntimeError):
        player.play_next()
    with pytest.raises(RuntimeError):
        player.burst(1)
    with pytest.raises(RuntimeError):
        player.seek(0)


def test_recorder_unconfigured():
    # Test that using a constructor without full configuration raises an error when trying to use
    # the recorder
    recorder = rosbag2_py.Recorder()
    with pytest.raises(RuntimeError):
        recorder.start_spin()
    with pytest.raises(RuntimeError):
        recorder.record()
    with pytest.raises(RuntimeError):
        recorder.stop()
    with pytest.raises(RuntimeError):
        recorder.pause()
    with pytest.raises(RuntimeError):
        recorder.resume()
    with pytest.raises(RuntimeError):
        recorder.is_paused()


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_record_cancel(tmp_path, storage_id):
    bag_path = tmp_path / 'test_record_cancel'
    storage_options, converter_options = get_rosbag_options(str(bag_path), storage_id)

    record_options = rosbag2_py.RecordOptions()
    record_options.all_topics = True
    record_options.is_discovery_disabled = False
    record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)

    recorder = rosbag2_py.Recorder()

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
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    play_options = rosbag2_py.PlayOptions()
    play_options.loop = True
    play_options.start_paused = True

    player = rosbag2_py.Player(storage_options, play_options)

    player_thread = threading.Thread(
        target=player.play,
        args=(storage_options, play_options),
        daemon=True)
    player_thread.start()

    def check_playback_start_output(cap_streams):
        out, err = capfd.readouterr()
        cap_streams['err'] += err
        cap_streams['out'] += out
        expected_string_regex = re.compile(PLAYBACK_UNTIL_TIMESTAMP_REGEX_STRING)
        matches = expected_string_regex.search(cap_streams['err'])
        return matches is not None

    captured_streams = {'out': '', 'err': ''}

    if not wait_for(lambda: check_playback_start_output(captured_streams),
                    timeout=rclpy.duration.Duration(seconds=5)):
        with capfd.disabled():
            print('\nCaptured stdout:', captured_streams['out'])
            print('\nCaptured stderr:', captured_streams['err'])
        player.cancel()
        player_thread.join()
        assert False

    player.cancel()
    player_thread.join(3)
    assert not player_thread.is_alive()


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_play_process_sigint_in_python_handler(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'talker')
    assert os.path.exists(bag_path), 'Could not find test bag file: ' + bag_path

    storage_options, _ = get_rosbag_options(bag_path, storage_id)

    play_options = rosbag2_py.PlayOptions()
    play_options.loop = True
    play_options.topics_to_filter = ['topic']

    player = rosbag2_py.Player(storage_options, play_options)

    player.start_spin()
    player.play()
    player.wait_for_playback_to_start()

    sigint_triggered = False
    try:
        signal.raise_signal(signal.SIGINT)
    except KeyboardInterrupt:
        sigint_triggered = True
        pass
    finally:
        assert sigint_triggered
        # The player hasn't been stopped by signal need to call stop() method to stop it.
        player.stop()
        player.stop_spin()
        assert player.wait_for_playback_to_finish(15.0)


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_record_process_sigint_in_python_handler(tmp_path, storage_id, capfd):
    bag_path = tmp_path / 'test_recorder_api'
    storage_options, _ = get_rosbag_options(str(bag_path), storage_id)

    record_options = rosbag2_py.RecordOptions()
    record_options.start_paused = False
    record_options.topics = ['/topic']
    record_options.is_discovery_disabled = False
    record_options.topic_polling_interval = datetime.timedelta(milliseconds=10)
    record_options.disable_keyboard_controls = True

    recorder = rosbag2_py.Recorder(
        storage_options, record_options, 'info', 'rosbag2_recorder_test')
    assert not recorder.is_paused()
    recorder.start_spin()
    recorder.record()

    sigint_triggered = False
    try:
        signal.raise_signal(signal.SIGINT)
    except KeyboardInterrupt:
        sigint_triggered = True
        pass
    finally:
        assert sigint_triggered
        # The recorder hasn't been stopped by signal need to call stop() method to stop it.
        recorder.stop()
        recorder.stop_spin()

    metadata_io = rosbag2_py.MetadataIo()
    assert wait_for(lambda: metadata_io.metadata_file_exists(str(bag_path)),
                    timeout=rclpy.duration.Duration(seconds=3))
    metadata = metadata_io.read_metadata(str(bag_path))
    assert len(metadata.relative_file_paths)
    storage_path = bag_path / metadata.relative_file_paths[0]
    assert wait_for(lambda: storage_path.is_file(),
                    timeout=rclpy.duration.Duration(seconds=3))
