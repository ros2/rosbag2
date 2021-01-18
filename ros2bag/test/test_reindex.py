# Copyright 2020 DCS Corporation, All Rights Reserved.
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
#
# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
# Part 252.227-7013 or 7014 (Feb 2014).
#
# This notice must appear in all copies of this file and its derivatives.

import contextlib
from pathlib import Path
from typing import Any
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest
import yaml


RESOURCES_PATH = Path(__file__).parent / 'resources' / 'reindexing'
# TEST_NODE = 'ros2bag_record_qos_profile_test_node'
# TEST_NAMESPACE = 'ros2bag_record_qos_profile'
TEST_NODE = 'ros2bag_reindex_node'
TEST_NAMESPACE = 'ros2bag_reindex'
ERROR_STRING = r'\[ERROR] \[ros2bag]:'


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


def check_version(target_base_node: Any, test_base_node: Any) -> None:
    target_version = target_base_node.get('version')
    test_version = test_base_node.get('version')
    assert target_base_node.get('version') == test_base_node.get('version'), \
        print('Reindex generated wrong version. Expected "{}"; got "{}"'.format(
                target_version, test_version
            ))


def check_storage_identifier(target_base_node: Any, test_base_node: Any) -> None:
    target_si = target_base_node.get('storage_identifier')
    test_si = test_base_node.get('storage_identifier')
    assert target_si == test_si, \
        print('Reindex generated wrong storage identifier. '
              'Expected "{}"; got "{}"'.format(
                  target_si, test_si
              ))


def check_relative_filepaths(target_base_node: Any, test_base_node: Any) -> None:
    target_fp = target_base_node.get('relative_file_paths')
    test_fp = test_base_node.get('relative_file_paths')
    assert target_fp == test_fp, \
        print('Reindex generated wrong relative file paths. '
              'Expected "{}"; got "{}"'.format(
                target_fp, test_fp
                ))


def check_duration(target_base_node: Any, test_base_node: Any) -> None:
    target_duration = target_base_node.get('duration')
    test_duration = test_base_node.get('duration')
    assert target_duration == test_duration, \
        print('Reindex generated wrong duration. '
              'Expected "{}"; got "{}"'.format(
                target_duration, test_duration
                ))


def check_starting_time(target_base_node: Any, test_base_node: Any) -> None:
    target_start = target_base_node.get('starting_time')
    test_start = test_base_node.get('starting_time')
    assert target_start == test_start, \
        print('Reindex generated wrong starting time. '
              'Expected "{}"; got "{}"'.format(
                target_start, test_start
                ))


def check_message_count(target_base_node: Any, test_base_node: Any) -> None:
    target_mc = target_base_node.get('message_count')
    test_mc = test_base_node.get('message_count')
    assert target_mc == test_mc, \
        print('Reindex generated wrong message count. '
              'Expected "{}"; got "{}'.format(
                target_mc, test_mc
                ))


def check_topic_type(target_topic_metadata: Any, test_topic_metadata: Any) -> None:
    target_name = target_topic_metadata.get('topic_metadata').get('name')
    test_type = test_topic_metadata.get('topic_metadata').get('type')
    target_type = target_topic_metadata.get('topic_metadata').get('type')
    assert test_type == target_type, \
        print('Reindex generated incorrect target type for "{}". '
              'Expected "{}", got "{}'.format(target_name, target_type, test_type))


def check_topic_ser_fmt(target_topic_metadata: Any, test_topic_metadata: Any) -> None:
    target_name = target_topic_metadata.get('topic_metadata').get('name')
    test_ser_fmt = test_topic_metadata.get('topic_metadata').get('serialization_format')
    target_ser_fmt = target_topic_metadata.get('topic_metadata').get('serialization_format')
    assert test_ser_fmt == target_ser_fmt, \
        print('Reindex generated incorrect serialization format for "{}". '
              'Expected "{}", got "{}"'.format(target_name, target_ser_fmt, test_ser_fmt))


def check_topic_qos(target_topic_metadata: Any, test_topic_metadata: Any) -> None:
    target_name = target_topic_metadata.get('topic_metadata').get('name')
    test_qos = test_topic_metadata.get('topic_metadata').get('offered_qos_profiles')
    target_qos = test_topic_metadata.get('topic_metadata').get('offered_qos_profiles')
    assert test_qos == target_qos, \
        print('Reindex generated incorrect QOS profiles for "{}". '
              'Expected "{}", got "{}"'.format(target_name, target_qos, test_qos))


def check_topic_message_count(target_topic_metadata: Any, test_topic_metadata: Any) -> None:
    target_name = target_topic_metadata.get('topic_metadata').get('name')
    test_count = test_topic_metadata.get('message_count')
    target_count = target_topic_metadata.get('message_count')
    assert test_count == target_count, \
        print('Reindex generated incorrect message count for "{}". '
              'Expected "{}", got "{}"'.format(target_name, target_count, test_count))


def check_topics(target_base_node: Any, test_base_node: Any) -> None:
    target_topics = target_base_node.get('topics_with_message_count')
    test_topics = test_base_node.get('topics_with_message_count')
    assert test_topics, print('Reindex generated no topics block')

    if test_topics:
        for test_topic in test_topics:
            test_name = test_topic.get('topic_metadata').get('name')
            target_topic = next((x for x in target_topics
                                 if x.get('topic_metadata').get('name') == test_name))

            assert target_topic, print('Reindex generated extra topic: "{}"'.format(test_name))
            if target_topic:
                check_topic_type(target_topic, test_topic)
                check_topic_ser_fmt(target_topic, test_topic)
                check_topic_qos(target_topic, test_topic)
                check_topic_message_count(target_topic, test_topic)


def check_compression_fmt(target_base_node: Any, test_base_node: Any) -> None:
    target_c_fmt = target_base_node.get('compression_format')
    test_c_fmt = test_base_node.get('compression_format')
    assert target_c_fmt == test_c_fmt, \
        print('Reindex generated incorrect compression format. '
              'Expected "{}", got "{}"'.format(target_c_fmt, test_c_fmt))


def check_compression_mode(target_base_node: Any, test_base_node: Any) -> None:
    target_mode = target_base_node.get('compression_mode')
    test_mode = test_base_node.get('compression_mode')
    assert target_mode == test_mode, \
        print('Reindex generated incorrect compression mode. '
              'Expected "{}", got "{}"'.format(target_mode, test_mode))


def compare_metadata_files(target_file: Path, test_file: Path):
    target_yaml = yaml.safe_load(target_file.open())
    generated_yaml = yaml.safe_load(test_file.open())

    # Check that base node exists
    target_base_node = target_yaml.get('rosbag2_bagfile_information')
    test_base_node = generated_yaml.get('rosbag2_bagfile_information')
    assert test_base_node, print('Reindex was unable to generate base node')

    check_version(target_base_node, test_base_node)
    check_storage_identifier(target_base_node, test_base_node)

    # INCONSISTENT BETWEEN COMPRESSED / NON COMPRESSED BAGS
    # Disabling for now
    # check_relative_filepaths(target_base_node, test_base_node)

    # MAY NOT BE ABLE TO GUARANTEE THIS #
    # check_duration(target_base_node, test_base_node)
    # check_starting_time(target_base_node, test_base_node)

    check_message_count(target_base_node, test_base_node)
    check_topics(target_base_node, test_base_node)
    check_compression_fmt(target_base_node, test_base_node)
    check_compression_mode(target_base_node, test_base_node)


class TestRos2BagReindexMultiFile(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2bag-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                    launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_bag_command = launch_bag_command

    @classmethod
    def tearDown(cls) -> None:
        metadata_file = RESOURCES_PATH / 'multiple_files' / 'metadata.yaml'
        metadata_file.unlink(True)

    def test_multiple_files(self):
        bag_path = RESOURCES_PATH / 'multiple_files'
        metadata_file = bag_path / 'metadata.yaml'
        target_file = bag_path / 'multiple_files_target.yaml'

        arguments = ['reindex', bag_path.as_posix()]
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)

        # Metadata.yaml file should be created at this point
        compare_metadata_files(target_file, metadata_file)


class TestRos2BagReindexMessageCompression(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2bag-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                    launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_bag_command = launch_bag_command

    @classmethod
    def tearDown(cls) -> None:
        metadata_file = RESOURCES_PATH / 'message_compression' / 'metadata.yaml'
        metadata_file.unlink(True)

    def test_compressed(self):
        bag_path = RESOURCES_PATH / 'message_compression'
        metadata_file = bag_path / 'metadata.yaml'
        target_file = bag_path / 'message_compression_target.yaml'

        arguments = ['reindex', bag_path.as_posix(),
                     '--compression-format', 'zstd',
                     '--compression-mode', 'message']
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)

        # Metadata.yaml file should be created at this point
        compare_metadata_files(target_file, metadata_file)


class TestRos2BagReindexFileCompression(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_bag_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'bag', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2bag-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                    launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_bag_command = launch_bag_command

    @classmethod
    def tearDown(cls) -> None:
        metadata_file = RESOURCES_PATH / 'file_compression' / 'metadata.yaml'
        metadata_file.unlink(True)

    def test_compressed(self):
        bag_path = RESOURCES_PATH / 'file_compression'
        metadata_file = bag_path / 'metadata.yaml'
        target_file = bag_path / 'file_compression_target.yaml'

        arguments = ['reindex', bag_path.as_posix(),
                     '--compression-format', 'zstd',
                     '--compression-mode', 'file']
        with self.launch_bag_command(arguments=arguments) as bag_command:
            bag_command.wait_for_shutdown(timeout=5)

        # Metadata.yaml file should be created at this point
        compare_metadata_files(target_file, metadata_file)