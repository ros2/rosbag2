# Copyright 2024 Apex.AI, Inc.
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
import datetime
from pathlib import Path

import pytest

from ros2bag.verb.record import add_recorder_arguments
from ros2bag.verb.record import validate_parsed_arguments

RESOURCES_PATH = Path(__file__).parent / 'resources'
ERROR_STRING_MSG = 'ros2bag record CLI did not produce the expected output' \
                   '\n Expected output pattern: {}\n Actual output: {}'


@pytest.fixture(scope='function')
def test_arguments_parser():
    parser = argparse.ArgumentParser()
    add_recorder_arguments(parser)
    return parser


def test_recorder_positional_topics_list_argument(test_arguments_parser):
    """Test recorder positional topics list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['topic1', 'topic2', '--output', output_path.as_posix()]
    )
    assert ['topic1', 'topic2'] == args.topics_positional
    assert output_path.as_posix() == args.output


def test_recorder_optional_topics_list_argument(test_arguments_parser):
    """Test recorder optional --topics list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--topics', 'topic1', 'topic2', '--output', output_path.as_posix()]
    )
    assert ['topic1', 'topic2'] == args.topics
    assert output_path.as_posix() == args.output


def test_recorder_services_list_argument(test_arguments_parser):
    """Test recorder --services list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--services', 'service1', 'service2', '--output', output_path.as_posix()]
    )
    assert ['service1', 'service2'] == args.services
    assert output_path.as_posix() == args.output


def test_recorder_services_and_positional_topics_list_arguments(test_arguments_parser):
    """Test recorder --services list and positional topics list arguments parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--output', output_path.as_posix(),
         '--services', 'service1', 'service2', '--', 'topic1', 'topic2'])
    assert ['service1', 'service2'] == args.services
    assert ['topic1', 'topic2'] == args.topics_positional
    assert output_path.as_posix() == args.output


def test_recorder_services_and_optional_topics_list_arguments(test_arguments_parser):
    """Test recorder --services list and optional --topics list arguments parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--output', output_path.as_posix(),
         '--services', 'service1', 'service2', '--topics', 'topic1', 'topic2'])
    assert ['service1', 'service2'] == args.services
    assert ['topic1', 'topic2'] == args.topics
    assert output_path.as_posix() == args.output


def test_recorder_topic_types_list_argument(test_arguments_parser):
    """Test recorder --topic-types list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--topic-types', 'topic_type1', 'topic_type2', '--output', output_path.as_posix()])
    assert ['topic_type1', 'topic_type2'] == args.topic_types
    assert output_path.as_posix() == args.output


def test_recorder_exclude_topic_types_list_argument(test_arguments_parser):
    """Test recorder --exclude-topic-types list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-topic-types', 'topic_type1', 'topic_type2', '--output',
         output_path.as_posix()])
    assert ['topic_type1', 'topic_type2'] == args.exclude_topic_types
    assert output_path.as_posix() == args.output


def test_recorder_exclude_topics_list_argument(test_arguments_parser):
    """Test recorder --exclude-topics list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-topics', 'topic1', 'topic2', '--output', output_path.as_posix()]
    )
    assert ['topic1', 'topic2'] == args.exclude_topics
    assert output_path.as_posix() == args.output


def test_recorder_exclude_services_list_argument(test_arguments_parser):
    """Test recorder --exclude-services list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-services', 'service1', 'service2', '--output', output_path.as_posix()]
    )
    assert ['service1', 'service2'] == args.exclude_services
    assert output_path.as_posix() == args.output


def test_recorder_custom_data_list_argument(test_arguments_parser):
    """Test recorder --custom-data list argument parser."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--custom-data', 'Key1=Value1', 'key2=value2', '--output', output_path.as_posix()]
    )
    assert ['Key1=Value1', 'key2=value2'] == args.custom_data
    assert output_path.as_posix() == args.output


def test_recorder_validate_exclude_regex_needs_inclusive_args(test_arguments_parser):
    """Test that --exclude-regex needs inclusive arguments."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-regex', '%s-%s', '--services', 'service1', '--topics', 'topic1',
         '--output', output_path.as_posix()]
    )
    assert '%s-%s' == args.exclude_regex
    assert args.all is False
    assert args.all_topics is False
    assert [] == args.topic_types
    assert args.all_services is False
    assert '' == args.regex

    uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
    error_str = validate_parsed_arguments(args, uri)
    assert error_str is not None
    expected_output = '--exclude-regex argument requires either --all, ' \
                      '--all-topics, --topic-types, --all-services or --regex'
    matches = expected_output in error_str
    assert matches, ERROR_STRING_MSG.format(expected_output, error_str)


def test_recorder_validate_exclude_topics_needs_inclusive_args(test_arguments_parser):
    """Test that --exclude-topics inclusive arguments."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-topics', 'topic1', '--all-services', '--topics', 'topic1',
         '--output', output_path.as_posix()]
    )
    assert ['topic1'] == args.exclude_topics
    assert args.all is False
    assert args.all_topics is False
    assert [] == args.topic_types
    assert args.all_services is True
    assert '' == args.regex
    assert '' == args.exclude_regex

    uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
    error_str = validate_parsed_arguments(args, uri)
    assert error_str is not None
    expected_output = '--exclude-topics argument requires either --all, --all-topics, ' \
                      '--topic-types or --regex'
    matches = expected_output in error_str
    assert matches, ERROR_STRING_MSG.format(expected_output, error_str)


def test_recorder_validate_exclude_topics_types_needs_inclusive_args(test_arguments_parser):
    """Test that --exclude-topic-types needs inclusive argument."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-topic-types', '/topic_type1', '--all-services', '--topics', 'topic1',
         '--output', output_path.as_posix()]
    )
    assert ['/topic_type1'] == args.exclude_topic_types
    assert args.all is False
    assert args.all_topics is False
    assert [] == args.topic_types
    assert args.all_services is True
    assert '' == args.regex
    assert '' == args.exclude_regex

    uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
    error_str = validate_parsed_arguments(args, uri)
    assert error_str is not None
    expected_output = '--exclude-topic-types argument requires either --all, ' \
                      '--all-topics or --regex'
    matches = expected_output in error_str
    assert matches, ERROR_STRING_MSG.format(expected_output, error_str)


def test_recorder_validate_exclude_services_needs_inclusive_args(test_arguments_parser):
    """Test that --exclude-services needs at least --all, --all-services or --regex arguments."""
    output_path = RESOURCES_PATH / 'ros2bag_tmp_file'
    args = test_arguments_parser.parse_args(
        ['--exclude-services', 'service1', '--services', 'service1', '--all-topics',
         '--output', output_path.as_posix()]
    )
    assert ['service1'] == args.exclude_services
    assert args.all is False
    assert args.all_topics is True
    assert [] == args.topic_types
    assert args.all_services is False
    assert '' == args.regex
    assert '' == args.exclude_regex

    uri = args.output or datetime.datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')
    error_str = validate_parsed_arguments(args, uri)
    assert error_str is not None
    expected_output = '--exclude-services argument requires either --all, --all-services '
    'or --regex'
    matches = expected_output in error_str
    assert matches, ERROR_STRING_MSG.format(expected_output, error_str)
