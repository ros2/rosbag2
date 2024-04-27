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
from pathlib import Path

import pytest

from ros2bag.verb.play import add_player_arguments

RESOURCES_PATH = Path(__file__).parent / 'resources'


@pytest.fixture(scope='function')
def test_arguments_parser():
    parser = argparse.ArgumentParser()
    add_player_arguments(parser)
    return parser


def test_topics_list_arguments(test_arguments_parser):
    """Test player topic list arguments parser."""
    bag_path = RESOURCES_PATH / 'empty_bag'
    args = test_arguments_parser.parse_args(['--topics', 'topic1 topic2', bag_path.as_posix()])
    assert ['topic1', 'topic2'] == args.topics


def test_services_list_arguments(test_arguments_parser):
    """Test player services list arguments parser."""
    bag_path = RESOURCES_PATH / 'empty_bag'
    args = test_arguments_parser.parse_args(
        ['--services', 'service1 service2', bag_path.as_posix()]
    )
    assert ['service1', 'service2'] == args.services
