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

import os
from pathlib import Path

from common import get_rosbag_options, TESTED_STORAGE_IDS

import pytest

import rosbag2_py


RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_reset_filter(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id=storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=['AAA', 'CCC', 'DDD'])
    reader.set_filter(storage_filter)

    (topic, data, t) = reader.read_next()

    assert topic == 'AAA'
    assert t == 1001

    (topic, data, t) = reader.read_next()

    assert topic == 'CCC'
    assert t == 1002

    (topic, data, t) = reader.read_next()

    assert topic == 'AAA'
    assert t == 1004

    # No filter and bag continues same location
    reader.reset_filter()

    (topic, data, t) = reader.read_next()

    assert topic == 'FFF'
    assert t == 1004

    (topic, data, t) = reader.read_next()

    assert topic == 'BBB'
    assert t == 1004

    (topic, data, t) = reader.read_next()

    assert topic == 'EEE'
    assert t == 1005


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_seek_forward(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # seek forward
    reader.seek(1822)

    (topic, data, t) = reader.read_next()

    assert topic == 'CCC'
    assert t == 1822

    # set filter continues in same location
    storage_filter = rosbag2_py.StorageFilter(topics=['BBB', 'GGG'])
    reader.set_filter(storage_filter)

    (topic, data, t) = reader.read_next()

    assert topic == 'GGG'
    assert t == 1822

    (topic, data, t) = reader.read_next()

    assert topic == 'GGG'
    assert t == 1822

    (topic, data, t) = reader.read_next()

    assert topic == 'BBB'
    assert t == 1826


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_seek_backward(storage_id):
    bag_path = str(RESOURCES_PATH / storage_id / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path, storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # seek forward first
    reader.seek(1822)
    storage_filter = rosbag2_py.StorageFilter(topics=['BBB', 'GGG'])
    reader.set_filter(storage_filter)
    (topic, data, t) = reader.read_next()

    # seek backwards & filter preserved
    reader.seek(1408)

    (topic, data, t) = reader.read_next()

    assert topic == 'BBB'
    assert t == 1408

    (topic, data, t) = reader.read_next()

    assert topic == 'GGG'
    assert t == 1408

    (topic, data, t) = reader.read_next()

    assert topic == 'BBB'
    assert t == 1413
