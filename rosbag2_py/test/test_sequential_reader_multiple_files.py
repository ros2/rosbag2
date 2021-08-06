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
import sys


if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)

from common import get_rosbag_options  # noqa
import rosbag2_py  # noqa


def test_reset_filter():
    bag_path = str(Path(__file__).parent.parent / 'resources' / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path)

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


def test_seek_forward():
    bag_path = str(Path(__file__).parent.parent / 'resources' / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path)

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


def test_seek_backward():
    bag_path = str(Path(__file__).parent.parent / 'resources' / 'wbag')
    storage_options, converter_options = get_rosbag_options(bag_path)

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
