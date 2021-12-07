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

import os
from pathlib import Path
import sys
import tempfile
import unittest

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)

from common import get_rosbag_options  # noqa
import rosbag2_py  # noqa
from rosbag2_py import (
    bag_rewrite,
    StorageOptions,
)  # noqa

RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])


class TestConvert(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.tmp_path = Path(cls.tmpdir.name)

    @classmethod
    def tearDownClass(cls):
        try:
            cls.tmpdir.cleanup()
        except OSError:
            pass

    def test_no_toplevel_key(self):
        output_options_path = self.tmp_path / 'no_toplevel_key.yml'
        output_options_content = """
- key: value
"""
        with output_options_path.open('w') as f:
            f.write(output_options_content)
        with self.assertRaises(RuntimeError):
            bag_rewrite([], str(output_options_path))

    def test_output_bags_not_a_list(self):
        output_options_path = self.tmp_path / 'not_a_list.yml'
        output_options_content = """
output_bags:
  key: value
"""
        with output_options_path.open('w') as f:
            f.write(output_options_content)
        with self.assertRaises(RuntimeError):
            bag_rewrite([], str(output_options_path))

    def test_basic_convert(self):
        # This test is just to test that the rosbag2_py wrapper parses input
        # It is not a comprehensive test of bag_rewrite.
        bag_a_path = RESOURCES_PATH / 'convert_a'
        bag_b_path = RESOURCES_PATH / 'convert_b'
        output_uri_1 = self.tmp_path / 'converted_1'
        output_uri_2 = self.tmp_path / 'converted_2'
        input_options = [
            StorageOptions(uri=str(bag_a_path)),
            StorageOptions(uri=str(bag_b_path), storage_id='sqlite3'),
        ]
        output_options_path = self.tmp_path / 'simple_convert.yml'
        output_options_content = f"""
output_bags:
- uri: {output_uri_1}
  storage_id: sqlite3
  topics: [a_empty]
- uri: {output_uri_2}
  storage_id: sqlite3
  exclude: "*empty*"
"""
        with output_options_path.open('w') as f:
            f.write(output_options_content)
        bag_rewrite(input_options, str(output_options_path))
        self.assertTrue(output_uri_1.exists() and output_uri_1.is_dir())
        self.assertTrue((output_uri_1 / 'metadata.yaml').exists())
        self.assertTrue(output_uri_2.exists() and output_uri_2.is_dir())
        self.assertTrue((output_uri_2 / 'metadata.yaml').exists())
