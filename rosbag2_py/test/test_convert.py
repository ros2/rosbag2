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

import pytest

from rosbag2_py import bag_rewrite, StorageOptions
from rosbag2_test_common import TESTED_STORAGE_IDS

RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])


def test_no_toplevel_key(tmpdir):
    output_options_path = tmpdir / 'no_toplevel_key.yml'
    output_options_content = '[{key: value}]'
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    with pytest.raises(RuntimeError):
        bag_rewrite([], '', str(output_options_path))


def test_output_bags_not_a_list(tmpdir):
    output_options_path = tmpdir / 'not_a_list.yml'
    output_options_content = '{output_bags: {key: value}}'
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    with pytest.raises(RuntimeError):
        bag_rewrite([], '', str(output_options_path))


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_basic_convert(tmpdir, storage_id):
    # This test is just to test that the rosbag2_py wrapper parses input
    # It is not a comprehensive test of bag_rewrite.
    bag_a_path = RESOURCES_PATH / storage_id / 'convert_a'
    bag_b_path = RESOURCES_PATH / storage_id / 'convert_b'
    output_uri_1 = tmpdir / storage_id / 'converted_1'
    output_uri_2 = tmpdir / storage_id / 'converted_2'
    input_options = [
        StorageOptions(uri=str(bag_a_path)),
        StorageOptions(uri=str(bag_b_path)),
    ]
    output_options_path = tmpdir / 'simple_convert.yml'
    output_options_content = f"""
output_bags:
- uri: {output_uri_1}
  storage_id: {storage_id}
  topics: [a_empty]
- uri: {output_uri_2}
  storage_id: {storage_id}
  exclude: ".*empty.*"
"""
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    bag_rewrite(input_options, '', str(output_options_path))
    assert output_uri_1.exists()
    assert output_uri_1.isdir()
    assert (output_uri_1 / 'metadata.yaml').exists()

    assert output_uri_2.exists()
    assert output_uri_2.isdir()
    assert (output_uri_2 / 'metadata.yaml').exists()


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_input_options_yaml_parsing(tmpdir, storage_id):
    """Test that input options can be loaded from YAML file."""
    bag_a_path = RESOURCES_PATH / storage_id / 'convert_a'
    bag_b_path = RESOURCES_PATH / storage_id / 'convert_b'

    # Write input options YAML
    input_options_path = tmpdir / 'input_options.yml'
    input_options_content = f"""
input_bags:
- uri: {bag_a_path}
  storage_id: {storage_id}
- uri: {bag_b_path}
  storage_id: {storage_id}
"""
    with input_options_path.open('w') as f:
        f.write(input_options_content)

    # Write output YAML
    output_uri = tmpdir / storage_id / 'converted_from_input_yaml'
    output_options_path = tmpdir / 'out.yml'
    output_options_content = f"""
output_bags:
- uri: {output_uri}
  storage_id: {storage_id}
  all_topics: true
  all_services: true
"""
    with output_options_path.open('w') as f:
        f.write(output_options_content)

    # Call bag_rewrite with input config file (empty input_options vector)
    bag_rewrite([], str(input_options_path), str(output_options_path))

    assert output_uri.exists()
    assert os.path.isdir(output_uri)
    assert (output_uri / 'metadata.yaml').exists()


def test_input_options_yaml_missing_uri(tmpdir):
    """Test that missing URI in input YAML raises error."""
    input_options_path = tmpdir / 'bad_input.yml'
    input_options_content = """
input_bags:
- storage_id: mcap
"""
    with input_options_path.open('w') as f:
        f.write(input_options_content)

    output_options_path = tmpdir / 'out.yml'
    output_options_content = """
output_bags:
- uri: /tmp/test
  storage_id: mcap
  all_topics: true
"""
    with output_options_path.open('w') as f:
        f.write(output_options_content)

    with pytest.raises(RuntimeError,
                       match="Input bag StorageOptions must specify a non-empty 'uri'."):
        bag_rewrite([], str(input_options_path), str(output_options_path))


def test_both_input_sources_provided(tmpdir):
    """Test that providing both input_options and input_config_file raises error."""
    bag_path = RESOURCES_PATH / TESTED_STORAGE_IDS[0] / 'convert_a'
    input_options = [StorageOptions(uri=str(bag_path))]

    input_config_path = tmpdir / 'input.yml'
    with input_config_path.open('w') as f:
        f.write('input_bags: []')

    output_options_path = tmpdir / 'out.yml'
    with output_options_path.open('w') as f:
        f.write('output_bags: []')

    with pytest.raises(RuntimeError, match='Exactly one input source must be provided.*'):
        bag_rewrite(input_options, str(input_config_path), str(output_options_path))
