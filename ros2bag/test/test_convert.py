# Copyright 2026 TinyGeek
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
from unittest.mock import Mock

import pytest

from ros2bag.verb import convert


@pytest.mark.parametrize('input_source', ['--input', '--input-options'])
@pytest.mark.parametrize('output_uri', [None, 'converted bag'])
def test_convert_output_uri(tmp_path, monkeypatch, input_source, output_uri):
    parser = argparse.ArgumentParser()
    verb = convert.ConvertVerb()
    verb.add_arguments(parser, 'ros2 bag')
    arguments = [input_source, str(tmp_path), '-o', 'output.yaml']
    if output_uri is not None:
        arguments.extend(['--output-uri', output_uri])
    args = parser.parse_args(arguments)
    rewrite = Mock()
    monkeypatch.setattr(convert, 'bag_rewrite', rewrite)

    verb.main(args=args)

    input_options, input_config_file, output_config_file = rewrite.call_args.args
    assert output_config_file == 'output.yaml'
    assert rewrite.call_args.kwargs.get('output_uri', '') == (output_uri or '')
    if input_source == '--input':
        assert len(input_options) == 1
        assert input_options[0].uri == str(tmp_path)
        assert input_config_file == ''
    else:
        assert input_options == []
        assert input_config_file == str(tmp_path)
    assert rewrite.call_count == 1


def test_convert_rejects_empty_output_uri(tmp_path, monkeypatch):
    parser = argparse.ArgumentParser()
    verb = convert.ConvertVerb()
    verb.add_arguments(parser, 'ros2 bag')
    args = parser.parse_args([
        '--input', str(tmp_path), '-o', 'output.yaml', '--output-uri', '',
    ])
    rewrite = Mock()
    monkeypatch.setattr(convert, 'bag_rewrite', rewrite)

    with pytest.raises(RuntimeError, match='--output-uri must not be empty'):
        verb.main(args=args)

    rewrite.assert_not_called()
