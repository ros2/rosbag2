# Copyright 2021, Robotec.ai sp. z o.o.
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
import pathlib
import sys

from ament_index_python import get_package_share_directory

import launch

import launch_ros

import yaml

_batch_cfg = None
_producers_cfg = None

_producer_idx = 0
_producer_nodes = []


def _parse_arguments(args=sys.argv[4:]):
    """Parse benchmark and producers config files."""
    batch_cfg = None
    producers_cfg_path = None
    err_str = 'Missing or invalid arguments detected.' \
        'Launchfile requires "benchmark:=" and "producers:=" arguments ' \
        'with coresponding config files.'

    if len(args) != 2:
        raise RuntimeError(err_str)

    else:
        for arg in args:
            if 'benchmark:=' in arg:
                bench_cfg_path = pathlib.Path(arg.replace('benchmark:=', ''))
                if not bench_cfg_path.is_file():
                    raise RuntimeError(
                        'Batch config file {} does not exist.'.format(bench_cfg_path)
                    )
                with open(bench_cfg_path, 'r') as config_file:
                    batch_cfg_yaml = yaml.load(config_file, Loader=yaml.FullLoader)
                    batch_cfg = (batch_cfg_yaml['rosbag2_performance_benchmarking']
                                 ['benchmark_node']
                                 ['ros__parameters'])
            elif 'producers:=' in arg:
                producers_cfg_path = pathlib.Path(arg.replace('producers:=', ''))
                if not producers_cfg_path.is_file():
                    raise RuntimeError(
                        'Producers config file {} does not exist.'.format(producers_cfg_path)
                        )
            else:
                raise RuntimeError(err_str)
        return batch_cfg, producers_cfg_path


def _launch_producers():
    """Launch next writer."""
    global _producer_idx, _producer_nodes
    if _producer_idx == len(_producer_nodes):
        return launch.actions.LogInfo(msg='Benchmark finished!')
    node = _producer_nodes[_producer_idx]['node']
    return node


def _producer_node_started(event, context):
    """Log current status on producer start."""
    global _producer_idx
    return launch.actions.LogInfo(
            msg='-----------{}/{}-----------'.format(_producer_idx+1, len(_producer_nodes))
    )


def _producer_node_exited(event, context):
    """Launch new producer when previously has finished."""
    global _producer_idx, _producer_nodes
    node_params = _producer_nodes[_producer_idx]['parameters']

    if event.returncode != 0:
        return [
            launch.actions.LogInfo(msg='Writer error. Shutting down benchmark.'),
            launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason='Writer error'
                )
            )
        ]

    # Handle clearing bag files
    if not node_params['preserve_bags']:
        db_files = pathlib.Path.cwd().joinpath(node_params['db_folder']).glob('*.db3')
        for f in db_files:
            f.unlink()

    _producer_idx += 1
    return [
        launch.actions.LogInfo(
            msg='---------------------------'
        ),
        _launch_producers()
    ]


def generate_launch_description():
    """Generate launch description for ros2 launch system."""
    global _producer_idx, _producer_nodes, _batch_cfg, _producers_cfg
    _batch_cfg, _producers_cfg = _parse_arguments()

    # Benchmark options
    benchmark_params = _batch_cfg['benchmark']

    repeat_each = benchmark_params.get('repeat_each')
    db_root_folder = benchmark_params.get('db_root_folder')
    summary_result_file = benchmark_params.get('summary_result_file')
    no_transport = benchmark_params.get('no_transport')
    preserve_bags = benchmark_params.get('preserve_bags')

    # Producers options
    producers_params = _batch_cfg['benchmark']['parameters']

    max_cache_size_params = producers_params.get('max_cache_size')
    max_bag_size_params = producers_params.get('max_bag_size')
    compression_params = producers_params.get('compression')
    compression_queue_size_params = producers_params.get('compression_queue_size')
    compression_threads_params = producers_params.get('compression_threads')
    storage_config_file_params = producers_params.get('storage_config_file')

    # Parameters cross section for whole benchmark
    # Parameters cross section is a list of all possible parameters variants
    params_cross_section = []
    timestamp_name = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    # Helper function for generating cross section list
    def __generate_cross_section_parameter(i,
                                           cache,
                                           compression,
                                           compression_queue_size,
                                           compression_threads,
                                           storage_config,
                                           max_bag_size):
        # Storage conf parameter for each producer
        st_conf_filename = storage_config.replace('.yaml', '')
        storage_conf_path = ''
        if storage_config != '':
            storage_conf_path = pathlib.Path(
                get_package_share_directory(
                    'rosbag2_performance_benchmarking'
                    )
            ).joinpath('config', 'storage', storage_config)
            if not storage_conf_path.exists():
                raise RuntimeError(
                    'Config {} does not exist.'.format(storage_config))
            st_conf_filename = pathlib.Path(storage_config).with_suffix('')

        # Generates unique title for producer
        node_title = 'run_' + \
            '{i}_{cache}_{comp}_{comp_q}_{comp_t}_{st_conf}_{bag_size}'.format(
                i=i,
                cache=cache,
                comp=compression if compression else 'default_compression',
                comp_q=compression_queue_size,
                comp_t=compression_threads,
                st_conf=st_conf_filename if st_conf_filename else 'default_config',
                bag_size=max_bag_size
            )

        # Result file path for producer
        result_file = pathlib.Path(db_root_folder).joinpath(
            timestamp_name,
            summary_result_file
        )

        # Database folder path for producer
        db_folder = pathlib.Path(db_root_folder).joinpath(
            timestamp_name,
            node_title
        )

        # Filling up parameters cross section list for benchmark
        params_cross_section.append(
            {
                'node_title': node_title,
                'db_folder': str(db_folder),
                'cache': cache,
                'preserve_bags': preserve_bags,
                'no_transport': no_transport,
                'result_file': str(result_file),
                'compression_format': compression,
                'compression_queue_size': compression_queue_size,
                'compression_threads': compression_threads,
                'storage_config_file': str(storage_conf_path),
                'config_file': str(_producers_cfg),
                'max_bag_size': max_bag_size
            }
        )

    # For the sake of python indentation, multiple for loops in alternative way with helper func
    [
        __generate_cross_section_parameter(
            i,
            cache,
            compression,
            compression_queue_size,
            compression_threads,
            storage_config,
            max_bag_size)
        for i in range(0, repeat_each)
        for cache in max_cache_size_params
        for compression in compression_params
        for compression_queue_size in compression_queue_size_params
        for compression_threads in compression_threads_params
        for storage_config in storage_config_file_params
        for max_bag_size in max_bag_size_params
    ]

    ld = launch.LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )

    # Create all required nodes for benchmark
    for producer_param in params_cross_section:
        parameters = [
            producer_param['config_file'],
            {'max_cache_size': producer_param['cache']},
            {'max_bag_size': producer_param['max_bag_size']},
            {'db_folder': producer_param['db_folder']},
            {'results_file': producer_param['result_file']},
            {'compression_queue_size': producer_param['compression_queue_size']},
            {'compression_threads': producer_param['compression_threads']}
        ]

        if producer_param['storage_config_file'] != '':
            parameters.append({'storage_config_file': producer_param['storage_config_file']})
        if producer_param['compression_format'] != '':
            parameters.append({'compression_format': producer_param['compression_format']})

        # TODO(piotr.jaroszek): choose node based on 'no_transport' parameter
        producer_node = launch_ros.actions.Node(
            package='rosbag2_performance_benchmarking',
            executable='writer_benchmark',
            name='rosbag2_performance_benchmarking_node',
            parameters=parameters
        )

        # Fill up dict with producer nodes and their corresponding parameters
        _producer_nodes.append({'node': producer_node, 'parameters': producer_param})

    # Connect start and exit events for each producer
    for producer_node in _producer_nodes:
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=producer_node['node'],
                    on_exit=_producer_node_exited
                )
            )
        )
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessStart(
                    target_action=producer_node['node'],
                    on_start=_producer_node_started
                )
            )
        )

    # Launch nodes one after another. Next node is launched after previous is finished.
    ld.add_action(_launch_producers())

    return ld


if __name__ == '__main__':
    raise RuntimeError('Batch benchmark launchfile does not support standalone execution.')
