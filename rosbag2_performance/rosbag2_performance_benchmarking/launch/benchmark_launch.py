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

"""
Launchfile for benchmarking rosbag2.

This launchfile can only be launched with 'ros2 launch' command.

Two launch arguments are required:
* benchmark - path to benchmark description in yaml format ('benchmark:=<PATH>'),
* producers - path to producers description in yaml format ('producers:=<PATH>').

Goal of this launchfile is to launch in sequence all processes and/or nodes with right parameters
required for selected benchmark. Cross section of parameters is generated based on parameters from
'benchmark' yaml description file.

Based on 'no_transport' parameter in benchmark description, a single run in launch sequence
looks as follow:

NO TRANSPORT:
Only 'writer_benchmark' node is used as 'producer node' (PN). It directly writes messages to
a storage and then fill up a result file. No additional processes are required.

PN starts -> PN exits

TRANSPORT:
For end-to-end benchmark, `ros2 bag record` (ROSBAG) process and 'result writer' (RW) are also
included in a single launch sequence run. In this case 'benchmark_publishers' node act as
producer node. Result writer node writes final result file.

ROSBAG starts -> PN starts -> PN exits -> ROSBAG exits -> RW starts

After the whole sequence is finished, both producers and benchmark description files are copied
to benchmark folder.
"""

import datetime
import os
import pathlib
import shutil
import signal
import sys
import time

from ament_index_python import get_package_share_directory

import launch

import launch_ros

import yaml

_bench_cfg_path = None
_producers_cfg_path = None

_producer_idx = 0
_producer_nodes = []

_rosbag_processes = []
_rosbag_pid = None

_result_writers = []


def _parse_arguments(args=sys.argv[4:]):
    """Parse benchmark and producers config file paths."""
    bench_cfg_path = None
    producers_cfg_path = None
    err_str = 'Missing or invalid arguments detected. ' \
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
            elif 'producers:=' in arg:
                producers_cfg_path = pathlib.Path(arg.replace('producers:=', ''))
                if not producers_cfg_path.is_file():
                    raise RuntimeError(
                        'Producers config file {} does not exist.'.format(producers_cfg_path)
                    )
            else:
                raise RuntimeError(err_str)
        return bench_cfg_path, producers_cfg_path


def _copy_config_files():
    """Copy benchmark and producers config files to benchmark folder."""
    global _bench_cfg_path, _producers_cfg_path
    # Copy yaml configs for current benchmark after benchmark is finished
    benchmark_path = pathlib.Path(_producer_nodes[0]['parameters']['db_folder'])
    shutil.copy(str(_bench_cfg_path), str(benchmark_path.with_name('benchmark.yaml')))
    shutil.copy(str(_producers_cfg_path), str(benchmark_path.with_name('producers.yaml')))


def _launch_sequence(transport):
    """
    Continue with launch sequence (launch entry action of next run).

    Launches next producer node or rosbag2 record process, based on transport (end to end)
    or transportless type of benchmark.

    :param" transport If True launch a 'ros2 bag record' process, else a producer node.
    """
    global _producer_idx, _producer_nodes, _rosbag_processes
    if _producer_idx == len(_producer_nodes):
        _copy_config_files()
        return launch.actions.LogInfo(msg='Benchmark finished!')
    action = None
    if transport:
        action = _rosbag_processes[_producer_idx]
    else:
        action = _producer_nodes[_producer_idx]['node']
    return action


def _rosbag_proc_started(event, context):
    """Register current rosbag2 PID so we can terminate it when producer exits."""
    global _rosbag_pid
    _rosbag_pid = event.pid


def _rosbag_ready_check(event):
    """Consider rosbag2 ready when 'Listening for topics...' string is printed.

    Launches producer node if ready.
    """
    target_str = 'Listening for topics...'
    if target_str in event.text.decode():
        return _launch_sequence(transport=False)


def _rosbag_proc_exited(event, context):
    """
    Start next rosbag2 record process after current one exits.

    Launches result writer on exit.
    """
    global _producer_idx, _result_writers, _rosbag_pid

    # ROS2 bag returns 2 if terminated with SIGINT, which we expect here
    if event.returncode != 2:
        _rosbag_pid = None
        return [
            launch.actions.LogInfo(msg='Rosbag2 record error. Shutting down benchmark.'),
            launch.actions.EmitEvent(
                event=launch.events.Shutdown(
                    reason='Rosbag2 record error'
                )
            )
        ]
    return [
            _result_writers[_producer_idx-1]
    ]


def _producer_node_started(event, context):
    """Log current benchmark progress on producer start."""
    global _producer_idx
    return launch.actions.LogInfo(
        msg='-----------{}/{}-----------'.format(_producer_idx + 1, len(_producer_nodes))
    )


def _producer_node_exited(event, context):
    """
    Launch new producer when current has finished.

    If transport is on, then stops rosbag2 recorder process. 

    Handles clearing of bags.
    """
    global _producer_idx, _producer_nodes, _rosbag_pid
    node_params = _producer_nodes[_producer_idx]['parameters']
    transport = node_params['transport']

    # Handle clearing bag files
    if not node_params['preserve_bags']:
        db_files = pathlib.Path.cwd().joinpath(node_params['db_folder']).glob('*.db3')
        for f in db_files:
            f.unlink()

    # If we have non empty rosbag PID, then we need to kill it (end-to-end transport case)
    if _rosbag_pid is not None and transport:
        os.kill(_rosbag_pid, signal.SIGINT)
        _rosbag_pid = None

    # Shutdown benchmark with error if producer node crashes
    if event.returncode != 0:
        return [
            launch.actions.LogInfo(msg='Writer error. Shutting down benchmark.'),
            launch.actions.EmitEvent(
                event=launch.events.Shutdown(
                    reason='Writer error'
                )
            )
        ]

    # Bump up producer index, so the launch sequence can continue
    _producer_idx += 1
    return [
        launch.actions.LogInfo(
            msg='---------------------------'
        ),
        _launch_sequence(transport=transport)
    ]


def generate_launch_description():
    """Generate launch description for ros2 launch system."""
    global _producer_nodes, _bench_cfg_path, _producers_cfg_path
    _bench_cfg_path, _producers_cfg_path = _parse_arguments()

    # Parse yaml config for benchmark
    bench_cfg = None
    with open(_bench_cfg_path, 'r') as config_file:
        bench_cfg_yaml = yaml.load(config_file, Loader=yaml.FullLoader)
        bench_cfg = (bench_cfg_yaml['rosbag2_performance_benchmarking']
                                   ['benchmark_node']
                                   ['ros__parameters'])

    # Benchmark options
    benchmark_params = bench_cfg['benchmark']

    repeat_each = benchmark_params.get('repeat_each')
    db_root_folder = benchmark_params.get('db_root_folder')
    summary_result_file = benchmark_params.get('summary_result_file')
    transport = not benchmark_params.get('no_transport')
    preserve_bags = benchmark_params.get('preserve_bags')

    # Producers options
    producers_params = bench_cfg['benchmark']['parameters']

    max_cache_size_params = producers_params.get('max_cache_size')
    max_bag_size_params = producers_params.get('max_bag_size')
    compression_params = producers_params.get('compression')
    compression_queue_size_params = producers_params.get('compression_queue_size')
    compression_threads_params = producers_params.get('compression_threads')
    storage_config_file_params = producers_params.get('storage_config_file')

    # Parameters cross section for whole benchmark
    # Parameters cross section is a list of all possible parameters variants
    params_cross_section = []

    # Generate unique benchmark directory name
    benchmark_cfg_name = pathlib.Path(_bench_cfg_path).name.replace('.yaml', '')
    producer_cfg_name = pathlib.Path(_producers_cfg_path).name.replace('.yaml', '')
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    transport_postfix = 'transport' if transport else 'no_transport'
    benchmark_dir_name = benchmark_cfg_name + \
        '_' + producer_cfg_name + \
        '_' + transport_postfix + \
        '_' + timestamp

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
            benchmark_dir_name,
            summary_result_file
        )

        # Database folder path for producer
        db_folder = pathlib.Path(db_root_folder).joinpath(
            benchmark_dir_name,
            node_title
        )

        # Filling up parameters cross section list for benchmark
        params_cross_section.append(
            {
                'node_title': node_title,
                'db_folder': str(db_folder),
                'cache': cache,
                'preserve_bags': preserve_bags,
                'transport': transport,
                'result_file': str(result_file),
                'compression_format': compression,
                'compression_queue_size': compression_queue_size,
                'compression_threads': compression_threads,
                'storage_config_file': str(storage_conf_path),
                'config_file': str(_producers_cfg_path),
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

    # Create all required nodes and processes for benchmark
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

        if not transport:
            # Writer benchmark node writes messages directly to a storage, uses no publishers
            producer_node = launch_ros.actions.Node(
                package='rosbag2_performance_benchmarking',
                executable='writer_benchmark',
                name='rosbag2_performance_benchmarking_node',
                parameters=parameters
            )
        else:
            # Benchmark publishers node uses standard publishers for publishing messages
            producer_node = launch_ros.actions.Node(
                package='rosbag2_performance_benchmarking',
                executable='benchmark_publishers',
                name='rosbag2_performance_benchmarking_node',
                parameters=parameters
            )

            # ROS2 bag process for recording messages
            rosbag_process = launch.actions.ExecuteProcess(
                sigkill_timeout=launch.substitutions.LaunchConfiguration(
                    'sigkill_timeout', default=60),
                sigterm_timeout=launch.substitutions.LaunchConfiguration(
                    'sigterm_timeout', default=60),
                cmd=['ros2', 'bag', 'record', '-a'] +
                    ['-o', str(producer_param['db_folder'])]
            )

            # Result writer node walks through output metadata files and generates
            # output results file
            result_writer = launch_ros.actions.Node(
                package='rosbag2_performance_benchmarking',
                executable='results_writer',
                name='rosbag2_performance_benchmarking_node',
                parameters=parameters
            )

            # Fill up list with rosbag record process and result writers actions
            _rosbag_processes.append(rosbag_process)
            _result_writers.append(result_writer)

        # Fill up dict with producer nodes and their corresponding parameters
        _producer_nodes.append({'node': producer_node, 'parameters': producer_param})

    # Connect start and exit events for a proper sequence
    if not transport:
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
    else:
        for producer_node, rosbag_proc in zip(_producer_nodes, _rosbag_processes):
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
            ),
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessStart(
                        target_action=rosbag_proc,
                        on_start=_rosbag_proc_started
                    )
                )
            )
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessIO(
                        target_action=rosbag_proc,
                        on_stdout=_rosbag_ready_check,
                        on_stderr=_rosbag_ready_check
                    )
                )
            )
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessExit(
                        target_action=rosbag_proc,
                        on_exit=_rosbag_proc_exited
                    )
                )
            )

    # Launch nodes one after another. Next node is launched after previous is finished.
    ld.add_action(_launch_sequence(transport=transport))

    return ld


if __name__ == '__main__':
    raise RuntimeError('Benchmark launchfile does not support standalone execution.')
