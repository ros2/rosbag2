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

from ament_index_python import get_package_share_directory
import launch
import launch_ros
import psutil
from rosbag2_py import get_default_storage_id

import yaml

_bench_cfg_path = None
_producers_cfg_path = None

_producer_idx = 0
_producer_nodes = []

_rosbag_processes = []
_rosbag_pid = None
_producer_pid = None
_rosbag_process = None
_producer_process = None

_parameters = []

_cpu_usage_per_core = []
_producer_cpu_usage = 0.0
_recorder_cpu_usage = 0.0

_producer_cpu_affinity = []
_recorder_cpu_affinity = []


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
    benchmark_path = pathlib.Path(_producer_nodes[0]['parameters']['bag_folder'])
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
    global _rosbag_pid, _rosbag_process, _recorder_cpu_affinity
    _rosbag_pid = event.pid
    _rosbag_process = psutil.Process(_rosbag_pid)
    if len(_recorder_cpu_affinity) > 0:
        _rosbag_process.cpu_affinity(_recorder_cpu_affinity)


def _rosbag_ready_check(event):
    """
    Consider rosbag2 ready when 'Listening for topics...' string is printed.

    Launches producer node if ready.
    """
    target_str = 'Listening for topics...'
    if target_str in event.text.decode():
        global _recorder_cpu_usage, _rosbag_process
        _rosbag_process.cpu_percent()
        _recorder_cpu_usage = 0.0
        return _launch_sequence(transport=False)


def _rosbag_proc_exited(event, context):
    """Check if rosbag2 record process finished correctly."""
    global _rosbag_pid, _rosbag_process

    _rosbag_pid = None
    _rosbag_process = None

    # ROS2 bag returns 0 if terminated with SIGINT, which we expect here
    if event.returncode != 0:
        return [
            launch.actions.LogInfo(msg='Rosbag2 record error. Shutting down benchmark. '
                                       'Return code = ' + str(event.returncode)),
            launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Rosbag2 record error'))
        ]


def _results_writer_exited(event, context):
    """Start next rosbag2 record process or launch new producer on exit."""
    global _producer_idx, _producer_nodes

    node_params = _producer_nodes[_producer_idx]['parameters']
    transport = node_params['transport']

    if event.returncode != 0:
        return [
            launch.actions.LogInfo(msg='Results writer error. Shutting down benchmark. '
                                       'Return code = ' + str(event.returncode)),
            launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Results writer error'))
        ]

    # Bump up producer index, so the launch sequence can continue
    _producer_idx += 1

    return [
        launch.actions.LogInfo(msg='---------------------------'),
        _launch_sequence(transport=transport)
    ]


def _producer_node_started(event, context):
    """Log current benchmark progress on producer start."""
    global _producer_idx, _producer_pid, _producer_process, _producer_cpu_usage
    global _cpu_usage_per_core, _producer_cpu_affinity
    _producer_pid = event.pid
    _producer_process = psutil.Process(_producer_pid)
    if len(_producer_cpu_affinity) > 0:
        _producer_process.cpu_affinity(_producer_cpu_affinity)
    _producer_process.cpu_percent()
    _producer_cpu_usage = 0.0
    psutil.cpu_percent(None, True)
    _cpu_usage_per_core = []
    return launch.actions.LogInfo(
        msg='-----------{}/{}-----------'.format(_producer_idx + 1, len(_producer_nodes))
    )


def _producer_node_finished_check(event):
    """
    Consider producer node finished when 'Producer threads finished' string is printed.

    Measure producer node CPU load
    """
    target_str = 'Producer threads finished'
    if target_str in event.text.decode():
        global _producer_cpu_usage, _producer_process
        try:
            _producer_cpu_usage = _producer_process.cpu_percent()
        except psutil.NoSuchProcess as e:
            return [
                launch.actions.LogInfo(msg="Warning! Can't measure producer node CPU load "
                                           f"Producer's process ({e.pid}) doesn't exist"),
            ]


def _producer_node_exited(event, context):
    """
    Launch result writer on exit.

    If transport is on, then stops rosbag2 recorder process.

    Handles clearing of bags.
    """
    global _producer_idx, _producer_nodes, _rosbag_pid, _recorder_cpu_usage, _rosbag_process
    global _producer_cpu_usage, _cpu_usage_per_core
    parameters = _parameters[_producer_idx]
    node_params = _producer_nodes[_producer_idx]['parameters']
    transport = node_params['transport']
    _cpu_usage_per_core = psutil.cpu_percent(None, True)

    # If we have non empty rosbag PID, then we need to kill it (end-to-end transport case)
    if _rosbag_pid is not None and transport:
        # Check if rosbag process still alive
        if not psutil.pid_exists(_rosbag_pid):
            return [
                launch.actions.LogInfo(msg="Rosbag2 process doesn't exist. "
                                           'Shutting down benchmark.'),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown(reason="Rosbag2 process doesn't exist")
                )
            ]
        _recorder_cpu_usage = _rosbag_process.cpu_percent()
        os.kill(_rosbag_pid, signal.SIGINT)
        # Wait for rosbag2 process to exit for 10 seconds
        rosbag_return_code = _rosbag_process.wait(10)
        _rosbag_pid = None
        if rosbag_return_code is not None and rosbag_return_code != 0:
            return [
                launch.actions.LogInfo(msg='Rosbag2 record error. Shutting down benchmark. '
                                           'Return code = ' + str(rosbag_return_code)),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown(reason='Rosbag2 record error')
                )
            ]

    # Check if recorded bag files exists
    storage_id = get_default_storage_id()
    bag_files = []
    if node_params['storage_id'] != '':
        storage_id = node_params['storage_id']
    if storage_id == 'sqlite3' or storage_id == 'mcap':
        file_ext_mask = '*.mcap' if storage_id == 'mcap' else '*.db3'
        bag_files = pathlib.Path.cwd().joinpath(node_params['bag_folder']).glob(file_ext_mask)
        #  Raise error if bag_files is empty.
        if not bag_files:
            return [
                launch.actions.LogInfo(msg='Error! Rosbag2 files not found. '
                                           'Shutting down benchmark.'),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown(reason='Rosbag2 files not found.')
                )
            ]
    else:
        return [
            launch.actions.LogInfo(msg=f'Unsupported storage_id = {storage_id}'
                                       'Shutting down benchmark.'),
            launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason=f'Unsupported storage_id = {storage_id}')
            )
        ]

    # Handle clearing bag files
    if not node_params['preserve_bags']:
        stats_path = pathlib.Path.cwd().joinpath(node_params['bag_folder'], 'bagfiles_info.yaml')
        stats = {
            'total_size': 0,
            'bagfiles': []
        }

        # Delete rosbag files
        for f in bag_files:
            filesize = f.stat().st_size
            f.unlink()
            stats['bagfiles'].append({f.name: {'size': filesize}})
            stats['total_size'] += filesize

        # Dump files size information
        with open(stats_path, 'w') as stats_file:
            yaml.dump(stats, stats_file)

    # Shutdown benchmark with error if producer node crashes
    if event.returncode != 0:
        return [
            launch.actions.LogInfo(msg='Writer error. Shutting down benchmark.'),
            launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Writer error'))
        ]

    # Add cpu load as parameters
    parameters.append({'cpu_usage_per_core': _cpu_usage_per_core})
    if transport:
        parameters.append({'producer_cpu_usage': _producer_cpu_usage})
        parameters.append({'recorder_cpu_usage': _recorder_cpu_usage})
    else:
        # If no transport, recorder will be part of the producer. Swap them out to avoid confusion
        parameters.append({'producer_cpu_usage': _recorder_cpu_usage})
        parameters.append({'recorder_cpu_usage': _producer_cpu_usage})

    # Result writer node walks through output metadata files and generates
    # output results file
    result_writer = launch_ros.actions.Node(
        package='rosbag2_performance_benchmarking',
        executable='results_writer',
        name='rosbag2_performance_benchmarking_node',
        parameters=parameters
    )

    return [
        result_writer,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=result_writer,
                on_exit=_results_writer_exited
            )
        )
    ]


def generate_launch_description():
    """Generate launch description for ros2 launch system."""
    global _producer_nodes, _bench_cfg_path, _producers_cfg_path
    global _producer_cpu_affinity, _recorder_cpu_affinity
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
    bag_root_folder = benchmark_params.get('bag_root_folder')
    summary_result_file = benchmark_params.get('summary_result_file')
    transport = not benchmark_params.get('no_transport')
    preserve_bags = benchmark_params.get('preserve_bags')

    # CPU affinity for producers and recorder
    _producer_cpu_affinity = benchmark_params.get('producers_cpu_affinity', [])
    _recorder_cpu_affinity = benchmark_params.get('recorder_cpu_affinity', [])
    if not transport:
        _producer_cpu_affinity = _recorder_cpu_affinity
        print('Warning! With no_transport = True, producer_cpu_affinity will be ignored')

    if len(_producer_cpu_affinity) > 0:
        # Set CPU affinity for current process to avoid impact on the recorder
        psutil.Process().cpu_affinity(_producer_cpu_affinity)

    # Producers options
    producers_params = bench_cfg['benchmark']['parameters']

    storage_id = producers_params.get('storage_id', [''])
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
                                           max_bag_size,
                                           storage):
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
            '{i}_{storage}_{cache}_{comp}_{comp_q}_{comp_t}_{st_conf}_{bag_size}'.format(
                i=i,
                storage=storage,
                cache=cache,
                comp=compression if compression else 'default_compression',
                comp_q=compression_queue_size,
                comp_t=compression_threads,
                st_conf=st_conf_filename if st_conf_filename else 'default_config',
                bag_size=max_bag_size
            )

        # Result file path for producer
        result_file = pathlib.Path(bag_root_folder).joinpath(
            benchmark_dir_name,
            summary_result_file
        )

        # Bag folder path for producer
        bag_folder = pathlib.Path(bag_root_folder).joinpath(
            benchmark_dir_name,
            node_title
        )

        # Filling up parameters cross section list for benchmark
        params_cross_section.append(
            {
                'node_title': node_title,
                'bag_folder': str(bag_folder),
                'cache': cache,
                'preserve_bags': preserve_bags,
                'transport': transport,
                'result_file': str(result_file),
                'compression_format': compression,
                'compression_queue_size': compression_queue_size,
                'compression_threads': compression_threads,
                'storage_config_file': str(storage_conf_path),
                'config_file': str(_producers_cfg_path),
                'max_bag_size': max_bag_size,
                'storage_id': str(storage)
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
            max_bag_size,
            storage)
        for i in range(0, repeat_each)
        for cache in max_cache_size_params
        for compression in compression_params
        for compression_queue_size in compression_queue_size_params
        for compression_threads in compression_threads_params
        for storage_config in storage_config_file_params
        for max_bag_size in max_bag_size_params
        for storage in storage_id
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
            {'bag_folder': producer_param['bag_folder']},
            {'results_file': producer_param['result_file']},
            {'compression_queue_size': producer_param['compression_queue_size']},
            {'compression_threads': producer_param['compression_threads']}
        ]

        if producer_param['storage_id'] != '':
            parameters.append({'storage_id': producer_param['storage_id']})
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
            rosbag_args = []
            if producer_param['storage_id']:
                rosbag_args += [
                    '-s',
                    str(producer_param['storage_id'])
                ]
            if producer_param['storage_config_file']:
                rosbag_args += [
                    '--storage-config-file',
                    str(producer_param['storage_config_file'])
                ]
            if producer_param['cache']:
                rosbag_args += [
                    '--max-cache-size',
                    str(producer_param['cache'])
                ]
            if producer_param['compression_format']:
                rosbag_args += [
                    '--compression-mode',
                    'message'
                ]
                rosbag_args += [
                    '--compression-format',
                    str(producer_param['compression_format'])
                ]
            if producer_param['compression_queue_size']:
                rosbag_args += [
                    '--compression-queue-size',
                    str(producer_param['compression_queue_size'])
                ]
            if producer_param['compression_threads']:
                rosbag_args += [
                    '--compression-threads',
                    str(producer_param['compression_threads'])
                ]
            if producer_param['max_bag_size']:
                rosbag_args += [
                    '-b',
                    str(producer_param['max_bag_size'])
                ]
            rosbag_args += ['-o', str(producer_param['bag_folder'])]
            rosbag_process = launch.actions.ExecuteProcess(
                sigkill_timeout=launch.substitutions.LaunchConfiguration(
                    'sigkill_timeout', default=60),
                sigterm_timeout=launch.substitutions.LaunchConfiguration(
                    'sigterm_timeout', default=60),
                cmd=['ros2', 'bag', 'record', '-e',
                     r'\/.*_benchmarking_node\/.*'] + rosbag_args
            )

            # Fill up list with rosbag record process and result writers actions
            _rosbag_processes.append(rosbag_process)

        # Fill up dict with producer nodes and their corresponding parameters
        _producer_nodes.append({'node': producer_node, 'parameters': producer_param})
        # Fill up list with parameters
        _parameters.append(parameters)

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
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessIO(
                        target_action=producer_node['node'],
                        on_stdout=_producer_node_finished_check,
                        on_stderr=_producer_node_finished_check
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
            )
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessIO(
                        target_action=producer_node['node'],
                        on_stdout=_producer_node_finished_check,
                        on_stderr=_producer_node_finished_check
                    )
                )
            )
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
