# Copyright 2020, Robotec.ai sp. z o.o.
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

"""Module for launching a rosbag2 benchmark."""

import pathlib
import sys
from typing import cast


from launch import LaunchDescription
import launch.actions
import launch.events
import launch.logging
import launch.substitutions
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml

VERBOSE = True
logger = launch.logging.get_logger('BENCHMARK')


def get_worker(
    worker_executable,
    benchmark_path=None,
    name=None,
    topic=None,
    max_count=100,
    frequency=100,
    delay=1000,
    size=1000,
    instances=1,
    same_topic=True
):
    """Instantiate worker launch action."""
    available_workers = [
        'image_worker',
        'pointcloud2_worker',
        'bytearray_worker'
    ]

    if worker_executable not in available_workers:
        raise RuntimeError(
            "Worker '{}' is not supported. Available: {}"
            .format(available_workers)
        )
    if not name:
        raise RuntimeError('You must set an unique worker name.')
    if not topic:
        raise RuntimeError('You must set an unique worker topic.')
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError('Invalid report dir.')

    if not isinstance(max_count, int) or not isinstance(frequency, int) or \
       not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError('Invalid worker parameters.')

    worker_launch_info = {
        'name': name,
        'topic': topic,
        'max_count': max_count,
        'frequency': frequency,
        'delay': delay,
        'size': size,
        'same_topic': same_topic
        }

    with open(pathlib.Path(benchmark_path).joinpath(
                '{}-bytearray-worker.yaml'.format(name)
            ), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)

    all_topics = []
    if same_topic:
        all_topics = [topic for i in range(0, instances)]
    else:
        all_topics = [topic + str(i) for i in range(0, instances)]

    message_type = ''
    logger.info('Worker executable: ' + worker_executable)
    if worker_executable == 'image_worker':
        message_type = 'sensor_msgs/msg/Image'
    elif worker_executable == 'pointcloud2_worker':
        message_type = 'sensor_msgs/msg/PointCloud2'
    else:
        message_type = 'std_msgs/msg/ByteMultiArray'

    all_types = [message_type] * instances
    logger.info(
        'Creating bytearray worker - name: {},  topic: {}, \
        max_count: {}, frequency: {}, size: {}'
        .format(name, all_topics, max_count, frequency, size))
    return all_topics, all_types, Node(
            package='rosbag2_performance_workers',
            executable=worker_executable,
            name=name,
            parameters=[
                {
                    'max_count': max_count,
                    'frequency': frequency,
                    'size': size,
                    'delay': delay,
                    'benchmark_path': str(benchmark_path),
                    'instances': instances,
                    'topic': topic,
                    'same_topic': same_topic
                }],
        ), name


def get_dummy_publisher(topics, types):
    """Instantiate dummy publisher action for warming up topics."""
    if len(topics) != len(types):
        raise RuntimeError('Number of topics must match number od types')

    return Node(
            package='rosbag2_benchmarking',
            executable='dummy_publishers',
            name='dummy_publishers',
            parameters=[{'topics': topics, 'types': types}],
        )


def get_report_generator(benchmark_path=None):
    """Instantiate generator node action."""
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError('Invalid report dir.')

    return Node(
            package='rosbag2_benchmarking',
            executable='report_gen',
            name='report_generator',
            parameters=[{'benchmark_path': str(benchmark_path)}]
        )


def get_system_monitor(benchmark_path=None, frequency=10):
    """Instantiate system monitor action."""
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError('Invalid report dir.')

    return Node(
            package='rosbag2_benchmarking',
            executable='system_monitor',
            name='system_monitor',
            parameters=[{
                'benchmark_path': str(benchmark_path),
                'frequency': frequency
            }]
        )


def parse_workers(config, benchmark_path):
    """Parse workers and check topic and name conflicts."""
    worker_topics = []
    worker_types = []
    workers_to_run = []

    worker_topic_check = {}
    names_check = []

    for worker in config['workers']:
        worker_type = list(worker)[0]
        kwargs = {'benchmark_path': str(benchmark_path)}
        if worker[worker_type].get('name') is not None:
            kwargs.update({'name': worker[worker_type]['name']})
        if worker[worker_type].get('topic') is not None:
            kwargs.update({'topic': worker[worker_type]['topic']})
        if worker[worker_type].get('delay') is not None:
            kwargs.update({'delay': worker[worker_type]['delay']})
        if worker[worker_type].get('max_count') is not None:
            kwargs.update({'max_count': worker[worker_type]['max_count']})
        if worker[worker_type].get('size') is not None:
            kwargs.update({'size': worker[worker_type]['size']})
        if worker[worker_type].get('frequency') is not None:
            kwargs.update({'frequency': worker[worker_type]['frequency']})
        if worker[worker_type].get('same_topic') is not None:
            kwargs.update({'same_topic': worker[worker_type]['same_topic']})
        if worker[worker_type].get('instances') is not None:
            kwargs.update({'instances': worker[worker_type]['instances']})
        topics, type_, node, name = get_worker(
            **kwargs,
            worker_executable=worker_type + '_worker'
        )
        if name in names_check:
            logger.error(
                "Multiple workers with same name '{}'. Exiting.".format(name)
            )
            sys.exit(0)
        names_check.append(name)
        for topic in topics:
            if worker_topic_check.get(topic):
                if len(worker_topic_check.get(topic)) > 0 and \
                   name not in worker_topic_check.get(topic, []):
                    logger.error(
                        "Multiple workers on same topic '{}'. Exiting."
                        .format(topic)
                    )
                    sys.exit(0)
            worker_topic_check.update({topic: name})
        workers_to_run.append(node)
        worker_topics += (topics)
        worker_types += (type_)
    return worker_topics, worker_types, workers_to_run


def generate_launch_description():
    """Benchmark launch description."""
    # Register workers in a dict, False mean worker is not finished
    def worker_started(event, context):
        worker_hooks.update({event.pid: False})

    def monitor_exited(event, context):
        return launch.actions.EmitEvent(event=launch.events.Shutdown(
            reason='Monitor exited'
            ))

    def bag_exited(event, context):
        return launch.actions.EmitEvent(event=launch.events.Shutdown(
            reason='Bag exited'
            ))

    # Exits benchmark when all workers are finished
    def worker_exited(event, context):
        if event.returncode != 0:
            logger.error(
                'One of the workers returned error. Shutting down launch.'
            )
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason='Worker error'
                ))
        worker_hooks.update({event.pid: True})
        logger.info('Worker exited, pid: {}'.format(event.pid))
        if sum(worker_hooks.values()) == len(workers_to_run):
            logger.info('Benchmark done, shutting down.')
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason='Benchmark done'
                ))

    # Run dummy publishers to warm up topics when rosbag is ready and
    # listening for new topics
    def bag_warm_check(event):
        target_str = 'Listening for topics...'
        if target_str in event.text.decode():
            logger.info('Launching dummy pub')
            return get_dummy_publisher(worker_topics, worker_types)

    # Run workers when rosbag is warmed up with required topics
    def bag_run_check(event):
        target_str = \
            'All requested topics are subscribed. Stopping discovery...'
        if target_str in event.text.decode():
            logger.info('Launching workers')
            return workers_to_run

    # Log verbose output for workers
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(
                    launch.events.process.ProcessIO, event).process_name, line
                ))

    # Starts rosbag2 when system monitor is ready
    def monitor_check(event):
        target_str = 'Monitor ready.'
        # Monitor ready, start rosbag2
        if target_str in event.text.decode():
            return bag_worker

    ld = LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )

    # Manage config file
    config = None
    report_dir = pathlib.Path.cwd()
    for arg in sys.argv[4:]:
        if 'description:=' in arg:
            path = pathlib.Path(arg[len('description:='):])
            if path.is_file():
                with open(path) as config_file:
                    config = yaml.load(config_file)
            else:
                raise RuntimeError(
                    '{} is not correct yaml config file.'.format(path)
                )
        if 'report_dir:=' in arg:
            report_dir = pathlib.Path(arg[len('report_dir:='):]).expanduser()

    if config is None:
        raise RuntimeError("Missing 'description' parameter! \
            Use 'description:=config.yaml' parameter.")

    benchmark_path = pathlib.Path.joinpath(
        pathlib.Path(report_dir).expanduser(),
        pathlib.Path(
            str(config['benchmark']['id']) + '-' + config['benchmark']['tag'])
        )
    benchmark_path.mkdir(parents=True, exist_ok=True)

    # Prepare system usage monitor
    system_monitor = get_system_monitor(
        benchmark_path, config['benchmark'].get('monitor_frequency', 10)
    )
    ld.add_action(system_monitor)
    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=system_monitor,
            on_exit=monitor_exited
        )
    ))
    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=system_monitor,
            on_stdout=monitor_check,
            on_stderr=monitor_check
        )
    ))
    if VERBOSE:
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessIO(
                    target_action=system_monitor,
                    on_stdout=on_output,
                    on_stderr=on_output
                )
            )
        )

    # Setup workers
    worker_hooks = {}
    worker_topics, worker_types, workers_to_run = parse_workers(
        config,
        benchmark_path
    )

    # Prepare workers
    for worker in workers_to_run:
        ld.add_action(launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessStart(
                target_action=worker,
                on_start=worker_started
            )
        ))
        ld.add_action(launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=worker,
                on_exit=worker_exited
            )
        ))
        if VERBOSE:
            ld.add_action(launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessIO(
                    target_action=worker,
                    on_stdout=on_output,
                    on_stderr=on_output,
                )
            ))

    # Prepare rosbag
    reports_bag_location = pathlib.Path.joinpath(
        benchmark_path,
        pathlib.Path('bag')
    )
    if config['benchmark'].get('overwrite_existing'):
        import shutil
        shutil.rmtree(reports_bag_location, ignore_errors=True)

    args_list = [arg.split(' ') for arg in config['rosbag']['args']]
    parsed_args_list = [item for sublist in args_list for item in sublist]

    bag_worker = launch.actions.ExecuteProcess(
        sigkill_timeout=LaunchConfiguration(
            'sigkill_timeout', default=15),
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=15),
        cmd=['ros2', 'bag', 'record'] + list(set(worker_topics)) +
            ['-o', str(reports_bag_location)] + parsed_args_list
    )

    # Hook rosbag with readiness checks
    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=bag_worker,
            on_stdout=bag_run_check,
            on_stderr=bag_run_check,
        )
    ))
    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=bag_worker,
            on_stdout=bag_warm_check,
            on_stderr=bag_warm_check,
        )
    ))
    ld.add_action(launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=bag_worker,
            on_exit=bag_exited
        )
    ))

    return ld
