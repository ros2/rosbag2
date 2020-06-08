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
import sys
import yaml
import pathlib
from typing import cast

import launch.logging
import launch.actions
import launch.events
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node

VERBOSE = True
logger = launch.logging.get_logger("BENCHMARK")

def get_image_worker(name=None, topic=None, max_count=100, dt=10, delay=1000, dimensions=32, benchmark_path=None):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(dt, int) or not isinstance(delay, int) or not isinstance(dimensions, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "dt": dt, "delay": delay, "dimensions": dimensions}
    with open(pathlib.Path(benchmark_path).joinpath("{}-image-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)
    
    logger.info("Creating image worker: {} {} {} {} {}".format(name, topic, max_count, dt, dimensions))
    return topic, "sensor_msgs/msg/Image", Node(
            package='rosbag2_performance_workers',
            executable='image_worker',
            name=name,
            parameters=[
                {
                    'max_count':max_count, 
                    'dt':dt, 
                    'dimensions':dimensions, 
                    'delay':delay,
                    'benchmark_path': benchmark_path
                }],
            remappings=[
                ('image', topic)
            ]
        )

def get_pointcloud_worker(name=None, topic=None, max_count=100, dt=10, delay=1000, size=10000, benchmark_path=None):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(dt, int) or not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "dt": dt, "delay": delay, "size": size}
    with open(pathlib.Path(benchmark_path).joinpath("{}-pointcloud2-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)

    logger.info("Creating pointcloud worker: {} {} {} {} {}".format(name, topic, max_count, dt, size))
    return topic, "sensor_msgs/msg/PointCloud2", Node(
            package='rosbag2_performance_workers',
            executable='pointcloud2_worker',
            name=name,
            parameters=[
                {
                    'max_count': max_count, 
                    'dt': dt, 
                    'size': size, 
                    'delay': delay,
                    'benchmark_path': str(benchmark_path)
                }],
            remappings=[
                ('pointcloud2', topic)
            ]
        )

def get_dummy_publisher(topics, types):
    if len(topics) != len(types):
        raise RuntimeError("Number of topics must match number od types")
    
    return Node(
            package='rosbag2_benchmarking',
            executable='dummy_publishers',
            name="dummy_publishers",
            parameters=[{'topics':topics, 'types':types}],
        )

def get_raport_generator(benchmark_path=None):
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    return Node(
            package='rosbag2_benchmarking',
            executable='raport_gen',
            name="raport_generator",
            parameters=[{'benchmark_path': str(benchmark_path)}]
        )

def get_system_monitor(benchmark_path=None):
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    return Node(
            package='rosbag2_benchmarking',
            executable='system_monitor',
            name="system_monitor",
            parameters=[{'benchmark_path': str(benchmark_path)}]
        )

def parse_workers(config, benchmark_path):
    worker_topics = []
    worker_types = []
    workers_to_run = []

    for worker in config["workers"]:
        if list(worker)[0] == "image":
            kwargs = {"benchmark_path": str(benchmark_path)}
            if worker["image"].get("name"):
                kwargs.update({"name":worker["image"]["name"]})
            if worker["image"].get("topic"):
                kwargs.update({"topic":worker["image"]["topic"]})
            if worker["image"].get("max_count"):
                kwargs.update({"max_count":worker["image"]["max_count"]})
            if worker["image"].get("dimensions"):
                kwargs.update({"dimensions":worker["image"]["dimensions"]})
            if worker["image"].get("dt"):
                kwargs.update({"dt":worker["image"]["dt"]})
            topic, type_, node = get_image_worker(**kwargs)
            workers_to_run.append(node)
            worker_topics.append(topic)
            worker_types.append(type_)
        elif list(worker)[0] == "pointcloud2":
            kwargs = {"benchmark_path": str(benchmark_path)}
            if worker["pointcloud2"].get("name"):
                kwargs.update({"name":worker["pointcloud2"]["name"]})
            if worker["pointcloud2"].get("topic"):
                kwargs.update({"topic":worker["pointcloud2"]["topic"]})
            if worker["pointcloud2"].get("max_count"):
                kwargs.update({"max_count":worker["pointcloud2"]["max_count"]})
            if worker["pointcloud2"].get("size"):
                kwargs.update({"size":worker["pointcloud2"]["size"]})
            if worker["pointcloud2"].get("dt"):
                kwargs.update({"dt":worker["pointcloud2"]["dt"]})
            topic, type_, node = get_pointcloud_worker(**kwargs)
            workers_to_run.append(node)
            worker_topics.append(topic)
            worker_types.append(type_)
        else:
            pass
    
    return worker_topics, worker_types, workers_to_run


def generate_launch_description():
    # Register workers in a dict, False mean worker is not finished
    def worker_started(event, context):
        worker_hooks.update({event.pid:False})

    # Exits benchmark when all workers are finished
    def worker_exited(event, context):
        worker_hooks.update({event.pid:True})
        logger.info("Worker exited, pid: {}".format(event.pid))
        if sum(worker_hooks.values()) == len(workers_to_run):
            logger.info("Benchmark done, shutting down.")
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="Benchmark done"
                ))

    # Run dummy publishers to warm up topics when rosbag is ready and listening for new topics
    def bag_warm_check(event):
        target_str = 'Listening for topics...'
        if target_str in event.text.decode():
            logger.info("Launching dummy pub")
            return get_dummy_publisher(worker_topics, worker_types)

    # Run workers when rosbag is warmed up with required topics
    def bag_run_check(event):
        target_str = 'All requested topics are subscribed. Stopping discovery...'
        if target_str in event.text.decode():
            logger.info("Launching workers")
            return workers_to_run

    # Log verbose output for workers
    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    ld = LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )
    
    # Manage config file
    config = None
    if "description" in sys.argv[4]:
        path = pathlib.Path(sys.argv[4][len("description:="):])
        if path.is_file():
            with open(path) as config_file:
                config = yaml.load(config_file)
        else:
            raise RuntimeError("{} is not correct yaml config file.".format(path))
    else:
        raise RuntimeError("Missing 'description' parameter! Use 'description:=config.yaml' parameter.")
    benchmark_path = pathlib.Path.joinpath(pathlib.Path(config["raport_dir"]).expanduser(), pathlib.Path(str(config["benchmark"]["id"]) + "-" + config["benchmark"]["tag"]))
    benchmark_path.mkdir(parents=True, exist_ok=True)

    # Setup system usage monitor
    system_monitor = get_system_monitor(benchmark_path)
    ld.add_action(system_monitor)
    if VERBOSE:
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
            target_action=system_monitor,
            on_stdout=on_output,
            on_stderr=on_output,
        )))

    # Setup workers
    worker_hooks = {}
    worker_topics, worker_types, workers_to_run = parse_workers(config, benchmark_path)
    logger.info(worker_topics)
    
    # Run rosbag
    raports_bag_location = pathlib.Path.joinpath(benchmark_path, pathlib.Path("bag"))
    if config["benchmark"].get("overwrite_existing"):
        import shutil
        shutil.rmtree(raports_bag_location, ignore_errors=True)
    bag_worker = launch.actions.ExecuteProcess(cmd=["ros2", 'bag', 'record'] + worker_topics + ["-o", str(raports_bag_location)] + ["-p", str(config["rosbag"]["polling_interval"])])

    # Run workers
    index = 0
    for worker in workers_to_run:
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessStart(target_action=worker, on_start=worker_started)))
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(target_action=worker, on_exit=worker_exited)))
        if VERBOSE:
            ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
                target_action=worker,
                on_stdout=on_output,
                on_stderr=on_output,
            )))

    # Hook rosbag with readiness checks
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=bag_worker,
        on_stdout=bag_run_check,
        on_stderr=bag_run_check,
    )))
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=bag_worker,
        on_stdout=bag_warm_check,
        on_stderr=bag_warm_check,
    )))
    ld.add_action(bag_worker)

    return ld

if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())