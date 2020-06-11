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

def get_image_worker(name=None, topic=None, max_count=100, frequency=100, delay=1000, dimensions=32, benchmark_path=None, instance=0, same_topic=True):
    name += "_{}".format(instance)
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(frequency, int) or not isinstance(delay, int) or not isinstance(dimensions, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "frequency": frequency, "delay": delay, "dimensions": dimensions}
    with open(pathlib.Path(benchmark_path).joinpath("{}-image-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)
    
    worker_topic = topic if same_topic else topic + str(instance)
    logger.info("Creating image worker - name: {},  topic: {}, max_count: {}, frequency: {}, size: {}".format(name, worker_topic, max_count, frequency, dimensions))
    return worker_topic, "sensor_msgs/msg/Image", Node(
            package='rosbag2_performance_workers',
            executable='image_worker',
            name=name,
            parameters=[
                {
                    'max_count':max_count, 
                    'frequency':frequency, 
                    'dimensions':dimensions, 
                    'delay':delay,
                    'benchmark_path': benchmark_path
                }],
            remappings=[
                ('image', worker_topic)
            ]
        ), name

def get_pointcloud_worker(name=None, topic=None, max_count=100, frequency=100, delay=1000, size=10000, benchmark_path=None, instance=0, same_topic=True):
    name += "_{}".format(instance)
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(frequency, int) or not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "frequency": frequency, "delay": delay, "size": size}
    with open(pathlib.Path(benchmark_path).joinpath("{}-pointcloud2-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)

    worker_topic = topic if same_topic else topic + str(instance)
    logger.info("Creating pointcloud worker - name: {},  topic: {}, max_count: {}, frequency: {}, size: {}".format(name, worker_topic, max_count, frequency, size))
    return worker_topic, "sensor_msgs/msg/PointCloud2", Node(
            package='rosbag2_performance_workers',
            executable='pointcloud2_worker',
            name=name,
            parameters=[
                {
                    'max_count': max_count, 
                    'frequency': frequency, 
                    'size': size, 
                    'delay': delay,
                    'benchmark_path': str(benchmark_path)
                }],
            remappings=[
                ('pointcloud2', worker_topic)
            ]
        ), name

def get_bytearray_worker(name=None, topic=None, max_count=100, frequency=100, delay=1000, size=10000, benchmark_path=None, instances=0, same_topic=True):
    # name += "_{}".format(instance)
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(frequency, int) or not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "frequency": frequency, "delay": delay, "size": size, "same_topic": same_topic}
    with open(pathlib.Path(benchmark_path).joinpath("{}-bytearray-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)

    all_topics = [topic + str(i) for i in range(0, instances)]
    all_types = ["sensor_msgs/msg/ByteMultiArray"] * instances
    logger.info("Creating bytearray worker - name: {},  topic: {}, max_count: {}, frequency: {}, size: {}".format(name, all_topics, max_count, frequency, size))
    return all_topics, all_types, Node(
            package='rosbag2_performance_workers',
            executable='bytearray_worker',
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

def get_worker(
    worker_executable,
    benchmark_path=None,
    name=None,
    topic=None,
    max_count=100,
    frequency=100,
    delay=1000,
    size=10000,
    instances=0,
    same_topic=True
):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    if not isinstance(max_count, int) or not isinstance(frequency, int) or not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError("Invalid worker parameters.")

    worker_launch_info = {"name": name, "topic": topic, "max_count": max_count, "frequency": frequency, "delay": delay, "size": size, "same_topic": same_topic}
    with open(pathlib.Path(benchmark_path).joinpath("{}-bytearray-worker.yaml".format(name)), 'w') as f:
        yaml.dump(worker_launch_info, f, default_flow_style=False)

    all_topics = [topic + str(i) for i in range(0, instances)]
    all_types = ["sensor_msgs/msg/ByteMultiArray"] * instances
    logger.info("Creating bytearray worker - name: {},  topic: {}, max_count: {}, frequency: {}, size: {}".format(name, all_topics, max_count, frequency, size))
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

def get_system_monitor(benchmark_path=None, frequency=10):
    if not pathlib.Path(benchmark_path).is_dir():
        raise RuntimeError("Invalid raport dir.")

    return Node(
            package='rosbag2_benchmarking',
            executable='system_monitor',
            name="system_monitor",
            parameters=[{'benchmark_path': str(benchmark_path), 'frequency':frequency}]
        )

def parse_workers(config, benchmark_path):
    worker_topics = []
    worker_types = []
    workers_to_run = []

    worker_topic_check = {}

    for worker in config["workers"]:
        if list(worker)[0] == "image":
            kwargs = {"benchmark_path": str(benchmark_path)}
            if worker["image"].get("name") is not None:
                kwargs.update({"name":worker["image"]["name"]})
            if worker["image"].get("topic") is not None:
                kwargs.update({"topic":worker["image"]["topic"]})
            if worker["image"].get("delay") is not None:
                kwargs.update({"delay":worker["image"]["delay"]})
            if worker["image"].get("max_count") is not None:
                kwargs.update({"max_count":worker["image"]["max_count"]})
            if worker["image"].get("dimensions") is not None:
                kwargs.update({"dimensions":worker["image"]["dimensions"]})
            if worker["image"].get("frequency") is not None:
                kwargs.update({"frequency":worker["image"]["frequency"]})
            if worker["image"].get("same_topic") is not None:
                kwargs.update({"same_topic":worker["image"]["same_topic"]})
            workers_in_boundle = [worker["image"].get("name")+"_{}".format(i) for i in range(0,worker["image"].get("instances", 1))]
            for instance in range(0, worker["image"].get("instances", 1)):
                topic, type_, node, name = get_image_worker(**kwargs, instance=instance)
                if worker_topic_check.get(topic):
                    if len(worker_topic_check.get(topic)) > 0 and name not in worker_topic_check.get(topic):
                        logger.error("Multiple workers on same topic {}. Exiting.".format(topic))
                        sys.exit(0)
                worker_topic_check.update({topic: workers_in_boundle})
                workers_to_run.append(node)
                worker_topics.append(topic)
                worker_types.append(type_)
        elif list(worker)[0] == "pointcloud2":
            kwargs = {"benchmark_path": str(benchmark_path)}
            if worker["pointcloud2"].get("name") is not None:
                kwargs.update({"name":worker["pointcloud2"]["name"]})
            if worker["pointcloud2"].get("topic") is not None:
                kwargs.update({"topic":worker["pointcloud2"]["topic"]})
            if worker["pointcloud2"].get("delay") is not None:
                kwargs.update({"delay":worker["pointcloud2"]["delay"]})
            if worker["pointcloud2"].get("max_count") is not None:
                kwargs.update({"max_count":worker["pointcloud2"]["max_count"]})
            if worker["pointcloud2"].get("size") is not None:
                kwargs.update({"size":worker["pointcloud2"]["size"]})
            if worker["pointcloud2"].get("frequency") is not None:
                kwargs.update({"frequency":worker["pointcloud2"]["frequency"]})
            if worker["pointcloud2"].get("same_topic") is not None:
                kwargs.update({"same_topic":worker["pointcloud2"]["same_topic"]})
            workers_in_boundle = [worker["pointcloud2"].get("name")+"_{}".format(i) for i in range(0,worker["pointcloud2"].get("instances", 1))]
            for instance in range(0, worker["pointcloud2"].get("instances", 1)):
                topic, type_, node, name = get_pointcloud_worker(**kwargs, instance=instance)
                if worker_topic_check.get(topic):
                    if len(worker_topic_check.get(topic)) > 0 and name not in worker_topic_check.get(topic):
                        logger.error("Multiple workers on same topic {}. Exiting.".format(topic))
                        sys.exit(0)
                worker_topic_check.update({topic: workers_in_boundle})
                workers_to_run.append(node)
                worker_topics.append(topic)
                worker_types.append(type_)
        elif list(worker)[0] == "bytearray":
            kwargs = {"benchmark_path": str(benchmark_path)}
            if worker["bytearray"].get("name") is not None:
                kwargs.update({"name":worker["bytearray"]["name"]})
            if worker["bytearray"].get("topic") is not None:
                kwargs.update({"topic":worker["bytearray"]["topic"]})
            if worker["bytearray"].get("delay") is not None:
                kwargs.update({"delay":worker["bytearray"]["delay"]})
            if worker["bytearray"].get("max_count") is not None:
                kwargs.update({"max_count":worker["bytearray"]["max_count"]})
            if worker["bytearray"].get("size") is not None:
                kwargs.update({"size":worker["bytearray"]["size"]})
            if worker["bytearray"].get("frequency") is not None:
                kwargs.update({"frequency":worker["bytearray"]["frequency"]})
            if worker["bytearray"].get("same_topic") is not None:
                kwargs.update({"same_topic":worker["bytearray"]["same_topic"]})
            workers_in_boundle = [worker["bytearray"].get("name")+"_{}".format(i) for i in range(0,worker["bytearray"].get("instances", 1))]
            # for instance in range(0, worker["bytearray"].get("instances", 1)):
            topics, type_, node, name = get_worker(**kwargs, worker_executable="bytearray_worker", instances=worker["bytearray"].get("instances", 1))
            for topic in topics:
                if worker_topic_check.get(topic):
                    if len(worker_topic_check.get(topic)) > 0 and name not in worker_topic_check.get(topic, []):
                        logger.error("Multiple workers on same topic {}. Exiting.".format(topic))
                        sys.exit(0)
            worker_topic_check.update({topic: workers_in_boundle})
            workers_to_run.append(node)
            worker_topics += (topics)
            worker_types += (type_)
            pass

    return worker_topics, worker_types, workers_to_run


def generate_launch_description():
    print(sys.argv)
    
    # Register workers in a dict, False mean worker is not finished
    def worker_started(event, context):
        worker_hooks.update({event.pid:False})

    def monitor_exited(event, context):
        return launch.actions.EmitEvent(event=launch.events.Shutdown(
            reason="Monitor exited"
            ))

    # Exits benchmark when all workers are finished
    def worker_exited(event, context):
        if event.returncode != 0:
            logger.error("One of the workers returned error. Shutting down launch.")
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="Worker error"
                ))
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

    # Prepare system usage monitor
    system_monitor = get_system_monitor(benchmark_path, config["benchmark"].get("monitor_frequency",10))
    ld.add_action(system_monitor)
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(target_action=system_monitor, on_exit=monitor_exited)))
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=system_monitor,
        on_stdout=monitor_check,
        on_stderr=monitor_check
    )))
    if VERBOSE:
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
            target_action=system_monitor,
            on_stdout=on_output,
            on_stderr=on_output,
        )))

    # Setup workers
    worker_hooks = {}
    worker_topics, worker_types, workers_to_run = parse_workers(config, benchmark_path)
    # logger.info("Worker all topics: {}".format(worker_topics))
    
    # Prepare workers
    for worker in workers_to_run:
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessStart(target_action=worker, on_start=worker_started)))
        ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(target_action=worker, on_exit=worker_exited)))
        if VERBOSE:
            ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
                target_action=worker,
                on_stdout=on_output,
                on_stderr=on_output,
            )))
            
    # Prepare rosbag
    raports_bag_location = pathlib.Path.joinpath(benchmark_path, pathlib.Path("bag"))
    if config["benchmark"].get("overwrite_existing"):
        import shutil
        shutil.rmtree(raports_bag_location, ignore_errors=True)

    args_list = [arg.split(" ") for arg in config["rosbag"]["args"]]
    parsed_args_list = [item for sublist in args_list for item in sublist]

    bag_worker = launch.actions.ExecuteProcess(cmd= ["ros2", 'bag', 'record'] + \
        list(set(worker_topics)) + \
        ["-o", str(raports_bag_location)] + \
        parsed_args_list)
    
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

    return ld

if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
