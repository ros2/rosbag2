import os
import platform
import sys
import yaml
from typing import cast

import pathlib

from launch import LaunchDescription
import launch.logging
import launch.actions  # noqa: E402
import launch.events  # noqa: E402
import launch.substitutions  # noqa: E402
from launch_ros.actions import Node

VERBOSE = True
CONTEXT = None

logger = launch.logging.get_logger("BENCHMARK")

def get_image_worker(name=None, topic=None, max_count=100, dt=10, delay=1000, dimensions=32):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")

    if not isinstance(max_count, int) or not isinstance(dt, int) or not isinstance(delay, int) or not isinstance(dimensions, int):
        raise RuntimeError("Invalid worker parameters.")
    
    logger.info("Creating image worker: {} {} {} {} {}".format(name, topic, max_count, dt, dimensions))
    return topic, "sensor_msgs/msg/Image", Node(
            package='rosbag2_performance_workers',
            executable='image_worker',
            name=name,
            parameters=[{'max_count':max_count, 'dt':dt, 'dimensions':dimensions, 'delay':delay}],
            remappings=[
                ('image', topic)
            ]
        )

def get_pointcloud_worker(name=None, topic=None, max_count=100, dt=10, delay=1000, size=10000):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker topic.")

    if not isinstance(max_count, int) or not isinstance(dt, int) or not isinstance(delay, int) or not isinstance(size, int):
        raise RuntimeError("Invalid worker parameters.")

    logger.info("Creating pointcloud worker: {} {} {} {} {}".format(name, topic, max_count, dt, size))
    return topic, "sensor_msgs/msg/PointCloud2", Node(
            package='rosbag2_performance_workers',
            executable='pointcloud2_worker',
            name=name,
            parameters=[{'max_count':max_count, 'dt':dt, 'size':size, 'delay':delay}],
            remappings=[
                ('image', topic)
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

def get_raport_generator():
    return Node(
            package='rosbag2_benchmarking',
            executable='raport_gen',
            name="raport_generator",
        )

def parse_workers(config):
    worker_topics = []
    worker_types = []
    workers_to_run = []

    for worker in config["workers"]:
        if list(worker)[0] == "image":
            kwargs = {}
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
        elif list(worker)[0] == "pointcloud":
            pass
        else:
            pass
    
    return worker_topics, worker_types, workers_to_run


def generate_launch_description():

    ld = LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )


    config = None
    if "description" in sys.argv[4]:
        path = pathlib.Path(sys.argv[4][len("description:="):])
        if path.is_file():
            with open(path) as config_file:
                config = yaml.load(config_file)
        else:
            raise RuntimeError("{} is not correct yaml config file.".format(path))
        print(pathlib.Path.cwd(), path)
    else:
        raise RuntimeError("Missing 'description' parameter! Use 'description:=config.yaml' parameter.")

    # return ld

    def on_output(event: launch.Event) -> None:
        for line in event.text.decode().splitlines():
            print('[{}] {}'.format(
                cast(launch.events.process.ProcessIO, event).process_name, line))

    def raport_finished(event, context):
        logger.info("Benchmark done, shutting down.")
        return launch.actions.EmitEvent(event=launch.events.Shutdown(
            reason="Benchmark done"
            ))
    raport_generator = get_raport_generator()
    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(target_action=raport_generator, on_exit=raport_finished)))

    # (piotr.jaroszek) TODO: launch something to track system usage here
    worker_hooks = {}
    worker_topics = []
    worker_types = []
    workers_to_run = []
    worker_topics, worker_types, workers_to_run = parse_workers(config)

    ## (piotr.jaroszek) TODO: move workers setup to parameters file
    # topic, type_, node = get_image_worker(name="cam1", topic="image_cam1", max_count=1000)
    # workers_to_run.append(node)
    # worker_topics.append(topic)
    # worker_types.append(type_)

    # topic, type_, node = get_image_worker(name="cam2", topic="image_cam2", max_count=1000)
    # workers_to_run.append(node)
    # worker_topics.append(topic)
    # worker_types.append(type_)
    ## ENDTODO

    logger.info(worker_topics)
        
    raports_bag_location = pathlib.Path.joinpath(pathlib.Path(config["raport_dir"]).expanduser(), pathlib.Path(str(config["benchmark"]["id"]) + "-" + config["benchmark"]["tag"]), pathlib.Path("bag"))
    if config.get("overwrite_existing"):
        import shutil
    bag_worker = launch.actions.ExecuteProcess(cmd=["ros2", 'bag', 'record'] + worker_topics + ["-o", str(raports_bag_location)] + ["-p", "10"])

    # Register workers in a dict, False mean worker is not finished
    def worker_started(event, context):
        worker_hooks.update({event.pid:False})

    # Exits benchmark when all workers are finished
    def worker_exited(event, context):
        worker_hooks.update({event.pid:True})
        logger.info("Worker exited, pid: {}".format(event.pid))
        if sum(worker_hooks.values()) == len(workers_to_run):
            logger.info("All workers done, launching raport generator")
            return raport_generator

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
    CONTEXT = ls.context
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())