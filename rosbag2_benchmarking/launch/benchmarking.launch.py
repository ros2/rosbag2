import os
import platform
import sys
from typing import cast

from launch import LaunchDescription
import launch.logging
import launch.actions  # noqa: E402
import launch.events  # noqa: E402
import launch.substitutions  # noqa: E402
from launch_ros.actions import Node

VERBOSE = False

logger = launch.logging.get_logger("BENCHMARK")

def get_image_worker(name=None, topic=None, max_count=100, dt=10, dimensions=32):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker out topic.")

    return topic, "sensor_msgs/msg/Image", Node(
            package='rosbag2_performance_workers',
            executable='image_worker',
            name=name,
            parameters=[{'max_count':max_count, 'dt':dt, 'dimensions':dimensions}],
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

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )

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

    ## (piotr.jaroszek) TODO: move workers setup to parameters file
    topic, type_, node = get_image_worker(name="cam1", topic="image_cam1", max_count=1000)
    workers_to_run.append(node)
    worker_topics.append(topic)
    worker_types.append(type_)

    topic, type_, node = get_image_worker(name="cam2", topic="image_cam2", max_count=1000)
    workers_to_run.append(node)
    worker_topics.append(topic)
    worker_types.append(type_)
    ## ENDTODO

    logger.info(worker_topics)
    bag_worker = launch.actions.ExecuteProcess(cmd=["ros2", 'bag', 'record'] + worker_topics + ["-p 100"])

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