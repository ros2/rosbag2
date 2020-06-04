import os
import platform
import sys
from typing import cast

from launch import LaunchDescription
import launch.actions  # noqa: E402
import launch.events  # noqa: E402
import launch.substitutions  # noqa: E402
from launch_ros.actions import Node

VERBOSE = False

def get_image_worker(name=None, topic=None, max_count=100, dt=10, dimensions=32):
    if not name:
        raise RuntimeError("You must set an unique worker name.")
    if not topic:
        raise RuntimeError("You must set an unique worker out topic.")

    return topic, Node(
            package='rosbag2_performance_workers',
            executable='image_worker',
            name=name,
            parameters=[{'max_count':max_count, 'dt':dt, 'dimensions':dimensions}],
            remappings=[
                ('image', topic)
            ]
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
    
    worker_hooks = {}
    worker_topics = []
    workers_to_run = []

    ## (piotr.jaroszek) TODO: move workers setup to parameters file
    topic, node = get_image_worker(name="cam1", topic="image_cam1", max_count=100)
    workers_to_run.append(node)
    worker_topics.append(topic)

    topic, node = get_image_worker(name="cam2", topic="image_cam2", max_count=100)
    workers_to_run.append(node)
    worker_topics.append(topic)
    ## ENDTODO

    print(worker_topics)
    bag_worker = launch.actions.ExecuteProcess(cmd=["ros2", 'bag', 'record'] + worker_topics + ["-p 10"])

    def worker_started(event, context):
        worker_hooks.update({event.pid:False})

    def worker_exited(event, context):
        worker_hooks.update({event.pid:True})
        print("Worker exited, pid: {}".format(event.pid))
        if sum(worker_hooks.values()) == len(workers_to_run):
            print("All workers done, shutting down")
            return launch.actions.EmitEvent(event=launch.events.Shutdown(
                    reason="Benchmark done"
                ))

    def bag_run_check(event):
        # (piotr.jaroszek) maybe add some dummy worker to warm up topics, so the subscription would be already ready when the 'main' worker kicks in?
        target_str = 'Listening for topics...'
        # target_str = 'All requested topics are subscribed. Stopping discovery...'
        if target_str in event.text.decode():
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
        # dummy workers
        # dummy = launch.actions.ExecuteProcess(cmd=["ros2", 'topic', 'pub'] + [worker_topics[index]] + ["sensor_msgs/msg/Image", "-1"])
        # ld.add_action(dummy)
        # index += 1
        # dummy = Node(
        #     package='rosbag2_performance_workers',
        #     executable='dummy_image_pub',
        #     name="dumm"
        # )
        # ld.add_action(dummy)

    ld.add_action(launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessIO(
        target_action=bag_worker,
        on_stdout=bag_run_check,
        on_stderr=bag_run_check,
    )))
    ld.add_action(bag_worker)

    return ld