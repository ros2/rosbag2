import sys
import pathlib
import yaml
import time
import launch
from typing import List
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

BATCH_CONFIG = None
WRITER_INDEX = 0
WRITER_NODES = []


def parse_batch_config(args=sys.argv[4:]):
    """Parse batch benchmark config file."""
    print(args)
    if len(args) != 1:
        raise RuntimeError('Multiple launch arguments detected.')
    else:
        config_path = pathlib.Path(args[0].replace('config:=', ''))
        if not config_path.is_file():
            raise RuntimeError('Config file {} does not exist.'.format(config_path))
        with open(config_path, 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.FullLoader)
            return config['rosbag2_performance_benchmarking']['benchmark_node']['ros__parameters']


def launch_writers():
    """Launch writer."""
    global WRITER_INDEX, WRITER_NODES
    WRITER_INDEX += 1
    if WRITER_INDEX - 1 == len(WRITER_NODES):
        return launch.actions.LogInfo(msg='Benchmark finished!')
    return WRITER_NODES[WRITER_INDEX-1]['node']


def writer_node_exited(event, context):
    """Launch new writer when previously has finished."""
    global WRITER_INDEX, WRITER_NODES
    node_params = WRITER_NODES[WRITER_INDEX-1]['parameters']
    print(node_params)

    if event.returncode != 0:
        return launch.actions.EmitEvent(event=launch.events.Shutdown(
            reason="Writer error"
            ))

    # Handle clearing bag files
    if not node_params['preserve_bags']:
        db_files = pathlib.Path.cwd().joinpath(node_params['db_folder']).glob('*.db3')
        for f in db_files:
            f.unlink()

    return [launch.actions.LogInfo(msg='<<<<<<<<<<<<<<<<'), launch_writers()]

def generate_launch_description():
    """Generate launch description for ros2 launch system."""
    global WRITER_INDEX, WRITER_NODES, BATCH_CONFIG

    BATCH_CONFIG = parse_batch_config()
    
    # Batch benchmark options
    repeat_each = BATCH_CONFIG['benchmark']['repeat_each']
    db_root_folder = BATCH_CONFIG['benchmark']['db_root_folder']
    summary_result_file = BATCH_CONFIG['benchmark']['summary_result_file']
    no_transport = BATCH_CONFIG['benchmark']['no_transport']
    preserve_bags = BATCH_CONFIG['benchmark']['preserve_bags']
    
    # Producers options
    batch_params = BATCH_CONFIG['benchmark']['parameters']
    max_cache_size_params = batch_params.get('max_cache_size', None)
    compression_params = batch_params.get('compression', None)
    storage_config_file_params = batch_params.get('storage_config_file', None)
    instances_params = batch_params.get('instances', None)
    size_params = batch_params.get('size', None)
    max_count_params = batch_params.get('max_count', [10])

    # Params crosssection dir
    params_crosssection = []
    for i in range(0, repeat_each):
        for instances in instances_params:
            for cache in max_cache_size_params:
                for size in size_params:
                    for compression in compression_params:
                        for max_count in max_count_params:
                            for storage_config in storage_config_file_params:

                                storage_conf_path = pathlib.Path(
                                    get_package_share_directory('rosbag2_performance_benchmarking')
                                ).joinpath('config', 'storage', storage_config)
                                if not storage_conf_path.exists():
                                    raise RuntimeError(
                                        "Config {} does not exist.".format(storage_config))
                                
                                node_name = 'writer_benchmark_' + \
                                    'c{cache}_i{inst}_s{size}_mc{mc}_{comp}_{opt}'.format(
                                    cache=cache,
                                    inst=instances,
                                    size=size,
                                    mc=max_count,
                                    comp=compression if compression else "default",
                                    opt=pathlib.Path(storage_config).with_suffix('')
                                )

                                result_file = str(
                                    pathlib.Path(db_root_folder).joinpath(
                                        pathlib.Path(storage_config).with_suffix(''),
                                        compression if compression else "default",
                                        'results.csv'
                                    )
                                )

                                db_folder = pathlib.Path(db_root_folder).joinpath(
                                    pathlib.Path(storage_config).with_suffix(''),
                                    compression if compression else "default",
                                    node_name
                                )

                                params_crosssection.append(
                                    {
                                        'node_name': node_name,
                                        'db_folder': str(db_folder),
                                        'cache': cache,
                                        'size': size,
                                        'max_count': max_count,
                                        'instances': instances,
                                        'preserve_bags': preserve_bags,
                                        'no_transport': no_transport,
                                        'result_file': result_file,
                                        'compression_format': compression,
                                        'storage_config_file': str(storage_conf_path)
                                    }
                                )

    ld = LaunchDescription()
    ld.add_action(
        launch.actions.LogInfo(msg='Launching benchmark!'),
    )

    for parameters in params_crosssection:
        writer_node = Node(
            package='rosbag2_performance_benchmarking',
            namespace='writer_benchmark',
            executable='writer_benchmark',
            name=parameters['node_name'],
            parameters=[
                {'frequency': 100},
                {'max_count': parameters['max_count']},
                {'size': parameters['size']},
                {'instances': parameters['instances']},
                {'max_cache_size': parameters['cache']},
                {'max_bag_size': 0},
                {'db_folder': parameters['db_folder']},
                {'results_file': parameters['result_file']},
                {'storage_config_file': parameters['storage_config_file']},
                {'compression_format': parameters['compression_format']},
                {'compression_queue_size': 1},
                {'compression_threads': 0}
            ]
        )
        WRITER_NODES.append({'node': writer_node, 'parameters': parameters})

    for writer_node in WRITER_NODES:
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=writer_node['node'],
                    on_exit=writer_node_exited
                )
            )
        )

    ld.add_action(launch_writers())

    return ld

def main(argv=sys.argv[1:]):
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description())
    return ls.run()

if __name__ == '__main__':
    main()