#!/usr/bin/env python3

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

"""Tool for generating human friendly benchmark report."""

import argparse
import csv
import pathlib
import statistics

import yaml


class Postprocess:
    """Base class for posprocess calculations."""

    def process(self, grouped_data, benchmark_config, producers_config):
        raise NotImplementedError


class PostprocessStorageConfig(Postprocess):
    """
    Postprocess.

    Calculate percent of recorded messages per storage config for different
    benchmark parameters.
    """

    def process(self, grouped_data, benchmark_config, producers_config):
        """
        Process grouped data and prints human friendly informations.

        :param: grouped data List of grouped results. Grouped result is a list with rows from
            single benchmark run (ie. run with two publisher groups returns two rows in results
            file).
        :param: benchmark_config Benchmark description from yaml config.
        :param: producers_config Producers description from yaml config.
        """
        benchmark_config_cleaned = (benchmark_config['rosbag2_performance_benchmarking']
                                                    ['benchmark_node']
                                                    ['ros__parameters'])
        benchmark_parameters = benchmark_config_cleaned['benchmark']['parameters']
        repeat_each = benchmark_config_cleaned['benchmark']['repeat_each']

        producers_config_publishers = (producers_config['rosbag2_performance_benchmarking_node']
                                                       ['ros__parameters']
                                                       ['publishers'])

        # Split data for storage configs
        splitted_data = {}
        for data in grouped_data:
            storage_cfg_name = data[0]['storage_config']
            storage_cfg_name = storage_cfg_name if storage_cfg_name != '' else 'default'
            if storage_cfg_name not in splitted_data.keys():
                splitted_data.update({storage_cfg_name: []})
            splitted_data[storage_cfg_name].append(data)

        cache_data_per_storage_conf = {}

        print(yaml.dump(producers_config_publishers))

        def __process_test(compression_selected,
                           compression_queue_size_selected,
                           compression_threads_selected,
                           max_bagfile_size_selected):
            for storage_cfg_name, data in splitted_data.items():
                cache_size_pub_groups_percentage = {}
                for pub_groups in data:
                    # Single sample contains multiple rows
                    if len(pub_groups) != len(producers_config_publishers['publisher_groups']):
                        raise RuntimeError('Invalid number of records in results detected.')

                    # These parameters are same for all rows in sample
                    # (multiple publishers in publisher group)
                    if pub_groups[0]['compression'] != compression_selected:
                        continue
                    if int(pub_groups[0]['compression_queue']) != compression_queue_size_selected:
                        continue
                    if int(pub_groups[0]['compression_threads']) != compression_threads_selected:
                        continue
                    if int(pub_groups[0]['max_bagfile_size']) != max_bagfile_size_selected:
                        continue

                    if pub_groups[0]['cache_size'] not in cache_size_pub_groups_percentage.keys():
                        cache_size_pub_groups_percentage.update({pub_groups[0]['cache_size']: []})

                    for pub_group in pub_groups:
                        if pub_group['cache_size'] not in cache_size_pub_groups_percentage.keys():
                            cache_size_pub_groups_percentage.update({pub_group['cache_size']: []})
                        total_produced = int(pub_group['total_produced'])
                        total_recorded = int(pub_group['total_recorded_count'])
                        cache_size_pub_groups_percentage[pub_group['cache_size']].append(
                            total_recorded / total_produced)

                cache_recorded_percentage_stats = {
                    cache: {
                        'avg': statistics.mean(pub_groups),
                        'min': min(pub_groups),
                        'max': max(pub_groups)
                    }
                    for cache, pub_groups in cache_size_pub_groups_percentage.items()
                }
                cache_data_per_storage_conf.update(
                    {storage_cfg_name: cache_recorded_percentage_stats}
                )

            result = {
                'repeat_each': repeat_each,
                'max_bagfile_size': max_bagfile_size_selected,
                'compression': compression_selected,
                'compression_threads': compression_threads_selected,
                'compression_queue_size': compression_queue_size_selected,
                'cache_data': cache_data_per_storage_conf
            }

            print('Results: ')
            print('\tRepetitions: {}'.format(result['repeat_each']))
            print('\tMax bagfile size: {}'.format(result['max_bagfile_size']))
            print('\tCompression: {}'.format(
                result['compression'] if result['compression'] else '<default>')
            )
            print('\tCompression threads: {}'.format(result['compression_threads']))
            print('\tCompression queue size: {}'.format(result['compression_queue_size']))
            print('\tRecorded messages for different caches and storage config:')
            for storage_cfg, caches in result['cache_data'].items():
                print('\t\tstorage config: {}:'.format(pathlib.Path(storage_cfg).name))
                for cache, percent_recorded in caches.items():
                    print('\t\t\tcache {:,} - min: {:.2%}, average: {:.2%}, max: {:.2%}'.format(
                        int(cache),
                        percent_recorded['min'],
                        percent_recorded['avg'],
                        percent_recorded['max']))
            print('======================== end of report ========================')

        [
            __process_test(
                compression_selected,
                compression_queue_size_selected,
                compression_threads_selected,
                max_bagfile_size_selected)
            for compression_selected in benchmark_parameters['compression']
            for compression_queue_size_selected in benchmark_parameters['compression_queue_size']
            for compression_threads_selected in benchmark_parameters['compression_threads']
            for max_bagfile_size_selected in benchmark_parameters['max_bag_size']
        ]


class Report:
    """Report generator main class."""

    def __init__(self, benchmark_dir):
        """Initialize with config and results data."""
        self.__benchmark_dir = benchmark_dir
        self.__load_configs()
        self.__load_results()

    def generate(self):
        """Handle data posprocesses."""
        psc = PostprocessStorageConfig()
        psc.process(
            self.__results_data,
            self.__benchmark_config,
            self.__producers_config
        )

    def __load_configs(self):
        producers_config_path = pathlib.Path(self.__benchmark_dir).joinpath('producers.yaml')
        benchmark_config_path = pathlib.Path(self.__benchmark_dir).joinpath('benchmark.yaml')

        with open(producers_config_path, 'r') as fp:
            self.__producers_config = yaml.load(fp, Loader=yaml.FullLoader)
        with open(benchmark_config_path, 'r') as fp:
            self.__benchmark_config = yaml.load(fp, Loader=yaml.FullLoader)

    def __load_results(self):
        results_path = pathlib.Path(self.__benchmark_dir).joinpath('results.csv')

        with open(results_path, mode='r') as fp:
            reader = csv.DictReader(fp, delimiter=' ')
            results = []

            for result in reader:
                results.append(result)

            publishers_groups = (
                self.__producers_config['rosbag2_performance_benchmarking_node']
                                       ['ros__parameters']
                                       ['publishers']
                                       ['publisher_groups'])

            publishers_groups_num = len(publishers_groups)

            # Group rows in results file, so that rows within same benchmark run are
            # in one list
            # Example: one benchmark run with two publisher groups returns two rows in results
            # file. We want to group these.
            results_grouped = [
                results[i:i+(publishers_groups_num)]
                for i in range(0, len(results), publishers_groups_num)
            ]

            self.__results_data = results_grouped


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', help='Benchmark results folder.')
    args = parser.parse_args()
    benchmark_dir = args.input

    if benchmark_dir:
        raport = Report(benchmark_dir)
        raport.generate()
    else:
        parser.print_help()
