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

import argparse
import csv
import pathlib
import statistics

import yaml


class Postprocess:

    def process(self, grouped_data, benchmark_config, producers_config):
        raise NotImplementedError


class PostprocessStorageConfig(Postprocess):

    def process(self, grouped_data, benchmark_config, producers_config):
        benchmark_config_cleaned = (benchmark_config['rosbag2_performance_benchmarking']
                                                    ['benchmark_node']
                                                    ['ros__parameters'])
        benchmark_parameters = benchmark_config_cleaned['benchmark']['parameters']
        repeat_each = benchmark_config_cleaned['benchmark']['repeat_each']

        producers_config_publishers = (producers_config['rosbag2_performance_benchmarking_node']
                                                       ['ros__parameters']
                                                       ['publishers'])

        # Split for storage configs
        splitted_data = {}
        for data in grouped_data:
            storage_cfg_name = data[0]['storage_config']
            storage_cfg_name = storage_cfg_name if storage_cfg_name != '' else 'default'
            if storage_cfg_name not in splitted_data.keys():
                splitted_data.update({storage_cfg_name: []})
            splitted_data[storage_cfg_name].append(data)

        # {'storage_optimized': {10000: 78%, 100000: 55%, ...}, 'storage_resilient': ... }
        storage_pack = {}

        print(yaml.dump(producers_config_publishers))

        def __process_test(compression_selected,
                           compression_queue_size_selected,
                           compression_threads_selected,
                           max_bagfile_size_selected):
            for storage_cfg_name, data in splitted_data.items():
                # {10000: [sample0_rec_%, ..., sampleM_rec_%], 100000: [sampleN_rec_%..]}
                cache_samples = {}
                for sample in data:
                    # sample contains multiple rows
                    if len(sample) != len(producers_config_publishers['publisher_groups']):
                        raise RuntimeError('Invalid number of records in results detected.')

                    if sample[0]['compression'] != compression_selected:
                        continue
                    if int(sample[0]['compression_queue']) != compression_queue_size_selected:
                        continue
                    if int(sample[0]['compression_threads']) != compression_threads_selected:
                        continue
                    if int(sample[0]['max_bagfile_size']) != max_bagfile_size_selected:
                        continue

                    # process only default compression and 0 max_bag_filesize
                    if sample[0]['cache_size'] not in cache_samples.keys():
                        cache_samples.update({sample[0]['cache_size']: []})

                    # TODO(piotr.jaroszek) WARNING, currently results returns 'total_produced'
                    # correctly (per publisher group), but 'total_recorded' is already summed for
                    # all the publisher groups!
                    sample_total_produced = 0
                    for row in sample:
                        sample_total_produced += int(row['total_produced'])
                    cache_samples[sample[0]['cache_size']].append(
                        int(sample[0]['total_recorded_count'])/sample_total_produced)

                # {10000: 78%, 100000: 55%}
                cache_avg_recorded_percentage = {
                    cache: statistics.mean(samples)
                    for cache, samples in cache_samples.items()
                }
                storage_pack.update({storage_cfg_name: cache_avg_recorded_percentage})

            result = {
                'repeat_each': repeat_each,
                'max_bagfile_size': max_bagfile_size_selected,
                'compression': compression_selected,
                'compression_threads': compression_threads_selected,
                'compression_queue_size': compression_queue_size_selected,
                'cache_data': storage_pack
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
                    print('\t\t\tcache [bytes]: {:,}: {:.2%} recorded'.format(
                        int(cache),
                        percent_recorded))

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

    def __init__(self, benchmark_dir):
        self.__benchmark_dir = benchmark_dir
        self.__load_configs()
        self.__load_results()

    def generate(self):
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
