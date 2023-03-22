# Copyright 2021 Apex.AI, Inc.
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

import csv
import fnmatch
import os
import pathlib
import statistics
import unittest

import ament_index_python

import launch
import launch.actions
import launch_testing

import yaml

# Postprocess and PostprocessStorageConfig classes have been adapted from report_gen.py script
# to make easier results processing


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
                cache_samples = {}
                for sample in data:
                    # Single sample contains multiple rows
                    if len(sample) != len(producers_config_publishers['publisher_groups']):
                        raise RuntimeError('Invalid number of records in results detected.')

                    # These parameters are same for all rows in sample
                    # (multiple publishers in publisher group)
                    if sample[0]['compression'] != compression_selected:
                        continue
                    if int(sample[0]['compression_queue']) != compression_queue_size_selected:
                        continue
                    if int(sample[0]['compression_threads']) != compression_threads_selected:
                        continue
                    if int(sample[0]['max_bagfile_size']) != max_bagfile_size_selected:
                        continue

                    if sample[0]['cache_size'] not in cache_samples.keys():
                        cache_samples.update({sample[0]['cache_size']: []})

                    # TODO(piotr.jaroszek) WARNING, currently results in 'total_produced' column
                    # are correct (per publisher group), but 'total_recorded' is already summed
                    # for all the publisher groups!
                    sample_total_produced = 0
                    for row in sample:
                        sample_total_produced += int(row['total_produced'])
                    cache_samples[sample[0]['cache_size']].append(
                        int(sample[0]['total_recorded_count']) / sample_total_produced)

                cache_recorded_percentage_stats = {
                    cache: {
                        'avg': statistics.mean(samples),
                        'min': min(samples),
                        'max': max(samples)
                    }
                    for cache, samples in cache_samples.items()
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
            return result

        results = [
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
        return results


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
        results = psc.process(
            self.__results_data,
            self.__benchmark_config,
            self.__producers_config
        )
        return results

    def __load_configs(self):
        producers_config_path = pathlib.Path(self.__benchmark_dir).joinpath('producers.yaml')
        benchmark_config_path = pathlib.Path(self.__benchmark_dir).joinpath('benchmark.yaml')

        with open(producers_config_path, 'r') as fp:
            self.__producers_config = yaml.safe_load(fp)
        with open(benchmark_config_path, 'r') as fp:
            self.__benchmark_config = yaml.safe_load(fp)

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
                results[i:i + publishers_groups_num]
                for i in range(0, len(results), publishers_groups_num)
            ]

            self.__results_data = results_grouped


def find_directory(pattern, path):
    """Find all directories paths matching a pattern."""
    result = []
    for root, dirs, files in os.walk(path):
        for name in dirs:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result


def check_benchmark_result(result, expectation):
    """Check if the benchmark results passes the expected target."""
    for storage_cfg, caches in result['cache_data'].items():
        for cache, percent_recorded in caches.items():
            if ((percent_recorded['min'] < expectation['min'])
                    or (percent_recorded['avg'] < expectation['avg'])
                    or (percent_recorded['max'] < expectation['max'])):
                return False
    return True


class BenchmarkResults:
    """Rosbag2 benchmark results data."""

    def __init__(self, benchmark, producer):
        self.benchmark = benchmark
        self.producer = producer
        self.directory = ''
        self.expectation = {'avg': 0.0, 'min': 0.0, 'max': 0.0}
        self.reports = {}
        self.test_passed = False

    def get_error_message(self):
        """Return an informative error message."""
        msg = 'Test case: ' + self.benchmark + '.yaml + ' + self.producer + '.yaml failed. '
        info = ''
        if not self.directory:
            info = 'No results directory found. The benchmark probably failed'
        elif not self.reports:
            info = 'No results report generated. The results output format is probably wrong'
        elif not self.test_passed:
            info = 'Recorded messages percentage is lower than expected '
        return msg + info


class BenchmarkResultsChecker:
    """Rosbag2 benchmark results checker."""

    def __init__(self, root_folder, benchmarks, producers):
        self.root_folder = root_folder
        self.results = {}
        for benchmark in benchmarks:
            for producer in producers:
                self.results[(benchmark, producer)] = BenchmarkResults(benchmark, producer)

    def find_benchmark_directories(self):
        """
        Return all the benchmark directories in the root folder cases.

        If multiple directories are found only the most recent one is returned.
        """
        for (benchmark, producer), result in self.results.items():
            pattern = benchmark + '_' + producer + '*'
            files = find_directory(pattern, self.root_folder)
            if files:
                # add just the latest one
                sorted_files = sorted(files)
                result.directory = sorted_files[-1]
            else:
                print('No benchmark directories found in ' + self.root_folder)

    def load_results(self):
        """Process the benchmark results and generates a report."""
        self.find_benchmark_directories()
        for (benchmark, producer), result in self.results.items():
            print('read_results')
            if not result.directory:
                print('No results found for ' + benchmark + ' ' + producer)
            else:
                print('Generate report for: ' + result.directory)
                report = Report(result.directory)
                result.reports = report.generate()

    def check_expectations(self):
        """Check the generated report against the expected results and stores the result."""
        self.load_results()

        print(self.results.items())
        for (benchmark, producer), result in self.results.items():
            if not result.reports:
                print('No results found for ' + benchmark + ' ' + producer)
                result.test_passed = False
            else:
                for report in result.reports:
                    if check_benchmark_result(report, result.expectation):
                        print('Test passed: ' + result.directory)
                        result.test_passed = True
                    else:
                        print('Test failed: ' + result.directory)
                        result.test_passed = False

    def set_expectations(self, benchmark, producer, expectation):
        self.results[(benchmark, producer)].expectation = expectation


def generate_test_description():
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    config_path = os.path.join(
        ament_index_python.get_package_share_directory('rosbag2_performance_benchmarking'),
        'config'
    )

    benchmark_path = os.path.join(config_path, 'benchmarks/test')
    producers_path = os.path.join(config_path, 'producers')

    bag_root_folder = '/tmpfs/rosbag2_performance_test'
    benchmarks = [
        'test_no_transport',
        'test_benchmark_producers'
    ]
    producers = [
        '100MBs_raw',
        'automotive',
    ]

    checker = BenchmarkResultsChecker(bag_root_folder, benchmarks, producers)

    for benchmark in benchmarks:
        for producer in producers:
            checker.set_expectations(benchmark, producer, {'avg': 0.95, 'min': 0.95, 'max': 0.95})

    # Create benchmarks launch actions list
    benchmark_processes = []
    for benchmark in benchmarks:
        for producer in producers:
            benchmark_yaml = benchmark + '.yaml'
            producer_yaml = producer + '.yaml'
            args = []
            args += ['benchmark:=' + str(os.path.join(benchmark_path, benchmark_yaml))]
            args += ['producers:=' + str(os.path.join(producers_path, producer_yaml))]

            process = launch.actions.ExecuteProcess(
                cmd=['ros2', 'launch', 'rosbag2_performance_benchmarking',
                     'benchmark_launch.py'] + args,
                env=proc_env,
                output='screen'
            )
            benchmark_processes.append(process)

    # Configure the benchmarks to be launched sequentially
    ld = launch.LaunchDescription()
    ld.add_action(benchmark_processes[0])
    for action, next_action in zip(benchmark_processes, benchmark_processes[1:]):
        ld.add_action(
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=action,
                    on_exit=next_action
                )
            )
        )

    ld.add_action(launch_testing.util.KeepAliveProc())
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {
        'last_benchmark': benchmark_processes[-1],
        'checker': checker,
    }


class TestBenchmarkProcessStops(unittest.TestCase):

    def test_proc_terminates(self, last_benchmark):
        # Wait for the last benchmark to complete before processing the results
        self.proc_info.assertWaitForShutdown(process=last_benchmark, timeout=600)


@launch_testing.post_shutdown_test()
class TestBenchmarkResults(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_benchmark_results(self, checker):
        checker.check_expectations()
        for result in checker.results.values():
            self.assertTrue(result.test_passed, result.get_error_message())
