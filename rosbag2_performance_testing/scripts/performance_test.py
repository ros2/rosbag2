# Copyright 2019-2020, Martin Idel
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

from subprocess import Popen
from psutil import Process
from os.path import join, isdir
from os import getcwd
from time import sleep
from shutil import rmtree
from signal import SIGINT
from yaml import safe_load, YAMLError
from typing import Dict

from ament_index_python import get_package_prefix

# Time monitoring = sleep_between_reading * number_of_readings
sleep_between_reading = 5.0
cpu_averaging_time = 1.0  # must be smaller than sleep_between_reading
number_of_readings = 12


def run_record_process(bagfile_name: str):
    record_process_handle = Popen(['ros2', 'bag', 'record', '-a', '-o', bagfile_name], shell=False)
    sleep(2.0)  # give the process some time to start
    record_process = Process(record_process_handle.pid)
    resident_memory = []
    cpu_usage = []
    for i in range(0, number_of_readings):
        resident_memory.append(record_process.memory_info()[0])
        cpu_usage.append(record_process.cpu_percent(cpu_averaging_time))
        sleep(sleep_between_reading - cpu_averaging_time)
    record_process_handle.send_signal(SIGINT)
    sleep(2.0)  # need to wait to finish writing bagfile

    resident_memory_average = sum(resident_memory) / len(resident_memory) / 1000000
    resident_memory_max = max(resident_memory) / 1000000
    cpu_usage_average = sum(cpu_usage) / len(cpu_usage)
    cpu_usage_max = max(cpu_usage)
    print(f'Memory usage: maximally {resident_memory_max} MB at an average of {resident_memory_average} MB')
    print(f'CPU usage (in % of one core): maximally {cpu_usage_max}% at an average of {cpu_usage_average}%')


def read_expectations_file(test_name) -> Dict[str, int]:
    package_dir = get_package_prefix('rosbag2_performance_testing')
    expected_dir = join(package_dir, 'share', 'rosbag2_performance_testing', 'expected')
    with open(join(expected_dir, test_name + '_expected.yaml'), 'r') as stream:
        try:
            expectations = safe_load(stream)[test_name]
            print('Running test ' + test_name + ': ' + expectations['description'] + '\n'
                  + 'Expected I/O-Load: '
                  + str(expectations['expected_size_in_mb']/expectations['expected_duration_in_sec']) + 'MB/sec\n'
                  + 'WARNING: Test will take ' + str(expectations['expected_duration_in_sec']) + 's and take '
                  + str(expectations['expected_size_in_mb']) + 'MB of disk space')
            sleep(2.0)
            return {topic['topic_metadata']['name']: int(topic['message_count'])
                    for topic in expectations['topics_with_expected_message_count']}
        except YAMLError as exc:
            print(exc)


def inspect_bagfile(bagfile_name) -> Dict[str, int]:
    with open(join(getcwd(), bagfile_name, 'metadata.yaml'), 'r') as stream:
        try:
            topics = safe_load(stream)['rosbag2_bagfile_information']['topics_with_message_count']
            return {topic['topic_metadata']['name']: int(topic['message_count']) for topic in topics}
        except YAMLError as exc:
            print(exc)


def has_test_passed(actual_topics: Dict[str, int], expected_topics: Dict[str, int]) -> bool:
    if not set(expected_topics).issubset(set(actual_topics)):
        print('Bag file does not contain all expected topics.')
        return False
    topic_count_present = {topic: min(actual_topics[topic]/expected_topics[topic], 1)
                           for topic in expected_topics.keys()}
    for topic_name in topic_count_present.keys():
        if topic_count_present[topic_name] < .95:
            topic_count_strings = ['Topic ' + topic_name + ': ' + str(topic_count_present[topic_name] * 100)
                                   + ' % of messages received' for topic_name in topic_count_present.keys()]
            print(topic_count_strings)
            return False
    return True


def run_test(bagfile_name: str):
    if isdir(join(getcwd(), bagfile_name)):
        rmtree(join(getcwd(), bagfile_name))
    expected_topics = read_expectations_file('large_message_test')
    run_record_process(bagfile_name)
    actual_topics = inspect_bagfile(bagfile_name)
    filtered_actual_topics = {topic: count for topic, count in actual_topics.items() if count > 0}
    failed_passed = 'passed' if has_test_passed(filtered_actual_topics, expected_topics) else 'failed'
    print(f'Performance test {failed_passed}')


if __name__ == "__main__":
    run_test('performance_bag')
