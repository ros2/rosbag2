import subprocess
import psutil
import os.path
import time
import shutil
import signal
import yaml


def run_record_process(bagfile_name):
    record_process_handle = subprocess.Popen(['ros2', 'bag', 'record', '-a', '-o', bagfile_name], shell=False)
    time.sleep(2.0)  # give the process some time to start
    record_process = psutil.Process(record_process_handle.pid)
    resident_memory = []
    cpu_usage = []
    for i in range(0, 12):
        resident_memory.append(record_process.memory_info()[0])
        cpu_usage.append(record_process.cpu_percent(1.0))
        time.sleep(4.0)
    record_process_handle.send_signal(signal.SIGINT)
    time.sleep(2.0)  # need to wait to finish writing bagfile

    resident_memory_average = sum(resident_memory) / len(resident_memory)
    resident_memory_max = max(resident_memory)
    cpu_usage_average = sum(cpu_usage) / len(cpu_usage)
    cpu_usage_max = max(cpu_usage)
    print('Memory usage: maximally ' + str(resident_memory_max / 1000000) +
          'MB at an average of ' + str(resident_memory_average / 1000000) + 'MB')
    print('CPU usage (in % of one core): maximally ' + str(cpu_usage_max) +
          '% at an average of ' + str(cpu_usage_average) + '%')


def run_launch_file(name):
    return subprocess.Popen(['ros2', 'launch', 'rosbag2_performance_publishers', name], shell=False)


def read_expectations_file(test_name):
    with open(os.path.join(os.getcwd(), test_name + '_expected.yaml'), 'r') as stream:
        try:
            expectations = yaml.safe_load(stream)[test_name]
            print('Running test ' + test_name + expectations['description'] + '\n'
                  + 'Expected I/O-Load: '
                  + str(expectations['expected_size_in_mb']/expectations['expected_duration_in_sec']) + 'MB/sec\n'
                  + 'WARNING: Test will take ' + str(expectations['expected_duration_in_sec']) + 's and take '
                  + str(expectations['expected_size_in_mb']) + 'MB of disk space')
            time.sleep(2.0)
            return {topic['topic_metadata']['name']: int(topic['message_count'])
                    for topic in expectations['topics_with_expected_message_count']}
        except yaml.YAMLError as exc:
            print(exc)


def inspect_bagfile(bagfile_name):
    with open(os.path.join(os.getcwd(), bagfile_name, 'metadata.yaml'), 'r') as stream:
        try:
            topics = yaml.safe_load(stream)['rosbag2_bagfile_information']['topics_with_message_count']
            return {topic['topic_metadata']['name']: int(topic['message_count']) for topic in topics}
        except yaml.YAMLError as exc:
            print(exc)


def has_test_passed(actual_topics, expected_topics):
    if len(set(expected_topics) - set(actual_topics)) > 0 or len(set(actual_topics) - set(expected_topics)) > 0:
        print('Bag file does not contain all expected topics or additional topics are present.')
        return False
    topic_count_present = {topic: min(actual_topics[topic]/expected_topics[topic], 1)
                           for topic in actual_topics.keys()}
    for topic_name in topic_count_present.keys():
        if topic_count_present[topic_name] < .95:
            topic_count_strings = ['Topic ' + topic_name + ': ' + str(topic_count_present[topic_name] * 100)
                                   + ' % of messages received' for topic_name in topic_count_present.keys()]
            print(topic_count_strings)
            return False
    return True


def run_test(bagfile_name, launchfile_name):
    shutil.rmtree(bagfile_name)
    expected_topics = read_expectations_file('large_message_test')
    launch_process = run_launch_file(launchfile_name)
    run_record_process(bagfile_name)
    launch_process.send_signal(signal.SIGINT)
    actual_topics = inspect_bagfile(bagfile_name)
    print('Performance test passed' if has_test_passed(actual_topics, expected_topics) else 'Performance test failed')


run_test('performance_bag', 'large_message_launch.launch.py')
