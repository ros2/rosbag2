# Copyright 2020, Robotec.ai sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os, pathlib, re
import matplotlib.pyplot as plt
import numpy as np
import sys
import argparse

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
filename = "record.1000.100.1000.txt"

parser = argparse.ArgumentParser()
parser.add_argument('-i', '--input')
args = parser.parse_args()
filename = args.input

print("processing file " + filename)

raport_path = pathlib.Path(path).joinpath(filename)

callback_times = []
begin_trans_times = []
commit_trans_times = []
insert_start_times = []
insert_end_times = []

with open(str(raport_path), "r") as raport1:
    first_line = raport1.readline()
    start_time = float(re.search('([0-9]*\.[0-9]*).*', first_line).group(1))
    print(start_time)

    lines = raport1.readlines()
    for line in [first_line] + lines:

        callback_match = re.search('([0-9]*\.[0-9]*).*\[Callback\]', line)
        if callback_match:
            callback_times.append(float(callback_match.group(1))-start_time)

        begin_trans_match = re.search('([0-9]*\.[0-9]*).*begin transaction', line)
        if begin_trans_match:
            begin_trans_times.append(float(begin_trans_match.group(1))-start_time)

        commit_trans_match = re.search('([0-9]*\.[0-9]*).*commit transaction', line)
        if commit_trans_match:
            commit_trans_times.append(float(commit_trans_match.group(1))-start_time)

        insert_start_match = re.search('([0-9]*\.[0-9]*).*Executing insert start', line)
        if insert_start_match:
            insert_start_times.append(float(insert_start_match.group(1))-start_time)

        insert_end_match = re.search('([0-9]*\.[0-9]*).*Executing insert end', line)
        if insert_end_match:
            insert_end_times.append(float(insert_end_match.group(1))-start_time)

dt_between_callbacks = []
last_callback_time = None
for callback_time in callback_times:
    if last_callback_time is None:
        last_callback_time = callback_time
    dt_between_callbacks.append(callback_time - last_callback_time)
    last_callback_time = callback_time

# print(callback_times[0], dt_between_callbacks[0])

callback_times_v = [1] * len(callback_times)
begin_trans_times_v = [3] * len(begin_trans_times)
commit_trans_times_v = [3] * len(commit_trans_times)
insert_start_times_v = [2] * len(insert_start_times)
insert_end_times_v = [2] * len(insert_end_times)

fig, ax = plt.subplots(figsize=(15, 6))
fig.suptitle(filename + " msg_count=" + str(len(callback_times)), fontsize=16)
if (len(begin_trans_times_v)>0):
    ax.stem(begin_trans_times, begin_trans_times_v, label='begin_transaction', markerfmt=' ', linefmt='r-', basefmt='k')
if (len(commit_trans_times_v)>0):
    ax.stem(commit_trans_times, commit_trans_times_v, label='commit_transaction', markerfmt=' ', linefmt='g-', basefmt='k')
if (len(insert_start_times)>0):
    ax.stem(insert_start_times, insert_start_times_v, label='insert_start', markerfmt=' ', linefmt='y-', basefmt='k')
if (len(insert_end_times)>0):
    ax.stem(insert_end_times, insert_end_times_v, label='insert_end', markerfmt=' ', linefmt='m-', basefmt='k')
if (len(callback_times)>0):
    ax.stem(callback_times, callback_times_v, label='callback', markerfmt=' ', linefmt='b-', basefmt='k')
plt.yticks([])

ax.legend(loc='upper right')
ax.grid(True)

plt.show()
