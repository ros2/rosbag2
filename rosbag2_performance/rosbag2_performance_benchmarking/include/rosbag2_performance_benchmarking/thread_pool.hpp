// Copyright 2022 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBAG2_PERFORMANCE_BENCHMARKING__THREAD_POOL_HPP_
#define ROSBAG2_PERFORMANCE_BENCHMARKING__THREAD_POOL_HPP_

#include <mutex>
#include <queue>
#include <vector>
#include <thread>
#include <functional>
#include <condition_variable>

class ThreadPool
{
public:
  using job_type = std::function<void ()>;

  void start(size_t size)
  {
    if (!threads_.empty()) {
      throw std::runtime_error("thread pool already started");
    }

    for (size_t i = 0; i < size; ++i) {
      threads_.emplace_back(
        [this] {
          thread_task();
        });
    }
  }

  void queue(job_type job)
  {
    if (job == nullptr) {
      throw std::invalid_argument("job is nullptr");
    }

    std::lock_guard<std::mutex> l(mutex_);
    jobs_.push(job);
    cv_.notify_one();
  }

  void terminate()
  {
    terminate_ = true;
    cv_.notify_all();
    for (auto & t : threads_) {
      if (t.joinable()) {t.join();}
    }
    threads_.clear();
  }

private:
  void thread_task()
  {
    while (true) {
      job_type job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(
          l, [this] {
            return !jobs_.empty() || terminate_;
          });

        if (terminate_) {
          break;
        }

        job = jobs_.front();
        jobs_.pop();
      }
      job();
    }
  }

  bool terminate_ = false;
  std::mutex mutex_;
  std::queue<job_type> jobs_;
  std::condition_variable cv_;
  std::vector<std::thread> threads_;
};

#endif  // ROSBAG2_PERFORMANCE_BENCHMARKING__THREAD_POOL_HPP_
