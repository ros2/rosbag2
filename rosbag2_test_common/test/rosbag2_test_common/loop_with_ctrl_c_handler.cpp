// Copyright 2023 Apex.AI, Inc. or its affiliates. All Rights Reserved.
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

#include <cstdio>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <windows.h>
int main(void)
{
  // Enable default CTRL+C handler first. This is workaround and needed for the cases when
  // process created with CREATE_NEW_PROCESS_GROUP flag. Without it, installing custom Ctrl+C
  // handler will not work.
  if (!SetConsoleCtrlHandler(nullptr, false)) {
    std::cerr << "Error: Failed to enable default CTL+C handler. \n";
  }

  static std::atomic_bool running = true;
  // Installing our own control handler
  auto CtrlHandler = [](DWORD fdwCtrlType) -> BOOL {
      switch (fdwCtrlType) {
        case CTRL_C_EVENT:
          printf("Ctrl-C event\n");
          running = false;
          return TRUE;
        default:
          return FALSE;
      }
    };
  if (!SetConsoleCtrlHandler(CtrlHandler, TRUE)) {
    std::cerr << "\nError. Can't install SIGINT handler\n";
    return EXIT_FAILURE;
  } else {
    std::cout << "\nWaiting in a loop for CTRL+C event\n";
    std::cout.flush();
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }
  return EXIT_SUCCESS;
}
#else
#include <csignal>

int main()
{
  auto old_sigint_handler = std::signal(
    SIGINT, [](int /* signal */) {
      printf("Ctrl-C event\n");
      exit(EXIT_SUCCESS);
    });

  if (old_sigint_handler != SIG_ERR) {
    std::cout << "\nWaiting in a loop for CTRL+C event\n";
    std::cout.flush();
    while (1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  } else {
    std::cerr << "\nError. Can't install SIGINT handler\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

#endif
