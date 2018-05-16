cmake_minimum_required(VERSION 3.5)

project(ros2_rosbag_evaluation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")
set(BUILD_SHARED_LIBS ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(common_sources
  src/common/strings.cpp)

set(profiler_sources
  src/profiler/profiler.cpp)

set(sqlite_sources
  src/writer/sqlite/sqlite.cpp
  src/writer/sqlite/sqlite_writer.cpp
  src/writer/sqlite/one_table_sqlite_writer.cpp
  src/writer/sqlite/separate_topic_table_sqlite_writer.cpp)

set(trivial_writer_benchmark_sources
  src/benchmark/writer/trivial/trivial_writer_benchmark.cpp
  src/benchmark/benchmark.cpp
  src/writer/stream/message_stream_writer.cpp
  src/generators/message_generator.cpp)

set(sqlite_writer_benchmark_cmd_sources
  src/benchmark/writer/sqlite/sqlite_writer_benchmark_cmd.cpp
  src/benchmark/writer/sqlite/sqlite_writer_benchmark.cpp
  src/benchmark/benchmark.cpp
  src/generators/message_generator.cpp)

set(small_messages_benchmark_sources
  src/benchmark/small_messages_benchmark.cpp
  src/benchmark/writer/sqlite/sqlite_writer_benchmark.cpp
  src/benchmark/benchmark.cpp
  src/generators/message_generator.cpp)

set(big_messages_benchmark_sources
  src/benchmark/big_messages_benchmark.cpp
  src/benchmark/writer/sqlite/sqlite_writer_benchmark.cpp
  src/benchmark/benchmark.cpp
  src/generators/message_generator.cpp)

set(mixed_messages_benchmark_sources
  src/benchmark/mixed_messages_benchmark.cpp
  src/benchmark/writer/sqlite/sqlite_writer_benchmark.cpp
  src/benchmark/benchmark.cpp
  src/generators/message_generator.cpp)

add_library(common ${common_sources})
target_include_directories(common PRIVATE src)

add_library(profiler ${profiler_sources})
target_include_directories(profiler PRIVATE src)

add_library(sqlite ${sqlite_sources})
target_include_directories(sqlite PRIVATE src)
target_link_libraries(sqlite sqlite3 common)

add_executable(trivial_writer_benchmark ${trivial_writer_benchmark_sources})
target_link_libraries(trivial_writer_benchmark profiler sqlite)
target_include_directories(trivial_writer_benchmark PRIVATE src)

add_executable(sqlite_writer_benchmark_cmd ${sqlite_writer_benchmark_cmd_sources})
target_link_libraries(sqlite_writer_benchmark_cmd profiler sqlite)
target_include_directories(sqlite_writer_benchmark_cmd PRIVATE src)

add_executable(small_messages_benchmark ${small_messages_benchmark_sources})
target_link_libraries(small_messages_benchmark profiler sqlite)
target_include_directories(small_messages_benchmark PRIVATE src)

add_executable(big_messages_benchmark ${big_messages_benchmark_sources})
target_link_libraries(big_messages_benchmark profiler sqlite)
target_include_directories(big_messages_benchmark PRIVATE src)

add_executable(mixed_messages_benchmark ${mixed_messages_benchmark_sources})
target_link_libraries(mixed_messages_benchmark profiler sqlite)
target_include_directories(mixed_messages_benchmark PRIVATE src)
