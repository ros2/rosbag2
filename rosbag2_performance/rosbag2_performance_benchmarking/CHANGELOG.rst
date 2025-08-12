^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_performance_benchmarking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.9 (2025-08-11)
-------------------
* Fix warning: initialize number_of_threads (`#2121 <https://github.com/ros2/rosbag2/issues/2121>`_) (`#2123 <https://github.com/ros2/rosbag2/issues/2123>`_)
  Co-authored-by: Cristóbal Arroyo <cristobal.arroyo@ekumenlabs.com>
* Bugfix: `prosbag2_performance_benchmarking` outputs incorrect results for topics with frequency more than 1 kHz (`#2077 <https://github.com/ros2/rosbag2/issues/2077>`_) (`#2080 <https://github.com/ros2/rosbag2/issues/2080>`_)
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Fixes in the rosbag2_performance_benchmarking message data generator (`#2078 <https://github.com/ros2/rosbag2/issues/2078>`_) (`#2084 <https://github.com/ros2/rosbag2/issues/2084>`_)
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Enable `rosbag2_performance_benchmarking` package to be built by default (`#2093 <https://github.com/ros2/rosbag2/issues/2093>`_) (`#2097 <https://github.com/ros2/rosbag2/issues/2097>`_)
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Fix for failure in the benchmark_launch when using Process.wait twice (`#2076 <https://github.com/ros2/rosbag2/issues/2076>`_) (`#2082 <https://github.com/ros2/rosbag2/issues/2082>`_)
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Contributors: mergify[bot]

0.26.8 (2025-07-10)
-------------------
* [jazzy] Upstream quality changes from Apex.AI part-2 (backport `#1924 <https://github.com/ros2/rosbag2/issues/1924>`_) (`#1987 <https://github.com/ros2/rosbag2/issues/1987>`_)
* Contributors: Michael Orlov <morlovmr@gmail.com>, Christophe Bedard <bedard.christophe@gmail.com>

0.26.7 (2025-04-22)
-------------------

0.26.6 (2024-12-18)
-------------------

0.26.5 (2024-09-06)
-------------------

0.26.4 (2024-06-27)
-------------------

0.26.3 (2024-05-15)
-------------------

0.26.2 (2024-04-24)
-------------------

0.26.1 (2024-04-17)
-------------------

0.26.0 (2024-04-16)
-------------------
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`_)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`_)
* Contributors: Chris Lalancette, jmachowinski

0.25.0 (2024-03-27)
-------------------
* Add option to set compression threads priority (`#1457 <https://github.com/ros2/rosbag2/issues/1457>`_)
* Add per group statistics for rosbag2_performance_benchmarking report (`#1306 <https://github.com/ros2/rosbag2/issues/1306>`_)
* Contributors: Michael Orlov, jmachowinski

0.24.0 (2023-07-11)
-------------------
* Set CPU affinity for producers and recorder from benchmark parameters (`#1305 <https://github.com/ros2/rosbag2/issues/1305>`_)
* Add CPU usage to rosbag2_performance_benchmarking results report (`#1304 <https://github.com/ros2/rosbag2/issues/1304>`_)
* Add config option to use storage_id parameter in benchmark_launch.py (`#1303 <https://github.com/ros2/rosbag2/issues/1303>`_)
* Contributors: Michael Orlov

0.23.0 (2023-04-28)
-------------------

0.22.0 (2023-04-18)
-------------------
* Add tests for rosbag2_performance_benchmarking pkg (`#1268 <https://github.com/ros2/rosbag2/issues/1268>`_)
* Contributors: Michael Orlov

0.21.0 (2023-04-12)
-------------------
* Fix expectations for rosbag2 return code in rosbag2_performance_benchmarking (`#1267 <https://github.com/ros2/rosbag2/issues/1267>`_)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`_)
* Use thread pool to run benchmark publishers in rosbag2_performance_benchmarking (`#1250 <https://github.com/ros2/rosbag2/issues/1250>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov, Shane Loretz, carlossvg

0.20.0 (2023-02-14)
-------------------
* Skip ament_package() call when not building rosbag2_performance_benchmarking (`#1242 <https://github.com/ros2/rosbag2/issues/1242>`_)
* Contributors: Shane Loretz

0.19.0 (2023-01-13)
-------------------
* Add option to specify a message type (`#1153 <https://github.com/ros2/rosbag2/issues/1153>`_)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`_)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`_)
* Contributors: Emerson Knapp, Michael Orlov, carlossvg

0.18.0 (2022-11-15)
-------------------

0.17.0 (2022-07-30)
-------------------

0.16.0 (2022-05-11)
-------------------

0.15.1 (2022-04-06)
-------------------

0.15.0 (2022-04-05)
-------------------

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Install headers to include/${PROJECT_NAME} (`#958 <https://github.com/ros2/rosbag2/issues/958>`_)
* Contributors: Shane Loretz

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------
* Enable YAML encoding/decoding for RecordOptions and StorageOptions (`#916 <https://github.com/ros2/rosbag2/issues/916>`_)
  * Enable YAML encoding/decoding for RecordOptions and StorageOptions
* Contributors: Emerson Knapp

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Contributors: Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Updated node declare_parameter to new syntax (`#882 <https://github.com/ros2/rosbag2/issues/882>`_)
* Updated benchmark package to use writer close() instead of old reset() (`#881 <https://github.com/ros2/rosbag2/issues/881>`_)
* Contributors: Adam Dąbrowski

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------

0.7.0 (2021-03-18)
------------------
* fixed a memory leak in no-transport benchmark (`#674 <https://github.com/ros2/rosbag2/issues/674>`_)
* report of performance improvements in rosbag2 (roughly since Foxy) (`#651 <https://github.com/ros2/rosbag2/issues/651>`_)
* Performance benchmarking improvements (`#634 <https://github.com/ros2/rosbag2/issues/634>`_)
* Contributors: Adam Dąbrowski, Piotr Jaroszek

0.6.0 (2021-02-01)
------------------
* Performance benchmarking refactor (`#594 <https://github.com/ros2/rosbag2/issues/594>`_)
* Contributors: Adam Dąbrowski

0.5.0 (2020-12-02)
------------------
* Sqlite storage double buffering (`#546 <https://github.com/ros2/rosbag2/issues/546>`_)
* Contributors: Adam Dąbrowski

0.4.0 (2020-11-19)
------------------
* read yaml config file (`#497 <https://github.com/ros2/rosbag2/issues/497>`_)
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* performance testing packages (`#442 <https://github.com/ros2/rosbag2/issues/442>`_)
* Contributors: Adam Dąbrowski, Karsten Knese, Michael Jeronimo

0.3.2 (2020-06-03)
------------------

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------

0.2.5 (2020-04-30)
------------------

0.2.4 (2019-11-18 17:51)
------------------------

0.2.3 (2019-11-18 13:55)
------------------------

0.2.2 (2019-11-13)
------------------

0.2.1 (2019-10-23)
------------------

0.2.0 (2019-09-26)
------------------

0.1.2 (2019-05-20)
------------------

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------

0.0.3 (2018-12-14)
------------------

0.0.2 (2018-12-12)
------------------

0.0.1 (2018-12-11)
------------------
