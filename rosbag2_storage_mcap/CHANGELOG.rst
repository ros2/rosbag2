^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage_mcap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.1 (2023-05-18)
-------------------

0.22.0 (2023-04-18)
-------------------
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`_)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`_)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`_)
* Contributors: Michael Orlov, james-rms

0.21.0 (2023-04-12)
-------------------
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`_)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov, james-rms

0.20.0 (2023-02-14)
-------------------
* CLI: Get storage-specific values from plugin (`#1209 <https://github.com/ros2/rosbag2/issues/1209>`_)
* Contributors: Emerson Knapp

0.19.0 (2023-01-13)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_)
* rosbag2_cpp: test more than one storage plugin (`#1196 <https://github.com/ros2/rosbag2/issues/1196>`_)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`_)
* rosbag2_storage_mcap: merge into rosbag2 repo (`#1163 <https://github.com/ros2/rosbag2/issues/1163>`_)
* Contributors: Michael Orlov, james-rms

0.6.0 (2022-11-28)
------------------
* mcap_storage: 'none' is a valid storage preset profile (`#86 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/86>`_)
* mcap_storage: handle update_metadata call (`#83 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/83>`_)
* Update clang-format rules to fit ROS 2 style guide (`#80 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/80>`_)
* Revert "read_order: throw exception from set_read_order for unsupported orders"
* read_order: throw exception from set_read_order for unsupported orders
* Fix compile flags to work on rosbag_storage:0.17.x (`#78 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/78>`_)
* Fix Windows build (`#73 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/73>`_)
* Contributors: Andrew Symington, Emerson Knapp, James Smith, james-rms

0.5.0 (2022-11-02)
------------------
* set defaults for SQLite plugin parity (`#68 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/68>`_)
* rosbag2_storage_mcap: add storage preset profiles (`#57 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/57>`_)
* rename test_fixture_interfaces package to testdata (`#64 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/64>`_)
* Switch to using the vendored zstd library. (`#59 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/59>`_)
* Add set_read_order reader API (`#54 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/54>`_)
* Contributors: Chris Lalancette, Emerson Knapp, James Smith

0.4.0 (2022-10-06)
------------------
* Some minor improvements in rosbag2_storage_mcap after review (`#58 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/58>`_)
* Revert "rosbag2_storage_mcap: add storage preset profiles"
* rosbag2_storage_mcap: add storage preset profiles
* Contributors: James Smith, Michael Orlov

0.3.0 (2022-09-09)
------------------
* Store IDL message definitions in Schema records when no MSG definition is available (`#43 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/43>`_)
* Contributors: James Smith

0.2.0 (2022-09-08)
------------------
* Support timestamp-ordered playback (`#50 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/50>`_)
* Support regex topic filtering
* Contributors: James Smith

0.1.7 (2022-08-15)
------------------
* Add all lz4 sources to fix undefined symbols at runtime (`#46 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/46>`_)
* Contributors: Emerson Knapp

0.1.6 (2022-07-22)
------------------
* Upgrade mcap to fix LZ4 error and segfault (`#42 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/42>`_)
* Contributors: Jacob Bandes-Storch

0.1.5 (2022-04-25)
------------------
* Fix build for Foxy (`#34 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/34>`_)
* Contributors: Jacob Bandes-Storch

0.1.4 (2022-04-21)
------------------
* fix: minor issues (`#31 <https://github.com/wep21/rosbag2_storage_mcap/issues/31>`_)
  * remove unnecessary block
  * use target_link_libraries instead of ament_target_dependencies
  * remove ros environment
  * add prefix to compile definition
* Update email address for Foxglove maintainers (`#32 <https://github.com/wep21/rosbag2_storage_mcap/issues/32>`_)
* Contributors: Daisuke Nishimatsu, Jacob Bandes-Storch

0.1.3 (2022-04-20)
------------------

0.1.2 (2022-04-20)
------------------
* Added mcap_vendor package. Updated CMakeLists.txt to fetch dependencies with FetchContent rather than Conan.
* Contributors: Jacob Bandes-Storch

0.1.1 (2022-04-01)
------------------
* CMake build script will now execute pip install conan automatically.
* Contributors: Daisuke Nishimatsu

0.1.0 (2022-03-24)
------------------
* [1.0.0] Use Summary section for get_metadata() and seek(), implement remaining methods (`#17 <https://github.com/wep21/rosbag2_storage_mcap/issues/17>`_)
* feat: add play impl (`#16 <https://github.com/wep21/rosbag2_storage_mcap/issues/16>`_)
* chore: refine package.xml (`#15 <https://github.com/wep21/rosbag2_storage_mcap/issues/15>`_)
* Don't throw when READ_WRITE mode is used; add .mcap file extension to recorded files (`#14 <https://github.com/wep21/rosbag2_storage_mcap/issues/14>`_)
* Add dynamic message definition lookup (`#13 <https://github.com/wep21/rosbag2_storage_mcap/issues/13>`_)
* Switch C++ formatter to clang-format (`#12 <https://github.com/wep21/rosbag2_storage_mcap/issues/12>`_)
* Merge pull request `#7 <https://github.com/wep21/rosbag2_storage_mcap/issues/7>`_ from ros-tooling/jhurliman/reader-writer
* uninitialized struct
* lint
* lint
* lint
* Reader and writer implementation
* Merge pull request `#6 <https://github.com/wep21/rosbag2_storage_mcap/issues/6>`_ from wep21/add-metadata-impl
* feat: add metadata impl
* Merge pull request `#5 <https://github.com/wep21/rosbag2_storage_mcap/issues/5>`_ from wep21/mcap-storage-impl
* chore: update cmake minimum version
* chore: install mcap header
* chore: include mcap header
* fix: move fetch content into rosbag2 storage mcap
* Merge pull request `#3 <https://github.com/wep21/rosbag2_storage_mcap/issues/3>`_ from ros-tooling/emersonknapp/mcap_plugin_skeleton
* Add rosbag2_storage_mcap skeleton
* Contributors: Daisuke Nishimatsu, Emerson Knapp, Jacob Bandes-Storch, John Hurliman, wep21
