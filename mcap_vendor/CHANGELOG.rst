^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcap_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.15.7 (2023-07-18)
-------------------

0.15.6 (2023-06-05)
-------------------

0.15.5 (2023-04-25)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_) (`#1224 <https://github.com/ros2/rosbag2/issues/1224>`_)
* Contributors: mergify[bot]

0.15.4 (2023-01-10)
-------------------
* [backport humble `#1208 <https://github.com/ros2/rosbag2/issues/1208>`_] Fixes policy CMP0135 warning for CMake >= 3.24 for mcap_vendor (`#1227 <https://github.com/ros2/rosbag2/issues/1227>`_)
* mcap_vendor: only install public headers (backport `#1207 <https://github.com/ros2/rosbag2/issues/1207>`_) (`#1214 <https://github.com/ros2/rosbag2/issues/1214>`_)
* rosbag2_storage_mcap: fix rosbag2_cpp tests (`#1205 <https://github.com/ros2/rosbag2/issues/1205>`_)
* Use mcap tarball rather than git clone (`#1200 <https://github.com/ros2/rosbag2/issues/1200>`_)
* [Humble backport] rosbag2_storage_mcap: merge into rosbag2 repo (`#1163 <https://github.com/ros2/rosbag2/issues/1163>`_) (`#1189 <https://github.com/ros2/rosbag2/issues/1189>`_)
* Contributors: Michael Carroll, james-rms, mergify[bot]

0.6.0 (2022-11-28)
------------------
* Fix Windows build (`#73 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/73>`_)
  Update mcap version to newest windows-compatible release.
  Add visibility macros for tests.
  Add clang-format preprocessor indentation for visibility_control to be readable.
* Contributors: Emerson Knapp

0.5.0 (2022-11-02)
------------------
* mcap_vendor: update to v0.6.0 (`#69 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/69>`_)
* Cleanup in `mcap_vendor` package (`#62 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/62>`_)
* Switch to using the vendored zstd library. (`#59 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/59>`_)
* Contributors: Chris Lalancette, Michael Orlov, James Smith

0.4.0 (2022-10-06)
------------------

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
  Incorporates fixes from https://github.com/foxglove/mcap/pull/478 and https://github.com/foxglove/mcap/pull/482
* Add missing buildtool_depend on git (`#37 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/37>`_)
  This vendor package uses git to fetch sources for other packages. It should declare a dependency on that build tool.
  This should address the current cause of RPM build failures for RHEL: https://build.ros2.org/view/Rbin_rhel_el864/job/Rbin_rhel_el864__mcap_vendor__rhel_8_x86_64__binary/
* Contributors: Jacob Bandes-Storch, Scott K Logan

0.1.5 (2022-04-25)
------------------
* Test Foxy & Galactic in CI, fix missing test_depends in mcap_vendor/package.xml (`#33 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/33>`_)
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
