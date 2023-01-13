^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage_mcap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
  This reverts commit aef9b9a65293f9e5d80a858ef84e485a8655a0c0.
* read_order: throw exception from set_read_order for unsupported orders
* Fix compile flags to work on rosbag_storage:0.17.x (`#78 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/78>`_)
  This fixes the compile flags for rolling, which has two versions -- one that does not support read order (0.17.x) and one that does support read order (0.18.x).
* Fix Windows build (`#73 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/73>`_)
  Update mcap version to newest windows-compatible release.
  Add visibility macros for tests.
  Add clang-format preprocessor indentation for visibility_control to be readable.
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
  1. Fixed some findings from Clang-Tidy
  1. Some renames according to the ROS2 coding style
  1. Add default initializations for member variables
  1. Moved code responsible for adding schema and channel from write(msg)
  to create_topic(topic) method to reduce performance burden on first
  message write and in lieu to preparation for moving schema collection
  process to upper SequentialWriter layer.
* Revert "rosbag2_storage_mcap: add storage preset profiles"
  This reverts commit 38830add3935b978968fe2703d3180b413ccc8c2.
* rosbag2_storage_mcap: add storage preset profiles
* Contributors: James Smith, Michael Orlov

0.3.0 (2022-09-09)
------------------
* Store IDL message definitions in Schema records when no MSG definition is available
  (`#43 <https://github.com/ros-tooling/rosbag2_storage_mcap/issues/43>`_)
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
  Incorporates fixes from https://github.com/foxglove/mcap/pull/478 and https://github.com/foxglove/mcap/pull/482
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
  I may be missing something, but from a cursory glance at [this code](https://github.com/ros2/rosbag2/blob/342d8ed3c1c4ae0411a4a92b60e79a728b8974b8/rosbag2_storage/src/rosbag2_storage/impl/storage_factory_impl.hpp#L108-L135), it appears that the `APPEND` mode is never used. This means we need to support `READ_WRITE`.
  This also adds a `.mcap` extension to recorded file names.
* Add dynamic message definition lookup (`#13 <https://github.com/wep21/rosbag2_storage_mcap/issues/13>`_)
  Currently, an exception will be thrown if lookup fails.
* Switch C++ formatter to clang-format (`#12 <https://github.com/wep21/rosbag2_storage_mcap/issues/12>`_)
  Remove uncrustify linter in favor of clang-format, which is easier to configure for use in VS Code format-on-save.
* Merge pull request `#7 <https://github.com/wep21/rosbag2_storage_mcap/issues/7>`_ from ros-tooling/jhurliman/reader-writer
  Reader and writer implementation
* uninitialized struct
* lint
* lint
* lint
* Reader and writer implementation
* Merge pull request `#6 <https://github.com/wep21/rosbag2_storage_mcap/issues/6>`_ from wep21/add-metadata-impl
  feat: add metadata impl
* feat: add metadata impl
* Merge pull request `#5 <https://github.com/wep21/rosbag2_storage_mcap/issues/5>`_ from wep21/mcap-storage-impl
  feat: mcap storage impl
* chore: update cmake minimum version
* chore: install mcap header
* chore: include mcap header
* fix: move fetch content into rosbag2 storage mcap
* Merge pull request `#3 <https://github.com/wep21/rosbag2_storage_mcap/issues/3>`_ from ros-tooling/emersonknapp/mcap_plugin_skeleton
  Add mcap storage plugin skeleton and CI
* Add rosbag2_storage_mcap skeleton
* Contributors: Daisuke Nishimatsu, Emerson Knapp, Jacob Bandes-Storch, John Hurliman, wep21
