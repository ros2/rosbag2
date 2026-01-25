^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_test_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.9 (2025-08-11)
-------------------

0.26.8 (2025-07-10)
-------------------
* [jazzy] Address flakiness in tests where need to spin a node (backport `#2001 <https://github.com/ros2/rosbag2/issues/2001>`_) (`#2019 <https://github.com/ros2/rosbag2/issues/2019>`_)
* [jazzy] Upstream quality changes from Apex.AI part-2 (backport `#1924 <https://github.com/ros2/rosbag2/issues/1924>`_) (`#1987 <https://github.com/ros2/rosbag2/issues/1987>`_)
* [jazzy] Use DDS queue depth for subscriptions as a maximum value across publishers (backport `#1960 <https://github.com/ros2/rosbag2/issues/1960>`_) (`#1980 <https://github.com/ros2/rosbag2/issues/1980>`_)
* Contributors: Michael Orlov <morlovmr@gmail.com>, Christophe Bedard <bedard.christophe@gmail.com>

0.26.7 (2025-04-22)
-------------------
* [jazzy] Upstream quality changes from Apex.AI part 1 (backport `#1903 <https://github.com/ros2/rosbag2/issues/1903>`_) (`#1909 <https://github.com/ros2/rosbag2/issues/1909>`_)
* [jazzy] Use tmpfs in rosbag2 temporary_directory_fixture (backport `#1901 <https://github.com/ros2/rosbag2/issues/1901>`_) (`#1904 <https://github.com/ros2/rosbag2/issues/1904>`_)
* Contributors: mergify[bot], Michael Orlov

0.26.6 (2024-12-18)
-------------------
* Add debug information for flaky can_record_again_after_stop test (`#1871 <https://github.com/ros2/rosbag2/issues/1871>`_) (`#1874 <https://github.com/ros2/rosbag2/issues/1874>`_)
  (cherry picked from commit 4602b2ce829842e17ccb8bf4a74c135d6c8f2623)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* Improve the reliability of rosbag2 tests (`#1796 <https://github.com/ros2/rosbag2/issues/1796>`_) (`#1806 <https://github.com/ros2/rosbag2/issues/1806>`_)
* Contributors: Marco A. Gutierrez, mergify[bot]

0.26.5 (2024-09-06)
-------------------
* Small cleanups to the rosbag2 tests. (`#1792 <https://github.com/ros2/rosbag2/issues/1792>`_) (`#1793 <https://github.com/ros2/rosbag2/issues/1793>`_)
  (cherry picked from commit 604cebcf11775151efa94f7c30ba1aea68e90c5c)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: mergify[bot]

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
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`_)
* Contributors: Barry Xu

0.25.0 (2024-03-27)
-------------------
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`_)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`_)
* Contributors: Barry Xu, Chris Lalancette

0.24.0 (2023-07-11)
-------------------
* Add extra checks in execute_and_wait_until_completion(..) (`#1346 <https://github.com/ros2/rosbag2/issues/1346>`_)
* Address flakiness in rosbag2_play_end_to_end tests (`#1297 <https://github.com/ros2/rosbag2/issues/1297>`_)
* Contributors: Michael Orlov

0.23.0 (2023-04-28)
-------------------

0.22.0 (2023-04-18)
-------------------

0.21.0 (2023-04-12)
-------------------
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov

0.20.0 (2023-02-14)
-------------------

0.19.0 (2023-01-13)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_)
* rosbag2_py: parametrize tests across storage plugins (`#1203 <https://github.com/ros2/rosbag2/issues/1203>`_)
* Contributors: Michael Orlov, james-rms

0.18.0 (2022-11-15)
-------------------
* Fix for ros2 bag play exit with non-zero code on SIGINT (`#1126 <https://github.com/ros2/rosbag2/issues/1126>`_)
* Contributors: Michael Orlov

0.17.0 (2022-07-30)
-------------------
* Split up the include of rclcpp.hpp (`#1027 <https://github.com/ros2/rosbag2/issues/1027>`_)
* Contributors: Chris Lalancette

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

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Contributors: Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Make sure the subscription exists before publishing messages (`#804 <https://github.com/ros2/rosbag2/issues/804>`_)
* Handle SIGTERM gracefully in recording (`#792 <https://github.com/ros2/rosbag2/issues/792>`_)
* Add spin_and_wait_for_matched to PublicationManager and update test c… (`#797 <https://github.com/ros2/rosbag2/issues/797>`_)
* Avoid passing exception KeyboardInterrupt to the upper layer (`#788 <https://github.com/ros2/rosbag2/issues/788>`_)
* Contributors: Barry Xu, Emerson Knapp

0.9.0 (2021-05-17)
------------------
* Add play_next() API to the player class (`#762 <https://github.com/ros2/rosbag2/issues/762>`_)
* use rclcpp::SerializedMessage in MemoryManagement (`#750 <https://github.com/ros2/rosbag2/issues/750>`_)
* remodel publication manager (`#749 <https://github.com/ros2/rosbag2/issues/749>`_)
* use public recorder api in tests (`#741 <https://github.com/ros2/rosbag2/issues/741>`_)
* player owns the reader (`#725 <https://github.com/ros2/rosbag2/issues/725>`_)
* Contributors: Karsten Knese, Michael Orlov

0.8.0 (2021-04-19)
------------------
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* Fix bad_function_call by replacing rclcpp::spin_some with SingleThreadedExecutor (`#705 <https://github.com/ros2/rosbag2/issues/705>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Contributors: Emerson Knapp

0.7.0 (2021-03-18)
------------------
* Remove temporary directory platform-specific logic from test fixture (`#660 <https://github.com/ros2/rosbag2/issues/660>`_)
* Contributors: Emerson Knapp

0.6.0 (2021-02-01)
------------------
* Stabilize test_record by reducing copies of executors and messages (`#576 <https://github.com/ros2/rosbag2/issues/576>`_)
* Contributors: Emerson Knapp

0.5.0 (2020-12-02)
------------------

0.4.0 (2020-11-19)
------------------
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Contributors: Michael Jeronimo

0.3.2 (2020-06-03)
------------------

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Export targets (`#403 <https://github.com/ros2/rosbag2/issues/403>`_)
* Contributors: Karsten Knese

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------

0.2.5 (2020-04-30)
------------------
* use serialized message (`#386 <https://github.com/ros2/rosbag2/issues/386>`_)
* QoS Profile Overrides - Player (`#353 <https://github.com/ros2/rosbag2/issues/353>`_)
* Intelligently subscribe to topics according to their QoS profiles (`#355 <https://github.com/ros2/rosbag2/issues/355>`_)
* Override Subscriber QoS - Record (`#346 <https://github.com/ros2/rosbag2/issues/346>`_)
* fix cyclone tests (`#338 <https://github.com/ros2/rosbag2/issues/338>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Enhance E2E tests in Windows (`#278 <https://github.com/ros2/rosbag2/issues/278>`_)
* Add splitting e2e tests (`#247 <https://github.com/ros2/rosbag2/issues/247>`_)
* Make rosbag2 a metapackage (`#241 <https://github.com/ros2/rosbag2/issues/241>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Zachary Michaels

0.2.4 (2019-11-18)
------------------

0.2.3 (2019-11-18)
------------------

0.2.2 (2019-11-13)
------------------
* (API) Generate bagfile metadata in Writer (`#184 <https://github.com/ros2/rosbag2/issues/184>`_)
* Contributors: Zachary Michaels

0.2.1 (2019-10-23)
------------------
* Disable parameter event publishers on test nodes. (`#180 <https://github.com/ros2/rosbag2/issues/180>`_)
* Fix API for new Intra-Process communication. (`#143 <https://github.com/ros2/rosbag2/issues/143>`_)
* Contributors: Alberto Soragna, Dan Rose

0.2.0 (2019-09-26)
------------------

0.1.2 (2019-05-20)
------------------
* clean up test dependencies for rosbag2_test_common (`#118 <https://github.com/ros2/rosbag2/issues/118>`_)
  * clean up test dependencies for rosbag2_test_common
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * use build and exec depend
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#115 <https://github.com/ros2/rosbag2/issues/115>`_)
* fix compilation against master (`#111 <https://github.com/ros2/rosbag2/issues/111>`_)
* Compile tests (`#103 <https://github.com/ros2/rosbag2/issues/103>`_)
* enforce unique node names (`#86 <https://github.com/ros2/rosbag2/issues/86>`_)
* Contributors: Dirk Thomas, Karsten Knese, William Woodall

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
* 0.0.3
* Contributors: Karsten Knese

0.0.2 (2018-12-12)
------------------
* update maintainer email
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* Auto discovery of new topics (`#63 <https://github.com/ros2/rosbag2/issues/63>`_)
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* use uint8 for serialized message (`#61 <https://github.com/ros2/rosbag2/issues/61>`_)
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
* Use directory as bagfile and add additonal record options (`#43 <https://github.com/ros2/rosbag2/issues/43>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel
