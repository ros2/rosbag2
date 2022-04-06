^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_test_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
