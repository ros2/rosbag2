^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_converter_default_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Forthcoming
-----------

0.3.9 (2022-02-03)
------------------

0.3.8 (2021-07-19)
------------------

0.3.7 (2021-02-15)
------------------

0.3.6 (2021-01-05)
------------------
* Update maintainer list for Foxy (`#551 <https://github.com/ros2/rosbag2/issues/551>`_)
* Contributors: Michael Jeronimo

0.3.5 (2020-08-31)
------------------

0.3.4 (2020-08-05)
------------------

0.3.3 (2020-06-23)
------------------

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
* Correct usage of rcpputils::SharedLibrary loading. (`#400 <https://github.com/ros2/rosbag2/issues/400>`_)
* Contributors: Karsten Knese

0.2.5 (2020-04-30)
------------------
* Fixed string related with poco (`#374 <https://github.com/ros2/rosbag2/issues/374>`_)
* rename rosidl_generator_cpp namespace to rosidl_runtime_cpp (`#366 <https://github.com/ros2/rosbag2/issues/366>`_)
* added rosidl_runtime c and cpp depencencies (`#310 <https://github.com/ros2/rosbag2/issues/310>`_)
* Replace poco dependency by rcutils (`#322 <https://github.com/ros2/rosbag2/issues/322>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Make rosbag2 a metapackage (`#241 <https://github.com/ros2/rosbag2/issues/241>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Karsten Knese

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

0.2.0 (2019-09-26)
------------------
* disable plugins/tests which need rmw_fastrtps_cpp if unavailable (`#137 <https://github.com/ros2/rosbag2/issues/137>`_)
* Contributors: ivanpauno

0.1.2 (2019-05-20)
------------------

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* fix compilation against master (`#111 <https://github.com/ros2/rosbag2/issues/111>`_)
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
* use fastrtps static instead of dynamic (`#104 <https://github.com/ros2/rosbag2/issues/104>`_)
* Contributors: Dirk Thomas, Karsten Knese

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
* Split converters (`#70 <https://github.com/ros2/rosbag2/issues/70>`_)
* Fix master build and small renamings (`#67 <https://github.com/ros2/rosbag2/issues/67>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
  * GH-118 Rename rosbag2_storage::TopicMetadata to TopicInformation and rosbag2_storage::TopicwithType to TopicMetadata
* Use converters when playing back files (`#56 <https://github.com/ros2/rosbag2/issues/56>`_)
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
* Contributors: Alessandro Bottero, Andreas Holzner, Martin Idel
