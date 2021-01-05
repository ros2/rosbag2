^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_compression
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2021-01-05)
------------------
* Update maintainer list for Foxy (`#551 <https://github.com/ros2/rosbag2/issues/551>`_)
* Contributors: Michael Jeronimo

0.3.5 (2020-08-31)
------------------

0.3.4 (2020-08-05)
------------------
* Fix exception thrown given invalid arguments with compression enabled (`#488 <https://github.com/ros2/rosbag2/issues/488>`_) (`#489 <https://github.com/ros2/rosbag2/issues/489>`_)
* Contributors: Devin Bonnie

0.3.3 (2020-06-23)
------------------

0.3.2 (2020-06-03)
------------------
* Add user provided split size to error message (`#430 <https://github.com/ros2/rosbag2/issues/430>`_)
* Contributors: Anas Abou Allaban

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Fix playback of compressed bagfiles (`#417 <https://github.com/ros2/rosbag2/issues/417>`_)
* Export targets (`#403 <https://github.com/ros2/rosbag2/issues/403>`_)
* Contributors: Emerson Knapp, Karsten Knese

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------
* Remove relative include paths in rosbag2_compression tests (`#405 <https://github.com/ros2/rosbag2/issues/405>`_)
* Contributors: Karsten Knese, Zachary Michaels

0.2.5 (2020-04-30)
------------------
* Don't fail build if lsan isn't available. (`#397 <https://github.com/ros2/rosbag2/issues/397>`_)
* Correctly set all test dependencies. (`#392 <https://github.com/ros2/rosbag2/issues/392>`_)
* Deduplicate code in SequentialCompressionReader. (`#372 <https://github.com/ros2/rosbag2/issues/372>`_)
* Add filter for reading selective topics. (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities. (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* Fix compression log logic. (`#320 <https://github.com/ros2/rosbag2/issues/320>`_)
* Fix throw in playback of split+compressed bagfiles. (`#294 <https://github.com/ros2/rosbag2/issues/294>`_)
* Refactor Compression Reader/Writers to use the CompressionFactory. (`#315 <https://github.com/ros2/rosbag2/issues/315>`_)
* Add compression factory implementation. (`#313 <https://github.com/ros2/rosbag2/issues/313>`_)
* Include stdexcept. (`#314 <https://github.com/ros2/rosbag2/issues/314>`_)
* Add compression factory stubs. (`#311 <https://github.com/ros2/rosbag2/issues/311>`_)
* Replace rcutils_get_file_size with rcpputils::fs::file_size. (`#291 <https://github.com/ros2/rosbag2/issues/291>`_)
* [compression] Enable compression through ros2bag cli. (`#263 <https://github.com/ros2/rosbag2/issues/263>`_)
* [compression] Close storage before compression. (`#284 <https://github.com/ros2/rosbag2/issues/284>`_)
* Improve logging in rosbag2_compression. (`#287 <https://github.com/ros2/rosbag2/issues/287>`_)
* Change validation functions to accept output type of ZSTD_getFrameContentSize. (`#285 <https://github.com/ros2/rosbag2/issues/285>`_)
* code style only: wrap after open parenthesis if not in one line. (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Add more assertions on rosbag2_compression. (`#279 <https://github.com/ros2/rosbag2/issues/279>`_)
* [compression] Add SequentialCompressionWriter. (`#260 <https://github.com/ros2/rosbag2/issues/260>`_)
* Add a SequentialCompressionReader. (`#258 <https://github.com/ros2/rosbag2/issues/258>`_)
* Move compression artifacts from rosbag2_cpp to rosbag2_compression. (`#257 <https://github.com/ros2/rosbag2/issues/257>`_)
* remove rosbag2 filesystem helper. (`#249 <https://github.com/ros2/rosbag2/issues/249>`_)
* [Compression - 8] Enable reader to read from compressed files/messages. (`#246 <https://github.com/ros2/rosbag2/issues/246>`_)
* [compression] Follow ROS2 style conventions better and throw eagerly. (`#245 <https://github.com/ros2/rosbag2/issues/245>`_)
* [Compression] Use vector resize instead of reserve. (`#243 <https://github.com/ros2/rosbag2/issues/243>`_)
* [Compression - 6] Add Zstd file decompression implementation. (`#230 <https://github.com/ros2/rosbag2/issues/230>`_)
* Check output of fread/fwrite in compression. (`#237 <https://github.com/ros2/rosbag2/issues/237>`_)
* Fix compress uri. (`#234 <https://github.com/ros2/rosbag2/issues/234>`_)
* [Compression - 5] Add Zstd file compression. (`#220 <https://github.com/ros2/rosbag2/issues/220>`_)
* [Compression - 4] Add decompressor interface. (`#219 <https://github.com/ros2/rosbag2/issues/219>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Scott K Logan, Thomas Moulard, Zachary Michaels

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
