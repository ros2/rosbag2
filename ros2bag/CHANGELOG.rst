^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2bag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2021-02-15)
------------------

0.3.6 (2021-01-05)
------------------
* Update maintainer list for Foxy (`#551 <https://github.com/ros2/rosbag2/issues/551>`_)
* Contributors: Michael Jeronimo

0.3.5 (2020-08-31)
------------------
* Add pytest.ini so local tests don't display warning. (`#446 <https://github.com/ros2/rosbag2/issues/446>`_) (`#514 <https://github.com/ros2/rosbag2/issues/514>`_)
* Contributors: Emerson Knapp

0.3.4 (2020-08-05)
------------------
* Validate QoS profile values are not negative. (`#483 <https://github.com/ros2/rosbag2/issues/483>`_) (`#490 <https://github.com/ros2/rosbag2/issues/490>`_)
  Co-authored-by: Jesse Ikawa <64169356+jikawa-az@users.noreply.github.com>
* Contributors: Devin Bonnie

0.3.3 (2020-06-23)
------------------

0.3.2 (2020-06-03)
------------------
* Improve help message for CLI verbs (`#427 <https://github.com/ros2/rosbag2/issues/427>`_)
* Contributors: Jacob Perron

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Don't allow user to specify unimplemented compression mode 'message' (`#415 <https://github.com/ros2/rosbag2/issues/415>`_)
* Use consistent quotes in help messages (`#416 <https://github.com/ros2/rosbag2/issues/416>`_)
* Contributors: Dirk Thomas, Emerson Knapp

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------

0.2.5 (2020-04-30)
------------------
* add topic remapping option to rosbag2 play (`#388 <https://github.com/ros2/rosbag2/issues/388>`_)
* Add loop option to rosbag play (`#361 <https://github.com/ros2/rosbag2/issues/361>`_)
* Expose topic filter to command line (addresses `#342 <https://github.com/ros2/rosbag2/issues/342>`_) (`#363 <https://github.com/ros2/rosbag2/issues/363>`_)
* Override QoS Profiles in CLI - Playback (`#356 <https://github.com/ros2/rosbag2/issues/356>`_)
* Refactor utility functions in ros2bag (`#358 <https://github.com/ros2/rosbag2/issues/358>`_)
* Add QoS Profile override to CLI (`#347 <https://github.com/ros2/rosbag2/issues/347>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* include hidden topics (`#332 <https://github.com/ros2/rosbag2/issues/332>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Add playback rate command line arg (`#304 <https://github.com/ros2/rosbag2/issues/304>`_)
* [compression] Enable compression through ros2bag cli (`#263 <https://github.com/ros2/rosbag2/issues/263>`_)
* switch to not deprecated API (`#261 <https://github.com/ros2/rosbag2/issues/261>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Karsten Knese, Mabel Zhang, Sriram Raghunathan, Zachary Michaels, ketatam

0.2.4 (2019-11-18)
------------------

0.2.3 (2019-11-18)
------------------
* Add CLI option to expose option for bagfile splitting (`#203 <https://github.com/ros2/rosbag2/issues/203>`_)
* Contributors: Karsten Knese, Prajakta Gokhale

0.2.2 (2019-11-13)
------------------

0.2.1 (2019-10-23)
------------------
* Fix flake8 errors and add missing lint tests. (`#194 <https://github.com/ros2/rosbag2/issues/194>`_)
* Import rosbag2_transport Python module on demand. (`#190 <https://github.com/ros2/rosbag2/issues/190>`_)
* Contributors: Michel Hidalgo, Thomas Moulard

0.2.0 (2019-09-26)
------------------
* install resource marker file for package (`#167 <https://github.com/ros2/rosbag2/issues/167>`_)
* install package manifest (`#161 <https://github.com/ros2/rosbag2/issues/161>`_)
* Contributors: Dirk Thomas, Ruffin

0.1.2 (2019-05-20)
------------------
* remove disclaimer (`#122 <https://github.com/ros2/rosbag2/issues/122>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* Fix issue with ros2bag record python frontend (`#100 <https://github.com/ros2/rosbag2/issues/100>`_)
* Consistent node naming across ros2cli tools (`#60 <https://github.com/ros2/rosbag2/issues/60>`_)
* Contributors: AAlon, Sagnik Basu

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
* 0.0.3
* Play old bagfiles (`#69 <https://github.com/bsinno/rosbag2/issues/69>`_)
* Release fixups (`#72 <https://github.com/bsinno/rosbag2/issues/72>`_)
* Contributors: Andreas Holzner, Karsten Knese, Martin Idel

0.0.2 (2018-12-12)
------------------
* update maintainer email
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* Auto discovery of new topics (`#63 <https://github.com/ros2/rosbag2/issues/63>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* Use directory as bagfile and add additonal record options (`#43 <https://github.com/ros2/rosbag2/issues/43>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* initial command line interface (`#12 <https://github.com/ros2/rosbag2/issues/12>`_)
* (demo, sqlite3) First working rosbag2 implementation (`#6 <https://github.com/ros2/rosbag2/issues/6>`_)
* initial setup
* Contributors: Alessandro Bottero, Andreas Greimel, Karsten Knese, Martin Idel
