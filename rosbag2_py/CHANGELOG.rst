^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.15.14 (2025-03-25)
--------------------
* [humble] Expose more Writer methods in python interface (backport `#1220 <https://github.com/ros2/rosbag2/issues/1220>`_ and `#1339 <https://github.com/ros2/rosbag2/issues/1339>`_) (`#1829 <https://github.com/ros2/rosbag2/issues/1829>`_)
* Add in python3-dev build dependency. (`#1863 <https://github.com/ros2/rosbag2/issues/1863>`_) (`#1866 <https://github.com/ros2/rosbag2/issues/1866>`_)
* Contributors: Jannik Jose, mergify[bot]

0.15.13 (2024-11-25)
--------------------
* Fixes clang compiler error introduced by `#1404 <https://github.com/ros2/rosbag2/issues/1404>`_. (`#1769 <https://github.com/ros2/rosbag2/issues/1769>`_)
* Contributors: Halvor Platou

0.15.12 (2024-07-28)
--------------------
* [Humble] Add topics with zero message counts to the SQLiteStorage::get_metadata(). (`#1722 <https://github.com/ros2/rosbag2/issues/1722>`_)
* [humble] Bugfix for wrong timestamps in ros2 bag info (backport `#1745 <https://github.com/ros2/rosbag2/issues/1745>`_) (`#1754 <https://github.com/ros2/rosbag2/issues/1754>`_)
* [humble] Expose py Reader metadata, improve `rosbag2_py.BagMetadata` usability (backport `#1082 <https://github.com/ros2/rosbag2/issues/1082>`_) (`#1404 <https://github.com/ros2/rosbag2/issues/1404>`_)
* [humble] Remove explicit sqlite3 from code (backport `#1166 <https://github.com/ros2/rosbag2/issues/1166>`_) (`#1723 <https://github.com/ros2/rosbag2/issues/1723>`_)
* [humble] rosbag2_storage: expose default storage ID as method (backport `#1146 <https://github.com/ros2/rosbag2/issues/1146>`_) (`#1724 <https://github.com/ros2/rosbag2/issues/1724>`_)
* [rosbag2] Fix typo to properly restore SIGINT hanlder (`#1687 <https://github.com/ros2/rosbag2/issues/1687>`_)
* [humble] Fix for false negative tests in rosbag2_py (backport `#1592 <https://github.com/ros2/rosbag2/issues/1592>`_) (`#1689 <https://github.com/ros2/rosbag2/issues/1689>`_)
* Contributors: Eric Cousineau, Michael Orlov, mergify[bot]

0.15.11 (2024-05-20)
--------------------

0.15.10 (2024-05-17)
--------------------
* [humble] Add --log-level to ros2 bag play and record (`#1655 <https://github.com/ros2/rosbag2/issues/1655>`_)
* Contributors: Roman

0.15.9 (2024-01-24)
-------------------
* [humble] Install signal handlers in recorder only inside record method (backport `#1464 <https://github.com/ros2/rosbag2/issues/1464>`_) (`#1526 <https://github.com/ros2/rosbag2/issues/1526>`_)
* Contributors: mergify[bot]

0.15.8 (2023-09-19)
-------------------

0.15.7 (2023-07-18)
-------------------
* [humble] Gracefully handle SIGINT and SIGTERM in rosbag2 recorder (backport `#1301 <https://github.com/ros2/rosbag2/issues/1301>`_) (`#1395 <https://github.com/ros2/rosbag2/issues/1395>`_)
* Contributors: Michael Orlov, mergify[bot]

0.15.6 (2023-06-05)
-------------------

0.15.5 (2023-04-25)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_) (`#1224 <https://github.com/ros2/rosbag2/issues/1224>`_)
* Contributors: mergify[bot]

0.15.4 (2023-01-10)
-------------------

0.15.3 (2022-11-07)
-------------------
* Revert "[humble] Backport. Added support for filtering topics via regular expressions (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)- (`#1039 <https://github.com/ros2/rosbag2/issues/1039>`_)" (`#1069 <https://github.com/ros2/rosbag2/issues/1069>`_)
* [humble] Backport. Added support for filtering topics via regular expressions (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)- (`#1039 <https://github.com/ros2/rosbag2/issues/1039>`_)
* Backport. Add use_sim_time option to record verb (`#1017 <https://github.com/ros2/rosbag2/issues/1017>`_)
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`_) (`#1008 <https://github.com/ros2/rosbag2/issues/1008>`_)
* Contributors: Esteve Fernandez, Keisuke Shima, Sean Kelly

0.15.2 (2022-05-11)
-------------------
* Fix test rosbag2_py test compatibility with Python < 3.8 (`#987 <https://github.com/ros2/rosbag2/issues/987>`_)
* Contributors: Scott K Logan

0.15.1 (2022-04-06)
-------------------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Audrow Nash, Barry Xu, Jorge Perez, Tony Peng

0.15.0 (2022-04-05)
-------------------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Barry Xu, Jorge Perez, Tony Peng

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Make sure published messages are acknowledged for play mode (`#951 <https://github.com/ros2/rosbag2/issues/951>`_)
* Contributors: Barry Xu

0.13.0 (2022-01-13)
-------------------
* Fix relative path syntax for cpplint (`#947 <https://github.com/ros2/rosbag2/issues/947>`_)
* Update to pybind11 2.7.1 (`#945 <https://github.com/ros2/rosbag2/issues/945>`_)
* Contributors: Chris Lalancette, Jacob Perron

0.12.0 (2021-12-17)
-------------------
* Add start-offset play option (`#931 <https://github.com/ros2/rosbag2/issues/931>`_)
* Expose bag_rewrite as `ros2 bag convert` (`#921 <https://github.com/ros2/rosbag2/issues/921>`_)
* Add "ignore leaf topics" option to recorder (`#925 <https://github.com/ros2/rosbag2/issues/925>`_)
* Add a ReaderWriterFactory utility to share logic for reuse (`#923 <https://github.com/ros2/rosbag2/issues/923>`_)
* Add pause/resume options to the bag recorder (`#905 <https://github.com/ros2/rosbag2/issues/905>`_)
* Contributors: Abrar Rahman Protyasha, Emerson Knapp, Ivan Santiago Paunovic

0.11.0 (2021-11-08)
-------------------
* Add --start-paused option to `ros2 bag play` (`#904 <https://github.com/ros2/rosbag2/issues/904>`_)
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Fix converter plugin choices for record (`#897 <https://github.com/ros2/rosbag2/issues/897>`_)
* Contributors: Emerson Knapp, Ivan Santiago Paunovic, Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Metadata per file info (`#870 <https://github.com/ros2/rosbag2/issues/870>`_)
* keyboard controls for pause/resume toggle and play-next: (`#847 <https://github.com/ros2/rosbag2/issues/847>`_)
* Add --snapshot-mode argument to the "record" verb (`#851 <https://github.com/ros2/rosbag2/issues/851>`_)
* Add stopRecording into rosbag2_py (`#854 <https://github.com/ros2/rosbag2/issues/854>`_)
* added seek interface (`#836 <https://github.com/ros2/rosbag2/issues/836>`_)
* Refactor plugin query mechanism and standardize trait management (`#833 <https://github.com/ros2/rosbag2/issues/833>`_)
* Update `PlayOptions::delay` to `rclcpp::Duration` to get nanosecond resolution (`#843 <https://github.com/ros2/rosbag2/issues/843>`_)
* Load compression and serialization choices via plugin query (`#827 <https://github.com/ros2/rosbag2/issues/827>`_)
* Add delay option (`#789 <https://github.com/ros2/rosbag2/issues/789>`_)
* Handle SIGTERM gracefully in recording (`#792 <https://github.com/ros2/rosbag2/issues/792>`_)
* Contributors: Afonso da Fonseca Braga, Cameron Miller, Emerson Knapp, Kosuke Takeuchi, Wojciech Jaworski, sonia

0.9.0 (2021-05-17)
------------------
* remove rosbag2_transport header (`#742 <https://github.com/ros2/rosbag2/issues/742>`_)
* Include utility to quiet cpplint. (`#744 <https://github.com/ros2/rosbag2/issues/744>`_)
* player owns the reader (`#725 <https://github.com/ros2/rosbag2/issues/725>`_)
* Contributors: Chris Lalancette, Karsten Knese

0.8.0 (2021-04-19)
------------------
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* Split Rosbag2Transport into Player and Recorder classes - first pass to enable further progress (`#721 <https://github.com/ros2/rosbag2/issues/721>`_)
* /clock publisher in Player (`#695 <https://github.com/ros2/rosbag2/issues/695>`_)
* Introducing Reindexer CLI (`#699 <https://github.com/ros2/rosbag2/issues/699>`_)
* Fix rosbag2_py transport test for py capsule (`#707 <https://github.com/ros2/rosbag2/issues/707>`_)
* rosbag2_py pybind wrapper for "record" - remove rosbag2_transport_py (`#702 <https://github.com/ros2/rosbag2/issues/702>`_)
* Add rosbag2_py::Player::play to replace rosbag2_transport_python version (`#693 <https://github.com/ros2/rosbag2/issues/693>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Contributors: Emerson Knapp, jhdcs

0.7.0 (2021-03-18)
------------------
* RMW-implementation-searcher converter in rosbag2_cpp (`#670 <https://github.com/ros2/rosbag2/issues/670>`_)
* use rosbag2_py for ros2 bag info (`#673 <https://github.com/ros2/rosbag2/issues/673>`_)
* CLI query rosbag2_py for available storage implementations (`#659 <https://github.com/ros2/rosbag2/issues/659>`_)
* Contributors: Emerson Knapp, Karsten Knese

0.6.0 (2021-02-01)
------------------
* Fix build issues when rosbag2_storage is binary installed (`#585 <https://github.com/ros2/rosbag2/issues/585>`_)
* Fix the tests on cyclonedds by translating qos duration values (`#606 <https://github.com/ros2/rosbag2/issues/606>`_)
* Contributors: Emerson Knapp, P. J. Reed

0.5.0 (2020-12-02)
------------------

0.4.0 (2020-11-19)
------------------
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Workaround pybind11 bug on Windows when CMAKE_BUILD_TYPE=RelWithDebInfo (`#538 <https://github.com/ros2/rosbag2/issues/538>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Fix rosbag2_py on Windows debug and stop ignoring the package (`#531 <https://github.com/ros2/rosbag2/issues/531>`_)
* Fix rosbag2_py bug when using libc++ (`#529 <https://github.com/ros2/rosbag2/issues/529>`_)
* AMENT_IGNORE rosbag2_py for now (`#509 <https://github.com/ros2/rosbag2/issues/509>`_)
* rosbag2_py reader and writer (`#308 <https://github.com/ros2/rosbag2/issues/308>`_)
* Contributors: Ivan Santiago Paunovic, Karsten Knese, Mabel Zhang, Michael Jeronimo

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
