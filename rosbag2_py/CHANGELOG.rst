^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.9 (2025-08-11)
-------------------
* [jazzy] Add public API to get player's starting time and playback duration (backport `#2095 <https://github.com/ros2/rosbag2/issues/2095>`_) (`#2102 <https://github.com/ros2/rosbag2/issues/2102>`_)
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* [jazzy] Expose more of the player/recorder API through Python (backport `#2062 <https://github.com/ros2/rosbag2/issues/2062>`_) (`#2100 <https://github.com/ros2/rosbag2/issues/2100>`_)
  Co-authored-by: Christophe Bedard <bedard.christophe@gmail.com>
  Co-authored-by: Michael Orlov <morlovmr@gmail.com>
* Contributors: mergify[bot]

0.26.8 (2025-07-10)
-------------------
* Change Python player and recorder to be more as a classes (`#2047 <https://github.com/ros2/rosbag2/issues/2047>`_) (`#2054 <https://github.com/ros2/rosbag2/issues/2054>`_)
* [jazzy] Upstream quality changes from Apex.AI part-2 (backport `#1924 <https://github.com/ros2/rosbag2/issues/1924>`_) (`#1987 <https://github.com/ros2/rosbag2/issues/1987>`_)
* Bugfix: `ros2 bag convert` dropping messages with compression mode message (`#1975 <https://github.com/ros2/rosbag2/issues/1975>`_) (`#1985 <https://github.com/ros2/rosbag2/issues/1985>`_)
* Contributors: Michael Orlov <morlovmr@gmail.com>,
  Christophe Bedard <bedard.christophe@gmail.com>, Ben <benjamin.andrew@swri.org>

0.26.7 (2025-04-22)
-------------------
* Add bindings to close method in PyReader and PyCompressionReader (`#1935 <https://github.com/ros2/rosbag2/issues/1935>`_) (`#1937 <https://github.com/ros2/rosbag2/issues/1937>`_)
* Remove SHARED from pybind11_add_module (`#1929 <https://github.com/ros2/rosbag2/issues/1929>`_) (`#1931 <https://github.com/ros2/rosbag2/issues/1931>`_)
* [jazzy] Upstream quality changes from Apex.AI part 1 (backport `#1903 <https://github.com/ros2/rosbag2/issues/1903>`_) (`#1909 <https://github.com/ros2/rosbag2/issues/1909>`_)
* Contributors: mergify[bot], Øystein Sture, Silvio Traversaro, Michael Orlov

0.26.6 (2024-12-18)
-------------------
* [jazzy] Add support for replaying multiple bags (backport `#1848 <https://github.com/ros2/rosbag2/issues/1848>`_) (`#1873 <https://github.com/ros2/rosbag2/issues/1873>`_)
  Co-authored-by: Christophe Bedard <christophe.bedard@apex.ai>
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* [jazzy] Add "--sort" CLI option to the "ros2 bag info" command (backport `#1804 <https://github.com/ros2/rosbag2/issues/1804>`_) (`#1838 <https://github.com/ros2/rosbag2/issues/1838>`_)
  Co-authored-by: Soenke Prophet <soenke.prophet@gmail.com>
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
  Co-authored-by: Sanoronas <soenke.prophet@gmail.com>
* Add in python3-dev build dependency. (`#1863 <https://github.com/ros2/rosbag2/issues/1863>`_) (`#1864 <https://github.com/ros2/rosbag2/issues/1864>`_)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* [jazzy] Add computation of size contribution to info verb (backport `#1726 <https://github.com/ros2/rosbag2/issues/1726>`_) (`#1872 <https://github.com/ros2/rosbag2/issues/1872>`_)
  Co-authored-by: Nicola Loi <nicolaloi@outlook.com>
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* Contributors: Marco A. Gutierrez, mergify[bot]

0.26.5 (2024-09-06)
-------------------
* Added method to introspect QoS in Python (`#1648 <https://github.com/ros2/rosbag2/issues/1648>`_) (`#1790 <https://github.com/ros2/rosbag2/issues/1790>`_)
  (cherry picked from commit f0f3cc5f57ba9142b763247a68acc571d2500bb5)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Add cli option compression-threads-priority (`#1768 <https://github.com/ros2/rosbag2/issues/1768>`_) (`#1778 <https://github.com/ros2/rosbag2/issues/1778>`_)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
  (cherry picked from commit 25c3e1c2effdaea3b880c39ff7580b2f38a44b1c)
  Co-authored-by: Roman <rsokolkov@gmail.com>
* [jazzy] Update CI scripts to use Ubuntu Noble distros and bump action scripts to latest versions (backport `#1709 <https://github.com/ros2/rosbag2/issues/1709>`_) (`#1779 <https://github.com/ros2/rosbag2/issues/1779>`_)
  Co-authored-by: Roman <rsokolkov@gmail.com>
  (cherry picked from commit 27a6b600c2a813ec1f2154145fe77392c88b314b)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* Bugfix for wrong timestamps in ros2 bag info (`#1745 <https://github.com/ros2/rosbag2/issues/1745>`_) (`#1752 <https://github.com/ros2/rosbag2/issues/1752>`_)
  (cherry picked from commit da28c9da82824b8ce5f6fc18935d1a954e52b636)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* Contributors: mergify[bot]

0.26.4 (2024-06-27)
-------------------
* Add bindings for LocalMessageDefinitionSource (`#1697 <https://github.com/ros2/rosbag2/issues/1697>`_) (`#1701 <https://github.com/ros2/rosbag2/issues/1701>`_)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
  Co-authored-by: methylDragon <methylDragon@gmail.com>
* Add --log-level to ros2 bag play and record (`#1625 <https://github.com/ros2/rosbag2/issues/1625>`_) (`#1674 <https://github.com/ros2/rosbag2/issues/1674>`_)
  Co-authored-by: Roman Sokolkov <rsokolkov@gmail.com>
* Contributors: mergify[bot]

0.26.3 (2024-05-15)
-------------------
* Included to_rclcpp_qos_vector to Python wrappers (`#1642 <https://github.com/ros2/rosbag2/issues/1642>`_) (`#1650 <https://github.com/ros2/rosbag2/issues/1650>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

0.26.2 (2024-04-24)
-------------------

0.26.1 (2024-04-17)
-------------------

0.26.0 (2024-04-16)
-------------------
* Add option to disable recorder keyboard controls (`#1607 <https://github.com/ros2/rosbag2/issues/1607>`_)
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`_)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`_)
* Switch rclpy to be an exec_depend here. (`#1606 <https://github.com/ros2/rosbag2/issues/1606>`_)
* Gracefully handle SIGINT and SIGTERM signals for play and burst CLI (`#1557 <https://github.com/ros2/rosbag2/issues/1557>`_)
* Added exclude-topic-types to record (`#1582 <https://github.com/ros2/rosbag2/issues/1582>`_)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Bernd Pfrommer, Chris Lalancette, Michael Orlov, jmachowinski

0.25.0 (2024-03-27)
-------------------
* Fix for false negative tests in rosbag2_py (`#1592 <https://github.com/ros2/rosbag2/issues/1592>`_)
* Update rosbag2_py stubs (`#1593 <https://github.com/ros2/rosbag2/issues/1593>`_)
* Add Python stubs for rosbag2_py (`#1459 <https://github.com/ros2/rosbag2/issues/1459>`_) (`#1569 <https://github.com/ros2/rosbag2/issues/1569>`_)
* Filter topic by type  (`#1577 <https://github.com/ros2/rosbag2/issues/1577>`_)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`_)
* Install signal handlers in recorder only inside record method (`#1464 <https://github.com/ros2/rosbag2/issues/1464>`_)
* add missing import otherwise it doesnt compile (`#1524 <https://github.com/ros2/rosbag2/issues/1524>`_)
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`_)
* Make `rosbag2_transport::Player::play()` run in a separate thread (`#1503 <https://github.com/ros2/rosbag2/issues/1503>`_)
* Switch to target_link_libraries everywhere. (`#1504 <https://github.com/ros2/rosbag2/issues/1504>`_)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`_)
* ros2 bag convert now excludes messages not in [start_time;end_time] (`#1455 <https://github.com/ros2/rosbag2/issues/1455>`_)
* Add support for compression to python API (`#1425 <https://github.com/ros2/rosbag2/issues/1425>`_)
* Contributors: Alejandro Hernández Cordero, Andrew Symington, Barry Xu, Chris Lalancette, Michael Orlov, Mikael Arguedas, Patrick Roncagliolo, Peter Favrholdt, Roman Sokolkov

0.24.0 (2023-07-11)
-------------------
* Gracefully handle SIGINT and SIGTERM in rosbag2 recorder (`#1301 <https://github.com/ros2/rosbag2/issues/1301>`_)
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`_)
* Add binding to close the writer (`#1339 <https://github.com/ros2/rosbag2/issues/1339>`_)
* Contributors: Emerson Knapp, Michael Orlov, Yadu

0.23.0 (2023-04-28)
-------------------

0.22.0 (2023-04-18)
-------------------
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`_)
* Store message definitions in SQLite3 storage plugin (`#1293 <https://github.com/ros2/rosbag2/issues/1293>`_)
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

0.19.0 (2023-01-13)
-------------------
* Expose more Writer methods in python interface (`#1220 <https://github.com/ros2/rosbag2/issues/1220>`_)
* rosbag2_storage: set MCAP as default plugin (`#1160 <https://github.com/ros2/rosbag2/issues/1160>`_)
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_)
* rosbag2_py: parametrize tests across storage plugins (`#1203 <https://github.com/ros2/rosbag2/issues/1203>`_)
* Added option to change node name for the recorder from the Python API (`#1180 <https://github.com/ros2/rosbag2/issues/1180>`_)
* Replace language for "db3"/"db"/"database" (`#1194 <https://github.com/ros2/rosbag2/issues/1194>`_)
* Remove explicit sqlite3 from code (`#1166 <https://github.com/ros2/rosbag2/issues/1166>`_)
* Move python get_default_storage_id to storage module instead of writer (`#1165 <https://github.com/ros2/rosbag2/issues/1165>`_)
* Contributors: Emerson Knapp, Michael Orlov, james-rms, ricardo-manriquez

0.18.0 (2022-11-15)
-------------------
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`_)
* rosbag2_py: set defaults for config when bag rewriting (`#1121 <https://github.com/ros2/rosbag2/issues/1121>`_)
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`_)
* expose py Reader metadata, improve `rosbag2_py.BagMetadata` usability (`#1082 <https://github.com/ros2/rosbag2/issues/1082>`_)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`_)
* Contributors: Emerson Knapp, Esteve Fernandez, james-rms

0.17.0 (2022-07-30)
-------------------
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`_)
* Add additional mode of publishing sim time updates triggered by replayed messages (`#1050 <https://github.com/ros2/rosbag2/issues/1050>`_)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`_)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`_)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`_)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`_)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`_)
* Contributors: Agustin Alba Chicar, Esteve Fernandez, Geoffrey Biggs, Hunter L. Allen, kylemarcey, Michael Orlov, Tony Peng

0.16.0 (2022-05-11)
-------------------
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`_)
* Fix test rosbag2_py test compatibility with Python < 3.8 (`#987 <https://github.com/ros2/rosbag2/issues/987>`_)
* Contributors: Michael Orlov, Scott K Logan, Sean Kelly

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
