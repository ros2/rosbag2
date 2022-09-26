^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add stopRecording into rosbag2_py (`#854 <https://github.com/ros2/rosbag2/issues/854>`_) (`#995 <https://github.com/ros2/rosbag2/issues/995>`_)
* [backport galactic] Fix `ros2 bag record` race at startup (`#911 <https://github.com/ros2/rosbag2/issues/911>`_)
* Contributors: Afonso da Fonseca Braga, Ivan Santiago Paunovic

0.9.1 (2021-07-08)
------------------
* [backport galactic] Handle SIGTERM gracefully in recording (`#792 <https://github.com/ros2/rosbag2/issues/792>`_) (`#807 <https://github.com/ros2/rosbag2/issues/807>`_)
  Backport `#792 <https://github.com/ros2/rosbag2/issues/792>`_ to galactic
  * Handle SIGTERM gracefully in recording
* [backport galactic] Add delay option (`#789 <https://github.com/ros2/rosbag2/issues/789>`_) (`#812 <https://github.com/ros2/rosbag2/issues/812>`_)
  Backport `#789 <https://github.com/ros2/rosbag2/issues/789>`_ to galactic
  * Add delay option
* Contributors: Emerson Knapp, Kosuke Takeuchi

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
