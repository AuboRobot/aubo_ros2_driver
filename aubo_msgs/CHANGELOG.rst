^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2022-04-12)
------------------
* Porting to ros2 (`#12 <https://github.com/destogl/ur_msgs/issues/12>`_)
* Document pin mapping in SetIO service (`#16 <https://github.com/destogl/ur_msgs/issues/16>`_)
* Contributors: Denis Štogl, Felix Exner, gavanderhoorn

1.3.4 (2021-05-25)
------------------
* Bump CMake version to ignore warning
* Contributors: gavanderhoorn

1.3.3 (2021-05-25)
------------------
* Migrate to Github Actions
* Fix SetPayload srv separater invalid (`#10 <https://github.com/ros-industrial/ur_msgs/issues/10>`_)
* Contributors: Chen Bainian, gavanderhoorn

1.3.2 (2020-10-12)
------------------
* Fix domain constants in ``Analog.msg`` (`#8 <https://github.com/ros-industrial/ur_msgs/issues/8>`_)
* Contributors: Gaël Écorchard

1.3.1 (2020-06-24)
------------------
* Change ``payload`` field name to ``mass`` (`#5 <https://github.com/ros-industrial/ur_msgs/issues/5>`_)
* Added ``center_of_gravity`` field to ``SetPayload`` service (`#2 <https://github.com/ros-industrial/ur_msgs/issues/2>`_)
* Mark package as architecture independent (`#1 <https://github.com/ros-industrial/ur_msgs/issues/1>`_)
* Contributors: Felix Exner, gavanderhoorn

1.3.0 (2020-06-22)
------------------
* First release of this package from its new repository.
* Contributors: gavanderhoorn

1.2.7 (2019-11-23)
------------------

1.2.6 (2019-11-19)
------------------
* Added a service to set the speed slider position (`#454 <https://github.com/ros-industrial/universal_robot/issues/454>`_)
* Added a domain field to Analog.msg (`#450 <https://github.com/ros-industrial/universal_robot/issues/450>`_)
* Migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Contributors: Felix Mauch

1.2.5 (2019-04-05)
------------------
* Correct width and type of 'digital\_*_bits'. (`#416 <https://github.com/ros-industrial/universal_robot/issues/416>`_)
* Clarify use of fields in SetIO svc. (`#415 <https://github.com/ros-industrial/universal_robot/issues/415>`_)
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* Add RobotModeDataMsg (`#395 <https://github.com/ros-industrial/universal_robot/issues/395>`_)
* Update maintainer and author information.
* Contributors: Felix von Drigalski, gavanderhoorn

1.2.1 (2018-01-06)
------------------

1.2.0 (2017-08-04)
------------------

1.1.9 (2017-01-02)
------------------
* No changes.

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Add message definition for ToolData.
* Contributors: Nikolas Engelhard

1.1.6 (2016-04-01)
------------------
* Moved SetIO FUN constants from driver.py to relevant srv file for easier interaction from other files
* catkin_lint
* Contributors: Thomas Timm Andersen, ipa-fxm
