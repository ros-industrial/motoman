^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2016-07-03)
------------------
* Cleaned up issues with Changelogs
* Contributors: Shaun Edwards

0.3.4 (2016-07-03)
------------------
* driver: remove deprecated 'robot_interface.launch' file.
  Deprecated since at least Aug-2013 (9acb8550).
* Support for multiple motion group control
* v1.3.4 MotoROS driver
  If a multi-group command is being processed, but one of the groups has
  a full queue, it will respond with a proper ROS_RESULT_BUSY message.  To
  keep everything in sync, none of the groups will be processed.
* v1.3.3 MotoROS driver
  Improved initialization speed on DX200 controllers.
  Updates to the Parameter Extraction library on DX200 allow faster
  reading from the controller at startup.
* v1.3.0 MotoROS driver - minor bug fix
  Fix expected byte length for multi-group messages
* v1.3.1 MotoROS driver
  Added assertion to verify axis-type is valid
  Removed commented code line
* v1.3.0 MotoROS driver
  Add support for linear axes (such as base track).  Linear position
  data in Meters.
  Fix support for external axes on DX100.
  Fix error code returned for an unknown msgType.
  Modified the size verification on ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX
  messages.  You are not required to send data for four groups if you
  system doesn't have that many groups.* correcting comment to match with launch files: dx100 does not use bswap, FS100 does
* Fix Issue `#62 <https://github.com/shaun-edwards/motoman/issues/62>`_: motoman_driver CMakeLists.txt missing motoman_msgs dependencies
* v1.2.5 MotoROS driver
  Fix disconnect logic in the State Server
  Fixed issue with re-connecting to the State Server after closing
  multiple concurrent connections.
* v1.2.4 Update multi-group message id's
* v1.2.3 Rename subdirectory for DX200 Inform-job
  No change actual job; just to folder structure.
  Corrected ROS_MSG_JOINT_FEEDBACK_EX message.
  This message will only be sent if the controller has more than one
  control-group.
  Single arm systems will not send this message.
* v1.2.2
  Corrected the ROS_MSG_MOTO_MOTION_REPLY when replying to
  ROS_MSG_JOINT_TRAJ_PT_FULL_EX.  A motion-reply message will be sent for
  each control group affected by the multi-group-motion message.  The
  motion-reply will correctly indicate the control group index for what it
  represents
* v1.2.1
  Primitive I/O support
  Added custom Motoman-specific message for reading and writing a single
  I/O point in the controller.
  Note: Write-support is limited to only certain addresses in the robot
  controller.  See wiki for details.
  Fixed multiple-arm support for the DX100 controller.
* v1.2.0
  Add support for multiple control groups.
  Support for SDA robots, or multiple individual robots and/or external
  axes.
  Add new command message for controlling up to 4 groups.
  Add new position-feedback message to send all group data.
  Add compatibility for DX200 controller.
  Convert MotoPlusIDE projects into Visual Studio solution.
  Maintained legacy compatibility for MPIDE.
  Improve I/O feedback signals.
  Allocate additional signals for future expansion.
  Add more cases where feedback signals are used.
  Improve error handling
  Add additional text and I/O feedback in error cases
* Contributors: Jeremy Zoss, Maarten de Vries, Sachin Chitta, Shaun Edwards, Ted Miller, Thiago de Freitas Oliveira Araujo, gavanderhoorn, thiagodefreitas

0.3.3 (2014-02-07)
------------------
* No changes

0.3.2 (2014-01-31)
------------------
* No changes

0.3.1 (2014-01-30)
------------------
* Synchronized versions for bloom release
* driver: move DEPENDS to CATKIN_DEPENDS. Fix `#24 <https://github.com/shaun-edwards/motoman/issues/24>`_.
* driver: link against catkin_LIBRARIES. Fix `#23 <https://github.com/shaun-edwards/motoman/issues/23>`_.
* driver: avoid hardcoded python path. Fix `#19 <https://github.com/shaun-edwards/motoman/issues/19>`_.
* Update move_to_joint.py
* Add proper install targets to driver pkg.
  This fixes `#10 <https://github.com/shaun-edwards/motoman/issues/10>`_.
* Added binaries of motoplus driver.  These can be directly loaded on the controller
* Added controller specific INFORM files
* Commiting motoplus changes required to support DX100 using new incremental motion interface
* Renamed fs100 package to motoman_driver.  The new package now contains drivers for all controllers.  Package name reflects new naming convention
* Contributors: Shaun Edwards, Thomas Timm Andersen, gavanderhoorn
