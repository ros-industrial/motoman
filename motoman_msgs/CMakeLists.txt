cmake_minimum_required(VERSION 2.8.3)
project(motoman_msgs)

find_package(catkin
  REQUIRED COMPONENTS
    industrial_msgs
    message_generation
    std_msgs
    trajectory_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    DynamicJointPoint.msg
    DynamicJointsGroup.msg
    DynamicJointState.msg
    DynamicJointTrajectory.msg
    DynamicJointTrajectoryFeedback.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    CmdJointTrajectoryEx.srv
)

generate_messages(
  DEPENDENCIES
    industrial_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
      industrial_msgs
      message_runtime
      std_msgs
      trajectory_msgs
)
