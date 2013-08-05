# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(fs100)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs control_msgs sensor_msgs urdf industrial_msgs industrial_robot_client industrial_utils simple_message)
find_package(Boost REQUIRED COMPONENTS system thread)

#######################################
## Adding directories and definitions #
#######################################
include_directories(include MotoPlus inform ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
add_definitions(-DROS=1)

catkin_package(
    DEPENDS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs control_msgs sensor_msgs simple_message urdf industrial_msgs industrial_robot_client simple_message
    CATKIN_DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES 
)

###########
## Build ##
###########
# NOTE: FS100 uses opposite byte-ordering from most i386-based PCs

# Changes below made for DX100

# Simple message library
add_library(fs100_simple_message
  src/simple_message/motoman_motion_ctrl.cpp
  src/simple_message/motoman_motion_ctrl_message.cpp
  src/simple_message/motoman_motion_reply.cpp
  src/simple_message/motoman_motion_reply_message.cpp)
target_link_libraries(fs100_simple_message 
  simple_message)

# Industrial client library
add_library(fs100_industrial_robot_client
                     src/industrial_robot_client/joint_relay_handler.cpp
                     src/industrial_robot_client/joint_feedback_relay_handler.cpp
                     src/industrial_robot_client/robot_state_interface.cpp
                     src/industrial_robot_client/joint_trajectory_interface.cpp
                     src/industrial_robot_client/joint_trajectory_streamer.cpp
                     )
target_link_libraries(fs100_industrial_robot_client 
  industrial_robot_client
  industrial_utils)

# Robot state node
add_executable(fs100_robot_state 
  src/fs100_robot_state_node.cpp)
target_link_libraries(fs100_robot_state	
  fs100_simple_message
  fs100_industrial_robot_client 
  industrial_utils)
set_target_properties(fs100_robot_state 
  PROPERTIES OUTPUT_NAME robot_state 
  PREFIX "")
              
# Motion streaming node                      
add_executable(fs100_motion_streaming_interface
	src/fs100_joint_streaming_node.cpp
	src/fs100_joint_trajectory_streamer.cpp
	src/fs100_motion_ctrl.cpp)
target_link_libraries(fs100_motion_streaming_interface 
  fs100_simple_message 
  fs100_industrial_robot_client 
  industrial_utils)
set_target_properties(fs100_motion_streaming_interface 
  PROPERTIES OUTPUT_NAME motion_streaming_interface 
  PREFIX "")


