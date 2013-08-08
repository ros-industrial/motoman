# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dx100)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs control_msgs sensor_msgs simple_message urdf industrial_msgs industrial_robot_client industrial_utils)
find_package(Boost REQUIRED COMPONENTS system thread)

#######################################
## Adding directories and definitions #
#######################################
include_directories(include motoplus inform ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
#add_definitions(-MOTOPLUS=1)
add_definitions(-DROS=1)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## Declare artifacts to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs control_msgs sensor_msgs simple_message urdf industrial_msgs industrial_robot_client
    CATKIN_DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES 
)

###########
## Build ##
###########
add_library(motoman_lib 
	src/utils.cpp
	src/trajectory_job.cpp
	src/ros_conversion.cpp
	)

add_executable(system_interface 
  src/system_interface.cpp)
  
target_link_libraries(system_interface  
  motoman_lib
  industrial_robot_client  
  simple_message
  ${catkin_LIBRARIES})


add_executable(dx100_robot_state 
  src/robot_state.cpp)
target_link_libraries(dx100_robot_state 
  motoman_lib
  industrial_robot_client  
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(dx100_robot_state PROPERTIES OUTPUT_NAME robot_state PREFIX "")


add_executable(dx100_motion_interface
  src/motion_interface.cpp
  src/joint_trajectory_handler.cpp)
target_link_libraries(dx100_motion_interface
  motoman_lib
  industrial_robot_client  
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(dx100_motion_interface PROPERTIES OUTPUT_NAME motion_interfacee PREFIX "")


add_executable(dx100_motion_download_interface
  src/motion_download_interface.cpp
  src/joint_trajectory_downloader.cpp
  )
target_link_libraries(dx100_motion_download_interface 
  motoman_lib
  industrial_robot_client  
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(dx100_motion_download_interface PROPERTIES OUTPUT_NAME motion_download_interface PREFIX "")


add_executable(dx100_joint_trajectory_action
  src/utils.cpp 
  src/joint_trajectory_action.cpp)
target_link_libraries(dx100_joint_trajectory_action ${catkin_LIBRARIES})
set_target_properties(dx100_joint_trajectory_action PROPERTIES OUTPUT_NAME joint_trajectory_action PREFIX "")

catkin_add_gtest(dx100_utest test/utest.cpp)
target_link_libraries(dx100_utest 
  motoman_lib 
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(dx100_utest PROPERTIES OUTPUT_NAME dx100 PREFIX "")
