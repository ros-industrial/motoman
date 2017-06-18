cmake_minimum_required(VERSION 2.8.3)
project(kinematics_base_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  rostest
  rosunit
  moveit_ros_planning
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_rostest_gtest(${PROJECT_NAME}_utest launch/test_kinematics_plugin.launch src/test_kinematics_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_utest ${catkin_LIBRARIES} ${boost_LIBRARIES})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinematics_base_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
