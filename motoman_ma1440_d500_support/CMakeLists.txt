cmake_minimum_required(VERSION 2.8.3)

project(motoman_ma1440_d500_support)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/launch_test.xml)
endif()

install(DIRECTORY config launch test urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
