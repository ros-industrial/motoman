cmake_minimum_required(VERSION 2.8.3)

project(motoman_ma1440_d500_support)

find_package(catkin REQUIRED)

catkin_package(catkin REQUIRED COMPONENTS
  motoman_ma1440_suppport
  motoman_motopos_d500_suppport
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/launch_test.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
