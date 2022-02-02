cmake_minimum_required(VERSION 3.0.2)

project(motoman_hc10_support)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/roslaunch_test_hc10.xml)
  roslaunch_add_file_check(test/roslaunch_test_hc10dt.xml)
  roslaunch_add_file_check(test/roslaunch_test_hc10dt_b10.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
