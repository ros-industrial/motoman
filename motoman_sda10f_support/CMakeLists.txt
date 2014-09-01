
cmake_minimum_required(VERSION 2.8.3)

project(motoman_sda10f_support)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED rostest)
endif()

catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/launch_test.xml)

foreach(dir config launch meshes urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
