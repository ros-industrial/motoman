cmake_minimum_required(VERSION 3.1.3)
project(motoman_hc10_moveit_config)

find_package(catkin REQUIRED COMPONENTS
rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS scripts/add_interferences.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
