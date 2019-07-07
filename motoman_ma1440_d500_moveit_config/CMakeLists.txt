cmake_minimum_required(VERSION 2.8.3)
project(motoman_ma1440_d500_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/dynamic_trajectory_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

