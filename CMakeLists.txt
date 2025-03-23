cmake_minimum_required(VERSION 3.0.2)
project(robot_4wd_dds115_base_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/base_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the ddsm115 Python module
install(DIRECTORY ddsm115/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)