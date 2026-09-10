cmake_minimum_required(VERSION 3.0.2)
project(five_ugv_formation_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/displacement_follower.py
  scripts/formation_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
