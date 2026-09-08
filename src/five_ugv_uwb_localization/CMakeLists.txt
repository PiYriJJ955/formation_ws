cmake_minimum_required(VERSION 3.0.2)
project(five_ugv_uwb_localization)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs nlink_parser)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/robust_range_ekf.py
  scripts/calibrate_uwb.py
  scripts/robust_uwb_localizer.py
  scripts/offline_logger.py
  scripts/formation_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
