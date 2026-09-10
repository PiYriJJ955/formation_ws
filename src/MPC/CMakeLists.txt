cmake_minimum_required(VERSION 3.0.2)
project(MPC)

## Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ##qpoases_vendor
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nlink_parser
  Eigen3 REQUIRED
  message_generation
  serial
  tf
)

find_library(QPOASES_LIB qpOASES PATHS /usr/local/lib)

catkin_package(CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs LIBRARIES ${QPOASES_LIB})

add_executable(MPC src/MPC.cpp)
add_dependencies(MPC ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MPC ${catkin_LIBRARIES})
target_link_libraries(MPC ${catkin_LIBRARIES} ${QPOASES_LIB})

add_executable(MPCv2 src/MPCv2.cpp)
add_dependencies(MPCv2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MPCv2 ${catkin_LIBRARIES})
target_link_libraries(MPCv2 ${catkin_LIBRARIES} ${QPOASES_LIB})

add_executable(getabsolutedata src/getabsolutedata.cpp)
add_dependencies(getabsolutedata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(getabsolutedata ${catkin_LIBRARIES})
target_link_libraries(getabsolutedata ${catkin_LIBRARIES} ${QPOASES_LIB})

add_executable(UWBdata src/UWBdata.cpp)
add_dependencies(UWBdata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(UWBdata ${catkin_LIBRARIES})

add_executable(imu_fusion src/imu_fusion.cpp)
add_dependencies(imu_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_fusion ${catkin_LIBRARIES})

add_executable(absolute_location src/absolute_location.cpp)
add_dependencies(absolute_location ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(absolute_location ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} 
	##/opt/ros/melodic/include/qpoases_vendor
   /usr/local/include
   ~/wheeltec_robot/src/qpOASES-master/include)


