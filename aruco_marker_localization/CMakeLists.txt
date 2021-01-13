cmake_minimum_required(VERSION 3.0.2)
project(aruco_marker_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  roslaunch
)

add_message_files(
  FILES
  MarkerPoseArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES aruco_marker_tracking
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/image_from_webcam_node.py
  scripts/pose_estimation_node.py
  scripts/test_pose_visualisation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
