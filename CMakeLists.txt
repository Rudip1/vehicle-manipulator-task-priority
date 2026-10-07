cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator_tp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  CustomPoseStamped.msg
)

add_service_files(
  FILES
  Goalreached.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs rospy sensor_msgs std_msgs
)

# The Python nodes are not installed: they are run from the source tree
# (catkin devel space) because they import the sibling `utils` package and
# `Manipulator_task` module relative to their own location.
