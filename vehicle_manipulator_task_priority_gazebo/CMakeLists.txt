cmake_minimum_required(VERSION 3.22)
project(vehicle_manipulator_task_priority_gazebo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_markers.py DESTINATION lib/${PROJECT_NAME})

ament_package()
