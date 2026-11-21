cmake_minimum_required(VERSION 3.22)
project(vehicle_manipulator_task_priority LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(behaviortree_cpp REQUIRED)
find_package(vehicle_manipulator_task_priority_interfaces REQUIRED)

include(cmake/learn_manipulator_control.cmake)  # target: manipulator_control

# ---- ROS-free core: uArm kinematics, the mobile-manipulator model, the pick-and-place controller logic
add_library(vmtp_core STATIC
    src/swift_pro.cpp
    src/mobile_swift_pro.cpp
    src/controller.cpp)
target_include_directories(vmtp_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(vmtp_core PUBLIC manipulator_control Eigen3::Eigen)

# ---- ROS 2 nodes
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node vmtp_core)
ament_target_dependencies(controller_node rclcpp rclcpp_action geometry_msgs nav_msgs sensor_msgs std_msgs tf2
    vehicle_manipulator_task_priority_interfaces)

add_executable(aruco_node src/aruco_node.cpp)
target_link_libraries(aruco_node ${OpenCV_LIBS})
ament_target_dependencies(aruco_node rclcpp geometry_msgs sensor_msgs tf2 tf2_ros tf2_eigen cv_bridge)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node rclcpp rclcpp_action geometry_msgs std_msgs behaviortree_cpp
    vehicle_manipulator_task_priority_interfaces)
target_link_libraries(mission_node Eigen3::Eigen)

install(TARGETS controller_node aruco_node mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/evaluate_run.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_kinematics test/test_kinematics.cpp)
    target_link_libraries(test_kinematics vmtp_core)
    ament_add_gtest(test_controller test/test_controller.cpp)
    target_link_libraries(test_controller vmtp_core)
endif()

ament_package()
