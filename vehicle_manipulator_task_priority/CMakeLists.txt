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

include(cmake/learn_manipulator_control.cmake)  # target: manipulator_control

# ---- ROS-free core: uArm kinematics, the mobile-manipulator model, the pick-and-place controller logic
add_library(vmtp_core STATIC
    src/swift_pro.cpp
    src/mobile_swift_pro.cpp
    src/controller.cpp)
target_include_directories(vmtp_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(vmtp_core PUBLIC manipulator_control Eigen3::Eigen)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_kinematics test/test_kinematics.cpp)
    target_link_libraries(test_kinematics vmtp_core)
    ament_add_gtest(test_controller test/test_controller.cpp)
    target_link_libraries(test_controller vmtp_core)
endif()

ament_package()
