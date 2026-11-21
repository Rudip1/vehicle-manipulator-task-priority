# The kinematic-control library (task objects, recursive task-priority solver, set-based inequality tasks,
# unicycle kinematics) from learn-manipulator-control, pinned to a commit. Only the C++ library target
# `manipulator_control` is built: no tests, examples or Python module.
#
# Offline or with a local checkout:
#   colcon build --cmake-args -DFETCHCONTENT_SOURCE_DIR_LEARN_MANIPULATOR_CONTROL=/path/to/learn-manipulator-control
include(FetchContent)
set(MC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MC_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MC_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    learn_manipulator_control
    GIT_REPOSITORY https://github.com/Rudip1/learn-manipulator-control.git
    GIT_TAG 199e4f56ce42dc0305335da2e86336a749f68f43
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(learn_manipulator_control)
