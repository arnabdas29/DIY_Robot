cmake_minimum_required(VERSION 3.10)
project(lidar_car_controller)

set(CMAKE_CXX_STANDARD 14)

set(RPLIDAR_ROOT "$ENV{HOME}/rplidar_sdk")

# Header search paths
include_directories(
    ${RPLIDAR_ROOT}/sdk/include
    ${RPLIDAR_ROOT}/sdk/src
)

# Recursively find all C++ source files in sdk/src (including all unpacker modes)
file(GLOB_RECURSE SDK_SOURCES
    "${RPLIDAR_ROOT}/sdk/src/*.cpp"
)

# Exclude any Windows/macOS architecture files that might have been caught
list(FILTER SDK_SOURCES EXCLUDE REGEX ".*/arch/win32/.*")
list(FILTER SDK_SOURCES EXCLUDE REGEX ".*/arch/macOS/.*")

add_executable(lidar_controller lidar_controller.cpp ${SDK_SOURCES})
target_link_libraries(lidar_controller pthread)
