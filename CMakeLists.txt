cmake_minimum_required(VERSION 3.16)
project(lidar_car_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(RPLIDAR_ROOT "$ENV{HOME}/rplidar_sdk")

include_directories(
  ${RPLIDAR_ROOT}/sdk/include
  ${RPLIDAR_ROOT}/sdk/src
)

file(GLOB_RECURSE SDK_SOURCES
  "${RPLIDAR_ROOT}/sdk/src/*.cpp"
)

list(FILTER SDK_SOURCES EXCLUDE REGEX ".*/arch/win32/.*")
list(FILTER SDK_SOURCES EXCLUDE REGEX ".*/arch/macOS/.*")

add_executable(lidar_controller_node
  lidar_controller.cpp
  ${SDK_SOURCES}
)

add_executable(esp32_bridge_node
  esp32_bridge_node.cpp
)

ament_target_dependencies(lidar_controller_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(esp32_bridge_node
  rclcpp
  geometry_msgs
)

target_link_libraries(lidar_controller_node pthread)

target_link_libraries(esp32_bridge_node pthread)

install(TARGETS lidar_controller_node esp32_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
