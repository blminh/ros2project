cmake_minimum_required(VERSION 3.8)
project(utils)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)

find_package(shm_interfaces REQUIRED)

add_library(utils SHARED
  src/conversions.cpp
)

target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(utils
  std_msgs
  sensor_msgs
  shm_interfaces
  OpenCV
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS utils
  EXPORT export_utils
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

ament_export_targets(export_utils)
ament_export_dependencies(std_msgs shm_interfaces)

ament_package()
