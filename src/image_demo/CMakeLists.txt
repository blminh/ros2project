cmake_minimum_required(VERSION 3.8)
project(image_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(shm_interfaces REQUIRED)
find_package(utils REQUIRED)

add_executable(shm_image_publisher src/image_publisher.cpp)
ament_target_dependencies(shm_image_publisher rclcpp
  shm_interfaces
  sensor_msgs
  OpenCV
  utils
)

add_executable(shm_image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(shm_image_subscriber rclcpp
  shm_interfaces
  sensor_msgs
  # OpenCV
  utils
)

install(TARGETS
  shm_image_publisher
  shm_image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images)

ament_package()
