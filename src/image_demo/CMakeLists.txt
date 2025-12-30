cmake_minimum_required(VERSION 3.8)
project(image_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
find_package(cv_bridge REQUIRED)

# add_executable(image_publisher src/image_publisher.cpp)
# add_executable(image_subscriber src/image_subscriber.cpp)

# ament_target_dependencies(image_publisher rclcpp sensor_msgs)
# ament_target_dependencies(image_subscriber rclcpp sensor_msgs)

# install(TARGETS
#   image_publisher
#   image_subscriber
#   DESTINATION lib/${PROJECT_NAME}
# )

############
# Component
add_library(image_publish_component SHARED src/img_pub_component.cpp)
ament_target_dependencies(
  image_publish_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
rclcpp_components_register_nodes(image_publish_component "image_demo::ImagePublisher")

add_library(image_subscribe_component SHARED src/img_sub_component.cpp)
ament_target_dependencies(
  image_subscribe_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)
rclcpp_components_register_nodes(image_subscribe_component "image_demo::ImageSubscriber")

install(
  TARGETS
    image_publish_component
    image_subscribe_component
  LIBRARY DESTINATION lib
)

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
