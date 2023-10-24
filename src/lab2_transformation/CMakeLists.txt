cmake_minimum_required(VERSION 3.5)
project(lab2_transformation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
find_package(geometry_msgs)
find_package(std_msgs)
find_package(visualization_msgs)

add_executable(dynamic_ball_transform src/dynamic_ball_transform.cpp)
target_include_directories(dynamic_ball_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_ball_transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dynamic_ball_transform
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "std_msgs"
  "visualization_msgs"
)

add_executable(static_camera_transform src/static_camera_transform.cpp)
target_include_directories(static_camera_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(static_camera_transform PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  static_camera_transform
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "std_msgs"
  "visualization_msgs"
)

install(TARGETS 
  dynamic_ball_transform 
  static_camera_transform
  DESTINATION lib/${PROJECT_NAME})

ament_package()