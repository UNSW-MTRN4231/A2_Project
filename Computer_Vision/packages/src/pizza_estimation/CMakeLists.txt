cmake_minimum_required(VERSION 3.8)
project(pizza_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# This line is not needed as yolov8_ros is a Python package
# find_package(yolov8_ros REQUIRED)

# Add the executable
add_executable(pizza_estimation src/pizza_estimation.cpp)
ament_target_dependencies(pizza_estimation rclcpp yolov8_msgs sensor_msgs OpenCV cv_bridge)

# Install the executable
install(TARGETS
  pizza_estimation
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(${rclcpp_INCLUDE_DIRS} ${yolov8_msgs_INCLUDE_DIRS})

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
