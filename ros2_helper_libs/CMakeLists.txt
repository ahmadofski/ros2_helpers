cmake_minimum_required(VERSION 3.8)
project(ros2_helper_libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

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


# geometry_msgs_utils library
add_library(geometry_msgs_utils
  src/geometry_msgs_utils.cpp)
ament_target_dependencies(geometry_msgs_utils
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)


# tf2_utils library
add_library(tf2_utils
  src/tf2_utils.cpp)
ament_target_dependencies(tf2_utils
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

# install libraries
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS geometry_msgs_utils tf2_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)


ament_package()
