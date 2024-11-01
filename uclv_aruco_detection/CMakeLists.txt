cmake_minimum_required(VERSION 3.8)
project(uclv_aruco_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS})


# Add the executable
add_executable(pose_conversion_node src/pose_conversion.cpp)
add_executable(aruco_marker_array_pub_test_node src/aruco_marker_array_pub_test.cpp)


# Link libraries
ament_target_dependencies(pose_conversion_node rclcpp aruco_msgs geometry_msgs tf2 yaml-cpp tf2_geometry_msgs)
target_link_libraries(pose_conversion_node yaml-cpp)

ament_target_dependencies(aruco_marker_array_pub_test_node rclcpp aruco_msgs geometry_msgs)



# Install targets
install(TARGETS
  pose_conversion_node
  aruco_marker_array_pub_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

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
