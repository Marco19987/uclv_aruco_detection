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
find_package(rclcpp_components REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(uclv_aruco_detection_interfaces REQUIRED)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS})


# Add the executable
add_executable(pose_conversion_node src/pose_conversion.cpp)
add_executable(aruco_marker_array_pub_test_node src/aruco_marker_array_pub_test.cpp)
add_executable(multiple_poses_elaboration_node src/multiple_poses_elaboration.cpp)
add_executable(pose_conversion_server_node src/pose_conversion_server.cpp)

add_library(pose_conversion_server_component SHARED src/pose_conversion_server_composable.cpp)
ament_target_dependencies(pose_conversion_server_component rclcpp aruco_msgs geometry_msgs tf2 yaml-cpp tf2_geometry_msgs uclv_aruco_detection_interfaces rclcpp_components)
target_link_libraries(pose_conversion_server_component yaml-cpp)
rclcpp_components_register_node(
  pose_conversion_server_component
    PLUGIN "uclv_aruco_detection::PoseConversionServer"
    EXECUTABLE composable_pose_conversion_server
)

# Link libraries
ament_target_dependencies(pose_conversion_node rclcpp aruco_msgs geometry_msgs tf2 yaml-cpp tf2_geometry_msgs)
target_link_libraries(pose_conversion_node yaml-cpp)

ament_target_dependencies(aruco_marker_array_pub_test_node rclcpp aruco_msgs geometry_msgs)

ament_target_dependencies(multiple_poses_elaboration_node rclcpp aruco_msgs geometry_msgs tf2 yaml-cpp tf2_geometry_msgs)
target_link_libraries(multiple_poses_elaboration_node yaml-cpp)

ament_target_dependencies(pose_conversion_server_node rclcpp aruco_msgs geometry_msgs tf2 yaml-cpp tf2_geometry_msgs uclv_aruco_detection_interfaces)
target_link_libraries(pose_conversion_server_node yaml-cpp)


# Install targets
install(TARGETS
  pose_conversion_node
  aruco_marker_array_pub_test_node
  multiple_poses_elaboration_node
  pose_conversion_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_pose_conversion_server_component)
install(TARGETS
  pose_conversion_server_component
  EXPORT export_pose_conversion_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

  add_executable(test_server_client test/test_server_client.cpp)
  ament_target_dependencies(test_server_client rclcpp uclv_aruco_detection_interfaces)

  install(TARGETS
    test_server_client
    DESTINATION lib/${PROJECT_NAME}
  )



endif()

ament_package()
