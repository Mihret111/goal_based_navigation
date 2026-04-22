cmake_minimum_required(VERSION 3.8)
project(nav_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)        # Core ROS2 C++ API
find_package(rclcpp_action REQUIRED)      # Needed for action client/server in C++
find_package(geometry_msgs REQUIRED)      #Needed for twist, pose kind of things
find_package(std_msgs REQUIRED)      #Needed for string, empty kind of thngs
find_package(nav_msgs REQUIRED)      #Needed for odometry kind of thngs
find_package(tf2 REQUIRED)           # frame reasoning
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)    # needed because the action client and server will be components
find_package(nav_interfaces REQUIRED)      # depends on the custom action defn

# -------------------------------
include_directories(include)

add_library(nav_action_server_component SHARED
  src/nav_action_server_component.cpp
)

ament_target_dependencies(nav_action_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rt2_nav_interfaces
)

rclcpp_components_register_nodes(nav_action_server_component
  "nav_system::NavActionServerComponent"
)

install(
  TARGETS nav_action_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#----------------------------------
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
