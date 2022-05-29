cmake_minimum_required(VERSION 3.5)
project(ros2-components)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

# BUILD
# Add talker component
add_library(component_talker
  SHARED
  src/component_talker.cpp
)
ament_target_dependencies(component_talker
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(component_talker
 PLUGIN "component::component_talker"
 EXECUTABLE component_talker_node
)

# Add listener component
add_library(component_listener
  SHARED
  src/component_listener.cpp
)
ament_target_dependencies(component_listener
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(component_listener
 PLUGIN "component::component_listener"
 EXECUTABLE component_listener_node
)

# Add lifecycle talker component
add_library(lifecycle_component_talker
  SHARED
  src/lifecycle_component_talker.cpp
)
ament_target_dependencies(lifecycle_component_talker
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
)
rclcpp_components_register_node(lifecycle_component_talker
 PLUGIN "lifecycle_component::component_talker"
 EXECUTABLE lifecycle_component_talker_node
)

# INSTALL
install(TARGETS
  component_talker
  component_listener
  lifecycle_component_talker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  tmux/start_lifecycle_demo.bash
  DESTINATION lib/${PROJECT_NAME}
)

# TEST
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
