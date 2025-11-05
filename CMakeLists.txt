cmake_minimum_required(VERSION 3.8)
project(leica_serial_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include)

# Create interface library
add_library(leica_interface SHARED
  src/total_station_interface.cpp
  src/serial_total_station_interface.cpp
)

target_link_libraries(leica_interface ${Boost_LIBRARIES})

ament_target_dependencies(leica_interface
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Create the main executable
add_executable(leica_serial_node
  src/leica_serial_node.cpp
)

target_link_libraries(leica_serial_node leica_interface)

ament_target_dependencies(leica_serial_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS
  leica_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  leica_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()