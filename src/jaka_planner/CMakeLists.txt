cmake_minimum_required(VERSION 3.8)
project(jaka_planner)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)


# Optionally find additional dependencies
# find_package(Boost REQUIRED)

# Declare executable targets
add_executable(moveit_server src/moveit_server.cpp)
add_executable(moveit_test src/moveit_test.cpp)

# Link libraries
ament_target_dependencies(moveit_server
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  moveit_msgs
  action_msgs
  control_msgs
  moveit_ros_planning_interface
)
ament_target_dependencies(moveit_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  jaka_msgs
  moveit_msgs
  action_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_core
)

# include_directories(
#   include
#   ${moveit_msgs_INCLUDE_DIRS}  # Include moveit_msgs headers
#   ${moveit_core_INCLUDE_DIRS}           # Make sure this is added
#   ${moveit_ros_planning_interface_INCLUDE_DIRS}
#   ${moveit_visual_tools_INCLUDE_DIRS}
#   # $ENV{HOME}/moveit-ros2/install/moveit_core/include
# )

target_link_libraries(moveit_test
  ${moveit_msgs_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES}
)

# For including directories in ROS 2
target_include_directories(moveit_server
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(moveit_test
  PRIVATE
    ${moveit_core_INCLUDE_DIRS}  # Include the moveit_core include directory
    $ENV{HOME}/moveit-ros2/install/moveit_core/include
    ${moveit_ros_planning_interface_INCLUDE_DIRS}
    ${moveit_visual_tools_INCLUDE_DIRS}
)

# Specify where to find external libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add the directory to the search path
link_directories(${LIBRARY_DIR})

# Find the library and link it properly
target_link_libraries(moveit_server ${LIBRARY_DIR}/libjakaAPI.so)
target_link_libraries(moveit_test ${LIBRARY_DIR}/libjakaAPI.so)

# Install executables
install(TARGETS
  moveit_server
  moveit_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()



