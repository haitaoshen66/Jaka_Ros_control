cmake_minimum_required(VERSION 3.8)
project(jaka_teleop)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # ✅ 新增
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# C++ node
add_executable(enable_node src/enable_node.cpp)
ament_target_dependencies(enable_node rclcpp std_msgs jaka_msgs)
ament_python_install_package(${PROJECT_NAME})
target_include_directories(enable_node
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

# 外部共享库
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
# Add the directory to the search path
link_directories(${LIBRARY_DIR})
target_link_libraries(enable_node ${LIBRARY_DIR}/libjakaAPI.so)

install(TARGETS enable_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本（脚本版本）
install(PROGRAMS
  jaka_teleop/base_node.py
  jaka_teleop/button_node.py
  jaka_teleop/lift_node.py
  jaka_teleop/gripper_node.py
  jaka_teleop/servo_control_node.py
  jaka_teleop/workspace_zone_node.py
  jaka_teleop/oculus_reader.py
  jaka_teleop/oculus_reader_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ 安装 resource
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# 测试部分
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
