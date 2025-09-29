cmake_minimum_required(VERSION 3.22)

# Suppress CMP0002 warning 
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(jaka_zu5_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()