cmake_minimum_required(VERSION 3.8)
project(r2_control)

find_package(ament_cmake REQUIRED)

# Configuration-only package: no compiled targets.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
