cmake_minimum_required(VERSION 3.8)
project(r2_description)

find_package(ament_cmake REQUIRED)

# Description-only package: no compiled targets, just installed data.
install(
  DIRECTORY urdf launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
