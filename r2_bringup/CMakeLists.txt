cmake_minimum_required(VERSION 3.8)
project(r2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Integration checks across the description, control and gazebo packages.
  # These packages hold no application logic; what breaks is the seams.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(configuration_consistency
    test/test_configuration_consistency.py
    TIMEOUT 300
  )
endif()

ament_package()
