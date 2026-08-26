cmake_minimum_required(VERSION 3.8)
project(r2_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Motion-validation tooling. Not part of the robot's runtime.
# One install() per script on purpose: ament_cmake_symlink_install has been
# observed to install only the first entry of a multi-file PROGRAMS list on a
# fresh configure, which silently leaves a tool missing from `ros2 run`.
install(PROGRAMS scripts/holonomic_check.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/odometry_check.py  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/stop_check.py      DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
