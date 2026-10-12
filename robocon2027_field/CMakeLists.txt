cmake_minimum_required(VERSION 3.8)
project(robocon2027_field)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
