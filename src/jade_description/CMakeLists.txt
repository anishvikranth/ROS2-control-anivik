cmake_minimum_required(VERSION 3.8)
project(jade_description)

find_package(ament_cmake REQUIRED)

# Install resource directories to share/jade_description
install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()