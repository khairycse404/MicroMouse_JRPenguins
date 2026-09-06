cmake_minimum_required(VERSION 3.8)
project(mouse_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
install(
  DIRECTORY urdf meshes rviz launch worlds config scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/maze_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
