
cmake_minimum_required(VERSION 2.8.3)
project(second_project)

## Find catkin and any catkin packages
find_package(
  catkin 
  REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
)

## Build the executable
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(oodmetry_node src/oodmetry_node.cpp)
add_dependencies(oodmetry_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(oodmetry_node ${catkin_LIBRARIES})



