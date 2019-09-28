
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
add_executable(node src/node.cpp)
add_dependencies(node ${catkin_EXPORTED_TARGETS})
target_link_libraries(node ${catkin_LIBRARIES})



