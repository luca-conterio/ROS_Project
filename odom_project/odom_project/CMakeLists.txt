
cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(
  catkin 
  REQUIRED COMPONENTS 
  roscpp
  dynamic_reconfigure 
  std_msgs 
  nav_msgs 
  message_generation
  message_filters
  tf
)

add_message_files( ## add custom message files
  FILES
  floatStamped.msg
  odometryMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options( ## must link th dynamic reconfigure file
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package (
  CATKIN_DEPENDS 
  message_runtime 
  std_msgs
  message_filters
)

## Build the executable
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(node src/node.cpp)
add_dependencies(node ${catkin_EXPORTED_TARGETS})
target_link_libraries(node ${catkin_LIBRARIES})



