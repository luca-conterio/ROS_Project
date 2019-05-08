cmake_minimum_required(VERSION 2.8.3)
project(custom_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(
 FILES
 Num.msg ## include the name of the custom message file (implicit to search in the msg folder)

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs  ## in the custom message we use a standard message
 )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pub_custom src/pub.cpp)
target_link_libraries(pub_custom ${catkin_LIBRARIES})

add_executable(sub_custom src/sub.cpp)
target_link_libraries(sub_custom ${catkin_LIBRARIES})

