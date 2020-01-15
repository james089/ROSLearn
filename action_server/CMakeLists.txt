cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs 
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server 
  ${action_server_EXPORTED_TARGETS}
)

add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries( 
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client
  ${action_server_EXPORTED_TARGETS}
)