cmake_minimum_required(VERSION 2.8.3)
project(image_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core Gui Widgets Test Concurrent OpenGL)

add_action_files(
  DIRECTORY action
  FILES Image.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  actionlib 
  actionlib_msgs 
  roscpp 
  cv_bridge 
  image_transport 
  sensor_msgs
  pcl_conversions
  pcl_ros
)

###########
## Build ##
###########

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    )

# server 
add_executable(server src/server.cpp)

target_link_libraries(
  server
  ${catkin_LIBRARIES}
  #${OpenCV_LIBRARIES}
)

add_dependencies(
  server ${image_server_EXPORTED_TARGETS}
)

# client 
add_executable(client src/client.cpp)

target_link_libraries( 
  client
  ${catkin_LIBRARIES}
)

add_dependencies(
  client ${image_server_EXPORTED_TARGETS}
)


# ImageConverter 
add_executable(image_converter src/image_converter.cpp)

target_link_libraries( 
  image_converter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(
  image_converter ${image_server_EXPORTED_TARGETS}
)

# Image Display 
add_executable(image_show src/image_show.cpp)

target_link_libraries( 
  image_show
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(
  image_converter ${image_server_EXPORTED_TARGETS}
)

# PCL 
add_executable(pcl_process src/pcl_process.cpp)

target_link_libraries(pcl_process ${catkin_LIBRARIES})

# Track circle 
add_executable(track_ball src/track_ball.cpp)

target_link_libraries(track_ball 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_dependencies(
  track_ball ${image_server_EXPORTED_TARGETS}
)

# Track ellipse 
add_executable(track_ellipse src/track_ellipse.cpp)

target_link_libraries(track_ellipse 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_dependencies(
  track_ellipse ${image_server_EXPORTED_TARGETS}
)