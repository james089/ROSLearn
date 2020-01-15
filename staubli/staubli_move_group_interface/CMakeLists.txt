cmake_minimum_required(VERSION 2.8.3)
project(staubli_move_group_interface)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    interactive_markers
    moveit_core 
    moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_perception
	pluginlib
	geometric_shapes
	pcl_ros
	pcl_conversions
	rosbag
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(staubli_move_group_interface src/staubli_move_group_interface.cpp)

target_link_libraries(staubli_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
