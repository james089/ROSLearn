cmake_minimum_required(VERSION 2.8.3)

project(staubli)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
gazebo_ros
)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	gazebo_ros
	LIBRARIES ${PROJECT_NAME}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
