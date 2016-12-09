

cmake_minimum_required(VERSION 2.8.3)

project(stitcher)

ADD_DEFINITIONS(-std=c++0x )



## Find catkin and any catkin packages



find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	sensor_msgs
	genmsg 
	message_generation 
	image_transport
	cv_bridge
)

find_package( OpenCV REQUIRED)



## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs image_transport cv_bridge
	)
	


include_directories(
    	include
	${catkin_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
	${rospy_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)





add_executable(stitcher_node src/stitch.cpp)
target_link_libraries(stitcher_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(stitcher_node roscpp rospy sensor_msgs std_msgs geometry_msgs image_transport cv_bridge)





