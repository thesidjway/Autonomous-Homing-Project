

cmake_minimum_required(VERSION 2.8.3)

project(homing_planner)

ADD_DEFINITIONS(-std=c++0x )



## Find catkin and any catkin packages



find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	sensor_msgs
	genmsg 
	message_generation 
	ardrone_autonomy
)



## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs ardrone_autonomy
	)
	


include_directories(
    	include
	${catkin_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
	${rospy_INCLUDE_DIRS}
)






add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner roscpp rospy sensor_msgs std_msgs geometry_msgs ardrone_autonomy)



