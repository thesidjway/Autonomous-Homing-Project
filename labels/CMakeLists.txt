

cmake_minimum_required(VERSION 2.8.3)

project(labels)

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
	ardrone_autonomy
)

find_package( OpenCV REQUIRED)

add_message_files(
  	FILES
	LabelAngles.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)



## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs std_msgs sensor_msgs image_transport cv_bridge ardrone_autonomy
	)
	



include_directories(
    	include
	${catkin_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
	${rospy_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)





add_executable(labeldetect src/labeldetect.cpp)
target_link_libraries(labeldetect ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(labeldetect roscpp rospy sensor_msgs std_msgs geometry_msgs image_transport cv_bridge ardrone_autonomy)



