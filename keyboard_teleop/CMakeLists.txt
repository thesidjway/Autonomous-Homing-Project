cmake_minimum_required(VERSION 2.8)
project(teleop_homing)

find_package(catkin REQUIRED)
catkin_package()

#catkin_python_setup()


install(PROGRAMS src/control_homing.py  
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)