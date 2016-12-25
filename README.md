# Autonomous-Homing-Project
Codes, Papers and resources for the autonomous homing project

###Resources
1. [ROS driver for AR Drone](https://github.com/AutonomyLab/ardrone_autonomy)
2. [Up and flying with the AR.Drone and ROS: Getting started](http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-getting-started/)
3. [Driver Documentation](https://ardrone-autonomy.readthedocs.io/en/latest/)
4. [Converting ROS formats to OpenCV](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
5. [Stitching 2 images OpenCV](https://ramsrigoutham.com/2012/11/22/panorama-image-stitching-in-opencv)
6. [Convex Hull Function usage](http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/hull/hull.html)
7. [Blob Detector Parameters](https://www.learnopencv.com/blob-detection-using-opencv-python-c/)

###Requirements
1. ARDrone v2
2. ROS Indigo
3. OpenCV 2.4.11+ with nonfree enabled

###Downloading, Compiling and Running the Code
1. git clone this repository, recursive flag enabled into src of a ros workspace.
2. catkin_make in the root of the workspace and source it.
3. roslaunch teleop_homing homing.launch.




