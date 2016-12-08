#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="Gray Image";

void process(const sensor_msgs::ImageConstPtr& cam_image){
   sensor_msgs::CvBridge bridge; 
   Mat img_rgb = bridge.imgMsgToCv(cam_image,"bgr8"); 
   Mat img_gray;

   cvtColor(img_rgb,img_gray,CV_RGB2GRAY);

   imshow(WINDOW,img_rgb);
   imshow(WINDOW2,img_gray);
   cvWaitKey(1);
}

int main(int argc, char **argv){
  ros::init(argc,argv,"Display_Images");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);

  cv::namedWindow(WINDOW);
  cv::namedWindow(WINDOW2);

  cv::destroyWindow(WINDOW);
  cv::destroyWindow(WINDOW2);

  ros::spin();
  return 0;
}