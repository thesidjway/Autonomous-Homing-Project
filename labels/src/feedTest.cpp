#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

static const std::string OPENCV_WINDOW = "Image window";
int i=0;
void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{
    i++;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream

    // Update GUI Window
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    std::stringstream newss;
    newss << i;

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    if(i<20)
    cv::imwrite("out"+newss.str()+".png",cv_ptr->image,compression_params);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "stitch_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 10, imageCallback);
  

  ros::spin();

  return 0;
}