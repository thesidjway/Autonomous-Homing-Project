#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ardrone_autonomy/Navdata.h"
#include <mutex>

#define Y_THRESHOLD 15
#define DIVISION 3
#define NUMIMAGES 36
#define dilation_size 2

using namespace cv;
using namespace std;

int distx,disty;

std::mutex completeLock,currAngleLock;
static const std::string OPENCV_WINDOW = "Image window";

Mat images[NUMIMAGES];
int currAngle;
Mat currImage,stitchedImage;
int complete=0;
Mat img_1,img_2,prevframe;

int firstAngle=0;

void detectFeatures()
{

  for (int iter=0;iter<NUMIMAGES;iter++)
  {
    Mat element = getStructuringElement( MORPH_CROSS,Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
    Mat srcred1,srcred2,srcred,srcwhite,dilatedRed,dilatedWhite,boundary;
    Mat detectionImg=images[iter];
    Mat detectionImgHSV;

    cvtColor(detectionImg,detectionImgHSV,CV_BGR2HSV);
    inRange(detectionImgHSV,Scalar(0,70,30),Scalar(15,255,255),srcred1);
    inRange(detectionImgHSV,Scalar(165,70,30),Scalar(180,255,255),srcred2);

    bitwise_or(srcred1,srcred2,srcred);
    inRange(detectionImg,Scalar(140,140,140),Scalar(255,255,255),srcwhite);
    
    dilate( srcwhite, dilatedWhite, element );
    dilate( srcred, dilatedRed, element );

    bitwise_and(dilatedRed,dilatedWhite,boundary);

    imshow("AfterAND",boundary);

    waitKey(0);
  }

}
void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 
    completeLock.lock();
    cout<<"Complete: "<<complete<<endl;
    if(complete==NUMIMAGES)
    {
      cout<<""<<endl;
    }
    completeLock.unlock();
   // cout<<"IMAGE CAK"<<endl;
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

    currImage=cv_ptr->image;

    if(!currImage.empty())
    {   
      completeLock.lock();
      if(complete<NUMIMAGES)
      {
      currAngleLock.lock();
      int indexVal=floor((currAngle+180)/360.0*NUMIMAGES);
      if(images[indexVal].empty())
      {
        // if(complete==0)
        // {
        //     firstAngle=indexVal;
        // }
       // cout<<"first"<<endl;
        images[indexVal]=currImage;
        complete++;
      }
      currAngleLock.unlock();
      }
      completeLock.unlock();
    }

    if (complete==NUMIMAGES)
    {
      complete++;
      detectFeatures();
    }

    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
   // cv::waitKey(3);
}

void NavcallBack(const ardrone_autonomy::Navdata::ConstPtr& msg) 
{
  currAngleLock.lock();
 // cout<<"rotZ is"<< msg->rotZ <<endl;
    currAngle=msg->rotZ;
  currAngleLock.unlock();

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stitch_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 10, imageCallback);
  ros::Subscriber sub2 = nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 1, NavcallBack); 


  

  ros::spin();

  return 0;
}