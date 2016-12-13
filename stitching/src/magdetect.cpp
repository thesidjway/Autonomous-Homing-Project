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
#define cannythresh 100
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
RNG rng(12345);

int countEdges(Mat * edgesrc)
{
  Mat src_gray= *edgesrc;
  Mat canny_output;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Canny( src_gray, canny_output, cannythresh, cannythresh*2, 3 );
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<vector<Point>> contours_poly( contours.size() );


}
void detectFeatures()
{

  for (int iter=0;iter<NUMIMAGES;iter++)
  {
    Mat element = getStructuringElement( MORPH_CROSS,Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

    Mat srcred1,srcred2,srcred,srcwhite,dilatedRed,dilatedWhite,srcblue,srcgreen,dilatedBlue,dilatedGreen;
    Mat boundaryGB, boundaryRW, boundaryGW, boundaryBW, boundaryRG, boundaryBR;
    Mat detectionImg=images[iter];
    Mat detectionImgHSV;
    cvtColor(detectionImg,detectionImgHSV,CV_BGR2HSV);

    inRange(detectionImgHSV,Scalar(0,70,70),Scalar(15,255,255),srcred1);
    inRange(detectionImgHSV,Scalar(165,70,70),Scalar(180,255,255),srcred2);
    bitwise_or(srcred1,srcred2,srcred);
    inRange(detectionImg,Scalar(140,140,140),Scalar(255,255,255),srcwhite);
    inRange(detectionImgHSV,Scalar(40,70,70),Scalar(70,255,255),srcgreen);
    inRange(detectionImg,Scalar(85,70,70),Scalar(115,255,255),srcblue);

    dilate( srcwhite, dilatedWhite, element );
    dilate( srcred, dilatedRed, element );
    dilate( srcblue, dilatedBlue, element );
    dilate( srcgreen, dilatedGreen, element );

    bitwise_and(dilatedGreen,dilatedBlue,boundaryGB);
    imshow("BLUEGREEN",boundaryGB);

    bitwise_and(dilatedGreen,dilatedWhite,boundaryGW);
    imshow("GREENWHITE",boundaryGW);

    bitwise_and(dilatedWhite,dilatedBlue,boundaryBW);
    imshow("BLUEWHITE",boundaryBW);

    bitwise_and(dilatedRed,dilatedWhite,boundaryRW);
    imshow("REDWHITE",boundaryRW);

    bitwise_and(dilatedRed,dilatedGreen,boundaryRG);
    imshow("REDGREEN",boundaryRG);

    bitwise_and(dilatedRed,dilatedBlue,boundaryBR);
    imshow("BLUERED",boundaryBR);

    vector<vector<Point>> contoursGB, contoursGW, contoursBW, contoursRW, contoursRG, contoursBR;
    vector<Vec4i> hierarchyGB, hierarchyGW, hierarchyBW, hierarchyRW, hierarchyRG, hierarchyBR;

    findContours( boundaryGB, contoursGB, hierarchyGB, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( boundaryGW, contoursGW, hierarchyGW, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( boundaryBW, contoursBW, hierarchyBW, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( boundaryRW, contoursRW, hierarchyRW, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( boundaryRG, contoursRG, hierarchyRG, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( boundaryBR, contoursBR, hierarchyBR, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    
    waitKey(33);
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