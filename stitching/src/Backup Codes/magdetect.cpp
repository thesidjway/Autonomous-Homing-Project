//Code to Detect features in a 360 yaw span.
//Siddharth S Jha

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
#define POLYDP 0.001

using namespace cv;
using namespace std;

int distx,disty;

std::mutex completeLock,currAngleLock;

Mat images[NUMIMAGES];
int currAngle;
Mat currImage,stitchedImage;
int complete=0;
Mat img_1,img_2,prevframe;

int countEdges(Mat * edgesrc)
{
  Mat hull= *edgesrc;
}



void detectFeatures()
{

  for (int iter=0;iter<NUMIMAGES;iter++)
  {
    Mat element = getStructuringElement( MORPH_CROSS,Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

    Mat srcred1,srcred2,srcred,srcwhite,dilatedRed,dilatedWhite,srcblue,srcgreen,dilatedBlue,dilatedGreen;
    Mat boundary[6];//boundaryGB, boundaryRW, boundaryGW, boundaryBW, boundaryRG, boundaryBR;
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

    bitwise_and(dilatedGreen,dilatedBlue,boundary[0]);
    imshow("BLUEGREEN",boundary[0]);

    bitwise_and(dilatedGreen,dilatedWhite,boundary[1]);
    imshow("GREENWHITE",boundary[1]);

    bitwise_and(dilatedWhite,dilatedBlue,boundary[2]);
    imshow("BLUEWHITE",boundary[2]);

    bitwise_and(dilatedRed,dilatedWhite,boundary[3]);
    imshow("REDWHITE",boundary[3]);

    bitwise_and(dilatedRed,dilatedGreen,boundary[4]);
    imshow("REDGREEN",boundary[4]);

    bitwise_and(dilatedRed,dilatedBlue,boundary[5]);
    imshow("BLUERED",boundary[5]);

    vector<vector<Point>> contours[6]; //contoursGB, contoursGW, contoursBW, contoursRW, contoursRG, contoursBR;
    vector<Vec4i> hierarchy[6]; //hierarchy GB, hierarchyGW, hierarchyBW, hierarchyRW, hierarchyRG, hierarchyBR;

    for (int clr=0;clr<6;clr++)
    {
       findContours( boundary[clr], contours[clr], hierarchy[clr], CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    }

    int largestArea[]={-1,-1,-1,-1,-1,-1};
    int largestContourIndex[]={-1,-1,-1,-1,-1,-1};

    for(int clr=0;clr<6;clr++)
    {
      for( int cnt = 0; cnt< contours[clr].size(); cnt++ ) 
      {
         double a=contourArea( contours[clr][cnt],false);  
         if(a>largestArea[clr])
         {
           largestArea[clr]=a;
           largestContourIndex[clr]=cnt;
         }
      }
    }

    vector<vector<Point>> hull[6];
    for (int clr=0;clr<6;clr++)
    {
      if(largestContourIndex[clr]!=-1)
      {
        vector<Point2f> hullPoints;
        convexHull(contours[largestContourIndex[clr]], hull[clr], false );
        approxPolyDP(hull[clr], hullPoints, POLYDP, true);
        cout<<hullPoints.size()<<endl;
      }
    }
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

    // currImage=cv_ptr->image;
    imshow("damn",cv_ptr->image);
    // if(!currImage.empty())
    // {   
    //   completeLock.lock();
    //   if(complete<NUMIMAGES)
    //   {
    //   currAngleLock.lock();
    //   int indexVal=floor((currAngle+180)/360.0*NUMIMAGES);
    //   if(images[indexVal].empty())
    //   {
    //     images[indexVal]=currImage;
    //     complete++;
    //   }
    //   currAngleLock.unlock();
    //   }
    //   completeLock.unlock();
    // }

    // if (complete==NUMIMAGES)
    // {
    //   complete++;
    //   detectFeatures();
    // }
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