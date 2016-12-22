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
#include <algorithm>
#include <labels/LabelAngles.h>

float angles[14]; //RWB,RGB,GWB,BRG,WRB,BGR,WBR,WRG,GBR,BWR,WGR,WGB,WBG,GWR
using namespace cv;
using namespace std;
float currAngle;
mutex currAngleLock, angleArrayLock;

labels::LabelAngles angleMessage;

float dist(Point2f a1,Point2f a2)
{
  return sqrt(pow(a2.x-a1.x,2)+pow(a2.y-a1.y,2));
}

int isnonzero(float* array, int arraysize)
{
    for(int i=0;i<arraysize;i++)
    {
        if (fabs(array[i])<0.001)
        {
            return 0;
        }
    }
    return 1;
}

void detectPoster(std::vector<KeyPoint> KR, std::vector<KeyPoint> KB, std::vector<KeyPoint> KG)
{
    if(KR.size()>0)
    {
        if(KB.size()>0)
        {
            if(KG.size()>0) //RGB
            {
                
                cout<<"BGR"<<endl;

            }
            else //RB
            {
                cout<<"BR"<<endl;

            }
        }
        else if(KG.size()>0) //RG
        {
            cout<<"GR"<<endl;

        }
        //cout<<"JUST RED."<<endl;
    }
    else if(KB.size()>0) //BG
    {
        if(KG.size()>0)
        {
            cout<<"BG"<<endl;
        }
        //cout<<"JUST BLUE."<<endl;
    }
    else
    {
        //cout<<"JUST GREEN OR NOTHING."<<endl;
    }
}
void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 

    Rect myROI(260,0,120,360);
    Mat srcred1,srcred2,srcred,srcdark,srcblue,srcgreen,srcwhite;
    Mat detectionImgFull,detectionImg;
    Mat detectionImgHSV,detectionImgGray;

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

    detectionImgFull=cv_ptr->image;
    detectionImg=detectionImgFull(myROI);

    cvtColor(detectionImg,detectionImgHSV,CV_BGR2HSV);
    cvtColor(detectionImg,detectionImgGray,CV_BGR2GRAY);

    inRange(detectionImgHSV,Scalar(0,40,40),Scalar(4,255,255),srcred1);
    inRange(detectionImgHSV,Scalar(170,40,40),Scalar(180,255,255),srcred2);
    bitwise_or(srcred1,srcred2,srcred);

    inRange(detectionImgHSV,Scalar(55,40,20),Scalar(82,255,255),srcgreen);
    inRange(detectionImgHSV,Scalar(108,40,20),Scalar(132,255,255),srcblue);

    inRange(detectionImgGray,0,99,srcdark);
    inRange(detectionImgGray,100,255,srcwhite);

    bitwise_and(srcdark,srcblue,srcblue);
    bitwise_and(srcdark,srcgreen,srcgreen);



    SimpleBlobDetector::Params params;

    params.filterByColor = true;
    params.blobColor = 255;
    params.minThreshold = 200;
    params.filterByArea = true;
    params.minArea = 50;
    params.filterByCircularity = false;
    params.filterByConvexity = true;
    params.minConvexity = 0.30;    
    params.filterByInertia = true;
    params.maxInertiaRatio = 0.8;

    SimpleBlobDetector detector(params);

    std::vector<KeyPoint> keypointsred,keypointsblue,keypointsgreen,keypointswhite;
    std::vector<Point2f> keypointsRB, keypointsRG, keypointsBG, keypointsBGR, keypointsall;

    detector.detect( srcred, keypointsred);
    detector.detect( srcblue, keypointsblue);
    detector.detect( srcgreen, keypointsgreen);

    Mat im_with_keypoints_red,im_with_keypoints_white,im_with_keypoints_green,im_with_keypoints_blue;

    Mat blankImg(360,120,CV_8UC3,Scalar(255,255,255));
    
    // drawKeypoints( detectionImg, keypointsred, im_with_keypoints_red, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // drawKeypoints( detectionImg, keypointsblue, im_with_keypoints_blue, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // drawKeypoints( detectionImg, keypointsgreen, im_with_keypoints_green, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    drawKeypoints( blankImg, keypointsred, blankImg, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( blankImg, keypointsblue, blankImg, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( blankImg, keypointsgreen, blankImg, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // imshow("keypointsR", im_with_keypoints_red);
    // imshow("keypointsG", im_with_keypoints_green);
    // imshow("keypointsB", im_with_keypoints_blue);
    imshow("Colour",blankImg);
    imshow("Original",detectionImg);


    detectPoster(keypointsred,keypointsgreen,keypointsblue);

    cv::waitKey(20);

    

}

void NavcallBack(const ardrone_autonomy::Navdata::ConstPtr& msg) 
{
  currAngleLock.lock();
  currAngle=msg->rotZ;
  currAngleLock.unlock();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "stitch_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 10, imageCallback);
  ros::Subscriber navSub = nh.subscribe <ardrone_autonomy::Navdata>("/ardrone/navdata", 100, NavcallBack); 
  ros::Publisher anglePub = nh.advertise <labels::LabelAngles>("labelAngles",100);
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    angleArrayLock.lock();
    if(isnonzero(angles,14)==1)
    {
        angleMessage.RWB = angles[0];
        angleMessage.RGB = angles[1];
        angleMessage.GWB = angles[2];
        angleMessage.BRG = angles[3];
        angleMessage.WRB = angles[4]; 
        angleMessage.BGR = angles[5];
        angleMessage.WBR = angles[6];
        angleMessage.WRG = angles[7]; 
        angleMessage.GBR = angles[8]; 
        angleMessage.BWR = angles[9]; 
        angleMessage.WGR = angles[10]; 
        angleMessage.WGB = angles[11]; 
        angleMessage.WBG = angles[12]; 
        angleMessage.GWR = angles[13];
        anglePub.publish(angleMessage);
    }
    angleArrayLock.unlock();
    ros::spin();
  }
  return 0;
}