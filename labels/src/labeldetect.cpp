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

#define erosion_size 2
#define DISTTHRES 35

int angles[14]; //RWB,RGB,GWB,BRG,WRB,BGR,WBR,WRG,GBR,BWR,WGR,WGB,WBG,GWR
using namespace cv;
using namespace std;

float dist(Point2f a1,Point2f a2)
{
  return sqrt(pow(a1.x-a2.x,2)+pow(a2.y-a1.y,2));
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 

    Rect myROI(240,0,160,360);
    Mat srcred1,srcred2,srcred,srcdark,srcblue,srcgreen,srcwhite;

    //Mat boundary[6];//boundaryGB, boundaryRW, boundaryGW, boundaryBW, boundaryRG, boundaryBR;
    Mat detectionImgFull,detectionImg;

    Mat detectionImgHSV,detectionImgGray;//,detectionImgHLS;

    Mat erodedblue,erodedred;

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

    Mat element = getStructuringElement( MORPH_CROSS,Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point(erosion_size, erosion_size ) );

    detectionImg=cv_ptr->image;
    //detectionImg=detectionImgFull(myROI);

    //cout<<detectionImg.rows<<" "<<detectionImg.cols<<endl;
    cvtColor(detectionImg,detectionImgHSV,CV_BGR2HSV);
    cvtColor(detectionImg,detectionImgGray,CV_BGR2GRAY);

    inRange(detectionImgHSV,Scalar(0,40,40),Scalar(4,255,255),srcred1);
    inRange(detectionImgHSV,Scalar(170,40,40),Scalar(180,255,255),srcred2);
    bitwise_or(srcred1,srcred2,srcred);

    inRange(detectionImgHSV,Scalar(55,40,20),Scalar(82,255,255),srcgreen);
    inRange(detectionImgHSV,Scalar(108,40,20),Scalar(132,255,255),srcblue);

    inRange(detectionImgGray,0,99,srcdark);
    inRange(detectionImgGray,100,255,srcwhite);
    // dilate( srcdark, dilatedWhite, element );
    // dilate( srcred, dilatedRed, element );
    // dilate( srcblue, dilatedBlue, element );
    // dilate( srcgreen, dilatedGreen, element );
    bitwise_and(srcdark,srcblue,srcblue);
    bitwise_and(srcdark,srcgreen,srcgreen);
    //bitwise_and(srcdark,srcred,srcred);

    erode( srcblue, erodedblue, element );
    erode( srcred, erodedred, element );




    SimpleBlobDetector::Params params;

    params.filterByColor = true;

    params.blobColor = 255;

    params.minThreshold = 200;

    params.filterByArea = true;
    params.minArea = 10;
    params.minArea = 200;
     
    // Filter by Circularity
    params.filterByCircularity = false;
    
    //params.minCircularity = 0.1;
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.30;    
    // Filter by Inertia
    params.filterByInertia = true;
    params.maxInertiaRatio = 0.8;

    // Set up detector with params
    SimpleBlobDetector detector(params);

    std::vector<KeyPoint> keypointsred,keypointsblue,keypointsgreen,keypointswhite;
    std::vector<Point2f> keypointsRB, keypointsRG, keypointsBG, keypointsBGR;

    detector.detect( srcred, keypointsred);
    detector.detect( srcblue, keypointsblue);
    detector.detect( srcgreen, keypointsgreen);
  //  detector.detect( srcdark, keypointswhite);

// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints_red,im_with_keypoints_white,im_with_keypoints_green,im_with_keypoints_blue;
    
    drawKeypoints( detectionImg, keypointsred, im_with_keypoints_red, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( detectionImg, keypointsblue, im_with_keypoints_blue, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( detectionImg, keypointsgreen, im_with_keypoints_green, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // drawKeypoints( detectionImg, keypointswhite, im_with_keypoints_white, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // drawKeypoints( srcred, keypointsred, im_with_keypoints_red, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // drawKeypoints( srcblue, keypointsblue, im_with_keypoints_blue, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // drawKeypoints( srcgreen, keypointsgreen, im_with_keypoints_green, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    //drawKeypoints( detectionImg, keypointswhite, im_with_keypoints_white, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
  //  imshow("ORIGINAL",detectionImg);
// Show blobs
    imshow("keypointsR", im_with_keypoints_red);
    imshow("keypointsG", im_with_keypoints_green);
    imshow("keypointsB", im_with_keypoints_blue);
  //  imshow("keypointsW", im_with_keypoints_white);

    int k=0,m=0;

    for(k=0;k<keypointsblue.size();k++)
    {
      keypointsBG.push_back(keypointsblue[k].pt);
      keypointsRB.push_back(keypointsblue[k].pt);
    }

    for(k=0;k<keypointsred.size();k++)
    {
      keypointsRG.push_back(keypointsred[k].pt);
      keypointsRB.push_back(keypointsred[k].pt);
    }
    for(k=0;k<keypointsgreen.size();k++)
    {
      keypointsRG.push_back(keypointsgreen[k].pt);
      keypointsBG.push_back(keypointsgreen[k].pt);
    }

    // for(k=0;k<keypointsRG.size();k++)
    // {
    //   for(m=k+1;m<keypointsall.size();m++)
    //   {
    //     if(dist(keypointsall[m],keypointsall[k])<DISTTHRES)
    //       cout<<"PAAS"<<endl;
    //   }
    // }
    cout<<"#############"<<endl;

    // imshow("BLUE",erodedblue);
    // imshow("RED",erodedred);
    // imshow("GREEN",srcgreen);
    // imshow("WHITE",srcdark);






    cv::waitKey(30);
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