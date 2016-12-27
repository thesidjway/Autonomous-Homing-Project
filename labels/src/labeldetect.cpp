#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
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

#define IDLE 0
#define READING 1
#define HOMING 2

#define PI 3.14159

float angles[14]; //RWB,RGB,GWB,BRG,WRB,BGR,WBR,WRG,GBR,BWR,WGR,WGB,WBG,GWR
using namespace cv;
using namespace std;
float currAngle;
mutex currAngleLock, angleArrayLock;
int returned[5]={-1,-1,-1,-1,-1};
int currStatus=IDLE;


geometry_msgs::Twist thetasMessage;
labels::LabelAngles angleMessage;

float fmodAng(float a)
{
    return (a<0)*360.0-(a>360)*360.0+a;
}
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

int detectPoster(std::vector<KeyPoint> KR, std::vector<KeyPoint> KG, std::vector<KeyPoint> KB)
{
    for(int i=0;i<KR.size();i++)
    {
        for(int j=0;j<KB.size();j++)
        {
            
            for(int k=0;k<KG.size();k++)
            {
                if(dist(KG[k].pt,KB[j].pt)<1.5*KG[k].size && dist(KG[k].pt,KR[i].pt)<1.5*KG[k].size)
                {
                    if(KB[j].pt.y < KR[i].pt.y)
                    {
                        //cout<<"BGR"<<endl;
                        return 5;
                    }
                    else if(KB[j].pt.y > KR[i].pt.y)
                    {
                        //cout<<"RGB"<<endl;
                        return 1;
                    }
                }
                else if(dist(KR[i].pt,KB[j].pt)<1.5*KR[i].size && dist(KG[k].pt,KR[i].pt)<1.5*KR[i].size)
                {
                    if(KB[j].pt.y < KG[k].pt.y)
                    {
                        //cout<<"BRG"<<endl;
                        return 3;
                    }
                }
                else if (dist(KR[i].pt,KB[j].pt)<1.5*KB[j].size && dist(KG[k].pt,KB[j].pt)<1.5*KB[j].size)
                {
                    if(KR[i].pt.y > KG[k].pt.y)
                    {
                       // cout<<"GBR"<<endl;
                        return 8;
                    }
                }
            }
            //cout<<dist(KR[i].pt,KB[j].pt)<<" "<<KR[i].size<<endl;
            if(dist(KR[i].pt,KB[j].pt)<2*KR[i].size)
            {
                if(KB[j].pt.y < KR[i].pt.y)
                {
                    //cout<<"WBR"<<endl;
                    return 6;
                }
                else
                {
                    //cout<<"WRB"<<endl;
                    return 4;
                }
            }
            else if (dist(KR[i].pt,KB[j].pt)<4*KR[i].size)
            { 
                if(KB[j].pt.y < KR[i].pt.y)
                {
                    //cout<<"BWR"<<endl;
                    return 9;
                }
                else
                {
                    //cout<<"RWB"<<endl;
                    return 0;
                }
            }
        }
        for(int k=0;k<KG.size();k++)
        {
            if(dist(KG[k].pt,KR[i].pt)<2*KG[k].size)
            {
                if(KR[i].pt.y < KG[k].pt.y)
                {
                    //cout<<"WRG"<<endl;
                    return 7;
                }
                else
                {
                    //cout<<"WGR"<<endl;
                    return 10;
                }
            }
            else if (dist(KG[k].pt,KR[i].pt)<4*KG[k].size)
            { 
                if(KR[i].pt.y > KG[i].pt.y)
                {
                    //cout<<"GWR"<<endl;
                    return 13;
                }
            }        
        }
    }
    for(int j=0;j<KB.size();j++)
    {
        for(int k=0;k<KG.size();k++)
        {
            if(dist(KG[k].pt,KB[j].pt)<1.5*KG[k].size)
            {
                if(KB[j].pt.y < KG[k].pt.y)
                {
                    //cout<<"WBG"<<endl;
                    return 12;
                }
                else
                {
                    //cout<<"WGB"<<endl;
                    return 11;
                }
            }
            else if (dist(KG[k].pt,KB[j].pt)<4*KG[k].size)
            { 
                if(KB[j].pt.y > KG[k].pt.y)
                {
                    //cout<<"GWB"<<endl;
                    return 2;
                }
            }               
        }

    }
return -1;
}

bool myfunction (int i,int j) { return (i<j); }

void statusCallBack(const std_msgs::Int32::ConstPtr& msg) 
{
    currStatus=msg->data;
}

void NavcallBack(const ardrone_autonomy::Navdata::ConstPtr& msg) 
{
  currAngleLock.lock();
  currAngle=msg->rotZ;
  currAngleLock.unlock();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 
        if(currStatus==IDLE)
        {
            return;
        }
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
        if(currStatus==READING)
        {
            detectionImg=detectionImgFull(myROI);
        }
        else if(currStatus==HOMING)
        {
            detectionImg=detectionImgFull;
        }

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

        std::vector<KeyPoint> keypointsred,keypointsblue,keypointsgreen;


        detector.detect( srcred, keypointsred);
        detector.detect( srcblue, keypointsblue);
        detector.detect( srcgreen, keypointsgreen);
       
        Mat im_with_keypoints_red,im_with_keypoints_white,im_with_keypoints_green,im_with_keypoints_blue;

        
        drawKeypoints( detectionImg, keypointsred, im_with_keypoints_red, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( detectionImg, keypointsblue, im_with_keypoints_blue, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( detectionImg, keypointsgreen, im_with_keypoints_green, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        if(currStatus==READING)
        {
        imshow("keypointsR", im_with_keypoints_red);
        imshow("keypointsG", im_with_keypoints_green);
        imshow("keypointsB", im_with_keypoints_blue);
        }

        // imshow("Original",detectionImg);

        if(currStatus==READING)
        {
            int a=detectPoster(keypointsred,keypointsgreen,keypointsblue);
            returned[4]=returned[3];
            returned[3]=returned[2];
            returned[2]=returned[1];
            returned[1]=returned[0];
            returned[0]=a;
            if(returned[4]==returned[3] && returned[3]==returned[2] && returned[2]==returned[1] && returned[1]==returned[0] && returned[1]!=-1)
            {

                angleArrayLock.lock();
                if(fabs(angles[returned[2]])<0.01)
                {
                    angles[returned[2]]=fmodAng(currAngle);
                    cout<<returned[2]<<endl;
                }
                angleArrayLock.unlock();

            }
        }
        else if(currStatus==HOMING)
        {
            std::vector<float> keypointsall;
            for(int l=0;l<keypointsred.size();l++)
                keypointsall.push_back(keypointsred[l].pt.x);
            for(int l=0;l<keypointsgreen.size();l++)
                keypointsall.push_back(keypointsgreen[l].pt.x);
            for(int l=0;l<keypointsblue.size();l++)
                keypointsall.push_back(keypointsblue[l].pt.x);


        int i=0;
        float temp=-1;
        int count=0;
        int marker[3];
        float xValues[3];
        float leftValues[3];
        while(i<520)
        {
            int k=i+120;
            int num=0;
            for (int j=0;j<keypointsall.size();j++)
            {
                if(keypointsall[j]<k && keypointsall[j]>i)
                {
                    num++;
                    temp=keypointsall[j];
                }

            }
            if (num>=2)
            {
                xValues[count]=temp;
                leftValues[count]=i;
                Rect newROI(i,0,120,360);
                detectionImg=detectionImgFull(newROI);
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



                SimpleBlobDetector::Params Hparams;

                Hparams.filterByColor = true;
                Hparams.blobColor = 255;
                Hparams.minThreshold = 200;
                Hparams.filterByArea = true;
                Hparams.minArea = 50;
                Hparams.filterByCircularity = false;
                Hparams.filterByConvexity = true;
                Hparams.minConvexity = 0.30;    
                Hparams.filterByInertia = true;
                Hparams.maxInertiaRatio = 0.8;

                SimpleBlobDetector Hdetector(Hparams);

                std::vector<KeyPoint> Hkeypointsred,Hkeypointsblue,Hkeypointsgreen;

                Hdetector.detect( srcred, Hkeypointsred);
                Hdetector.detect( srcblue, Hkeypointsblue);
                Hdetector.detect( srcgreen, Hkeypointsgreen);

                marker[count]=detectPoster(Hkeypointsred,Hkeypointsgreen,Hkeypointsblue);
                i=i+120;
                count++;

            }
            else
            {
                i=i+20;
            }
            if(count==3)
            {
                line(detectionImgFull,Point(leftValues[0],20),Point(leftValues[0],340),Scalar(0,255,0),1,8,0);
                line(detectionImgFull,Point(leftValues[1],20),Point(leftValues[1],340),Scalar(0,255,0),1,8,0);
                line(detectionImgFull,Point(leftValues[2],20),Point(leftValues[2],340),Scalar(0,255,0),1,8,0);
                line(detectionImgFull,Point(xValues[0],20),Point(xValues[0],340),Scalar(255,0,0),1,8,0);
                line(detectionImgFull,Point(xValues[1],20),Point(xValues[1],340),Scalar(255,0,0),1,8,0);
                line(detectionImgFull,Point(xValues[2],20),Point(xValues[2],340),Scalar(255,0,0),1,8,0);
                line(detectionImgFull,Point(leftValues[0]+120,20),Point(leftValues[0]+120,340),Scalar(0,0,255),1,8,0);
                line(detectionImgFull,Point(leftValues[1]+120,20),Point(leftValues[1]+120,340),Scalar(0,0,255),1,8,0);
                line(detectionImgFull,Point(leftValues[2]+120,20),Point(leftValues[2]+120,340),Scalar(0,0,255),1,8,0);
                imshow("xValues",detectionImgFull);
                if ((marker[1]-marker[0]==1) || (marker[2]-marker[1]==1))
                {
                    angleArrayLock.lock();
                    currAngleLock.lock();
                    int k2=marker[1];
                    int k1=(k2-1)%14;
                    int k3=(k2+1)%14;
                    thetasMessage.angular.x=angles[k1]*PI/180.0;
                    thetasMessage.angular.y=angles[k2]*PI/180.0;
                    thetasMessage.angular.z=angles[k3]*PI/180.0;
                    thetasMessage.linear.x = fmodAng(fmodAng(currAngle)-(xValues[0]-320.0)*0.14375)*PI/180.0;
                    thetasMessage.linear.y = fmodAng(fmodAng(currAngle)-(xValues[1]-320.0)*0.14375)*PI/180.0;
                    thetasMessage.linear.z = fmodAng(fmodAng(currAngle)-(xValues[2]-320.0)*0.14375)*PI/180.0;
                    currAngleLock.unlock();
                    angleArrayLock.unlock();

                }

                break;
            }
            
        }





        }
        cv::waitKey(20);

}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "stitch_node");

  ros::NodeHandle nh;
  ros::Subscriber statusSub = nh.subscribe <std_msgs::Int32>("homingStatus", 100, statusCallBack); 
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 10, imageCallback);
  ros::Subscriber navSub = nh.subscribe <ardrone_autonomy::Navdata>("/ardrone/navdata", 100, NavcallBack); 
  ros::Publisher homingTwistPub = nh.advertise <geometry_msgs::Twist>("thetaAngles",100);
  ros::Publisher anglePub = nh.advertise <labels::LabelAngles>("labelAngles",100);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
    angleArrayLock.lock();

    //cout<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<" "<<angles[3]<<" "<<angles[4]<<" "<<angles[5]<<" "<<angles[6]<<" "<<angles[7]<<" "<<angles[8]<<" "<<angles[9]<<" "<<angles[10]<<" "<<angles[11]<<" "<<angles[12]<<" "<<angles[13]<<endl;
    
    
    if(isnonzero(angles,14)==1)
    {
        //cout<<"READING DONE"<<endl;
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
    if(currStatus==HOMING)
    {
        homingTwistPub.publish(thetasMessage);
    }
    
    ros::spinOnce();
  }
  return 0;
}