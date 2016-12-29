#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>

using namespace cv;
using namespace std;

#define PI 3.14159



bool myfunction (int i,int j) { return (i<j); }

float dist(Point2f a1,Point2f a2)
{
  return sqrt(pow(a2.x-a1.x,2)+pow(a2.y-a1.y,2));
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

int main()
  {

        Mat srcred1,srcred2,srcred,srcdark,srcblue,srcgreen,srcwhite;
        Mat detectionImgFull,detectionImg;
        Mat detectionImgHSV,detectionImgGray;

        detectionImg=imread("Data/out1.png",CV_LOAD_IMAGE_COLOR);
        detectionImgFull=detectionImg;

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


        //imshow("keypointsR", im_with_keypoints_red);
        //imshow("keypointsG", im_with_keypoints_green);
        //imshow("keypointsB", im_with_keypoints_blue);

        std::vector<float> keypointsall;
        for(int l=0;l<keypointsred.size();l++)
            keypointsall.push_back(keypointsred[l].pt.x);
        for(int l=0;l<keypointsgreen.size();l++)
            keypointsall.push_back(keypointsgreen[l].pt.x);
        for(int l=0;l<keypointsblue.size();l++)
            keypointsall.push_back(keypointsblue[l].pt.x);

        for(int l=0;l<keypointsall.size();l++)
            cout<<keypointsall[l]<<endl;


        int i=0;
        float temp=-1;
        int count=0;
        int marker[3];
        float xValues[3];
        Mat posters[3];
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
                Rect newROI(i,0,120,360);
                cout<<i<<endl;
                xValues[count]=temp;
                detectionImg=detectionImgFull(newROI);
                posters[count]=detectionImg;

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

                count++;
                i=i+120;
            }
            else
            {
                i=i+20;
            }
            if(count==3)
            {
                if ((marker[1]-marker[0]==1) || (marker[2]-marker[1]==1))
                {
                    int k2=marker[1];
                    int k1=(k2-1)%14;
                    int k3=(k2+1)%14;
                    cout<<xValues[0]<<" "<<xValues[1]<<" "<<xValues[2]<<endl;

                }

                break;
            } 
        }


        cv::waitKey(0);
  }