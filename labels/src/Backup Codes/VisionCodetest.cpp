
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

int main()
{
VideoCapture v(0);
while(1)
{
	Mat element = getStructuringElement( MORPH_CROSS,Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

    Mat srcred1,srcred2,srcred,srcwhite,dilatedRed,dilatedWhite,srcblue,srcgreen,dilatedBlue,dilatedGreen;
    Mat boundary[6];//boundaryGB, boundaryRW, boundaryGW, boundaryBW, boundaryRG, boundaryBR;
    Mat detectionImg;
    Mat detectionImgHSV,detectionImgHLS;

    v>>detectionImg;
    cvtColor(detectionImg,detectionImgHSV,CV_BGR2HSV);
    cvtColor(detectionImg,detectionImgHLS,CV_BGR2HLS);

    inRange(detectionImgHSV,Scalar(0,70,70),Scalar(14,255,255),srcred1);
    inRange(detectionImgHSV,Scalar(165,70,70),Scalar(180,255,255),srcred2);
    bitwise_or(srcred1,srcred2,srcred);
    inRange(detectionImgHLS,Scalar(0,150,0),Scalar(180,255,255),srcwhite);
    inRange(detectionImgHSV,Scalar(60,40,70),Scalar(100,255,255),srcgreen);
    inRange(detectionImgHSV,Scalar(105,40,70),Scalar(135,255,255),srcblue);

    dilate( srcwhite, dilatedWhite, element );
    dilate( srcred, dilatedRed, element );
    dilate( srcblue, dilatedBlue, element );
    dilate( srcgreen, dilatedGreen, element );

    imshow("ORIGINAL",detectionImg);

    imshow("BLUE",dilatedBlue);
    imshow("RED",dilatedRed);
    imshow("WHITE",dilatedWhite);
    imshow("GREEN",dilatedGreen);
    // bitwise_and(dilatedGreen,dilatedBlue,boundary[0]);
    // imshow("BLUEGREEN",boundary[0]);

    // bitwise_and(dilatedGreen,dilatedWhite,boundary[1]);
    // imshow("GREENWHITE",boundary[1]);

    // bitwise_and(dilatedWhite,dilatedBlue,boundary[2]);
    // imshow("BLUEWHITE",boundary[2]);

    // bitwise_and(dilatedRed,dilatedWhite,boundary[3]);
    // imshow("REDWHITE",boundary[3]);

    // bitwise_and(dilatedRed,dilatedGreen,boundary[4]);
    // imshow("REDGREEN",boundary[4]);

    // bitwise_and(dilatedRed,dilatedBlue,boundary[5]);
    // imshow("BLUERED",boundary[5]);

    // vector<vector<Point>> contours[6]; //contoursGB, contoursGW, contoursBW, contoursRW, contoursRG, contoursBR;
    // vector<Vec4i> hierarchy[6]; //hierarchy GB, hierarchyGW, hierarchyBW, hierarchyRW, hierarchyRG, hierarchyBR;

    // for (int clr=0;clr<6;clr++)
    // {
    //    findContours( boundary[clr], contours[clr], hierarchy[clr], CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // }

    // int largestArea[]={-1,-1,-1,-1,-1,-1};
    // int largestContourIndex[]={-1,-1,-1,-1,-1,-1};

    // for(int clr=0;clr<6;clr++)
    // {
    //   for( int cnt = 0; cnt< contours[clr].size(); cnt++ ) 
    //   {
    //      double a=contourArea( contours[clr][cnt],false);  
    //      if(a>largestArea[clr])
    //      {
    //        largestArea[clr]=a;
    //        largestContourIndex[clr]=cnt;
    //      }
    //   }
    // }

    // vector<vector<Point>> hull[6];
    // for (int clr=0;clr<6;clr++)
    // {
    //   if(largestContourIndex[clr]!=-1)
    //   {
    //     vector<Point2f> hullPoints;
    //     convexHull(contours[largestContourIndex[clr]], hull[clr], false );
    //     approxPolyDP(hull[clr], hullPoints, POLYDP, true);
    //     cout<<hullPoints.size()<<endl;
    //   }
    // }

    char a=waitKey(10);
	if (a == 27)
		break;
}

}