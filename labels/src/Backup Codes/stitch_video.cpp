#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include <iostream>

# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/nonfree/features2d.hpp"

#define Y_THRESHOLD 10
#define DIVISION 2
using namespace cv;
using namespace std;

int distx,disty;
Mat prevframe,frame;
Mat img_1,img_2;

int main( int argc, char** argv )
{
  int iterator=0;
  VideoCapture cap("lab.mp4"); 
  std::cout<<cap.get(CV_CAP_PROP_FRAME_COUNT);
  for(iterator=0;iterator<cap.get(CV_CAP_PROP_FRAME_COUNT);iterator++)
  {
    cap.read(frame);
    img_2=frame;
    if(img_1.empty())
    {
      prevframe=img_2;
    }
    img_1 = prevframe;



    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_1, descriptors_2;
    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    // printf("-- Max dist : %f \n", max_dist );
    // printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= 0.1)//max(2*min_dist, 0.02) )
      { good_matches.push_back( matches[i]); }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Good Matches", img_matches );
    int less_dist=0;
    Mat img1cut,img2cut;
    for( int i = 0; i < (int)good_matches.size(); i++ )
    { 
    	//KeyPoint apt=keypoints_2[1];
    	Point2f apt=keypoints_1[(int)good_matches[i].queryIdx].pt;
    	Point2f bpt=keypoints_2[(int)good_matches[i].trainIdx].pt;

    	//std::cout<<apt.x<<" "<<bpt.x<<std::endl;
    	//std::cout<<apt.y<<" "<<bpt.y<<std::endl<<std::endl;

    	distx=abs(apt.x-bpt.x);
    	disty=abs(apt.y-bpt.y);

    	if(disty<Y_THRESHOLD)
    	{
    		less_dist++;
    	}
    	if(less_dist>(int)good_matches.size()/DIVISION)
    	{
    		Rect img1roi(0,0,apt.x,img_1.rows);
    		img1cut=img_1(img1roi);
    		Rect img2roi(bpt.x,0,img_2.cols-bpt.x,img_1.rows);
    		img2cut=img_2(img2roi);
    		break; 

    	}

    	//printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
    }

    // imshow("img1", img1cut);
    // imshow("img2", img2cut);

    Size sz1 = img1cut.size();
    Size sz2 = img2cut.size();

  //  cout<<sz1.height<<" "<<sz2.height<<endl;

    Mat result(sz1.height, sz1.width+sz2.width, CV_8UC1);
    Mat left(result, Rect(0, 0, sz1.width, sz1.height));
    img1cut.copyTo(left);
    Mat right(result, Rect(sz1.width, 0, sz2.width, sz1.height));
    img2cut.copyTo(right);
   // imshow("left", left);
   // imshow("right", right);
    Mat concat;
    hconcat(left,right,concat);
    imshow("result", concat);

    cout<<concat.cols<<endl;

    prevframe=concat;
     if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
       {
                std::cout << "esc key is pressed by user" << std::endl; 
                break; 
       }

  }

}

