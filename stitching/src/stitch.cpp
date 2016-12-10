#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;


#define Y_THRESHOLD 10
#define DIVISION 2

int distx,disty;
Mat prevframe,frame;
Mat img_1,img_2;
Mat concat;

static const std::string OPENCV_WINDOW = "Image window";


void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 
  Point2f apt,bpt;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_2=cv_ptr->image;
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

      std::vector< DMatch > good_matches;

      for( int i = 0; i < descriptors_1.rows; i++ )
      { if( matches[i].distance <= 0.1)//max(2*min_dist, 0.02) )
        { good_matches.push_back( matches[i]); }
      }

      //-- Draw only "good" matches
      // Mat img_matches;
      // drawMatches( img_1, keypoints_1, img_2, keypoints_2,
      //              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
      //              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Show detected matches
      //imshow( "Good Matches", img_matches );
      int less_dist=0;
      Mat img1cut,img2cut;
      for( int i = 0; i < (int)good_matches.size(); i++ )
      { 
        //KeyPoint apt=keypoints_2[1];
        apt=keypoints_1[(int)good_matches[i].queryIdx].pt;
        bpt=keypoints_2[(int)good_matches[i].trainIdx].pt;

        //std::cout<<apt.x<<" "<<bpt.x<<std::endl;
        //std::cout<<apt.y<<" "<<bpt.y<<std::endl<<std::endl;

        distx=abs(apt.x-bpt.x);
        disty=abs(apt.y-bpt.y);

        if(disty<Y_THRESHOLD)
        {
          less_dist++;
        }
        if(less_dist>2)
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

      hconcat(left,right,concat);

      cout<<concat.cols<<endl;

      prevframe=concat;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }




    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, concat);
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