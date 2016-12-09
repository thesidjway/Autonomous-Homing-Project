#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Mat previmage;


void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{ 
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

  Mat image1= cv_ptr->image;

  // imshow("second image" , image1);
  if(previmage.empty())
  {
    previmage=image1;
  }
  Mat image2= previmage;

  Mat gray_image1;
  Mat gray_image2;

   // Convert to Grayscale
  cvtColor( image1, gray_image1, CV_RGB2GRAY );
  cvtColor( image2, gray_image2, CV_RGB2GRAY );
   
  imshow("first image"  , image1);
  imshow("second image" , image2);
   
   
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
   
  SurfFeatureDetector detector( minHessian );
   
  std::vector< KeyPoint > keypoints_object, keypoints_scene;
   
  detector.detect( gray_image1, keypoints_object );
  detector.detect( gray_image2, keypoints_scene );
   
  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;
   
  Mat descriptors_object, descriptors_scene;
   
  extractor.compute( gray_image1, keypoints_object, descriptors_object );
  extractor.compute( gray_image2, keypoints_scene, descriptors_scene );
   
  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );
   
  double max_dist = 0; double min_dist = 100;
   
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
   if( dist < min_dist ) min_dist = dist;
   if( dist > max_dist ) max_dist = dist;
  }
   
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
   
  //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;
   
  for( int i = 0; i < descriptors_object.rows; i++ )
  { 
  if( matches[i].distance < 3*min_dist )
  { 
    good_matches.push_back( matches[i]); 
  }
  }
  std::vector< Point2f > obj;
  std::vector< Point2f > scene;
   
  for( int i = 0; i < good_matches.size(); i++ )
  {
   //-- Get the keypoints from the good matches
   obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
   scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
   
  // Find the Homography Matrix
  Mat H = findHomography( obj, scene, CV_RANSAC );
   // Use the Homography Matrix to warp the images
  cv::Mat result;
  warpPerspective(image1,result,H,cv::Size(image1.cols+image2.cols,image1.rows));
  cv::Mat half(result,cv::Rect(0,0,image2.cols,image2.rows));
  image2.copyTo(half);
  previmage=image1;
  imshow( "Result", result );
  // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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