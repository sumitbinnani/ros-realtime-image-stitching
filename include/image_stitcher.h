#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono> 


static const std::string OPENCV_WINDOW = "Stitched Images";
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSyncPolicy;
typedef image_transport::SubscriberFilter ImageSubscriber;

using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cv::xfeatures2d;

class ImageStitcher{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ImageSubscriber left_sub;
  ImageSubscriber right_sub;
  image_transport::Publisher image_pub_;
  message_filters::Synchronizer<ApproxSyncPolicy> sync;
  Mat homography;
  bool show_image;

public:
  ImageStitcher(string, string, int, bool);
  ~ImageStitcher();
  void callback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);
  void calibrate(Mat, Mat);
};

ImageStitcher::ImageStitcher(string left, string right, int buffersize, bool debugscreen = false):it_(nh_),
  left_sub(it_, left, buffersize),
  right_sub(it_, right, buffersize),
  sync(ApproxSyncPolicy(buffersize), left_sub, right_sub){
  sync.registerCallback( boost::bind( &ImageStitcher::callback, this, _1, _2 ) );
  image_pub_ = it_.advertise("/stitched_images/output", buffersize);
  show_image = debugscreen;
  if (show_image) cv::namedWindow(OPENCV_WINDOW);
}

void ImageStitcher::calibrate(Mat img_1, Mat img_2){
  Ptr<SIFT> detector = SIFT::create();
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  
  detector->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
  detector->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);
  
  // Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<std::vector<DMatch>> knn_matches;
  matcher.knnMatch(descriptors_1, descriptors_2, knn_matches, 2);
  
  float ratio_thresh = 0.7f;
  std::vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++){
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
      good_matches.push_back(knn_matches[i][0]);
    }
  }
               
  // Extract location of good matches
  std::vector<Point2f> points1, points2;
  
  if (good_matches.size() < 15){
    cout << "Not Enough Matches" << endl;
    return;
  }
   
  for( size_t i = 0; i < good_matches.size(); i++ ){
    points1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
  }
   
  // Find homography
  homography = findHomography(points1, points2, RANSAC);
}

void ImageStitcher::callback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg){
  cv_bridge::CvImagePtr left_ptr, right_ptr;
  try {
    left_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
    right_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);
    
    Mat padded_right; 
    copyMakeBorder(right_ptr->image, padded_right, 0, 0, right_ptr->image.size().height, 0, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    
    if (homography.empty()){
      calibrate(left_ptr->image, padded_right);
    }
    
    if (homography.empty()){
      return;
    }
    
    warpPerspective(left_ptr->image, padded_right, homography, padded_right.size(),INTER_LINEAR, BORDER_TRANSPARENT);
    
     // Update GUI Window
    if (show_image){
       imshow(OPENCV_WINDOW, padded_right);
       waitKey(3);
    }
    
    right_ptr->image = padded_right;
    image_pub_.publish(right_ptr->toImageMsg());
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

ImageStitcher::~ImageStitcher(){
  if (show_image) cv::destroyWindow(OPENCV_WINDOW);
}