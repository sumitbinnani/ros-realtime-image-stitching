#include <ros/ros.h>
#include "image_stitcher.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "ImageStitcher", ros::init_options::AnonymousName);
  
  ros::NodeHandle nh_("~");
  
  std::string left, right;
  if (nh_.getParam("left", left)){
      ROS_INFO("Got left stream: %s", left.c_str());
  }else{
      ROS_ERROR("Failed to get 'left' stream");
      return 1;
  }
  
  if (nh_.getParam("right", right)){
      ROS_INFO("Got right stream: %s", right.c_str());
  }else{
      ROS_ERROR("Failed to get 'right' stream");
      return 1;
  }
  
  ImageStitcher ic(left, right, 10, false);
  ros::spin();
  return 0;
}
