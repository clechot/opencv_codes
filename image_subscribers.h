#ifndef __IMAGE_SUBSCRIBERS_H__
#define __IMAGE_SUBSCRIBERS_H__

#include <iostream>
#include <stdio.h>
#include <limits>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define TM_CCORR_NORMED 3

const std::string real_rgb_topic = "/camera/rgb/image_raw";
const std::string real_depth_topic = "/camera/depth_registered/image_raw";

int match_method = TM_CCORR_NORMED;
int white = 254;
double dNan = std::numeric_limits<double>::quiet_NaN();


class ImageSubscribers {

  int compare_flag;
  int robot_flag;
  double probability;

 public:

  ImageSubscribers(const std::string topic, const std::string score);

  ~ImageSubscribers();

  void init(ros::NodeHandle *nodeHandlePtr);

  void run();

  void compare(cv::Mat img1, cv::Mat img2, cv::Mat img3, cv::Mat img4);

  // Convert any image to a viewable image. Code primarily taken from rqt_image_view::image_view.cpp.
  void conversionImages(const sensor_msgs::ImageConstPtr &image1, const sensor_msgs::ImageConstPtr &image2, const sensor_msgs::ImageConstPtr &image3, const sensor_msgs::ImageConstPtr &image4);
  void viewImage(const cv::Mat img1) const;

  bool getNodeHandle();
  double getProbability();
  void normalizeProbability(double norm);

  std::string synth_rgb_topic;
  std::string synth_depth_topic;
  std::string score_topic;

 protected:

  void imageCallbackRealC(const sensor_msgs::ImageConstPtr &image);
  void imageCallbackRealD(const sensor_msgs::ImageConstPtr &image);
  void imageCallbackSynthC(const sensor_msgs::ImageConstPtr &image);
  void imageCallbackSynthD(const sensor_msgs::ImageConstPtr &image);

  ros::NodeHandle *nodeHandle_;

  sensor_msgs::ImageConstPtr latest_rgb_real_image_ = NULL;
  sensor_msgs::ImageConstPtr latest_depth_real_image_ = NULL;
  sensor_msgs::ImageConstPtr latest_rgb_synth_image_ = NULL;
  sensor_msgs::ImageConstPtr latest_depth_synth_image_ = NULL;

  // Subscribers for real & synthetic images
  image_transport::Subscriber real_rgb_sub_;
  image_transport::Subscriber real_depth_sub_;
  image_transport::Subscriber synth_rgb_sub_;
  image_transport::Subscriber synth_depth_sub_;

  ros::Publisher score_pub;
};


#endif //__IMAGE_SUBSCRIBERS_H__
