#include "image_subscribers.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <memory>

#include <iostream>
#include <stdio.h>
#include <limits>

#include <ros/console.h>

ImageSubscribers::ImageSubscribers(const std::string topic, const std::string score)
        : nodeHandle_(NULL) {

  ROS_INFO("New image subscriber");

  compare_flag = false;
  robot_flag = false;

  probability = 1;

  std::cout << " Topic name: " << topic;
  std::cout << "\n";
  std::cout << " Score nber: " << score;
  std::cout << "\n";

  synth_rgb_topic = "/" + topic + "/rgb/image_raw";
  synth_depth_topic = "/" + topic + "/depth/image_raw";
  score_topic = "/" + score;
}

ImageSubscribers::~ImageSubscribers() {

  cv::destroyWindow(synth_depth_topic);
}

void ImageSubscribers::imageCallbackRealC(const sensor_msgs::ImageConstPtr &image) {
  
  latest_rgb_real_image_ = image;
}

void ImageSubscribers::imageCallbackRealD(const sensor_msgs::ImageConstPtr &image) {
  
  latest_depth_real_image_ = image;
}

void ImageSubscribers::imageCallbackSynthC(const sensor_msgs::ImageConstPtr &image) {
  
  latest_rgb_synth_image_ = image;
}

void ImageSubscribers::imageCallbackSynthD(const sensor_msgs::ImageConstPtr &image) {
  
  latest_depth_synth_image_ = image;
}

void ImageSubscribers::conversionImages(const sensor_msgs::ImageConstPtr &image1, const sensor_msgs::ImageConstPtr &image2, const sensor_msgs::ImageConstPtr &image3, const sensor_msgs::ImageConstPtr &image4) {
  
  if(!image1)
  {
    ROS_INFO("No real rgb image");
    return;
  }
  if(!image2)
  {
    ROS_INFO("No real depth image");
    return;
  }
  if(!image3)
  {
    ROS_INFO("No synthetic rgb image");
    return;
  }
  if(!image4)
  {
    ROS_INFO("No synthetic depth image");
    return;
  }

  compare_flag = true;

  /// Convert any image to a viewable image. Taken from rqt_image_view::image_view.cpp.
  cv::Mat conversion_mat1_;
  try
  {
    // First let cv_bridge do its magic on color images.
    cv_bridge::CvImageConstPtr cv_ptr1 = cv_bridge::toCvShare(image1, sensor_msgs::image_encodings::BGR8);
    conversion_mat1_ = cv_ptr1->image;
  } catch (cv_bridge::Exception& e)
  {
    // Convert depth image with automatic scaling between min and max.
    cv_bridge::CvImageConstPtr cv_ptr1 = cv_bridge::toCvShare(image1);
    double min = 0;
    double max = 2;
    minMaxLoc(cv_ptr1->image, &min, &max);
    if(min == max)
    {
      min = 0;
      max = 2;
    }
    cv::Mat img_scaled_8u1;
    cv::Mat(cv_ptr1->image - min).convertTo(img_scaled_8u1, CV_8UC1, 255. / (max - min));
    cv::cvtColor(img_scaled_8u1, conversion_mat1_, CV_GRAY2RGB);
  }

  cv::Mat conversion_mat2_;
  try
  {
    // First let cv_bridge do its magic on color images.
    cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(image2, sensor_msgs::image_encodings::BGR8);
    conversion_mat2_ = cv_ptr2->image;
  } catch (cv_bridge::Exception& e)
  {
    // Convert depth image with automatic scaling between min and max.
    cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(image2);
    double min = 0;
    double max = 2;
    minMaxLoc(cv_ptr2->image, &min, &max);
    if(min == max)
    {
      min = 0;
      max = 2;
    }
    cv::Mat img_scaled_8u2;
    cv::Mat(cv_ptr2->image - min).convertTo(img_scaled_8u2, CV_8UC1, 255. / (max - min));
    cv::cvtColor(img_scaled_8u2, conversion_mat2_, CV_GRAY2RGB);
  }

  cv::Mat conversion_mat3_;
  try
  {
    // First let cv_bridge do its magic on color images.
    cv_bridge::CvImageConstPtr cv_ptr3 = cv_bridge::toCvShare(image3, sensor_msgs::image_encodings::BGR8);
    conversion_mat3_ = cv_ptr3->image;
  } catch (cv_bridge::Exception& e)
  {
    // Convert depth image with automatic scaling between min and max.
    cv_bridge::CvImageConstPtr cv_ptr3 = cv_bridge::toCvShare(image3);
    double min = 0;
    double max = 2;
    minMaxLoc(cv_ptr3->image, &min, &max);
    if(min == max)
    {
      min = 0;
      max = 2;
    }
    cv::Mat img_scaled_8u3;
    cv::Mat(cv_ptr3->image - min).convertTo(img_scaled_8u3, CV_8UC1, 255. / (max - min));
    cv::cvtColor(img_scaled_8u3, conversion_mat3_, CV_GRAY2RGB);
  }

  cv::Mat conversion_mat4_;
  try
  {
    // First let cv_bridge do its magic on color images.
    cv_bridge::CvImageConstPtr cv_ptr4 = cv_bridge::toCvShare(image4, sensor_msgs::image_encodings::BGR8);
    conversion_mat4_ = cv_ptr4->image;
  } catch (cv_bridge::Exception& e)
  {
    // Convert depth image with automatic scaling between min and max.
    cv_bridge::CvImageConstPtr cv_ptr4 = cv_bridge::toCvShare(image4);
    double min = 0;
    double max = 2;
    minMaxLoc(cv_ptr4->image, &min, &max);
    if(min == max)
    {
      min = 0;
      max = 2;
    }
    cv::Mat img_scaled_8u4;
    cv::Mat(cv_ptr4->image - min).convertTo(img_scaled_8u4, CV_8UC1, 255. / (max - min));
    cv::cvtColor(img_scaled_8u4, conversion_mat4_, CV_GRAY2RGB);
  }


  if(compare_flag==true)
  {
    compare(conversion_mat1_, conversion_mat2_, conversion_mat3_, conversion_mat4_);
  }
}

void ImageSubscribers::viewImage(const cv::Mat img1) const {

  cv::imshow(synth_depth_topic, img1);
  cv::waitKey(1);  // Necessary to actually show images.
}

void ImageSubscribers::init(ros::NodeHandle *nodeHandlePtr) {
  
  ROS_INFO_STREAM("Subscribing to all topics.");
  nodeHandle_ = nodeHandlePtr;

  image_transport::ImageTransport image_transport(*nodeHandlePtr);
  
  real_rgb_sub_ = image_transport.subscribe(real_rgb_topic, 1, &ImageSubscribers::imageCallbackRealC, this);

  real_depth_sub_ = image_transport.subscribe(real_depth_topic, 1, &ImageSubscribers::imageCallbackRealD, this);

  synth_rgb_sub_ = image_transport.subscribe(synth_rgb_topic, 1, &ImageSubscribers::imageCallbackSynthC, this);

  synth_depth_sub_ = image_transport.subscribe(synth_depth_topic, 1, &ImageSubscribers::imageCallbackSynthD, this);

  score_pub = nodeHandlePtr->advertise<std_msgs::Float64>(score_topic, 1);

  cv::namedWindow(synth_depth_topic);
  cv::startWindowThread();
}

void ImageSubscribers::compare(cv::Mat img1, cv::Mat img2, cv::Mat img3, cv::Mat img4) {

  robot_flag = false;

  /// Source image to display
  cv::Mat img_display;
  img1.copyTo(img_display);
  cv::Mat templ_display;
  img3.copyTo(templ_display);

  /// Create the result matrix
  int result_cols =  img1.cols - img3.cols + 1;
  int result_rows = img1.rows - img3.rows + 1;
  cv::Mat result = cv::Mat(result_rows, result_cols, CV_32FC1);

  /// Conversion RGB to HSV
  cv::Mat img1_HSV;
  cv::Mat img3_HSV;
  cv::cvtColor(img1, img1_HSV, CV_BGR2HSV);
  cv::cvtColor(img3, img3_HSV, CV_BGR2HSV);

  /// Conversion Depth to Gray
  cv::cvtColor(img2, img2, CV_BGR2GRAY);
  cv::cvtColor(img4, img4, CV_BGR2GRAY);

  /// Merge the 3 channels image and the depth channel image into one 4 channels image
  cv::Mat new_img = cv::Mat(img1_HSV.rows, img1_HSV.cols, CV_8UC4);
  cv::Mat new_templ = cv::Mat(img3_HSV.rows, img3_HSV.cols, CV_8UC4);
  int from_to1[] = {0,0,1,1,2,2};
  int from_to2[] = {0,3};

  /*// in order to have no HSV transformation
  cv::Mat new_imgRGB = cv::Mat(img1.rows, img1.cols, CV_8UC4);
  cv::Mat new_templRGB = cv::Mat(img3.rows, img3.cols, CV_8UC4);
  mixChannels(&img1, 1, &new_imgRGB, 1, from_to1, 3);
  mixChannels(&img2, 1, &new_imgRGB, 1, from_to2, 1);
  mixChannels(&img3, 1, &new_templRGB, 1, from_to1, 3);
  mixChannels(&img4, 1, &new_templRGB, 1, from_to2, 1);*/


  mixChannels(&img1_HSV, 1, &new_img, 1, from_to1, 3);
  mixChannels(&img2, 1, &new_img, 1, from_to2, 1);
  mixChannels(&img3_HSV, 1, &new_templ, 1, from_to1, 3);
  mixChannels(&img4, 1, &new_templ, 1, from_to2, 1);

  /// Create the mask in HSV
  cv:: Mat mask = cv::Mat(new_templ.rows, new_templ.cols, new_templ.type(), cv::Scalar::all(white));

  for(int j = 0; j < new_templ.cols; j++)
  {
    for(int i = 0; i < new_templ.rows; i++)
    {
      if(new_templ.at<cv::Vec4b>(i,j)[0] == 0)
      {
        mask.at<cv::Vec4b>(i,j) = dNan;
      }
      if(new_templ.at<cv::Vec4b>(i,j)[1] < 100)
      {
        mask.at<cv::Vec4b>(i,j) = dNan;
      }
    }
  }


  /*cv:: Mat mask = cv::Mat(img3_HSV.rows,img3_HSV.cols,img3_HSV.type(),cv::Scalar::all(white)); // in order to have no depth channel

  for(int j = 0; j < img3_HSV.cols; j++)
  {
    for(int i = 0; i < img3_HSV.rows; i++)
    {
      if(img3_HSV.at<cv::Vec3b>(i,j)[0] == 0)
      {
        mask.at<cv::Vec3b>(i,j) = dNan;
      }
      if(img3_HSV.at<cv::Vec3b>(i,j)[1] < 100)
      {
        mask.at<cv::Vec3b>(i,j) = dNan;
      }
    }
  }

  cv::Point pixel_ul(mask.rows, mask.cols);
  cv::Point pixel_br(0, 0);

  for(int k = 0; k < mask.cols; k++)
  {
    for(int l = 0; l < mask.rows; l++)
    {
      if(mask.at<cv::Vec3b>(l,k)[0] == white)
      {
        robot_flag = true;

        if(k < pixel_ul.x)
        {
          pixel_ul.x = k;
        }
        if(k > pixel_br.x)
        {
          pixel_br.x = k;
        }
        if(l < pixel_ul.y)
        {
          pixel_ul.y = l;
        }
        if(l > pixel_br.y)
        {
          pixel_br.y = l;
        }
      }
    }
  }*/

  /// Find the minimum template size (upper left pixel and bottom right pixel)
  cv::Point pixel_ul(mask.rows, mask.cols);
  cv::Point pixel_br(0, 0);

  for(int k = 0; k < mask.cols; k++)
  {
    for(int l = 0; l < mask.rows; l++)
    {
      if(mask.at<cv::Vec4b>(l,k)[0] == white)
      {
        robot_flag = true;

        if(k < pixel_ul.x)
        {
          pixel_ul.x = k;
        }
        if(k > pixel_br.x)
        {
          pixel_br.x = k;
        }
        if(l < pixel_ul.y)
        {
          pixel_ul.y = l;
        }
        if(l > pixel_br.y)
        {
          pixel_br.y = l;
        }
      }
    }
  }

  double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc; cv::Point matchLoc;

  if(robot_flag == true)
  {  
    cv::Mat resized_templ = new_templ(cv::Rect(pixel_ul.x, pixel_ul.y, pixel_br.x - pixel_ul.x + 1, pixel_br.y - pixel_ul.y + 1));
    cv::Mat resized_mask = mask(cv::Rect(pixel_ul.x, pixel_ul.y, pixel_br.x - pixel_ul.x + 1,pixel_br.y - pixel_ul.y + 1));
    //cv::Mat resized_templRGB = new_templRGB(cv::Rect(pixel_ul.x, pixel_ul.y, pixel_br.x - pixel_ul.x + 1,pixel_br.y - pixel_ul.y + 1)); // in order to have no HSV transformation
    //cv::Mat resized_templ_nodepth = img3_HSV(cv::Rect(pixel_ul.x, pixel_ul.y, pixel_br.x - pixel_ul.x + 1, pixel_br.y - pixel_ul.y + 1)); // in order to have no depth channel

    /// Do the Matching
    matchTemplate( new_img, resized_templ, result, match_method, resized_mask);
    //matchTemplate( new_imgRGB, resized_templRGB, result, match_method, resized_mask); // in order to have no HSV transformation
    //matchTemplate( img1_HSV, resized_templ_nodepth, result, match_method, resized_mask); // in order to have no depth channel

    /// Localizing the best match with minMaxLoc
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For TM_CCORR_NORMED, the best matches are higher values.
    matchLoc = maxLoc;

    /// Show me what you got
    rectangle( img_display, matchLoc, cv::Point( matchLoc.x + resized_templ.cols , matchLoc.y + resized_templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

    viewImage(img_display);
  }
  else
  {
    maxVal = 0;

    viewImage(img_display);
  }

  std_msgs::Float64 score;

  score.data = maxVal;
  score_pub.publish(score);

  std::cout << "Score: " << score.data << " ";

  probability = probability * score.data;
}

bool ImageSubscribers::getNodeHandle() {
  
  return nodeHandle_->ok();
}

double ImageSubscribers::getProbability() {
  
  return probability;
}

void ImageSubscribers::normalizeProbability(double norm) {
  
  if(norm == 0)
  {
    norm = 1;
  }

  probability = probability / norm;
}

void ImageSubscribers::run() {
  
  if(!nodeHandle_)
  {
    ROS_ERROR("No node handle, cannot proceed.");
    return;
  }

  conversionImages(latest_rgb_real_image_, latest_depth_real_image_, latest_rgb_synth_image_, latest_depth_synth_image_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nodeHandle;

  int nb_virtual_cameras = 1;
  double sum = 0;
  std::unique_ptr<ImageSubscribers> tab_cameras[nb_virtual_cameras];

  for(int k = 0; k < nb_virtual_cameras; k++)
  {
      tab_cameras[k].reset(new ImageSubscribers("synthetic_camera" + std::to_string(k + 1),"score" + std::to_string(k + 1)));

      tab_cameras[k]->init(&nodeHandle);
  }

  ros::Rate rate(10.0);
  while (tab_cameras[0]->getNodeHandle())
  {  
      for(int k = 0; k < nb_virtual_cameras; k++)
      {
        tab_cameras[k]->run();

        sum = sum + tab_cameras[k]->getProbability();
      }

      for(int k = 0; k < nb_virtual_cameras; k++)
      {
        tab_cameras[k]->normalizeProbability(sum);
      }
      sum = 0;

      std::cout << ";\n";
      std::cout << "Probability: " << tab_cameras[0]->getProbability() << " " << ";\n";

      ros::spinOnce();
      rate.sleep();
  }

  return 0;
}
