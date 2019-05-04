/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <image_proc/CropDecimateConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <std_msgs/Bool.h>

namespace image_proc {

using namespace cv;
using namespace cv_bridge; // CvImage, toCvShare

class StopSignNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
<<<<<<< HEAD
  image_transport::Subscriber sub_;
=======
  image_transport::CameraSubscriber sub_;
>>>>>>> b4ce56b6d189db643386a029d51c3638e0a5fc54
  int queue_size_;
  std::string target_frame_id_;

  boost::mutex connect_mutex_;
<<<<<<< HEAD
  ros::Publisher pub_stop_sign_;
=======
  image_transport::CameraPublisher pub_h_scans_;
    ros::Publisher pub_stop_sign_;// image_transport::Publisher pub_stop_sign_;
>>>>>>> b4ce56b6d189db643386a029d51c3638e0a5fc54

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::CropDecimateConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

<<<<<<< HEAD
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
=======
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
>>>>>>> b4ce56b6d189db643386a029d51c3638e0a5fc54

  void configCb(Config &config, uint32_t level);

};

void StopSignNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);
  private_nh.param("target_frame_id", target_frame_id_, std::string());

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&StopSignNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
<<<<<<< HEAD
  image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it_in_->subscribe("image_raw", queue_size_, &StopSignNodelet::imageCb, this, hints);
  pub_stop_sign_ = nh.advertise<std_msgs::Bool>("stop_sign",  10);
}

void StopSignNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
=======

  // Monitor whether anyone is subscribed to the h_scans
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&StopSignNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&StopSignNodelet::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to pub_h_scans_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_h_scans_ = it_out_->advertiseCamera("h_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  pub_stop_sign_ = nh.advertise<std_msgs::Bool>("stop_sign",  10);
}

// Handles (un)subscribing when clients (un)subscribe
void StopSignNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_h_scans_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &StopSignNodelet::imageCb, this, hints);
  }
}

void StopSignNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
>>>>>>> b4ce56b6d189db643386a029d51c3638e0a5fc54
{
  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }

  // Get a Mat view of the source data
  CvImageConstPtr img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGB8);

  // https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color

  Mat img_hsv;
  // Mat img_ptr->image = imread("/home/gene/stop_sign_ws/stop_sign.png");
  cvtColor(img_ptr->image, img_hsv, CV_RGB2HSV);

  Mat mask_red_orange, mask_red_pink, mask;
  
  // Masking colors (low end of Hue - red orange)
<<<<<<< HEAD
  inRange(img_hsv, Scalar(  0, 75, 75), Scalar(  2, 255, 255), mask_red_orange);
=======
  inRange(img_hsv, Scalar(  0, 75, 75), Scalar(  2,255, 255), mask_red_orange);
>>>>>>> b4ce56b6d189db643386a029d51c3638e0a5fc54

  // Masking colors (high end of Hue - red pink)
  inRange(img_hsv, Scalar(178, 75, 75), Scalar(180, 255, 255), mask_red_pink);
  
  // Union two masks to get "red" mask
  mask = mask_red_orange | mask_red_pink;

  // Apply red mask
  Mat img_hsv_redmasked;
  bitwise_and(img_hsv, img_hsv, img_hsv_redmasked, mask=mask);

  // Convert to grayscale
  Mat img_redmasked_gray;
  cvtColor(img_hsv_redmasked, img_redmasked_gray, COLOR_BGR2GRAY);

  // Convert to binary image
  Mat img_redmasked_binary;
  threshold(img_redmasked_gray, img_redmasked_binary, 1,255,THRESH_BINARY);

  // https://stackoverflow.com/questions/30369031/remove-spurious-small-islands-of-noise-in-an-image-python-opencv

  // Define the closing and opening structuring elements
  Mat se_closing = getStructuringElement(MORPH_RECT, Size(20, 20));
  Mat se_opening = getStructuringElement(MORPH_RECT, Size(15, 15));

  // Perform closing then opening
  Mat mask_close_open;
  Mat img_masked_binary_closed_open;
  morphologyEx(img_redmasked_binary, mask_close_open, MORPH_CLOSE, se_closing);
  morphologyEx(mask_close_open, img_masked_binary_closed_open, MORPH_OPEN, se_opening);
  
  // Fill in holes in octogon
  Mat img_masked_filled;
  img_masked_filled = Mat::zeros( img_redmasked_binary.size(), CV_8UC1 );
  std::vector< std::vector<Point> > contours;
  std::vector<Point> approx;
  std::vector<Vec4i> hierarchy;
  
  // Invert binary masked image
  Mat img_masked_binary_closed_open_inv;
  bitwise_not(img_masked_binary_closed_open, img_masked_binary_closed_open_inv);
    
  // Show masked image
  imshow("view1", img_masked_binary_closed_open);

  // https://stackoverflow.com/questions/8076889/how-to-use-opencv-simpleblobdetector
  
  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;
  
  // Change thresholds
  params.minThreshold = 1;
  params.maxThreshold = 255;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 1000;
  params.maxArea = 153600;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.75;
  params.maxCircularity = 1.0;
  
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.9;
  params.maxConvexity = 1.0;
  
  // Filter by Inertia
  params.filterByInertia = false;
  
  #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
    // Set up detector with params
    cd::SimpleBlobDetector detector(params);
  #else
    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
  #endif

  // Detect blobs.
  std::vector<KeyPoint> keypoints;
  detector->detect( img_masked_binary_closed_open_inv, keypoints);
 
  // Draw detected blobs as red circles.
  Mat im_with_keypoints, img_blank;
  img_blank = Mat::zeros( img_redmasked_binary.size(), CV_8UC1 );
  drawKeypoints( img_blank, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  // Show blobs
  imshow("keypoints", im_with_keypoints );
  
  for(int i = 0; i < keypoints.size(); i++)
  {
    int x = keypoints[i].pt.x;
    int y = keypoints[i].pt.y;
    std::cout << "Stop Sign " << i << ": (" << x << ","<< y << ")" << std::endl;
  }

  std_msgs::Bool stop_sign;
  stop_sign.data = false;
  
  if(keypoints.size() > 0)
  {
    stop_sign.data = true;
  }

  // Keep imshow windows open
  waitKey(1);

  pub_stop_sign_.publish(stop_sign);
 
}

void StopSignNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_proc::StopSignNodelet, nodelet::Nodelet)
