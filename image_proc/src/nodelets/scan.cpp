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

namespace image_proc {

using namespace cv_bridge; // CvImage, toCvShare

class ScanNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  int queue_size_;
  std::string target_frame_id_;

  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_h_scans_;
  image_transport::CameraPublisher pub_h_scan_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::CropDecimateConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void ScanNodelet::onInit()
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
  ReconfigureServer::CallbackType f = boost::bind(&ScanNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the h_scans
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ScanNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ScanNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_h_scans_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_h_scans_ = it_out_->advertiseCamera("h_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  pub_h_scan_ = it_out_->advertiseCamera("h_scan",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  
}

// Handles (un)subscribing when clients (un)subscribe
void ScanNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_h_scans_.getNumSubscribers() == 0 && pub_h_scan_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &ScanNodelet::imageCb, this, hints);
  }
}

void ScanNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  /// @todo Check image dimensions match info_msg
  /// @todo Publish tweaks to config_ so they appear in reconfigure_gui

  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }
  int decimation_x = config.decimation_x;
  int decimation_y = config.decimation_y;

  int max_width = image_msg->width - config.x_offset;
  int max_height = image_msg->height - config.y_offset;
  int width = config.width;
  int height = config.height;
  if (width == 0 || width > max_width)
    width = max_width;
  if (height == 0 || height > max_height)
    height = max_height;

  // Get a cv::Mat view of the source data
  CvImageConstPtr source = toCvShare(image_msg);

  CvImage h_scans(source->header, source->encoding);
  CvImage h_scans_float(source->header, source->encoding);
  CvImage h_scan(source->header, "32FC1");

  h_scans.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, height));
  h_scans_float.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, height));
  h_scan.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, 1));

  ////////////////////////////////////
  // CALCULATE HORIZONTAL LINE SCAN //
  ////////////////////////////////////

  // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
  h_scans.image.convertTo(h_scans_float.image, CV_32FC1,0.001); // [mm] --> [m]
  
  cv::Mat row_mean;
  cv::Mat col_mean;
  reduce(h_scans_float.image, h_scan.image, 0, CV_REDUCE_AVG, CV_32FC1); // 0 = AVERAGE COLUMNS

  // std::cout << h_scan.image << std::endl;

  // Create h_scans Image message
  sensor_msgs::ImagePtr out_h_scans = h_scans.toImageMsg();
  sensor_msgs::ImagePtr out_h_scan = h_scan.toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  int binning_x = std::max((int)info_msg->binning_x, 1);
  int binning_y = std::max((int)info_msg->binning_y, 1);
  out_info->binning_x = binning_x;
  out_info->binning_y = binning_y;
  out_info->roi.x_offset += config.x_offset * binning_x;
  out_info->roi.y_offset += config.y_offset * binning_y;
  out_info->roi.height = height * binning_y;
  out_info->roi.width = width * binning_x;
  
  // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
  if (width != (int)image_msg->width || height != (int)image_msg->height)
    out_info->roi.do_rectify = true;

  if (!target_frame_id_.empty()) {
    out_h_scans->header.frame_id = target_frame_id_;
    out_info->header.frame_id = target_frame_id_;
  }

  pub_h_scans_.publish(out_h_scans, out_info);
  pub_h_scan_.publish(out_h_scan, out_info);

  
}

void ScanNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_proc::ScanNodelet, nodelet::Nodelet)
