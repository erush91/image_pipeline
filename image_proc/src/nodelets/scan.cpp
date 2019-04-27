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
  image_transport::CameraPublisher pub_;
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

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ScanNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ScanNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_ = it_out_->advertiseCamera("h_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  pub_h_scan_ = it_out_->advertiseCamera("h_scan",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  
}

// Handles (un)subscribing when clients (un)subscribe
void ScanNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &ScanNodelet::imageCb, this, hints);
  }
}

template <typename T>
void debayer2x2toBGR(const cv::Mat& src, cv::Mat& dst, int R, int G1, int G2, int B)
{
  typedef cv::Vec<T, 3> DstPixel; // 8- or 16-bit BGR
  dst.create(src.rows / 2, src.cols / 2, cv::DataType<DstPixel>::type);

  int src_row_step = src.step1();
  int dst_row_step = dst.step1();
  const T* src_row = src.ptr<T>();
  T* dst_row = dst.ptr<T>();

  // 2x2 downsample and debayer at once
  for (int y = 0; y < dst.rows; ++y)
  {
    for (int x = 0; x < dst.cols; ++x)
    {
      dst_row[x*3 + 0] = src_row[x*2 + B];
      dst_row[x*3 + 1] = (src_row[x*2 + G1] + src_row[x*2 + G2]) / 2;
      dst_row[x*3 + 2] = src_row[x*2 + R];
    }
    src_row += src_row_step * 2;
    dst_row += dst_row_step;
  }
}

// Templated on pixel size, in bytes (MONO8 = 1, BGR8 = 3, RGBA16 = 8, ...)
template <int N>
void decimate(const cv::Mat& src, cv::Mat& dst, int decimation_x, int decimation_y)
{
  dst.create(src.rows / decimation_y, src.cols / decimation_x, src.type());

  int src_row_step = src.step[0] * decimation_y;
  int src_pixel_step = N * decimation_x;
  int dst_row_step = dst.step[0];

  const uint8_t* src_row = src.ptr();
  uint8_t* dst_row = dst.ptr();
  
  for (int y = 0; y < dst.rows; ++y)
  {
    const uint8_t* src_pixel = src_row;
    uint8_t* dst_pixel = dst_row;
    for (int x = 0; x < dst.cols; ++x)
    {
      memcpy(dst_pixel, src_pixel, N); // Should inline with small, fixed N
      src_pixel += src_pixel_step;
      dst_pixel += N;
    }
    src_row += src_row_step;
    dst_row += dst_row_step;
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

  // Compute the ROI we'll actually use
  bool is_bayer = sensor_msgs::image_encodings::isBayer(image_msg->encoding);

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

  // Except in Bayer downsampling case, output has same encoding as the input
  CvImage output(source->header, source->encoding);
  // Apply ROI (no copy, still a view of the image_msg data)
  output.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, height));

  ////////////////////////////////////
  // CALCULATE HORIZONTAL LINE SCAN //
  ////////////////////////////////////

  // Average row of data and convert to meters
  CvImage output_f(source->header, source->encoding);
  CvImage h_scan(source->header, "32FC1");
  output_f.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, height));
  h_scan.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, 1));
  output.image.convertTo(output_f.image, CV_32FC1,0.001); // [mm] --> [m]
  cv::Mat row_mean, col_mean;
  reduce(output_f.image, h_scan.image, 0, CV_REDUCE_AVG, CV_32FC1); // 0 = AVERAGE COLUMNS
  std::cout << h_scan.image << std::endl;

  // Create output Image message
  /// @todo Could save copies by allocating this above and having output.image alias it
  sensor_msgs::ImagePtr out_image = output.toImageMsg();
  sensor_msgs::ImagePtr out_h_scan = h_scan.toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  int binning_x = std::max((int)info_msg->binning_x, 1);
  int binning_y = std::max((int)info_msg->binning_y, 1);
  out_info->binning_x = binning_x * config.decimation_x;
  out_info->binning_y = binning_y * config.decimation_y;
  out_info->roi.x_offset += config.x_offset * binning_x;
  out_info->roi.y_offset += config.y_offset * binning_y;
  out_info->roi.height = height * binning_y;
  out_info->roi.width = width * binning_x;
  // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
  if (width != (int)image_msg->width || height != (int)image_msg->height)
    out_info->roi.do_rectify = true;

  if (!target_frame_id_.empty()) {
    out_image->header.frame_id = target_frame_id_;
    out_info->header.frame_id = target_frame_id_;
  }

  pub_.publish(out_image, out_info);
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
