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

#include <image_proc/FourierCoefsMsg.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>


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
  ros::Publisher pub_h_scan_;
  ros::Publisher pub_h_nearness_; 
  ros::Publisher pub_wfi_fourier_;
  ros::Publisher pub_wfi_control_command_;

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
  image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &ScanNodelet::imageCb, this, hints);

  // Publisher rows of depth image used in horizontal line scan
  pub_h_scans_ = it_out_->advertiseCamera("h_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
  
  // Publisher horizontal line scan
  pub_h_scan_ = nh.advertise<std_msgs::Float32MultiArray>("h_scan", 10);

  // Publisher horizontal nearnes
  pub_h_nearness_ = nh.advertise<std_msgs::Float32MultiArray>("h_nearness", 10);

  // Publisher Fourier coefficients
  pub_wfi_fourier_ = nh.advertise<image_proc::FourierCoefsMsg>("wfi_fourier_coefficients", 10);

  // Publisher WFI control command
  pub_wfi_control_command_ = nh.advertise<geometry_msgs::Twist>("wfi_control_command", 10);
  
}

// Handles (un)subscribing when clients (un)subscribe;
void ScanNodelet::connectCb(){}

void ScanNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                          const sensor_msgs::CameraInfoConstPtr& info_msg)
{

//////////////////////////////////////////////////////////////////////////
// PROCESS DEPTH IMAGE ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

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
  {
    out_info->roi.do_rectify = true;
  }

  if (!target_frame_id_.empty())
  {
    out_h_scans->header.frame_id = target_frame_id_;
    out_info->header.frame_id = target_frame_id_;
  }

  pub_h_scans_.publish(out_h_scans, out_info);

  ////////////////////////////////////
  // Saturate horizontal depth scan //
  ////////////////////////////////////

  cv::Mat depth_sat = cv::Mat::zeros(cv::Size(1, 580), CV_32FC1);
  depth_sat = h_scan.image;

  depth_sat.setTo(0.5, depth_sat < 0.5);
  depth_sat.setTo(10, depth_sat > 10);
  
  /////////////////////////////////
  // Publish WFI horizontal scan //
  /////////////////////////////////

  std::vector<float> h_scan_array(depth_sat.begin<float>(), depth_sat.end<float>());

  std_msgs::Float32MultiArray h_scan_msg;
  h_scan_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  h_scan_msg.layout.dim[0].size = h_scan_array.size();
  h_scan_msg.data.clear();
  h_scan_msg.data.insert(h_scan_msg.data.end(), h_scan_array.begin(), h_scan_array.end());

  pub_h_scan_.publish(h_scan_msg);

//////////////////////////////////////////////////////////////////////////
// CALCULATE WIDE FIELD INTEGRATION //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

  ///////////////////////
  // Declare variables //
  ///////////////////////
  float dg;
  int num_points;
  num_points = 580;
  int num_fourier_terms;
  num_fourier_terms = 4;
  float fourier_terms;
  for (int i = 1; i < num_fourier_terms + 1; i++)
  {
    fourier_terms = i;
  }
  float gamma_arr[num_points];
  float cos_gamma_arr[num_fourier_terms][num_points];
  float sin_gamma_arr[num_fourier_terms][num_points];
  float a_0, a[num_fourier_terms], b[num_fourier_terms];

  float gamma_start_FOV;
  float gamma_end_FOV;
  float gamma_range_FOV;
  float gamma_delta_FOV;
 
  /////////////////////////
  // Intialize variables //
  /////////////////////////
  gamma_start_FOV = -0.25 * M_PI;
  gamma_end_FOV = 0.25 * M_PI;
  gamma_range_FOV = gamma_end_FOV - gamma_start_FOV;
  gamma_delta_FOV = gamma_range_FOV / num_points;
  
  ///////////////////////////////////
  // Calculate horizontal nearness //
  ///////////////////////////////////
  
  cv::Mat nearness = cv::Mat::zeros(cv::Size(1, 580), CV_32FC1);

  nearness = 1.0 / depth_sat;

  /////////////////////////////////////
  // Publish WFI horizontal nearness //
  /////////////////////////////////////

  std::vector<float> nearness_array(nearness.begin<float>(), nearness.end<float>());
  
  std_msgs::Float32MultiArray nearness_msg;
  nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  nearness_msg.layout.dim[0].size = nearness_array.size();
  nearness_msg.data.clear();
  nearness_msg.data.insert(nearness_msg.data.end(), nearness_array.begin(), nearness_array.end());

  pub_h_nearness_.publish(nearness_msg);

  cv::Mat cos_gamma_mat(num_fourier_terms + 1, num_points, CV_32FC1, cos_gamma_arr);
  cv::Mat sin_gamma_mat(num_fourier_terms + 1, num_points, CV_32FC1, sin_gamma_arr);

  ////////////////////////////////////
  // Calculate Fourier coefficients //
  ////////////////////////////////////
  for (int i = 0; i < num_fourier_terms + 1; i++)
  {
    for (int j = 0; j < num_points; j++)
    {
      gamma_arr[j] = gamma_start_FOV + gamma_delta_FOV * j;
      cos_gamma_arr[i][j] = cos(i * gamma_arr[j]);
      sin_gamma_arr[i][j] = sin(i * gamma_arr[j]);
    }
    
    if(i == 0)
    {
      a_0 = nearness.dot(cos_gamma_mat.row(i)) * gamma_delta_FOV / (0.5 * gamma_range_FOV);
    }
    else if(i > 0)
    {
      a[i-1] = nearness.dot(cos_gamma_mat.row(i)) * gamma_delta_FOV / (0.5 * gamma_range_FOV);
      b[i-1] = nearness.dot(sin_gamma_mat.row(i)) * gamma_delta_FOV / (0.5 * gamma_range_FOV);
    }
  }

  // /////////////////////////
  // // Print cos_gamma_arr //
  // /////////////////////////
  // std::cout << "cos_gamma_arr = [";
  // for (int i = 0; i < num_fourier_terms; i++)
  // {
  //   std::cout << "[";
  //   for (int j = 0; j < num_points; j++)
  //   {
  //     std::cout << cos_gamma_arr[i][j] << ",";
  //   }
  //   std::cout << "]" << std::endl;
  // }
  // std::cout << "]" << std::endl;  

  // /////////////////////////
  // // Print sin_gamma_arr //
  // /////////////////////////
  // std::cout << "sin_gamma_arr = [";
  // for (int i = 0; i < num_fourier_terms; i++)
  // {
  //   std::cout << "[";
  //   for (int j = 0; j < num_points; j++)
  //   {
  //     std::cout << sin_gamma_arr[i][j] << ",";
  //   }
  //   std::cout << "]" << std::endl;
  // }
  // std::cout << "]" << std::endl;  
  
  // //////////////////////////////////////////
  // // Print Fourier cosine coefficients, a //
  // //////////////////////////////////////////
  // std::cout << "a = [";
  // for (int i = 0; i < num_fourier_terms + 1; i++)
  // {
  //   std::cout << a[i] << ",";  
  // }
  // std::cout << "]" << std::endl;

  // ////////////////////////////////////////
  // // Print Fourier sine coefficients, b //
  // ////////////////////////////////////////
  // std::cout << "b = [";
  // for (int i = 0; i < num_fourier_terms + 1; i++)
  // {
  //   std::cout << b[i] << ",";  
  // }
  // std::cout << "]" << std::endl;

  // ////////////////////
  // // Print nearness //
  // ////////////////////
  // std::cout << "nearness = ";
  // std::cout << nearness << std::endl;

  //////////////////////////////////////
  // Publish WFI Fourier coefficients //
  //////////////////////////////////////

  // Convert array to vector
  std::vector<float> a_vector(a, a + sizeof a / sizeof a[0]);
  std::vector<float> b_vector(b, b + sizeof b / sizeof b[0]);

  image_proc::FourierCoefsMsg wfi_ctrl_cmd_msg;

  wfi_ctrl_cmd_msg.header.stamp = ros::Time::now();
  wfi_ctrl_cmd_msg.a_0 = a_0;
  wfi_ctrl_cmd_msg.a = a_vector;
  wfi_ctrl_cmd_msg.b = b_vector;

  pub_wfi_fourier_.publish(wfi_ctrl_cmd_msg);

  ////////////////////////////////////////
  // Calculate forward velocity command //
  ////////////////////////////////////////

  // Forward velocity gain
  float K_03 = 0.1;

  // Scaling factor
  float N = 10;

  // Reference velocity
  float v_0 = 0.5;

  // Fourier coefficient
  float a_1 = a[0];
  float a_2 = a[1];

  // Min and max forward velocity limits
  float v_min = 0.1;
  float v_max = 2.0;

  // Forward velocity control law
  float wfi_forward_velocity_control = 1 - v_max * (a_0 - a_2);

  // Saturate forward velocity command
  if(wfi_forward_velocity_control < v_min)
    wfi_forward_velocity_control = v_min;
  if(wfi_forward_velocity_control > v_max)
    wfi_forward_velocity_control = v_min;

  ////////////////////////////////
  // Calculate yaw rate command //
  ////////////////////////////////

  // Lateral position gain
  float K_1 = 0; // -0.500;  // K_01 < 0 for stability
  
  // Yaw angle gain
  float K_2 =  0.5; // 0.575;

  // Fourier coefficient
  float b_1 = b[1];
  float b_2 = b[2];

  float wfi_yaw_rate_control = K_1 * b_1 + K_2 * b_2;

  // Min and max yaw rate limits
  float yaw_rate_min = -2.0;
  float yaw_rate_max =  2.0;

  if(wfi_yaw_rate_control < yaw_rate_min)
    wfi_yaw_rate_control = yaw_rate_min;
  if(wfi_yaw_rate_control > yaw_rate_max)
    wfi_yaw_rate_control = yaw_rate_max;

  //////////////////////////////////
  // Publish WFI control commands //
  //////////////////////////////////

  geometry_msgs::Twist wfi_control_command;

  wfi_control_command.linear.x = wfi_forward_velocity_control;
  wfi_control_command.linear.y = 0;
  wfi_control_command.linear.z = 0;
  wfi_control_command.angular.x = 0;
  wfi_control_command.angular.y = 0;
  wfi_control_command.angular.z = wfi_yaw_rate_control;

  pub_wfi_control_command_.publish(wfi_control_command);

}

void ScanNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_proc::ScanNodelet, nodelet::Nodelet)
