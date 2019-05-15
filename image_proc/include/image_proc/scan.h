/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
#include <image_proc/ScanConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <image_proc/FourierCoefsMsg.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_proc/depth_traits.h>
#include <sstream>
#include <limits.h>
#include <math.h>

using namespace cv_bridge; // CvImage, toCvShare

namespace image_proc
{ 
class ScanNodelet : public nodelet::Nodelet
{


  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  int queue_size_;
  std::string target_frame_id_;
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_h_scans_;
  image_transport::CameraPublisher pub_v_scans_;
  ros::Publisher pub_wfi_h_scan_;
  ros::Publisher pub_wfi_v_scan_;
  ros::Publisher pub_wfi_h_nearness_; 
  ros::Publisher pub_wfi_v_nearness_; 
  ros::Publisher pub_wfi_h_fourier_coefficients_;
  ros::Publisher pub_wfi_v_fourier_coefficients_;
  ros::Publisher pub_wfi_control_commands_;
  ros::Publisher pub_wfi_junctionness_;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  
  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::ScanConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void onInit()
  {
    nh         = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    ros::NodeHandle nh_wfi (nh, "wfi");
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
    
    // Publisher rows of depth image used in vertical line scan
    pub_v_scans_ = it_out_->advertiseCamera("v_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
    
    // Publisher horizontal line scan
    pub_wfi_h_scan_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("horiz/scan", 10);

    // Publisher vertical line scan
    pub_wfi_v_scan_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("vert/scan", 10);

    // Publisher horizontal nearnes
    pub_wfi_h_nearness_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("horiz/nearness", 10);

    // Publisher vertical nearnes
    pub_wfi_v_nearness_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("vert/nearness", 10);

    // Publisher horizontaol Fourier coefficients
    pub_wfi_h_fourier_coefficients_ = nh_wfi.advertise<image_proc::FourierCoefsMsg>("horiz/fourier_coefficients", 10);

    // Publisher vertical Fourier coefficients
    pub_wfi_v_fourier_coefficients_ = nh_wfi.advertise<image_proc::FourierCoefsMsg>("vert/fourier_coefficients", 10);

    // Publisher WFI control command
    pub_wfi_control_commands_ = nh_wfi.advertise<geometry_msgs::Twist>("control_commands", 10);

    // Publisher WFI junctionness
    pub_wfi_junctionness_ = nh_wfi.advertise<std_msgs::Float32>("junctionness", 10);
  }

  // Handles (un)subscribing when clients (un)subscribe;
  void connectCb(){}

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
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

    int max_h_width = image_msg->width - config.h_x_offset;
    int max_h_height = image_msg->height - config.h_y_offset;
    int h_width = config.h_width;
    int h_height = config.h_height;

    if (h_width == 0 || h_width > max_h_width)
      h_width = max_h_width;
    if (h_height == 0 || h_height > max_h_height)
      h_height = max_h_height;

    int max_v_width = image_msg->width - config.v_x_offset;
    int max_v_height = image_msg->height - config.v_y_offset;
    int v_width = config.v_width;
    int v_height = config.v_height;

    if (v_width == 0 || v_width > max_v_width)
      v_width = max_v_width;
    if (v_height == 0 || v_height > max_v_height)
      v_height = max_v_height;

    // Get h_a cv::Mat view of the source data
    CvImageConstPtr source = toCvShare(image_msg);

    //////////////////////////////////
    // Get Selected Horizontal Rows //
    //////////////////////////////////

    CvImage h_scans(source->header, source->encoding);
    CvImage h_scans_float(source->header, source->encoding);
    CvImage h_scan(source->header, "32FC1");

    h_scans.image = source->image(cv::Rect(config.h_x_offset, config.h_y_offset, h_width, h_height));
    h_scans_float.image = source->image(cv::Rect(config.h_x_offset, config.h_y_offset, h_width, h_height));
    h_scan.image = source->image(cv::Rect(config.h_x_offset, config.h_y_offset, h_width, 1));

    ///////////////////////////////
    // Get Seleted Vertical Rows //
    ///////////////////////////////

    CvImage v_scans(source->header, source->encoding);
    CvImage v_scans_float(source->header, source->encoding);
    CvImage v_scan(source->header, "32FC1");

    v_scans.image = source->image(cv::Rect(config.v_x_offset, config.v_y_offset, v_width, v_height));
    v_scans_float.image = source->image(cv::Rect(config.v_x_offset, config.v_y_offset, v_width, v_height));
    v_scan.image = source->image(cv::Rect(config.v_x_offset, config.v_y_offset, 1, v_height));

    ////////////////////////////////////
    // Calculate Horizontal Line Scan //
    ////////////////////////////////////

    // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
    h_scans.image.convertTo(h_scans_float.image, CV_32FC1,0.001); // [mm] --> [m]
    
    reduce(h_scans_float.image, h_scan.image, 0, CV_REDUCE_AVG, CV_32FC1); // 0 = AVERAGE COLUMNS

    //////////////////////////////////
    // Calculate Vertical Line Scan //
    //////////////////////////////////

    // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
    v_scans.image.convertTo(v_scans_float.image, CV_32FC1,0.001); // [mm] --> [m]
    
    reduce(v_scans_float.image, v_scan.image, 1, CV_REDUCE_AVG, CV_32FC1); // 1 = AVERAGE ROWS
    
    ///////////////////////////////////
    // Publish Horizontal Line Scans //
    ///////////////////////////////////

    // Create h_scans Image message
    sensor_msgs::ImagePtr out_h_scans = h_scans.toImageMsg();

    // Create updated CameraInfo message
    sensor_msgs::CameraInfoPtr out_h_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
    int h_binning_x = std::max((int)info_msg->binning_x, 1);
    int h_binning_y = std::max((int)info_msg->binning_y, 1);
    out_h_info->binning_x = h_binning_x;
    out_h_info->binning_y = h_binning_y;
    out_h_info->roi.x_offset += config.h_x_offset * h_binning_x;
    out_h_info->roi.y_offset += config.h_y_offset * h_binning_y;
    out_h_info->roi.width = h_width * h_binning_x;
    out_h_info->roi.height = h_height * h_binning_y;
    
    // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
    if (h_width != (int)image_msg->width || h_height != (int)image_msg->height)
    {
      out_h_info->roi.do_rectify = true;
    }

    if (!target_frame_id_.empty())
    {
      out_h_scans->header.frame_id = target_frame_id_;
      out_h_info->header.frame_id = target_frame_id_;
    }

    pub_h_scans_.publish(out_h_scans, out_h_info);

    /////////////////////////////////
    // Publish Vertical Line Scans //
    /////////////////////////////////

    // Create h_scans Image message
    sensor_msgs::ImagePtr out_v_scans = v_scans.toImageMsg();

    // Create updated CameraInfo message
    sensor_msgs::CameraInfoPtr out_v_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
    int v_binning_x = std::max((int)info_msg->binning_x, 1);
    int v_binning_y = std::max((int)info_msg->binning_y, 1);
    out_v_info->binning_x = v_binning_x;
    out_v_info->binning_y = v_binning_y;
    out_v_info->roi.x_offset += config.v_x_offset * v_binning_x;
    out_v_info->roi.y_offset += config.v_y_offset * v_binning_y;
    out_v_info->roi.width = v_width * v_binning_x;
    out_v_info->roi.height = v_height * v_binning_y;
    
    // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
    if (v_width != (int)image_msg->width || v_height != (int)image_msg->height)
    {
      out_v_info->roi.do_rectify = true;
    }

    if (!target_frame_id_.empty())
    {
      out_v_scans->header.frame_id = target_frame_id_;
      out_v_info->header.frame_id = target_frame_id_;
    }

    pub_v_scans_.publish(out_v_scans, out_v_info);

    ////////////////////////////////////
    // Saturate horizontal depth scan //
    ////////////////////////////////////

    cv::Mat h_depth_sat = cv::Mat::zeros(cv::Size(1, h_width), CV_32FC1);
    h_depth_sat = h_scan.image;

    h_depth_sat.setTo(0.5, h_depth_sat < 0.5);
    h_depth_sat.setTo(10, h_depth_sat > 10);

    //////////////////////////////////
    // Saturate vertical depth scan //
    //////////////////////////////////

    cv::Mat v_depth_sat = cv::Mat::zeros(cv::Size(1, v_height), CV_32FC1);
    cv::transpose(v_scan.image, v_depth_sat);
    
    v_depth_sat.setTo(0.5, v_depth_sat < 0.5);
    v_depth_sat.setTo(10, v_depth_sat > 10);
    
    ////////////////////////////////////
    // Publish horizontal depth scan  //
    ////////////////////////////////////

    std::vector<float> h_scan_array(h_depth_sat.begin<float>(), h_depth_sat.end<float>());

    std_msgs::Float32MultiArray h_scan_msg;
    h_scan_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    h_scan_msg.layout.dim[0].size = h_scan_array.size();
    h_scan_msg.data.clear();
    h_scan_msg.data.insert(h_scan_msg.data.end(), h_scan_array.begin(), h_scan_array.end());

    pub_wfi_h_scan_.publish(h_scan_msg);

    /////////////////////////////////
    // Publish vertical depth scan //
    /////////////////////////////////

    std::vector<float> v_scan_array(v_depth_sat.begin<float>(), v_depth_sat.end<float>());

    std_msgs::Float32MultiArray v_scan_msg;
    v_scan_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    v_scan_msg.layout.dim[0].size = v_scan_array.size();
    v_scan_msg.data.clear();
    v_scan_msg.data.insert(v_scan_msg.data.end(), v_scan_array.begin(), v_scan_array.end());

    pub_wfi_v_scan_.publish(v_scan_msg);

    //////////////////////////////////////////////////////////////////////////
    // CALCULATE WIDE FIELD INTEGRATION //////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////
    // Declare horizontal variables //
    //////////////////////////////////

    int h_num_points;
    h_num_points = h_width;
    int h_num_fourier_terms;
    h_num_fourier_terms = 4;
    float h_fourier_terms;
    for (int i = 1; i < h_num_fourier_terms + 1; i++)
    {
      h_fourier_terms = i;
    }
    float h_gamma_arr[h_num_points];
    float h_cos_gamma_arr[h_num_fourier_terms][h_num_points];
    float h_sin_gamma_arr[h_num_fourier_terms][h_num_points];
    float h_a_0, h_a[h_num_fourier_terms], h_b[h_num_fourier_terms];

    float h_gamma_start_FOV;
    float h_gamma_end_FOV;
    float h_gamma_range_FOV;
    float h_gamma_delta_FOV;
  
    ////////////////////////////////
    // Declare vertical variables //
    ////////////////////////////////

    int v_num_points;
    v_num_points = v_height;
    int v_num_fourier_terms;
    v_num_fourier_terms = 4;
    float v_fourier_terms;
    for (int i = 1; i < v_num_fourier_terms + 1; i++)
    {
      v_fourier_terms = i;
    }
    float v_gamma_arr[v_num_points];
    float v_cos_gamma_arr[v_num_fourier_terms][v_num_points];
    float v_sin_gamma_arr[v_num_fourier_terms][v_num_points];
    float v_a_0, v_a[v_num_fourier_terms], v_b[v_num_fourier_terms];

    float v_gamma_start_FOV;
    float v_gamma_end_FOV;
    float v_gamma_range_FOV;
    float v_gamma_delta_FOV;
  
    ////////////////////////////////////
    // Intialize horizontal variables //
    ////////////////////////////////////

    h_gamma_start_FOV = -0.25 * M_PI;
    h_gamma_end_FOV = 0.25 * M_PI;
    h_gamma_range_FOV = h_gamma_end_FOV - h_gamma_start_FOV;
    h_gamma_delta_FOV = h_gamma_range_FOV / h_num_points;
    
    //////////////////////////////////
    // Intialize vertical variables //
    //////////////////////////////////

    v_gamma_start_FOV = -0.25 * M_PI;
    v_gamma_end_FOV = 0.25 * M_PI;
    v_gamma_range_FOV = v_gamma_end_FOV - v_gamma_start_FOV;
    v_gamma_delta_FOV = v_gamma_range_FOV / v_num_points;

    ///////////////////////////////////
    // Calculate horizontal nearness //
    ///////////////////////////////////
    
    cv::Mat h_nearness = cv::Mat::zeros(cv::Size(1, h_num_points), CV_32FC1);

    h_nearness = 1.0 / h_depth_sat;

    /////////////////////////////////
    // Calculate vertical nearness //
    /////////////////////////////////
    
    cv::Mat v_nearness = cv::Mat::zeros(cv::Size(v_num_points,1), CV_32FC1);

    v_nearness = 1.0 / v_depth_sat;

    /////////////////////////////////
    // Publish horizontal nearness //
    /////////////////////////////////

    std::vector<float> h_nearness_array(h_nearness.begin<float>(), h_nearness.end<float>());
    
    std_msgs::Float32MultiArray h_nearness_msg;
    h_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    h_nearness_msg.layout.dim[0].size = h_nearness_array.size();
    h_nearness_msg.data.clear();
    h_nearness_msg.data.insert(h_nearness_msg.data.end(), h_nearness_array.begin(), h_nearness_array.end());

    pub_wfi_h_nearness_.publish(h_nearness_msg);

    cv::Mat h_cos_gamma_mat(h_num_fourier_terms + 1, h_num_points, CV_32FC1, h_cos_gamma_arr);
    cv::Mat h_sin_gamma_mat(h_num_fourier_terms + 1, h_num_points, CV_32FC1, h_sin_gamma_arr);

    ///////////////////////////////
    // Publish vertical nearness //
    ///////////////////////////////

    std::vector<float> v_nearness_array(v_nearness.begin<float>(), v_nearness.end<float>());
    
    std_msgs::Float32MultiArray v_nearness_msg;
    v_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    v_nearness_msg.layout.dim[0].size = v_nearness_array.size();
    v_nearness_msg.data.clear();
    v_nearness_msg.data.insert(v_nearness_msg.data.end(), v_nearness_array.begin(), v_nearness_array.end());

    pub_wfi_v_nearness_.publish(v_nearness_msg);

    cv::Mat v_cos_gamma_mat(v_num_fourier_terms + 1, v_num_points, CV_32FC1, v_cos_gamma_arr);
    cv::Mat v_sin_gamma_mat(v_num_fourier_terms + 1, v_num_points, CV_32FC1, v_sin_gamma_arr);

    ///////////////////////////////////////////////
    // Calculate horizontal Fourier coefficients //
    ///////////////////////////////////////////////

    for (int i = 0; i < h_num_fourier_terms + 1; i++)
    {
      for (int j = 0; j < h_num_points; j++)
      {
        h_gamma_arr[j] = h_gamma_start_FOV + h_gamma_delta_FOV * j;
        h_cos_gamma_arr[i][j] = cos(i * h_gamma_arr[j]);
        h_sin_gamma_arr[i][j] = sin(i * h_gamma_arr[j]);
      }
      
      if(i == 0)
      {
        h_a_0 = h_nearness.dot(h_cos_gamma_mat.row(i)) * h_gamma_delta_FOV / (0.5 * h_gamma_range_FOV);
      }
      else if(i > 0)
      {
        h_a[i-1] = h_nearness.dot(h_cos_gamma_mat.row(i)) * h_gamma_delta_FOV / (0.5 * h_gamma_range_FOV);
        h_b[i-1] = h_nearness.dot(h_sin_gamma_mat.row(i)) * h_gamma_delta_FOV / (0.5 * h_gamma_range_FOV);
      }
    }

    /////////////////////////////////////////////
    // Calculate vertical Fourier coefficients //
    /////////////////////////////////////////////
    
    for (int i = 0; i < v_num_fourier_terms + 1; i++)
    {
      for (int j = 0; j < v_num_points; j++)
      {
        v_gamma_arr[j] = v_gamma_start_FOV + v_gamma_delta_FOV * j;
        v_cos_gamma_arr[i][j] = cos(i * v_gamma_arr[j]);
        v_sin_gamma_arr[i][j] = sin(i * v_gamma_arr[j]);
      }
      
      if(i == 0)
      {
        v_a_0 = v_nearness.dot(v_cos_gamma_mat.row(i)) * v_gamma_delta_FOV / (0.5 * v_gamma_range_FOV);
      }
      else if(i > 0)
      {
        v_a[i-1] = v_nearness.dot(v_cos_gamma_mat.row(i)) * v_gamma_delta_FOV / (0.5 * v_gamma_range_FOV);
        v_b[i-1] = v_nearness.dot(v_sin_gamma_mat.row(i)) * v_gamma_delta_FOV / (0.5 * v_gamma_range_FOV);
      }
    }

    // ///////////////////////////
    // // Print h_cos_gamma_arr //
    // ///////////////////////////
    // std::cout << "h_cos_gamma_arr = [";
    // for (int i = 0; i < h_num_fourier_terms; i++)
    // {
    //   std::cout << "[";
    //   for (int j = 0; j < h_num_points; j++)
    //   {
    //     std::cout << h_cos_gamma_arr[i][j] << ",";
    //   }
    //   std::cout << "]" << std::endl;
    // }
    // std::cout << "]" << std::endl;  

    // ///////////////////////////
    // // Print h_sin_gamma_arr //
    // ///////////////////////////
    // std::cout << "h_sin_gamma_arr = [";
    // for (int i = 0; i < h_num_fourier_terms; i++)
    // {
    //   std::cout << "[";
    //   for (int j = 0; j < h_num_points; j++)
    //   {
    //     std::cout << h_sin_gamma_arr[i][j] << ",";
    //   }
    //   std::cout << "]" << std::endl;
    // }
    // std::cout << "]" << std::endl;  
    
    // ////////////////////////////////////////////
    // // Print Fourier cosine coefficients, h_a //
    // ////////////////////////////////////////////
    // std::cout << "h_a = [";
    // for (int i = 0; i < h_num_fourier_terms + 1; i++)
    // {
    //   std::cout << h_a[i] << ",";  
    // }
    // std::cout << "]" << std::endl;

    // //////////////////////////////////////////
    // // Print Fourier sine coefficients, h_b //
    // //////////////////////////////////////////
    // std::cout << "h_b = [";
    // for (int i = 0; i < h_num_fourier_terms + 1; i++)
    // {
    //   std::cout << h_b[i] << ",";  
    // }
    // std::cout << "]" << std::endl;

    // //////////////////////
    // // Print h_nearness //
    // //////////////////////
    // std::cout << "h_nearness = ";
    // std::cout << h_nearness << std::endl;

    /////////////////////////////////////////////////
    // Publish horizontal WFI Fourier coefficients //
    /////////////////////////////////////////////////

    // Convert array to vector
    std::vector<float> h_a_vector(h_a, h_a + sizeof h_a / sizeof h_a[0]);
    std::vector<float> h_b_vector(h_b, h_b + sizeof h_b / sizeof h_b[0]);

    image_proc::FourierCoefsMsg h_wfi_ctrl_cmd_msg;

    h_wfi_ctrl_cmd_msg.header.stamp = ros::Time::now();
    h_wfi_ctrl_cmd_msg.a_0 = h_a_0;
    h_wfi_ctrl_cmd_msg.a = h_a_vector;
    h_wfi_ctrl_cmd_msg.b = h_b_vector;

    pub_wfi_h_fourier_coefficients_.publish(h_wfi_ctrl_cmd_msg);

    ///////////////////////////////////////////////
    // Publish vertical WFI Fourier coefficients //
    ///////////////////////////////////////////////

    // Convert array to vector
    std::vector<float> v_a_vector(v_a, v_a + sizeof v_a / sizeof v_a[0]);
    std::vector<float> v_b_vector(v_b, v_b + sizeof v_b / sizeof v_b[0]);

    image_proc::FourierCoefsMsg v_wfi_ctrl_cmd_msg;

    v_wfi_ctrl_cmd_msg.header.stamp = ros::Time::now();
    v_wfi_ctrl_cmd_msg.a_0 = v_a_0;
    v_wfi_ctrl_cmd_msg.a = v_a_vector;
    v_wfi_ctrl_cmd_msg.b = v_b_vector;

    pub_wfi_v_fourier_coefficients_.publish(v_wfi_ctrl_cmd_msg);

    ////////////////////////////////////////
    // Calculate forward velocity command //
    ////////////////////////////////////////

    // Forward velocity gain
    float h_K_03 = 0.1;

    // Scaling factor
    float h_N = 10;

    // Reference velocity
    float h_v_0 = 0.5;

    // Fourier coefficient
    float h_a_1 = h_a[0];
    float h_a_2 = h_a[1];

    // Min and max forward velocity limits
    float h_v_min = 0.1;
    float h_v_max = 2.0;

    // Forward velocity control law
    float wfi_forward_velocity_control = 1 - h_v_max * (h_a_0 - h_a_2);

    // Saturate forward velocity command
    if(wfi_forward_velocity_control < h_v_min)
      wfi_forward_velocity_control = h_v_min;
    if(wfi_forward_velocity_control > h_v_max)
      wfi_forward_velocity_control = h_v_min;

    ////////////////////////////////
    // Calculate yaw rate command //
    ////////////////////////////////

    // Lateral position gain
    float h_K_1 = 0; // -0.500;  // K_01 < 0 for stability
    
    // Yaw angle gain
    float h_K_2 =  0.5; // 0.575;

    // Fourier coefficient
    float h_b_1 = h_b[1];
    float h_b_2 = h_b[2];

    float wfi_yaw_rate_control = h_K_1 * h_b_1 + h_K_2 * h_b_2;

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

    pub_wfi_control_commands_.publish(wfi_control_command);

    //////////////////////////////
    // Calculation Junctionness //
    //////////////////////////////

    std_msgs::Float32 wfi_junctionness;

    // ***** THIS IS NOT ROBUST ***
    wfi_junctionness.data = 1.8 * h_a_2 - h_a_0;

    pub_wfi_junctionness_.publish(wfi_junctionness);

  }

  void configCb(Config &config, uint32_t level)
  {
    config_ = config;
  }

  sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  /**
   * Sets the scan time parameter.
   * 
   * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as 
   * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
   * left to the user to set correctly.
   * 
   * @param scan_time The value to use for outgoing sensor_msgs::LaserScan.
   * 
   */
  void set_scan_time(const float scan_time);
  
  /**
   * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
   * 
   * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
   * angular increment.  range_max is used to set the output message.
   * 
   * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
   * @param range_max Maximum range to use points in the output scan.
   * 
   */
  void set_range_limits(const float range_min, const float range_max);
  
  /**
   * Sets the number of image rows to use in the output LaserScan.
   * 
   * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
   * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
   * can be used to vertically compress obstacles into a single LaserScan.
   * 
   * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
   * 
   */
  void set_scan_height(const int scan_height);
  
  /**
   * Sets the frame_id for the output LaserScan.
   * 
   * Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
   * Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
   * 
   * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
   * 
   */
  void set_output_frame(const std::string output_frame_id);

private:
  /**
   * Computes euclidean length of a cv::Point3d (as a ray from origin)
   * 
   * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
   * 
   * @param ray The ray for which the magnitude is desired.
   * @return Returns the magnitude of the ray.
   * 
   */
  double magnitude_of_ray(const cv::Point3d& ray) const;

  /**
   * Computes the angle between two cv::Point3d
   * 
   * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
   * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
   * 
   * @param ray1 The first ray
   * @param ray2 The second ray
   * @return The angle between the two rays (in radians)
   * 
   */
  double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
  
  /**
   * Determines whether or not new_value should replace old_value in the LaserScan.
   * 
   * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
   * new_value is 'more ideal' (currently shorter range) than old_value.
   * 
   * @param new_value The current calculated range.
   * @param old_value The current range in the output LaserScan.
   * @param range_min The minimum acceptable range for the output LaserScan.
   * @param range_max The maximum acceptable range for the output LaserScan.
   * @return If true, insert new_value into the output LaserScan.
   * 
   */
  bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) const;

  /**
  * Converts the depth image to a laserscan using the DepthTraits to assist.
  * 
  * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
  * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
  * a specific angular measurement, then the shortest range is used.
  * 
  * @param depth_msg The UInt16 or Float32 encoded depth message.
  * @param cam_model The image_geometry camera model for this image.
  * @param scan_msg The output LaserScan.
  * @param scan_height The number of vertical pixels to feed into each angular_measurement.
  * 
  */
  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model, 
  const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) const
  {
    // Use correct principal point from calibration
    float center_x = cam_model.cx();
    float center_y = cam_model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = image_proc::DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / cam_model.fx();
    float constant_y = unit_scaling / cam_model.fy();
    
    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);

    int offset = (int)(cam_model.cy()-scan_height/2);
    depth_row += offset*row_step; // Offset to center of image

    for(int v = offset; v < offset+scan_height_; v++, depth_row += row_step){
      for (int u = 0; u < (int)depth_msg->width; u++) // Loop over each pixel in row
      {	
        T depth = depth_row[u];
        
        double r = depth; // Assign to pass through NaNs and Infs
        double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
        int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
        
        if (image_proc::DepthTraits<T>::valid(depth)){ // Not NaN or Inf
          // Calculate in XYZ
          double x = (u - center_x) * depth * constant_x;
          double z = image_proc::DepthTraits<T>::toMeters(depth);
          
          // Calculate actual distance
          r = sqrt(pow(x, 2.0) + pow(z, 2.0));
        }
      
      // Determine if this point should be used.
      // if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
      //   scan_msg->ranges[index] = r;
      scan_msg->ranges[index] = r;
      // }
      }
    }
  }
  
  image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
  
  float scan_time_; ///< Stores the time between scans.
  float range_min_; ///< Stores the current minimum range to use.
  float range_max_; ///< Stores the current maximum range to use.
  int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.
  std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.

};
  
}; // depthimage_to_laserscan
