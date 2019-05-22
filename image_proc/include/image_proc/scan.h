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

#ifndef SCAN_H
#define SCAN_H

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
  image_transport::CameraSubscriber sub_depth_image_;
  ros::Subscriber sub_laserscan_;
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
  ros::Publisher pub_h_laserscan_;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  
  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef image_proc::ScanConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void onInit();

  // Handles (un)subscribing when clients (un)subscribe;
  void connectCb();

  void configCb(Config &config, uint32_t level);

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void laserscanCb(const sensor_msgs::LaserScan laserscan_msg);

  void get_h_strip(const sensor_msgs::ImageConstPtr& image_msg,
                   const sensor_msgs::CameraInfoConstPtr& info_msg);

  void get_v_strip(const sensor_msgs::ImageConstPtr& image_msg,
                   const sensor_msgs::CameraInfoConstPtr& info_msg);

  void calc_h_wfi_fourier_coefficients(int h_width_cropped, float h_gamma_start_FOV, float h_gamma_end_FOV);

  void calc_v_wfi_fourier_coefficients();

  void calc_forward_velocity_command();

  void calc_yaw_rate_command();

  sensor_msgs::LaserScanPtr depthimage_to_horiz_laserscan(const sensor_msgs::ImageConstPtr& depth_msg,
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
  
  // Declare parameters

  int h_x_offset_;
  int h_y_offset_;
  int h_width_;
  int h_height_;

  int v_x_offset_;
  int v_y_offset_;
  int v_width_;
  int v_height_;
  
  float scan_time_; ///< Stores the time between scans.
  float range_min_; ///< Stores the current minimum range to use.
  float range_max_; ///< Stores the current maximum range to use.
  int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.
  std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.

  CvImageConstPtr source;
  int v_width_cropped; 
  int v_height_cropped;
  int h_width_cropped; 
  int h_height_cropped;
  cv::Mat h_depth_sat;
  cv::Mat v_depth_sat;
  cv::Mat h_depth_raw;
  float h_a_0, h_a_1, h_a[4], h_b[4]; // *** TO DO: Remove hard-coded length ***
  float h_a_2;
  float wfi_forward_velocity_control;
  float wfi_yaw_rate_control;
  float h_gamma_start_FOV;
  float h_gamma_end_FOV;
  


};
  
}; // depthimage_to_laserscan

#endif