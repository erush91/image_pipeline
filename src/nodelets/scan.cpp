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
#include <wfi_from_depth_sensor/ScanConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <wfi_from_depth_sensor/FourierCoefsMsg.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <wfi_from_depth_sensor/scan.h>

namespace wfi_from_depth_sensor
{
    void ScanNodelet::onInit()
    {
        nh         = getNodeHandle();
        private_nh = getPrivateNodeHandle();
        ros::NodeHandle nh_wfi (nh, "wfi");
        ros::NodeHandle nh_pcl2laser (nh, "pcl2laser");
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

        // Subscriber RealSense depth image
        sub_depth_image_ = it_in_->subscribeCamera("image_raw", queue_size_, &ScanNodelet::imageCb, this, hints);

        // Subscriber Laserscan
        sub_laserscan_ = nh_pcl2laser.subscribe("horiz/laser_scan", 1, &ScanNodelet::laserscanCb, this);

        // Publisher rows of depth image used in horizontal line scan
        pub_h_scans_ = it_out_->advertiseCamera("h_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
        
        // Publisher rows of depth image used in vertical line scan
        pub_v_scans_ = it_out_->advertiseCamera("v_scans",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
        
        // Publisher horizontal line scan
        pub_wfi_h_scan_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("horiz/image_scan", 10);

        // Publisher vertical line scan
        pub_wfi_v_scan_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("vert/imagescan", 10);

        // Publisher horizontal nearness
        pub_wfi_h_nearness_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("horiz/nearness", 10);

        // Publisher vertical nearness
        pub_wfi_v_nearness_ = nh_wfi.advertise<std_msgs::Float32MultiArray>("vert/nearness", 10);

        // Publisher horizontaol Fourier coefficients
        pub_wfi_h_fourier_coefficients_ = nh_wfi.advertise<wfi_from_depth_sensor::FourierCoefsMsg>("horiz/fourier_coefficients", 10);

        // Publisher vertical Fourier coefficients
        pub_wfi_v_fourier_coefficients_ = nh_wfi.advertise<wfi_from_depth_sensor::FourierCoefsMsg>("vert/fourier_coefficients", 10);

        // Publisher WFI control command
        pub_wfi_control_commands_ = nh_wfi.advertise<geometry_msgs::Twist>("control_commands", 10);

        // Publisher WFI junctionness
        pub_wfi_junctionness_ = nh_wfi.advertise<std_msgs::Float32>("junctionness", 10);

        // Publisher horizontal laserscan
        pub_h_laserscan_ = nh_wfi.advertise<sensor_msgs::LaserScan>("horiz/laser_scan", 10);

    };

    void ScanNodelet::connectCb()
    {
        
    };

    void ScanNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;
    };
    
    double ScanNodelet::magnitude_of_ray(const cv::Point3d& ray) const
    {
        return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
    }

    double ScanNodelet::angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const
    {
        double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
        double magnitude1 = magnitude_of_ray(ray1);
        double magnitude2 = magnitude_of_ray(ray2);;
        return acos(dot_product / (magnitude1 * magnitude2));
    }

    bool ScanNodelet::use_point(const float new_value, const float old_value, const float range_min, const float range_max) const
    {  
        // Check for NaNs and Infs, a real number within our limits is more desirable than these.
        bool new_finite = std::isfinite(new_value);
        bool old_finite = std::isfinite(old_value);
        
        // Infs are preferable over NaNs (more information)
        if(!new_finite && !old_finite)
        { // Both are not NaN or Inf.
            if(!isnan(new_value))
            { // new is not NaN, so use it's +-Inf value.
                return true;
            }
            return false; // Do not replace old_value
        }
    
        // If not in range, don't bother
        bool range_check = range_min <= new_value && new_value <= range_max;
        if(!range_check)
        {
            return false;
        }
        
        if(!old_finite)
        { // New value is in range and finite, use it.
            return true;
        }
        
        // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
        bool shorter_check = new_value < old_value;
        return shorter_check;
    }

    ///////////////////////////////////////////////////////////////////////////
    // PUBLISH HORIZONTAL LASERSCAN (direct from depth image, distorted) //////
    ///////////////////////////////////////////////////////////////////////////
    
    void ScanNodelet::get_h_strip(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        
        ///////////////////////////////
        // Get Horizontal Laser Scan //
        ///////////////////////////////

        int max_h_width = image_msg->width - h_x_offset_;
        int max_h_height = image_msg->height - h_y_offset_;
        h_width_cropped = h_width_;
        h_height_cropped = h_height_;

        if (h_width_cropped == 0 || h_width_cropped > max_h_width)
        h_width_cropped = max_h_width;
        if (h_height_cropped == 0 || h_height_cropped > max_h_height)
        h_height_cropped = max_h_height;

        int max_v_width = image_msg->width - v_x_offset_;
        int max_v_height = image_msg->height - v_y_offset_;
        v_width_cropped = v_width_;
        v_height_cropped = v_height_;

        if (v_width_ == 0 || v_width_ > max_v_width)
        v_width_cropped = max_v_width;
        if (v_height_ == 0 || v_height_ > max_v_height)
        v_height_cropped = max_v_height;

        // Get h_a cv::Mat view of the source data
        source = toCvShare(image_msg);

        //////////////////////////////////
        // Get Selected Horizontal Rows //
        //////////////////////////////////

        CvImage h_scans(source->header, source->encoding);
        CvImage h_scans_float(source->header, source->encoding);
        CvImage h_scan(source->header, "32FC1");

        h_scans.image = source->image(cv::Rect(h_x_offset_, h_y_offset_, h_width_cropped, h_height_cropped));
        h_scans_float.image = source->image(cv::Rect(h_x_offset_, h_y_offset_, h_width_cropped, h_height_cropped));
        h_scan.image = source->image(cv::Rect(h_x_offset_, h_y_offset_, h_width_cropped, 1));

        ////////////////////////////////////
        // Calculate Horizontal Line Scan //
        ////////////////////////////////////

        // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
        h_scans.image.convertTo(h_scans_float.image, CV_32FC1,0.001); // [mm] --> [m]
        
        reduce(h_scans_float.image, h_scan.image, 0, CV_REDUCE_AVG, CV_32FC1); // 0 = AVERAGE COLUMNS

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
        out_h_info->roi.x_offset += h_x_offset_ * h_binning_x;
        out_h_info->roi.y_offset += h_y_offset_ * h_binning_y;
        out_h_info->roi.width = h_width_cropped * h_binning_x;
        out_h_info->roi.height = h_height_cropped * h_binning_y;
        
        // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
        if (h_width_cropped != (int)image_msg->width || h_height_cropped != (int)image_msg->height)
        {
        out_h_info->roi.do_rectify = true;
        }

        if (!target_frame_id_.empty())
        {
        out_h_scans->header.frame_id = target_frame_id_;
        out_h_info->header.frame_id = target_frame_id_;
        }

        pub_h_scans_.publish(out_h_scans, out_h_info);

        ////////////////////////////////////
        // Saturate horizontal depth scan //
        ////////////////////////////////////

        h_depth_sat = h_scan.image;

        h_depth_sat.setTo(0.5, h_depth_sat < 0.5);
        h_depth_sat.setTo(10, h_depth_sat > 10);

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
    }

    ///////////////////////////////////////////////////////////////////////////
    // PUBLISH VERTICAL LASERSCAN (direct from depth image, distorted) ////////
    ///////////////////////////////////////////////////////////////////////////

    void ScanNodelet::get_v_strip(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        
        ///////////////////////////////
        // Get Seleted Vertical Rows //
        ///////////////////////////////

        CvImage v_scans(source->header, source->encoding);
        CvImage v_scans_float(source->header, source->encoding);
        CvImage v_scan(source->header, "32FC1");

        v_scans.image = source->image(cv::Rect(v_x_offset_, v_y_offset_, v_width_cropped, v_height_cropped));
        v_scans_float.image = source->image(cv::Rect(v_x_offset_, v_y_offset_, v_width_cropped, v_height_cropped));
        v_scan.image = source->image(cv::Rect(v_x_offset_, v_y_offset_, 1, v_height_cropped));

        //////////////////////////////////
        // Calculate Vertical Line Scan //
        //////////////////////////////////

        // Convert to from 2 bytes (16 bit) to float, and scale from mm to m
        v_scans.image.convertTo(v_scans_float.image, CV_32FC1,0.001); // [mm] --> [m]
        
        reduce(v_scans_float.image, v_scan.image, 1, CV_REDUCE_AVG, CV_32FC1); // 1 = AVERAGE ROWS

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
        out_v_info->roi.x_offset += v_x_offset_ * v_binning_x;
        out_v_info->roi.y_offset += v_y_offset_ * v_binning_y;
        out_v_info->roi.width = v_width_cropped * v_binning_x;
        out_v_info->roi.height = v_height_cropped * v_binning_y;
        
        // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
        if (v_width_cropped != (int)image_msg->width || v_height_cropped != (int)image_msg->height)
        {
        out_v_info->roi.do_rectify = true;
        }

        if (!target_frame_id_.empty())
        {
        out_v_scans->header.frame_id = target_frame_id_;
        out_v_info->header.frame_id = target_frame_id_;
        }

        pub_v_scans_.publish(out_v_scans, out_v_info);

        //////////////////////////////////
        // Saturate vertical depth scan //
        //////////////////////////////////

        v_depth_sat = cv::Mat::zeros(cv::Size(1, v_height_cropped), CV_32FC1);
        cv::transpose(v_scan.image, v_depth_sat);
        
        v_depth_sat.setTo(0.5, v_depth_sat < 0.5);
        v_depth_sat.setTo(100, v_depth_sat > 100);

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
    }

    ///////////////////////////////////////////////////////////////////////////
    // PUBLISH HORIZONTAL LASERSCAN (from depthimage_to_laserscan, accurate) //
    ///////////////////////////////////////////////////////////////////////////

    sensor_msgs::LaserScanPtr ScanNodelet::depthimage_to_horiz_laserscan(const sensor_msgs::ImageConstPtr& depth_msg,
                                                                         const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Set camera model
        cam_model_.fromCameraInfo(info_msg);
        
        // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
        cv::Point2d raw_pixel_left(0, cam_model_.cy());
        cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
        cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);
        
        cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_.cy());
        cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
        cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);
        
        cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
        cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
        cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);
        
        double angle_max = angle_between_rays(left_ray, center_ray);
        double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image
        
        // Fill in laserscan message
        sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
        scan_msg->header = depth_msg->header;
        if(output_frame_id_.length() > 0){
            scan_msg->header.frame_id = output_frame_id_;
        }
        scan_msg->angle_min = angle_min;
        scan_msg->angle_max = angle_max;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1) * 1.28;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;
        
        // Check scan_height vs image_height
        if(scan_height_/2 > cam_model_.cy() || scan_height_/2 > depth_msg->height - cam_model_.cy()){
            std::stringstream ss;
            ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
            throw std::runtime_error(ss.str());
        }

        // Calculate and fill the ranges
        uint32_t ranges_size = 500;//depth_msg->width;
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
        
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            convert<uint16_t>(depth_msg, cam_model_, scan_msg, scan_height_);
        }
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        {
            convert<float>(depth_msg, cam_model_, scan_msg, scan_height_);
        }
        else
        {
            std::stringstream ss;
            ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
            throw std::runtime_error(ss.str());
        }
        
        return scan_msg;
    }

    void ScanNodelet::calc_h_wfi_fourier_coefficients(int h_num_points, float h_gamma_start_FOV, float h_gamma_end_FOV)
    {

        //////////////////////////////////
        // Declare horizontal variables //
        //////////////////////////////////

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
        // float h_a_0, h_a[h_num_fourier_terms], h_b[h_num_fourier_terms];

   
        ////////////////////////////////////
        // Intialize horizontal variables //
        ////////////////////////////////////

        float h_gamma_range_FOV = h_gamma_end_FOV - h_gamma_start_FOV;
        float h_gamma_delta_FOV = h_gamma_range_FOV / h_num_points;

        ///////////////////////////////////
        // Calculate horizontal nearness //
        ///////////////////////////////////
        
        cv::Mat h_nearness = cv::Mat::zeros(cv::Size(1, h_num_points), CV_32FC1);

        h_nearness = 1.0 / h_depth_sat;

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

        /////////////////////////////////////////////////
        // Publish horizontal WFI Fourier coefficients //
        /////////////////////////////////////////////////

        // Convert array to vector
        std::vector<float> h_a_vector(h_a, h_a + sizeof h_a / sizeof h_a[0]);
        std::vector<float> h_b_vector(h_b, h_b + sizeof h_b / sizeof h_b[0]);

        wfi_from_depth_sensor::FourierCoefsMsg h_wfi_ctrl_cmd_msg;

        h_wfi_ctrl_cmd_msg.header.stamp = ros::Time::now();
        h_wfi_ctrl_cmd_msg.a_0 = h_a_0;
        h_wfi_ctrl_cmd_msg.a = h_a_vector;
        h_wfi_ctrl_cmd_msg.b = h_b_vector;

        pub_wfi_h_fourier_coefficients_.publish(h_wfi_ctrl_cmd_msg);
    }
    
    void ScanNodelet::calc_v_wfi_fourier_coefficients()
    {
        
        ////////////////////////////////
        // Declare vertical variables //
        ////////////////////////////////

        int v_num_points;
        v_num_points = v_height_cropped;
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
    
        //////////////////////////////////
        // Intialize vertical variables //
        //////////////////////////////////

        v_gamma_start_FOV = -0.25 * M_PI;
        v_gamma_end_FOV = 0.25 * M_PI;
        v_gamma_range_FOV = v_gamma_end_FOV - v_gamma_start_FOV;
        v_gamma_delta_FOV = v_gamma_range_FOV / v_num_points;

        /////////////////////////////////
        // Calculate vertical nearness //
        /////////////////////////////////
        
        cv::Mat v_nearness = cv::Mat::zeros(cv::Size(v_num_points,1), CV_32FC1);

        v_nearness = 1.0 / v_depth_sat;

        ///////////////////////////////
        // Publish vertical nearness //
        ///////////////////////////////

        std::vector<float> v_nearness_array(v_nearness.begin<float>(), v_nearness.end<float>());
        
        std_msgs::Float32MultiArray v_nearness_msg;
        v_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        v_nearness_msg. layout.dim[0].size = v_nearness_array.size();
        v_nearness_msg.data.clear();
        v_nearness_msg.data.insert(v_nearness_msg.data.end(), v_nearness_array.begin(), v_nearness_array.end());

        pub_wfi_v_nearness_.publish(v_nearness_msg);

        cv::Mat v_cos_gamma_mat(v_num_fourier_terms + 1, v_num_points, CV_32FC1, v_cos_gamma_arr);
        cv::Mat v_sin_gamma_mat(v_num_fourier_terms + 1, v_num_points, CV_32FC1, v_sin_gamma_arr);

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

        // Convert array to vector
        std::vector<float> v_a_vector(v_a, v_a + sizeof v_a / sizeof v_a[0]);
        std::vector<float> v_b_vector(v_b, v_b + sizeof v_b / sizeof v_b[0]);

        wfi_from_depth_sensor::FourierCoefsMsg v_wfi_ctrl_cmd_msg;

        v_wfi_ctrl_cmd_msg.header.stamp = ros::Time::now();
        v_wfi_ctrl_cmd_msg.a_0 = v_a_0;
        v_wfi_ctrl_cmd_msg.a = v_a_vector;
        v_wfi_ctrl_cmd_msg.b = v_b_vector;

        pub_wfi_v_fourier_coefficients_.publish(v_wfi_ctrl_cmd_msg);
    }

    ////////////////////////////////////////
    // Calculate forward velocity command //
    ////////////////////////////////////////

    void ScanNodelet::calc_forward_velocity_command()
    {

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
        wfi_forward_velocity_control = 1 - h_v_max * (h_a_0 - h_a_2);

        // Saturate forward velocity command
        if(wfi_forward_velocity_control < h_v_min)
        wfi_forward_velocity_control = h_v_min;
        if(wfi_forward_velocity_control > h_v_max)
        wfi_forward_velocity_control = h_v_min;
    }

    ////////////////////////////////
    // Calculate yaw rate command //
    ////////////////////////////////

    void ScanNodelet::calc_yaw_rate_command()
    {

        // Lateral position gain
        float h_K_1 = 0; // -0.500;  // K_01 < 0 for stability
        
        // Yaw angle gain
        float h_K_2 =  0.5; // 0.575;

        // Fourier coefficient
        float h_b_1 = h_b[0];
        float h_b_2 = h_b[1];

        wfi_yaw_rate_control = h_K_1 * h_b_1 + h_K_2 * h_b_2;

        // Min and max yaw rate limits
        float yaw_rate_min = -2.0;
        float yaw_rate_max =  2.0;

        if(wfi_yaw_rate_control < yaw_rate_min)
        wfi_yaw_rate_control = yaw_rate_min;
        if(wfi_yaw_rate_control > yaw_rate_max)
        wfi_yaw_rate_control = yaw_rate_max;
    };

    void ScanNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg)
    {

        // Get parameters
        Config config;
        {
            boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
            config = config_;
        }

        h_x_offset_ = config.h_x_offset;
        h_y_offset_ = config.h_y_offset;
        h_width_ = config.h_width;
        h_height_ = config.h_height;

        v_x_offset_ = config.v_x_offset;
        v_y_offset_ = config.v_y_offset;
        v_width_ = config.v_width;
        v_height_ = config.v_height;

        scan_time_ = config.scan_time;
        range_min_ = config.range_min;
        range_max_ = config.range_max;
        scan_height_ = config.scan_height;
        output_frame_id_ = config.output_frame_id;

        // Publish horizontal laserscan (from depthimage_to_laserscan, accurate)
        sensor_msgs::LaserScanPtr h_laserscan_msg = depthimage_to_horiz_laserscan(image_msg, info_msg);
        pub_h_laserscan_.publish(h_laserscan_msg);

        // Publish horizontal laserscan (direct from depth image, distorted)
        get_h_strip(image_msg, info_msg);
        
        // Publish vertical laserscan (direct from depth image, distorted)
        get_v_strip(image_msg, info_msg);

        h_gamma_start_FOV = -0.25 * M_PI;
        h_gamma_end_FOV = 0.25 * M_PI;

        std::vector<float> h_depth_vector = h_laserscan_msg->ranges;
        int h_scan_length = h_depth_vector.size();

        h_depth_sat = cv::Mat(1,h_scan_length,CV_32FC1);
        std::memcpy(h_depth_sat.data,h_depth_vector.data(),h_depth_vector.size()*sizeof(float));

        // Calculate horizontal WFI Fourier coefficients
        calc_h_wfi_fourier_coefficients(h_scan_length, h_gamma_start_FOV, h_gamma_end_FOV);

        // Calculate vertical WFI Fourier coefficients
        calc_v_wfi_fourier_coefficients();

        // Calculate forward velocity command
        calc_forward_velocity_command();

        // Calculate yaw rate command
        calc_yaw_rate_command();
        
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

        float i0 = 2/(h_a_0 * M_PI);
        
        const float i0_0 = 22; // 2.4 // Nominal value for a0 when in a tunnel
        const float g_i0 = 1.0/2.5; // 1/0.8;

        const float a1_0 = 0.0404; // 0.06;
        const float g_a1 = 1/0.015; // 1/0.04;

        float h0 = g_i0 * (i0 - i0_0); // Opening Indicator
        float h1 = g_a1 * (h_a_1 - a1_0); // Dead End Indicator

        // ***** THIS IS NOT ROBUST (FALSE POSITIVES/NEGATIVES) ***
        wfi_junctionness.data = h0 - h1; // If(J > 0.5)-->Opening // If(J < -0.5)-->Dead End 

        pub_wfi_junctionness_.publish(wfi_junctionness);

    };

    void ScanNodelet::laserscanCb(const sensor_msgs::LaserScan h_laserscan_msg)
    {

        // Get parameters
        Config config;
        {
            boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
            config = config_;
        }

        std::vector<float> h_depth_vector = h_laserscan_msg.ranges;
        int h_scan_length = h_depth_vector.size();

        h_depth_sat = cv::Mat(1,h_scan_length,CV_32FC1);;
        std::memcpy(h_depth_sat.data,h_depth_vector.data(),h_depth_vector.size()*sizeof(float));
        
        ////////////////////////////////////
        // Saturate horizontal depth scan //
        ////////////////////////////////////

        h_depth_sat.setTo(0.5, h_depth_sat < 0.5);
        h_depth_sat.setTo(100, h_depth_sat > 100);

        h_gamma_start_FOV = -M_PI;
        h_gamma_end_FOV = M_PI;

        // Calculate horizontal WFI Fourier coefficients
        calc_h_wfi_fourier_coefficients(h_scan_length, h_gamma_start_FOV, h_gamma_end_FOV);

        // Calculate vertical WFI Fourier coefficients
        calc_v_wfi_fourier_coefficients();

        // Calculate forward velocity command
        calc_forward_velocity_command();

        // Calculate yaw rate command
        calc_yaw_rate_command();
        
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

        float i0 = 2/(h_a_0 * M_PI);
        
        const float i0_0 = 22; // 2.4 // Nominal value for a0 when in a tunnel
        const float g_i0 = 1.0/2.5; // 1/0.8;

        const float a1_0 = 0.0404; // 0.06;
        const float g_a1 = 1/0.015; // 1/0.04;

        float h0 = g_i0 * (i0 - i0_0); // Opening Indicator
        float h1 = g_a1 * (h_a_1 - a1_0); // Dead End Indicator

        // ***** THIS IS NOT ROBUST (FALSE POSITIVES/NEGATIVES) ***
        wfi_junctionness.data = h0 - h1; // If(J > 0.5)-->Opening // If(J < -0.5)-->Dead End 

        pub_wfi_junctionness_.publish(wfi_junctionness);

    };

}
  
// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( wfi_from_depth_sensor::ScanNodelet, nodelet::Nodelet)

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

///////////////////////////////////////////////
// Publish vertical WFI Fourier coefficients //
///////////////////////////////////////////////
