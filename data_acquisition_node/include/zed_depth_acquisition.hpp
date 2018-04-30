#ifndef ZED_DATA_ACQUISITION_INCLUDE_ZED_DEPTH_ACQUISITION_HPP
#define ZED_DATA_ACQUISITION_INCLUDE_ZED_DEPTH_ACQUISITION_HPP

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <zed_data_acquisition/ZedDepthViewConfig.h>
#include <sensor_msgs/image_encodings.h>

#include "include/zed_acquisition_param.hpp"
#include <include/zed_file_manager.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/core/version.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

namespace enc = sensor_msgs::image_encodings;

namespace zed
{
class ZedDepthAcquisition 
{
  public:    
    
    explicit ZedDepthAcquisition();
    virtual ~ZedDepthAcquisition();

    typedef zed_data_acquisition::ZedDepthViewConfig Config;

    void imageNormalize(const sensor_msgs::ImageConstPtr& msg, cv::Mat& src);
    cv_bridge::CvImageConstPtr colorMap(const cv_bridge::CvImageConstPtr& source, const int color_map);
    void distanceThreshold(const sensor_msgs::ImageConstPtr& msg, cv::Mat& dst);
    // void distanceThreshold(cv::Mat& src, cv::Mat& dst);
    void reconfigureCb(Config &config, uint32_t level);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void initRosParams();
    void printPixels(const sensor_msgs::ImageConstPtr& msg, const cv::Mat& depth, const cv::Mat& raw); 

  protected:
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer; 

    ros::NodeHandle nh_;                                  ///< ROS image transport object
    image_transport::ImageTransport img_transport_;       ///< ROS image transport object
    image_transport::Subscriber depth_sub_;               ///< ROS depth image subscriber

    boost::shared_ptr<ReconfigureServer> server_; 

    bool   disturbance_;
    int    colormap_;
    int    camera_position_;
    int    object_position_;
    int    disturbance_type_;
    float  min_depth_value_;
    float  max_depth_value_;
    float  luminance_;

    ZedFileManager file_manager_;

    std::string files_path_;

    cv::Mat raw_disparity_;                               ///< cv 

    zed::ZedAcquisitionParam params_;
};

} // zed namespace
#endif //ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP 