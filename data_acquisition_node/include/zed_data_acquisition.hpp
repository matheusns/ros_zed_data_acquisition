#ifndef ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP
#define ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <zed_data_acquisition/ZedDepthViewConfig.h>

#include "include/zed_acquisition_param.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/core/version.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

namespace zed
{
class ZedAcquisition 
{
  public:    
    
    explicit ZedAcquisition();
    virtual ~ZedAcquisition();

    typedef zed_data_acquisition::ZedDepthViewConfig Config;

    void imageNormalize(const sensor_msgs::ImageConstPtr& msg, cv::Mat& src);
    void reconfigureCb(Config &config, uint32_t level);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void leftRawImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void initRosParams();

  protected:
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer; 

    ros::NodeHandle nh_;                                  ///< ROS image transport object
    image_transport::ImageTransport img_transport_;       ///< ROS image transport object
    image_transport::Subscriber depth_sub_;               ///< ROS depth image subscriber
    image_transport::Subscriber left_sub_;                ///< ROS left raw image subscriber
    image_transport::Subscriber right_sub_;               ///< ROS right raw image subscriber

    boost::shared_ptr<ReconfigureServer> server_; 

    bool   disturbance_;
    int    colormap_;
    int    camera_position_;
    int    object_position_;
    int    disturbance_type_;
    float  min_depth_value_;
    float  max_depth_value_;
    float  luminance_;

    std::string files_path_;

    cv::Mat right_image_;                                 ///< cv 
    cv::Mat left_image_;                                  ///< cv 
    cv::Mat raw_disparity_;                               ///< cv 

    zed::ZedAcquisitionParam params_;

    bool is_saving_;
    int cont_;
};

} // zed namespace
#endif //ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP 