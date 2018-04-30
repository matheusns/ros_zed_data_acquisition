#ifndef ZED_DATA_ACQUISITION_INCLUDE_ZED_RIGHT_ACQUISITION_HPP
#define ZED_DATA_ACQUISITION_INCLUDE_ZED_RIGHT_ACQUISITION_HPP

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "include/zed_acquisition_param.hpp"
#include <include/zed_file_manager.hpp>


#include "opencv2/core/core.hpp"
#include "opencv2/core/version.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

namespace zed
{
class ZedRightAcquisition 
{
  public:    
    
    explicit ZedRightAcquisition();
    virtual ~ZedRightAcquisition();

    void rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool saveImage(cv::Mat& src, const std::string file_name);
    void initRosParams();

  protected:
    ros::NodeHandle nh_;                                  ///< ROS image transport object
    image_transport::ImageTransport img_transport_;       ///< ROS image transport object
    image_transport::Subscriber depth_sub_;               ///< ROS depth image subscriber
    image_transport::Subscriber left_sub_;                ///< ROS left raw image subscriber
    image_transport::Subscriber right_sub_;               ///< ROS right raw image subscriber

    std::string files_path_;

    cv::Mat right_img_;                                 ///< cv 
    zed::ZedFileManager file_manager_;
    zed::ZedAcquisitionParam params_;
};

} // zed namespace
#endif //ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP 