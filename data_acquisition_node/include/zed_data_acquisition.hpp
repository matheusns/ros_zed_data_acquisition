#ifndef ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP
#define ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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
    void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg);

  protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport img_transport_;       ///< ROS image transport object
    image_transport::Subscriber depth_sub_;               ///< ROS depth image subscriber
    cv::VideoWriter *output_video_;
    bool is_saving_; 
    int frame_height_;
    int frame_width_;
};

} // zed namespace
#endif //ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP 