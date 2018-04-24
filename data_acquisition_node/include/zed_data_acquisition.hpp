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
    bool readPrf(int &cols, int &rows, char *&data, const char *path);
    bool readPrf(cv::Mat &frame, const char *path);
    bool writePrf(int cols, int rows, unsigned char *&data, const char *path);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void leftRawImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg);

  protected:

    ros::NodeHandle nh_;                                  ///< ROS image transport object
    image_transport::ImageTransport img_transport_;       ///< ROS image transport object
    image_transport::Subscriber depth_sub_;               ///< ROS depth image subscriber
    image_transport::Subscriber left_sub_;                ///< ROS left raw image subscriber
    image_transport::Subscriber right_sub_;               ///< ROS right raw image subscriber

    cv::VideoWriter *disparity_output_video_;                       ///< ROS 
    cv::VideoWriter *right_output_video_;                       ///< ROS 
    cv::VideoWriter *left_output_video_;                       ///< ROS 
    cv::Mat right_image_;                                 ///< cv 
    cv::Mat left_image_;                                  ///< cv 
    cv::Mat raw_disparity_;                               ///< cv 

    bool is_saving_;                                      ///< ROS 
    int frame_height_;                                    ///< ROS 
    int frame_width_;                                     ///< ROS 
    int cont_;
};

} // zed namespace
#endif //ZED_DATA_ACQUISITION_INCLUDE_ZED_DATA_ACQUISITION_HPP 