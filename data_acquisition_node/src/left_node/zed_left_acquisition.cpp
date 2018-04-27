#include "include/zed_left_acquisition.hpp"

namespace zed
{
ZedLeftAcquisition::ZedLeftAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , left_sub_(img_transport_.subscribe("/zed/left/image_raw_color", 1,&ZedLeftAcquisition::leftRawImageCallback, this))
    , left_img_()
    , params_()
    , files_path_("~/zed_data_acquisiton/")
{
    params_.readFromRosParameterServer(nh_);
    initRosParams();
}
ZedLeftAcquisition::~ZedLeftAcquisition()
{
}

void ZedLeftAcquisition::leftRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::ostringstream image_name;
    left_img_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    image_name << "/home/matheus/32bits/left/left_" << msg->header.stamp << ".png"; 
    cv::imwrite(image_name.str(), left_img_);
    
}

void ZedLeftAcquisition::initRosParams() 
{
    files_path_ = params_.filesDirectory(); 
}

}

