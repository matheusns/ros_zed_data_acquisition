#include "include/zed_right_acquisition.hpp"

namespace zed
{
ZedRightAcquisition::ZedRightAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , right_sub_(img_transport_.subscribe("/zed/right/image_raw_color", 1,&ZedRightAcquisition::rightRawImageCallback, this))
    , right_img_()
    , params_()
    , files_path_("~/zed_data_acquisiton/")
{
    params_.readFromRosParameterServer(nh_);
    initRosParams();
}
ZedRightAcquisition::~ZedRightAcquisition()
{
}
void ZedRightAcquisition::rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::ostringstream image_name;
    right_img_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    image_name << "/home/matheus/32bits/right/right_" << msg->header.stamp << ".png"; 
    cv::imwrite(image_name.str(), right_img_);
}

void ZedRightAcquisition::initRosParams() 
{
    files_path_ = params_.filesDirectory(); 
}

}

