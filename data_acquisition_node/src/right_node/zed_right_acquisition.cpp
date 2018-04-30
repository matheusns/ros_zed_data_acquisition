#include "include/zed_right_acquisition.hpp"

namespace zed
{
ZedRightAcquisition::ZedRightAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , right_sub_(img_transport_.subscribe("/zed/right/image_raw_color", 1,&ZedRightAcquisition::rightRawImageCallback, this))
    , right_img_()
    , params_()
    , files_path_("~/zed_data_acquisiton")
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
    image_name << "right_" << msg->header.stamp;
    saveImage(right_img_, image_name.str());
}

bool ZedRightAcquisition::saveImage(cv::Mat& src, const std::string file_name)
{
    std::ostringstream path;

    if (files_path_[0] == '~')
    {
        files_path_ = file_manager_.normalizeUserPath(files_path_);
    }

    path << files_path_ << "/right/";
    
    if ( !file_manager_.existsDir(path.str()) )
    {
        file_manager_.createDir(path.str());
    }
    path << file_name << ".png";
    cv::imwrite(path.str(), src);
}
void ZedRightAcquisition::initRosParams() 
{
    files_path_ = params_.filesDirectory(); 
}

}

