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
    image_name << "left_" << msg->header.stamp;
    saveImage(left_img_, image_name.str());
    
}

bool ZedLeftAcquisition::saveImage(cv::Mat& src, const std::string file_name)
{
    std::ostringstream path;

    if (files_path_[0] == '~')
    {
        files_path_ = file_manager_.normalizeUserPath(files_path_);
    }

    path << files_path_ << "/left/";
    
    if ( !file_manager_.existsDir(path.str()) )
    {
        file_manager_.createDir(path.str());
    }
    path << file_name << ".png";
    cv::imwrite(path.str(), src);
}


void ZedLeftAcquisition::initRosParams() 
{
    files_path_ = params_.filesDirectory(); 
}

}

