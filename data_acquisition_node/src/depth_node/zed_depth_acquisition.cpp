#include "include/zed_depth_acquisition.hpp"

namespace zed
{
ZedDepthAcquisition::ZedDepthAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , depth_sub_(img_transport_.subscribe("/zed/depth/depth_registered", 1,&ZedDepthAcquisition::depthImageCallback, this))
    , raw_disparity_()
    , params_()
    , colormap_(-1)
    , min_depth_value_(0)
    , max_depth_value_(0)
    , server_()
    , files_path_("~/zed_data_acquisiton/")
    , file_manager_()
{
    params_.readFromRosParameterServer(nh_);
    initRosParams();
    server_.reset(new ReconfigureServer(nh_));
    server_->setCallback(boost::bind(&ZedDepthAcquisition::reconfigureCb, this, _1, _2));
}
ZedDepthAcquisition::~ZedDepthAcquisition()
{
}
void ZedDepthAcquisition::reconfigureCb(Config &config, uint32_t level)
{
//   boost::mutex::scoped_lock lock(g_image_mutex);
  colormap_ = config.colormap;
  min_depth_value_ = config.min_image_value;
  max_depth_value_ = config.max_image_value;
}

void ZedDepthAcquisition::imageNormalize(const sensor_msgs::ImageConstPtr& msg, cv::Mat& src)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat dst;
    try 
    {
        cv_bridge::CvtColorForDisplayOptions options;
        options.colormap = colormap_;

        if (min_depth_value_ == max_depth_value_) 
        {
            options.min_image_value = 0;
            if (msg->encoding == "32FC1") options.max_image_value = 20;               // Probably 10 meters
            else if (msg->encoding == "16UC1")  options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
        } 
        else 
        {
            options.min_image_value = min_depth_value_;
            options.max_image_value = max_depth_value_;
        }
        cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
        dst = cv_ptr->image;
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
    if (!dst.empty()) src = dst;
}

void ZedDepthAcquisition::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::ostringstream image_name;
    cv::Mat depth, raw;
    if (msg->encoding == "32FC1") raw = cv_bridge::toCvShare(msg, "32FC1")->image;
    if (msg->encoding == "16UC1") raw = cv_bridge::toCvShare(msg, "16UC1")->image;

    imageNormalize(msg, depth);
    printPixels(msg, depth, raw);

    cv::namedWindow("Depth", CV_WINDOW_NORMAL);
    image_name << "/home/matheus/32bits/depth/depth_" << msg->header.stamp << ".png"; 
    cv::imshow("Depth", depth);
    cv::waitKey(1);

    cv::imwrite(image_name.str(), depth);
}

void ZedDepthAcquisition::printPixels(const sensor_msgs::ImageConstPtr& msg, const cv::Mat& depth, const cv::Mat& raw)
{
    if (msg->encoding == "32FC1")
    {
        std::cout << "First Object = "  << raw.at<float>(347, 733) << std::endl;
        std::cout << "Second Object = " << raw.at<float>(236, 723) << std::endl << std::endl << std::endl;
        
        std::cout << "First Normalized Object = "  << static_cast<int>(depth.at<cv::Vec3b>(347, 733)[0]) 
        + static_cast<int>(depth.at<cv::Vec3b>(347, 733)[1]) 
        + static_cast<int>(depth.at<cv::Vec3b>(347, 733)[2]) << std::endl;

        std::cout << "Second Normalized Object = "  << static_cast<int>(depth.at<cv::Vec3b>(236, 723)[0]) 
        + static_cast<int>(depth.at<cv::Vec3b>(236, 723)[1]) 
        + static_cast<int>(depth.at<cv::Vec3b>(236, 723)[2])  << std::endl;
    }

    if (msg->encoding == "16UC1")
    {
        int first = static_cast<int>(raw.at<int>(350, 748));
        int second = static_cast<int>(raw.at<int>(258, 721));

        std::cout << "First Object = "  << first/(int)1000 << std::endl;
        std::cout << "Second Object = " << second/(int)1000 << std::endl << std::endl << std::endl;
        
        std::cout << "First Normalized Object = "  << static_cast<int>(depth.at<cv::Vec3b>(350, 748)[0]) 
        + static_cast<int>(depth.at<cv::Vec3b>(350, 748)[1]) 
        + static_cast<int>(depth.at<cv::Vec3b>(350, 748)[2]) << std::endl;

        std::cout << "Second Normalized Object = "  << static_cast<int>(depth.at<cv::Vec3b>(258, 721)[0]) 
        + static_cast<int>(depth.at<cv::Vec3b>(258, 721)[1]) 
        + static_cast<int>(depth.at<cv::Vec3b>(258, 721)[2])  << std::endl;
    }
} 

void ZedDepthAcquisition::initRosParams() 
{
    colormap_         = params_.colormap();
    min_depth_value_  = params_.minDistance();
    max_depth_value_  = params_.maxDistance();
    disturbance_      = params_.disturbance();
    camera_position_  = params_.cameraPosition();
    object_position_  = params_.objectPosition();
    disturbance_type_ = params_.disturbanceType();
    luminance_        = params_.iluminance();
    files_path_ = params_.filesDirectory(); 
}

}

