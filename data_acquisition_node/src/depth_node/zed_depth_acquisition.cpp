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
            options.max_image_value = 20;
            // if (msg->encoding == "32FC1") options.max_image_value = 20;               // Probably 10 meters
            // else if (msg->encoding == "16UC1")  options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
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
    cv::Mat depth;
    imageNormalize(msg, depth);    
    // cv::namedWindow("ROS Frames", CV_WINDOW_NORMAL );
    // cv::imshow("ROS Frames", depth);
    // std::cout << depth.channels() << std::endl;
    // cv::resize(raw_disparity_, raw_disparity_, cv::Size(640,360));
    image_name << "/home/matheus/32bits/depth/depth_" << msg->header.stamp << ".png"; 
    cv::imwrite(image_name.str(), depth);
    // cv::waitKey(1);
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

