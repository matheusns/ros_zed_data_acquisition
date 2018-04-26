#include "include/zed_data_acquisition.hpp"

namespace zed
{
ZedAcquisition::ZedAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , depth_sub_(img_transport_.subscribe("/zed/depth/depth_registered", 1,&ZedAcquisition::depthImageCallback, this))
    , left_sub_(img_transport_.subscribe("/zed/left/image_raw_color", 1,&ZedAcquisition::leftRawImageCallback, this))
    , right_sub_(img_transport_.subscribe("/zed/right/image_raw_color", 1,&ZedAcquisition::rightRawImageCallback, this))
    , right_image_()
    , left_image_()
    , raw_disparity_()
    , frame_height_(0)
    , frame_width_(0)
    , is_saving_(false)
    , cont_(0)
    , colormap_(-1)
    , min_depth_value_(0)
    , max_depth_value_(0)
    , server_()

{
    server_.reset(new ReconfigureServer(nh_));
    server_->setCallback(boost::bind(&ZedAcquisition::reconfigureCb, this, _1, _2));
}
ZedAcquisition::~ZedAcquisition()
{
}
void ZedAcquisition::reconfigureCb(Config &config, uint32_t level)
{
//   boost::mutex::scoped_lock lock(g_image_mutex);
  colormap_ = config.colormap;
  min_depth_value_ = config.min_image_value;
  max_depth_value_ = config.max_image_value;
}

void ZedAcquisition::imageNormalize(const sensor_msgs::ImageConstPtr& msg, cv::Mat& src)
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

void ZedAcquisition::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cont_++;
    std::ostringstream image_name;

    cv::Mat depth;
    imageNormalize(msg, depth);    
    cv::namedWindow("ROS Frames", CV_WINDOW_NORMAL );
    cv::imshow("ROS Frames", depth);
    std::cout << depth.channels() << std::endl;
    // cv::resize(raw_disparity_, raw_disparity_, cv::Size(640,360));
    image_name << "/home/matheus/32bits/" << cont_ << ".png"; 
    cv::imwrite(image_name.str(), depth);
    cv::waitKey(1);
}

void ZedAcquisition::leftRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat left_image_;
    left_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (is_saving_)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        is_saving_ = false;
    }
}

void ZedAcquisition::rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    right_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (is_saving_)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        is_saving_ = false;
    }
}

}