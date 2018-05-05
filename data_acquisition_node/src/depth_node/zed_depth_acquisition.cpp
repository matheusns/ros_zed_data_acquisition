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
    , min_depth_value_(1)
    , max_depth_value_(0)
    , server_()
    , files_path_("~/zed_data_acquisiton")
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

cv_bridge::CvImageConstPtr ZedDepthAcquisition::colorMap(const cv_bridge::CvImageConstPtr& source, const int color_map, bool normalized)
{
    cv_bridge::CvImagePtr result(new cv_bridge::CvImage());
    result->header = source->header;
    std::string encoding = enc::BGR8;

    if (min_depth_value_ != max_depth_value_ && !normalized)
    {
        if (color_map == -1) 
        {
            result->encoding = enc::MONO8;
            cv::Mat(source->image-min_depth_value_).convertTo(result->image, CV_8UC1, 255.0/(max_depth_value_ - min_depth_value_));
            // cv::normalize(source->image, result->image, 0, 255, NORM_MINMAX, CV_8UC1);
        }
        else
        {
            result->encoding = enc::BGR8;
            cv::Mat(source->image-min_depth_value_).convertTo(result->image, CV_8UC3, 255.0/(max_depth_value_ - min_depth_value_));
            // cv::normalize(source->image, result->image, 0, 255, NORM_MINMAX, CV_8UC3);
            cv::applyColorMap(result->image, result->image, color_map); 
        }
        return cv_bridge::cvtColor(result, encoding);
    }
    else if (min_depth_value_ != max_depth_value_ && normalized)
    {
        if (color_map == -1) 
        {
            result->encoding = enc::MONO8;
            cv::Mat(source->image-min_depth_value_).convertTo(result->image, CV_8UC1, 255.0/(10));
        }
        else
        {
            result->encoding = enc::BGR8;
            cv::Mat(source->image-min_depth_value_).convertTo(result->image, CV_8UC3, 255.0/(10));
            cv::applyColorMap(result->image, result->image, color_map); 
        }
        return cv_bridge::cvtColor(result, encoding);
    }
}

void ZedDepthAcquisition::distanceThreshold(const sensor_msgs::ImageConstPtr& msg, cv::Mat& dst, cv::Mat& normalized)
{
    cv_bridge::CvImageConstPtr source = cv_bridge::toCvShare(msg);

    cv_bridge::CvImagePtr result(new cv_bridge::CvImage());
    result->header = source->header;
    result->encoding = source->encoding;
    result->image = cv::Mat(source->image.rows, source->image.cols, CV_32FC1);
    
    cv_bridge::CvImagePtr normalized_result(new cv_bridge::CvImage());
    normalized_result->header = source->header;
    normalized_result->encoding = source->encoding;
    normalized_result->image = source->image;

    normalized = colorMap(normalized_result, colormap_, true)->image;

    if (min_depth_value_ != 0 || max_depth_value_ != 0)
    {
        for (size_t j = 0; j < source->image.rows; ++j) 
        {
            for (size_t i = 0; i < source->image.cols; ++i) 
            {
                float float_value = source->image.at<float>(j, i);

                if (float_value <= min_depth_value_)
                {
                    result->image.at<float>(j, i) = 0.0f;
                }
                else if (float_value >= max_depth_value_)
                {
                    result->image.at<float>(j, i) = 0.0f;
                }
                else
                {
                    result->image.at<float>(j, i) = float_value;
                }
                if (std::isnan(float_value))    
                {
                    result->image.at<float>(j, i) = 0;
                }
                
            }
        }
        cv::Mat image_result  = colorMap(result, colormap_)->image;
        if (image_result.empty())
        {
            ROS_INFO_STREAM("Frame Empty. Frame's time stamp = " << msg->header.stamp);
        }

        if (!image_result.empty())
        {
            dst = image_result;
        }
        return;
    }
}

void ZedDepthAcquisition::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::ostringstream image_name;
    cv::Mat depth, raw, normalized;
    if (msg->encoding == "32FC1") raw = cv_bridge::toCvShare(msg, "32FC1")->image;
    if (msg->encoding == "16UC1") raw = cv_bridge::toCvShare(msg, "16UC1")->image;

    distanceThreshold(msg,depth, normalized);
    if (!depth.empty())
    {
        printPixels(msg, depth, raw);
        // cv::namedWindow("Normalized", CV_WINDOW_NORMAL);
        image_name << "depth_" << msg->header.stamp;
        // cv::imshow("Normalized", normalized);
        // cv::imshow("Normalized", depth);
        // cv::waitKey(1);
        saveImage(depth, normalized, image_name.str());
    }
}

bool ZedDepthAcquisition::saveImage(cv::Mat& depth, cv::Mat& normalized, const std::string file_name)
{
    std::ostringstream depth_path;
    std::ostringstream normalized_path;
    
    if (files_path_[0] == '~')
    {
        files_path_ = file_manager_.normalizeUserPath(files_path_);
    }

    depth_path << files_path_ << "/depth/";
    normalized_path << files_path_ << "/normalized/";
    
    if ( !file_manager_.existsDir(depth_path.str()) )
    {
        file_manager_.createDir(depth_path.str());
    }

    if ( !file_manager_.existsDir(normalized_path.str()) )
    {
        file_manager_.createDir(normalized_path.str());
    }
    depth_path << file_name << ".png";
    normalized_path << file_name << ".png";
    cv::imwrite(normalized_path.str(), normalized);
    cv::imwrite(depth_path.str(), depth);
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

