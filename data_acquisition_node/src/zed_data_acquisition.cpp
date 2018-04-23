#include "include/zed_data_acquisition.hpp"

namespace zed
{
    ZedAcquisition::ZedAcquisition()
        : nh_("~")
        , img_transport_(nh_)
        , depth_sub_(img_transport_.subscribe("/zed/depth/depth_registered", 5,&ZedAcquisition::DepthImageCallback, this))
        , output_video_(NULL)
        , frame_height_(0)
        , frame_width_(0)
        , is_saving_(true)
    {
        
    }
    ZedAcquisition::~ZedAcquisition()
    {
    }

    void ZedAcquisition::DepthImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat raw_disparity;
        raw_disparity = cv_bridge::toCvShare(msg, "32FC1")->image;

        if (is_saving_)
        {
            int codec = CV_FOURCC('M', 'J', 'P', 'G');
            output_video_ = new cv::VideoWriter("/home/matheus/video.avi",-1, 3, raw_disparity.size(), true);
            frame_height_ = raw_disparity.rows;
            frame_width_  = raw_disparity.cols;
            is_saving_ = false;
        }
        std::cout << frame_width_ << " x " << frame_height_ << std::endl; 
        output_video_->write(raw_disparity);
        cv::imshow("view", raw_disparity);
        cv::waitKey(30);
    }
}