#include "include/zed_data_acquisition.hpp"

namespace zed
{
ZedAcquisition::ZedAcquisition()
    : nh_("~")
    , img_transport_(nh_)
    , depth_sub_(img_transport_.subscribe("/zed/depth/depth_registered", 1,&ZedAcquisition::depthImageCallback, this))
    , left_sub_(img_transport_.subscribe("/zed/left/image_raw_color", 1,&ZedAcquisition::leftRawImageCallback, this))
    , right_sub_(img_transport_.subscribe("/zed/right/image_raw_color", 1,&ZedAcquisition::rightRawImageCallback, this))
    , disparity_output_video_(NULL)
    , right_output_video_(NULL)
    , left_output_video_(NULL)
    , right_image_()
    , left_image_()
    , raw_disparity_()
    , frame_height_(0)
    , frame_width_(0)
    , cont_(0)
    , is_saving_(false)
{
    
}
ZedAcquisition::~ZedAcquisition()
{
}

bool ZedAcquisition::readPrf(int &cols, int &rows, char *&data, const char *path)
{
	try
	{
		std::ifstream infile(path, std::ifstream::binary);

		// read data as a block:
		infile.read((char *)&cols, 4);
		infile.read((char *)&rows, 4);

		data = new char[rows * cols];
		infile.read(data, rows * cols);
	}
	catch (std::exception &e)
	{
		return false;
	}
	return true;
}

bool ZedAcquisition::readPrf(cv::Mat &frame, const char *path)
{

	int cols, rows;
	char *data;
	bool success = readPrf(cols, rows, data, path);
	frame = cv::Mat(cv::Size(cols, rows), CV_8UC1, data);

	return success;
}

bool ZedAcquisition::writePrf(int cols, int rows, unsigned char *&data, const char *path)
{
	try
	{
		std::ofstream outfile(path, std::ios::out | std::ios::binary);
		outfile.write((char *)&cols, 4);
		outfile.write((char *)&rows, 4);
		outfile.write((char *)data, cols * rows);
		outfile.close();
	}
	catch (std::exception &e)
	{
		return false;
	}
	return true;
}

void ZedAcquisition::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    raw_disparity_ = cv_bridge::toCvShare(msg, "32FC1")->image;
    cont_++;
    std::ostringstream image_name;

    if (is_saving_)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        disparity_output_video_ = new cv::VideoWriter("/home/matheus/video.avi",-1, 3, raw_disparity_.size(), true);
        is_saving_ = false;
    }
    frame_height_ = raw_disparity_.rows;
    frame_width_  = raw_disparity_.cols;
    std::cout << frame_width_ << " x " << frame_height_ << std::endl; 
    // disparity_output_video_->write(raw_disparity_);
    cv::resize(raw_disparity_, raw_disparity_, cv::Size(640,360));
    cv::imshow("view", raw_disparity_);
    image_name << "/home/matheus/32bits/" << cont_ << ".png"; 
    cv::imwrite(image_name.str(), raw_disparity_);
    cv::waitKey(1);
}

void ZedAcquisition::leftRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat left_image_;
    left_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (is_saving_)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        left_output_video_ = new cv::VideoWriter("/home/matheus/left.avi",codec, 3, left_image_.size(), true);
        is_saving_ = false;
    }
    // left_output_video_->write(left_image_);
    // cv::imshow("view", left_image_);
    // cv::waitKey(1);
}

void ZedAcquisition::rightRawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    right_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (is_saving_)
    {
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        right_output_video_ = new cv::VideoWriter("/home/matheus/right.avi",codec, 3, right_image_.size(), true);
        is_saving_ = false;
    }
    // right_output_video_->write(right_image_);
    // cv::imshow("view", right_image_);
    // cv::waitKey(1);
}

}