#include <ros/ros.h>
#include <iostream>
#include "include/zed_depth_acquisition.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_depth_acquisition_node");
    zed::ZedDepthAcquisition zed_obj;
    ros::spin();
}