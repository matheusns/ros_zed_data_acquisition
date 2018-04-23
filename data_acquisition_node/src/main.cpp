#include <ros/ros.h>
#include <iostream>
#include "include/zed_data_acquisition.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_acquisition_node");
    zed::ZedAcquisition zed_obj;
    ros::spin();
}