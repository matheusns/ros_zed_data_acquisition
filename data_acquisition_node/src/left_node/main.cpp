#include <ros/ros.h>
#include <iostream>
#include "include/zed_left_acquisition.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_left_acquisition_node");
    zed::ZedLeftAcquisition zed_obj;
    ros::spin();
}