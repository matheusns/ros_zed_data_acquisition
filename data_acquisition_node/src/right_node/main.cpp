#include <ros/ros.h>
#include <iostream>
#include "include/zed_right_acquisition.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_right_acquisition_node");
    zed::ZedRightAcquisition zed_obj;
    ros::spin();
}