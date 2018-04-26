#include "include/zed_acquisition_param.hpp"

namespace zed
{
ZedAcquisitionParam::ZedAcquisitionParam()
    : param_directory_path_("~/zed_data_acquisiton/")
    , param_max_distance_(10.0)
    , param_min_distance_(0.0)
    , param_iluminance_(0.0)
    , param_disturbance_(false)
    , param_camera_position_(0)
    , param_object_position_(0)
    , param_disturbance_type_(0)
{

}
    
ZedAcquisitionParam::~ZedAcquisitionParam()
{

}

bool ZedAcquisitionParam::readFromRosParameterServer(const ros::NodeHandle& nh)
{
    try
    {
        if ( nh.hasParam("files_directory") )
        {
            nh.getParam("files_directory", param_directory_path_);
        }

        if ( nh.hasParam("enviroment_iluminance") )
        {
            nh.getParam("enviroment_iluminance", param_iluminance_);
        }

        if ( nh.hasParam("max_distance") )
        {
            nh.getParam("max_distance", param_max_distance_);
        }

        if ( nh.hasParam("min_distance") )
        {
            nh.getParam("min_distance", param_min_distance_);
        }

        if ( nh.hasParam("disturbance") )
        {
            nh.getParam("disturbance", param_disturbance_);
        }

        if ( nh.hasParam("camera_position") )
        {
            nh.getParam("camera_position", param_camera_position_);
        }

        if ( nh.hasParam("disturbance_type") )
        {
            nh.getParam("disturbance_type", param_disturbance_type_);
        }

        if ( nh.hasParam("object_position") )
        {
            nh.getParam("object_position", param_object_position_);
        }

        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Problem during parameters checking: " << e.what() << '\n');
        return false;
    }
}

} // zed namespace

