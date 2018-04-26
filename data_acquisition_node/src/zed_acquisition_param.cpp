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
    , param_colormap_(-1)
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

        if ( nh.hasParam("colormap") )
        {
            nh.getParam("colormap", param_colormap_);
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

std::string ZedAcquisitionParam::filesDirectory() const
{
    return param_directory_path_;
}

bool  ZedAcquisitionParam::disturbance() const
{
    return param_disturbance_;    
}
float ZedAcquisitionParam::iluminance() const
{
    return param_iluminance_;
}
float ZedAcquisitionParam::maxDistance() const
{
    return param_max_distance_;
}
float ZedAcquisitionParam::minDistance() const
{
    return param_min_distance_;
}

int ZedAcquisitionParam::colormap() const
{
    return param_colormap_;
}

CameraPosition  ZedAcquisitionParam::cameraPosition() const
{
    switch (param_camera_position_)
    {
       case CAMERA_POSITION_0:
            return CAMERA_POSITION_0;
       case CAMERA_POSITION_1:
            return CAMERA_POSITION_1;
       case CAMERA_POSITION_2:
            return CAMERA_POSITION_2;
       case CAMERA_POSITION_3:
            return CAMERA_POSITION_3;
       default: 
            return CAMERA_POSITION_0;
    }
}
ObjectPosition  ZedAcquisitionParam::objectPosition() const
{
    switch (param_object_position_)
    {
       case OBJECT_POSITION_0:
            return OBJECT_POSITION_0;
       case OBJECT_POSITION_1:
            return OBJECT_POSITION_1;
       case OBJECT_POSITION_2:
            return OBJECT_POSITION_2;
       case OBJECT_POSITION_3:
            return OBJECT_POSITION_3;
       default: 
            return OBJECT_POSITION_0;
    }

}
DisturbanceType ZedAcquisitionParam::disturbanceType() const
{
    switch (param_disturbance_type_)
    {
       case DISTURBANCE_0:
            return DISTURBANCE_0;
       case DISTURBANCE_1:
            return DISTURBANCE_1;
       case DISTURBANCE_2:
            return DISTURBANCE_2;
       case DISTURBANCE_3:
            return DISTURBANCE_3;
       default: 
            return DISTURBANCE_0;
    }
}

} // zed namespace

