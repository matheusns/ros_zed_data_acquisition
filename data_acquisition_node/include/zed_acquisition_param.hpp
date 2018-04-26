/**
* @file zed_acquisition_param.hpp
* @brief Function prototypes for the ROS Parameter interacting structure.
*
* This file contains the prototypes for the Zed Parameters class.\
* This class is responsible for manages the parameters loaded from ROS Parameter Server.  
*
* @author Matheus Nascimento.
* @ date April 2018
*/

#ifndef ZED_DATA_ACQUISITION_ZED_ACQUISITON_PARAM_HPP
#define ZED_DATA_ACQUISITION_ZED_ACQUISITON_PARAM_HPP

#include <iostream>
#include <string>

#include <ros/ros.h>
namespace zed
{
enum CameraPosition
{
    // Definir as posicoes
    CAMERA_POSITION_0,
    CAMERA_POSITION_1,
    CAMERA_POSITION_2,
    CAMERA_POSITION_3
};

enum ObjectPosition
{
    // Definir as posicoes
    OBJECT_POSITION_0,
    OBJECT_POSITION_1,
    OBJECT_POSITION_2,
    OBJECT_POSITION_3
};

enum DisturbanceType
{
    // Definir as PERTUBAÇÕES
    DISTURBANCE_0,
    DISTURBANCE_1,
    DISTURBANCE_2,
    DISTURBANCE_3
};

class ZedAcquisitionParam
{
public:
    /**
    *    @brief  Default constructor for RosParameter.
    *    @return nothing.
    */
    explicit ZedAcquisitionParam();
    /**
    *    @brief  Default destructor for RosParameter.
    *    @return nothing.
    */
    virtual ~ZedAcquisitionParam();
    /**
     *   @brief Reads the parameters from the parameter server.
     *
     *   If invalid parameters can be detected, the interface will reset them
     *   to the default values.
     *
     *   @param nh the ros::NodeHandle.
     *   @return void.
     */
    bool readFromRosParameterServer(const ros::NodeHandle& nh);    
    /**
     *   @brief Getter for the compressed output state from the ROS parameter server.
     *   @return bool.
     */
 protected:

    bool         param_disturbance_;
    std::string  param_directory_path_;                ///< Snapshot file path

    float param_max_distance_;
    float param_min_distance_;
    float param_iluminance_;

    int param_camera_position_;
    int param_object_position_;
    int param_disturbance_type_;
};

} // zed namespace
#endif  // ZED_DATA_ACQUISITION_ZED_ACQUISITON_PARAM_HPP
