#ifndef RADARAYS_ROS_ROS_HELPER_H
#define RADARAYS_ROS_ROS_HELPER_H

#include <radarays_ros/RadarParams.h>
#include <rclcpp/rclcpp.hpp>

namespace radarays_ros
{

/**
 * @brief Load a single RadarMaterial from declared parameters
 *        e.g. base_param = "materials.0"
 */
RadarMaterial loadRadarMaterialFromParams(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& base_param);

/**
 * @brief Load full RadarMaterials structure from parameter server
 *        requires "materials.size" and "materials.N.{velocity,ambient,...}"
 */
RadarMaterials loadRadarMaterialsFromParameterServer(
    const std::shared_ptr<rclcpp::Node>& node);

/**
 * @brief Default RadarModel
 */
inline RadarModel default_params_model()
{
    RadarModel model;
    model.beam_width = 8.0 * M_PI / 180.0;
    model.n_samples = 200;
    model.n_reflections = 2;
    return model;
}

/**
 * @brief Default RadarParams
 */
inline RadarParams default_params()
{
    RadarParams ret;
    ret.model = default_params_model();
    return ret;
}

} // namespace radarays_ros

#endif // RADARAYS_ROS_ROS_HELPER_H
