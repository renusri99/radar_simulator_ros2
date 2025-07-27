#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core.hpp>

#include <rmagine/types/Transform.hpp>
#include <rmagine/types/simulation/SimulationResults.hpp>
#include <rmagine/types/sensors/RadarModel.hpp>
#include <rmagine/types/Bundle.hpp>

#include "radarays_ros/types/RadarMaterial.hpp"
#include "radarays_ros/types/RadarConfig.hpp"

namespace radarays_ros
{

class Radar
{
public:
    Radar(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame);

    std::optional<rmagine::Transform> getTsm(rclcpp::Time stamp);
    bool updateTsm(rclcpp::Time stamp);

protected:
    void loadParams();
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_sub;
    rclcpp::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &params);

    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    std::string m_map_frame;
    std::string m_sensor_frame;

    rmagine::RadarModel m_radar_model;
    RadarParameters m_params;
    RadarConfig m_cfg;

    cv::Mat m_polar_image;

    std::vector<int> m_object_materials;

    rmagine::Transform Tsm_last;
    rclcpp::Time Tsm_stamp_last;
    rclcpp::Time stamp_last;
    bool has_last = false;

    int m_material_id_air = 0;
    float m_wave_energy_threshold = 0.001;
    bool m_resample = true;
};

} // namespace radarays_ros
