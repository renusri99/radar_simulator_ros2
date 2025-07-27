#include "radarays_ros/Radar.hpp"

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/types.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

namespace radarays_ros
{

Radar::Radar(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame)
: m_node(node),
  m_tf_buffer(tf_buffer),
  m_tf_listener(tf_listener),
  m_map_frame(map_frame),
  m_sensor_frame(sensor_frame)
{
    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.theta.size = 400;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;
    m_radar_model.phi.size = 1;

    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);

    loadParams();

    // Register dynamic parameter callback
    m_param_sub = m_node->add_on_set_parameters_callback(
        std::bind(&Radar::onParamChange, this, std::placeholders::_1));
}

std::optional<rm::Transform> Radar::getTsm(rclcpp::Time stamp)
{
    rm::Transform Tsm;

    try {
        geometry_msgs::msg::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
            m_map_frame,
            m_sensor_frame,
            tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

    } catch(tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF-Error: %s", ex.what());
        return std::nullopt;
    }

    return Tsm;
}

bool Radar::updateTsm(rclcpp::Time stamp)
{
    std::optional<rm::Transform> Tsm_opt;
    rclcpp::Time Tsm_stamp;

    try {
        rm::Transform Tsm;
        geometry_msgs::msg::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
            m_map_frame,
            m_sensor_frame,
            tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

        Tsm_opt = Tsm;
        Tsm_stamp = rclcpp::Time(Tsm_ros.header.stamp);
    } catch(tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF-Error: %s", ex.what());
    }

    if(!Tsm_opt && !has_last)
    {
        RCLCPP_WARN(m_node->get_logger(), "No current or previous transform. Skipping.");
        return false;
    }

    rm::Transform Tsm;
    if(Tsm_opt) {
        Tsm = *Tsm_opt;
    } else {
        Tsm = Tsm_last;
        Tsm_stamp = Tsm_stamp_last + (m_node->now() - stamp_last);
    }

    Tsm_last = Tsm;
    Tsm_stamp_last = Tsm_stamp;
    stamp_last = m_node->now();
    has_last = true;

    return true;
}

rclcpp::SetParametersResult Radar::onParamChange(const std::vector<rclcpp::Parameter> &params)
{
    for (const auto &param : params)
    {
        if (param.get_name() == "beam_width") {
            m_cfg.beam_width = param.as_double();
            m_params.model.beam_width = m_cfg.beam_width * M_PI / 180.0;
            m_resample = true;
        }
        else if (param.get_name() == "n_samples") {
            m_params.model.n_samples = param.as_int();
            m_cfg.n_samples = param.as_int();
            m_resample = true;
        }
        else if (param.get_name() == "n_reflections") {
            m_params.model.n_reflections = param.as_int();
            m_cfg.n_reflections = param.as_int();
        }
    }
    RCLCPP_INFO(m_node->get_logger(), "Radar parameters updated dynamically.");
    return rclcpp::SetParametersResult().set_successful(true);
}

void Radar::loadParams()
{
    m_params.materials = loadRadarMaterialsFromNode(m_node);
    m_node->get_parameter_or("object_materials", m_object_materials, std::vector<int>{});
    m_node->get_parameter_or("material_id_air", m_material_id_air, 0);
}

} // namespace radarays_ros