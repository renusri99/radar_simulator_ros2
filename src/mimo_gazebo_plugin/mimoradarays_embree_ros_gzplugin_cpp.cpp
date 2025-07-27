#include <radarays_gazebo_plugins/radarays_embree_ros_gzplugin.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>

namespace ignition::gazebo
{

RadaRaysEmbreeROS::RadaRaysEmbreeROS()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysEmbreeROS"), "[RadaRaysEmbreeROS] Constructed.");
}

RadaRaysEmbreeROS::~RadaRaysEmbreeROS()
{
    RCLCPP_INFO(rclcpp::get_logger("RadaRaysEmbreeROS"), "[RadaRaysEmbreeROS] Destroyed.");
}

void RadaRaysEmbreeROS::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
    node_ = std::make_shared<rclcpp::Node>("rada_rays_embree_ros");

    sensorEntity_ = _entity;
    entityComponentManager_ = &_ecm;

    // ROS 2 Publisher Setup
    std::string topic = _sdf->Get<std::string>("topic", "/radar/mimo_data").first;
    std::string frame_id = _sdf->Get<std::string>("frame", "radar_frame").first;

    frame_id_ = frame_id;
    pub_img_ = node_->create_publisher<sensor_msgs::msg::Image>(topic, 1);

    // Register event for sensor update
    updateConnection_ = eventMgr.Connect<ignition::gazebo::events::Update>(
        std::bind(&RadaRaysEmbreeROS::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "[RadaRaysEmbreeROS] Loaded.");
}

void RadaRaysEmbreeROS::OnUpdate()
{
    if (!entityComponentManager_)
    {
        RCLCPP_ERROR(node_->get_logger(), "[RadaRaysEmbreeROS] EntityComponentManager not available!");
        return;
    }

    auto poseComp = entityComponentManager_->Component<components::Pose>(sensorEntity_);
    if (!poseComp)
    {
        RCLCPP_WARN(node_->get_logger(), "[RadaRaysEmbreeROS] No Pose Component Found!");
        return;
    }

    auto timestamp = node_->now();

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
                                                 std_msgs::msg::Header(),
                                                 "mono8",
                                                 radarays_sensor_->m_polar_image)
                                                 .toImageMsg();

    msg->header.stamp = timestamp;
    msg->header.frame_id = frame_id_;

    pub_img_->publish(*msg);
}

IGNITION_ADD_PLUGIN(RadaRaysEmbreeROS, ignition::gazebo::System,
                    RadaRaysEmbreeROS::ISystemConfigure,
                    RadaRaysEmbreeROS::ISystemUpdate)

} // namespace ignition::gazebo
