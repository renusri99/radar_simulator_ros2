#ifndef IGNITION_GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H
#define IGNITION_GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/events/Events.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <unordered_map>

namespace rm = rmagine;

namespace ignition::gazebo
{

class RadaRaysEmbreeROS : public ignition::gazebo::System,
                          public ISystemConfigure,
                          public ISystemUpdate
{
public:
    RadaRaysEmbreeROS();
    virtual ~RadaRaysEmbreeROS();

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    // Sensor object
    ignition::gazebo::Entity sensorEntity_;
    ignition::gazebo::EntityComponentManager *entityComponentManager_;

    // ROS 2 Node
    std::shared_ptr<rclcpp::Node> node_;

    // ROS 2 Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;

    // Sensor Frame
    std::string frame_id_;
};

} // namespace ignition::gazebo

#endif // IGNITION_GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H
