#ifndef IGNITION_RADARAYS_OPTIX_ROS_PLUGIN_H
#define IGNITION_RADARAYS_OPTIX_ROS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <unordered_map>

namespace rm = rmagine;

namespace ignition
{
namespace gazebo
{

class MIMORadarOptixROSPlugin : public ignition::gazebo::System,
                                public ignition::gazebo::ISystemConfigure,
                                public ignition::gazebo::ISystemPostUpdate
{
public:
    MIMORadarOptixROSPlugin();
    virtual ~MIMORadarOptixROSPlugin();

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    void parseOutputs(const std::shared_ptr<const sdf::Element> &_sdf);
    void publishRadarImage();

    ignition::gazebo::Entity entity_;
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;

    std::shared_ptr<rmagine::OnDnSimulatorOptix> simulator_;

    std::string frame_id_;
    std::string topic_;
};

} // namespace gazebo
} // namespace ignition

#endif // IGNITION_RADARAYS_OPTIX_ROS_PLUGIN_H
