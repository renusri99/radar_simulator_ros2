#include "mimo_radar_optix_ros_plugin.h"

#include <cv_bridge/cv_bridge.h>

namespace ignition
{
namespace gazebo
{

MIMORadarOptixROSPlugin::MIMORadarOptixROSPlugin() : System() {}

MIMORadarOptixROSPlugin::~MIMORadarOptixROSPlugin() {}

void MIMORadarOptixROSPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        ignition::gazebo::EntityComponentManager &_ecm,
                                        ignition::gazebo::EventManager &_eventMgr)
{
    entity_ = _entity;
    node_ = std::make_shared<rclcpp::Node>("mimo_radar_optix_ros_node");

    // Load parameters from SDF
    frame_id_ = _sdf->Get<std::string>("frame", "mimo_radar_frame").first;
    topic_ = _sdf->Get<std::string>("topic", "/mimo_radar/image").first;

    // Publisher for MIMO Radar Image
    pub_img_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_, 10);

    // Initialize Rmagine simulator for MIMO processing
    simulator_ = std::make_shared<rmagine::OnDnSimulatorOptix>();

    RCLCPP_INFO(node_->get_logger(), "[MIMORadarOptixROSPlugin] Loaded.");
}

void MIMORadarOptixROSPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                         const ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // Get the radar's pose
    auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(entity_);
    if (!poseComp)
        return;

    rmagine::Transform radar_pose;
    radar_pose.t.x = poseComp->Data().Pos().X();
    radar_pose.t.y = poseComp->Data().Pos().Y();
    radar_pose.t.z = poseComp->Data().Pos().Z();

    // Simulate radar image
    publishRadarImage(radar_pose);
}

void MIMORadarOptixROSPlugin::publishRadarImage(const rmagine::Transform &radar_pose)
{
    // Simulate MIMO radar scan
    rmagine::Scan scan;
    simulator_->simulate(scan);

    // Convert scan data to an image
    cv::Mat radar_image(scan.size(), 1, CV_8UC1);
    for (size_t i = 0; i < scan.size(); i++)
    {
        radar_image.at<uint8_t>(i, 0) = static_cast<uint8_t>(scan.ranges[i] * 255.0);
    }

    // Convert OpenCV image to ROS2 message
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "mono8", radar_image).toImageMsg();

    msg->header.stamp = node_->now();
    msg->header.frame_id = frame_id_;

    pub_img_->publish(*msg);
    RCLCPP_INFO(node_->get_logger(), "Published MIMO Radar Image.");
}

IGNITION_ADD_PLUGIN(MIMORadarOptixROSPlugin,
                    ignition::gazebo::System,
                    MIMORadarOptixROSPlugin::ISystemConfigure,
                    MIMORadarOptixROSPlugin::ISystemPostUpdate)

} // namespace gazebo
} // namespace ignition
