#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <radarays_ros/RadarCPU.hpp>
#include <radarays_ros/RadarGPU.hpp>
#include <radarays_ros/ros_helper.h>
#include <radarays_ros/GetRadarParams.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace radarays_ros;
namespace rm = rmagine;

class RadarSimulatorNode : public rclcpp::Node
{
public:
    RadarSimulatorNode()
    : Node("radar_simulator")
    {
        declare_parameter<std::string>("map_file", "");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("sensor_frame", "radar");
        declare_parameter<std::string>("sync_topic", "");
        declare_parameter<bool>("gpu", false);

        get_parameter("map_file", map_file_);
        get_parameter("map_frame", map_frame_);
        get_parameter("sensor_frame", sensor_frame_);
        get_parameter("sync_topic", sync_topic_);
        get_parameter("gpu", use_gpu_);

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Map load and radar init
        if (!loadMapAndInitRadar())
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize radar simulator.");
            rclcpp::shutdown();
            return;
        }

        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        pub_polar_ = image_transport_->advertise("radar/image", 1);

        pub_pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud>("radar/pcl_real", 1);

        radar_param_srv_ = this->create_service<radarays_ros::srv::GetRadarParams>(
            "get_radar_params",
            std::bind(&RadarSimulatorNode::getRadarParamsCB, this,
                      std::placeholders::_1, std::placeholders::_2));

        if (!sync_topic_.empty())
        {
            RCLCPP_INFO(get_logger(), "Radar simulator in SYNC mode. Subscribing to: %s", sync_topic_.c_str());

            sync_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                sync_topic_, 1,
                std::bind(&RadarSimulatorNode::syncCallback, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Radar simulator in UNSYNCED mode.");

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&RadarSimulatorNode::unsyncedLoop, this));
        }
    }

private:
    bool loadMapAndInitRadar()
    {
        if (use_gpu_)
        {
#if defined RADARAYS_WITH_GPU
            auto map = rm::import_optix_map(map_file_);
            radarays_sim_ = std::make_shared<RadarGPU>(
                shared_from_this(),
                tf_buffer_,
                tf_listener_,
                map_frame_,
                sensor_frame_,
                map);
            return true;
#else
            RCLCPP_ERROR(get_logger(), "GPU not supported in this build.");
            return false;
#endif
        }
        else
        {
#if defined RADARAYS_WITH_CPU
            auto map = rm::import_embree_map(map_file_);
            radarays_sim_ = std::make_shared<RadarCPU>(
                shared_from_this(),
                tf_buffer_,
                tf_listener_,
                map_frame_,
                sensor_frame_,
                map);
            return true;
#else
            RCLCPP_ERROR(get_logger(), "CPU not supported in this build.");
            return false;
#endif
        }
    }

    void syncCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        radarays_sim_->loadParams();
        auto result = radarays_sim_->simulate(msg->header.stamp);
        if (result)
        {
            pub_polar_.publish(result);
        }
    }

    void unsyncedLoop()
    {
        radarays_sim_->loadParams();
        auto result = radarays_sim_->simulate(this->get_clock()->now());
        if (result)
        {
            pub_polar_.publish(result);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Radar simulation returned null image.");
        }
    }

    void getRadarParamsCB(
        const std::shared_ptr<radarays_ros::srv::GetRadarParams::Request> request,
        std::shared_ptr<radarays_ros::srv::GetRadarParams::Response> response)
    {
        (void)request;
        response->params = radarays_sim_->getParams();
    }

    // Parameters
    std::string map_file_;
    std::string map_frame_;
    std::string sensor_frame_;
    std::string sync_topic_;
    bool use_gpu_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Radar sim
    RadarPtr radarays_sim_;

    // Publishers
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher pub_polar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_pcl_;

    // Sync sub
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sync_sub_;

    // Service
    rclcpp::Service<radarays_ros::srv::GetRadarParams>::SharedPtr radar_param_srv_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
