#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>

#include "radar_tools/msg/radar_info.hpp"

using radar_tools::msg::RadarInfo;

struct RadarPoint
{
    float x;
    float y;
    float z;
    float intensity;
};

class RadarImgToPclNode : public rclcpp::Node
{
public:
    RadarImgToPclNode()
    : Node("radar_img_to_pcl")
    {

        RCLCPP_INFO(this->get_logger(), "RadarImgToPclNode has started.");

        this->declare_parameter("frame", "");
        this->declare_parameter("cut_intensity_min", 0.2);
        this->declare_parameter("cut_intensity_max", 1.0);
        this->declare_parameter("cut_range_min", 1.0);
        this->declare_parameter("cut_range_max", 100.0);

        this->declare_parameter("radar.phi.min", -3.14);
        this->declare_parameter("radar.phi.n", 400);
        this->declare_parameter("radar.phi.inc", 0.0157);

        this->declare_parameter("radar.theta.min", 0.0);
        this->declare_parameter("radar.theta.n", 1);
        this->declare_parameter("radar.theta.inc", 0.0);

        this->declare_parameter("radar.range.min", 0.0);
        this->declare_parameter("radar.range.n", 3424);
        this->declare_parameter("radar.range.inc", 0.02921);

        frame_id_ = this->get_parameter("frame").as_string();
        cut_intensity_min_ = this->get_parameter("cut_intensity_min").as_double();
        cut_intensity_max_ = this->get_parameter("cut_intensity_max").as_double();
        cut_range_min_ = this->get_parameter("cut_range_min").as_double();
        cut_range_max_ = this->get_parameter("cut_range_max").as_double();

        radar_info_ = loadRadarInfo();

        pub_pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

  
        sub_ = image_transport::create_subscription(
            this,
            "radar_image",
            std::bind(&RadarImgToPclNode::imageCallback, this, std::placeholders::_1),
            "raw"
        );
    }

private:
    std::string frame_id_;
    double cut_intensity_min_, cut_intensity_max_;
    double cut_range_min_, cut_range_max_;
    RadarInfo radar_info_;

    image_transport::Subscriber sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_;

    RadarInfo loadRadarInfo()
    {
        RadarInfo info;
        info.phi_min  = this->get_parameter("radar.phi.min").as_double();
        info.phi_n   = this->get_parameter("radar.phi.n").as_int();
        info.phi_inc  = this->get_parameter("radar.phi.inc").as_double();

        info.theta_min = this->get_parameter("radar.theta.min").as_double();
        info.theta_n  = this->get_parameter("radar.theta.n").as_int();
        info.theta_inc = this->get_parameter("radar.theta.inc").as_double();

        info.range_min = this->get_parameter("radar.range.min").as_double();
        info.range_n   = this->get_parameter("radar.range.n").as_int();
        info.range_inc = this->get_parameter("radar.range.inc").as_double();

        return info;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {

        cv::Mat image;
        try
        {
            image = cv_bridge::toCvShare(msg, "mono8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image received: %d x %d", image.cols, image.rows);

        radar_info_.range_n = image.rows;
        radar_info_.phi_n = image.cols;

        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl_msg.header.stamp = msg->header.stamp;
        pcl_msg.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

        pcl_msg.height = 1;
        pcl_msg.is_dense = true;
        pcl_msg.point_step = sizeof(RadarPoint);

        pcl_msg.fields.clear();
        sensor_msgs::msg::PointField f;

        f.name = "x"; f.offset = offsetof(RadarPoint, x); f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
        pcl_msg.fields.push_back(f);
        f.name = "y"; f.offset = offsetof(RadarPoint, y);
        pcl_msg.fields.push_back(f);
        f.name = "z"; f.offset = offsetof(RadarPoint, z);
        pcl_msg.fields.push_back(f);
        f.name = "intensity"; f.offset = offsetof(RadarPoint, intensity);
        pcl_msg.fields.push_back(f);

        std::vector<RadarPoint> radar_points;
        for (int i = 0; i < image.rows; ++i)
        {
            for (int j = 0; j < image.cols; ++j)
            {
                uint8_t val = image.at<uint8_t>(i, j);
                float val_rel = static_cast<float>(val) / 255.0f;
                double phi = radar_info_.phi_min + j * radar_info_.phi_inc;
                double dist = radar_info_.range_min + (i + 0.5) * radar_info_.range_inc;

                
                if (val >= 0) // Accept all pixels, even dark ones
                {
                    RadarPoint p;
                    p.x = -dist * std::cos(phi);
                    p.y = dist * std::sin(phi);
                    p.z = 0.0;
                    p.intensity = val_rel;
                    radar_points.push_back(p);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Generated %lu radar points", radar_points.size());



        pcl_msg.width = radar_points.size();
        pcl_msg.row_step = pcl_msg.width * pcl_msg.point_step;
        pcl_msg.data.resize(pcl_msg.row_step);
        std::memcpy(pcl_msg.data.data(), radar_points.data(), pcl_msg.data.size());

        pub_pcl_->publish(pcl_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarImgToPclNode>());
    rclcpp::shutdown();
    return 0;
}
