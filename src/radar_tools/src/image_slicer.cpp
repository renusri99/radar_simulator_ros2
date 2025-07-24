#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <radar_tools/ImgSlicerConfig.hpp>  //  new config header

using namespace radar_tools;  

using std::placeholders::_1;
using std::placeholders::_2;

class ImageSlicer : public rclcpp::Node
{
public:
    ImageSlicer()
    : Node("image_slicer")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("col_min", 0);
        this->declare_parameter<int>("col_max", 100);
        this->declare_parameter<int>("row_min", 0);
        this->declare_parameter<int>("row_max", 100);

        config_.col_min = this->get_parameter("col_min").as_int();
        config_.col_max = this->get_parameter("col_max").as_int();
        config_.row_min = this->get_parameter("row_min").as_int();
        config_.row_max = this->get_parameter("row_max").as_int();

        cropper_.x = config_.col_min;
        cropper_.width = config_.col_max - config_.col_min;
        cropper_.y = config_.row_min;
        cropper_.height = config_.row_max - config_.row_min;

        // Setup parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ImageSlicer::on_parameter_change, this, _1)
        );

        // Setup image transport
        image_transport::ImageTransport it(shared_from_this());
        sub_ = it.subscribe("image_in", 1, std::bind(&ImageSlicer::imageCallback, this, _1));
        pub_ = it.advertise("image_out", 1);
    }

private:
    image_transport::Publisher pub_;
    image_transport::Subscriber sub_;
    cv::Rect cropper_;
    ImgSlicerConfig config_;  // radar_tools config struct
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & params)
    {
        RCLCPP_INFO(this->get_logger(), "image_slicer: param callback triggered");

        for (const auto & param : params)
        {
            if (param.get_name() == "col_min") {
                int value = param.as_int();
                config_.col_min = std::clamp(value, ImgSlicerConfig::MIN_VALUE, ImgSlicerConfig::MAX_VALUE);
            }
            else if (param.get_name() == "col_max") {
                int value = param.as_int();
                config_.col_max = std::clamp(value, ImgSlicerConfig::MIN_VALUE, ImgSlicerConfig::MAX_VALUE);
            }
            else if (param.get_name() == "row_min") {
                int value = param.as_int();
                config_.row_min = std::clamp(value, ImgSlicerConfig::MIN_VALUE, ImgSlicerConfig::MAX_VALUE);
            }
            else if (param.get_name() == "row_max") {
                int value = param.as_int();
                config_.row_max = std::clamp(value, ImgSlicerConfig::MIN_VALUE, ImgSlicerConfig::MAX_VALUE);
            }
        }

        cropper_.x = config_.col_min;
        cropper_.width = config_.col_max - config_.col_min;
        cropper_.y = config_.row_min;
        cropper_.height = config_.row_max - config_.row_min;

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg_in)
    {
        cv::Mat image_in;

        try
        {
            image_in = cv_bridge::toCvShare(msg_in)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (cropper_.x < 0 || cropper_.y < 0 ||
            cropper_.x + cropper_.width > image_in.cols ||
            cropper_.y + cropper_.height > image_in.rows)
        {
            RCLCPP_WARN(this->get_logger(), "Cropper out of bounds. Skipping frame.");
            return;
        }

        cv::Mat image_out = image_in(cropper_);

        auto msg_out = cv_bridge::CvImage(
            msg_in->header, 
            msg_in->encoding,
            image_out).toImageMsg();

        pub_.publish(msg_out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSlicer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
