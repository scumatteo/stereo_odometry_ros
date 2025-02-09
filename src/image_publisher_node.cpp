#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
        : Node("image_publisher")   
    {
        std::string left_prefix = "left";
        std::string right_prefix = "right";

        this->declare_parameter<std::string>("root", "");
        this->declare_parameter<std::string>("left_path", "");
        this->declare_parameter<std::string>("right_path", "");
        this->declare_parameter<int>("fps", 0);
        this->declare_parameters(left_prefix);
        this->declare_parameters(right_prefix);

        std::string root = this->get_parameter("root").as_string();

        RCLCPP_INFO(this->get_logger(), "Parameters declared");

        this->create_image_publisher(left_prefix, this->left_publisher_);
        this->create_image_publisher(right_prefix, this->right_publisher_);

        RCLCPP_INFO(this->get_logger(), "Camera publishers created");

        RCLCPP_INFO(this->get_logger(), "Searching for images in %s", cv::String(root + "/" + this->get_parameter("left_path").as_string() + "/*.png").c_str());

        cv::glob(cv::String(
                     root + "/" + this->get_parameter("left_path").as_string() + "/*.png"),
                 this->left_paths_);
        cv::glob(cv::String(
                     root + "/" + this->get_parameter("right_path").as_string() + "/*.png"),
                 this->right_paths_);

        RCLCPP_INFO(this->get_logger(), "Founded %d left images!", this->left_paths_.size());
        RCLCPP_INFO(this->get_logger(), "Founded %d right images!", this->right_paths_.size());

        int fps = this->get_parameter("fps").as_int();
        this->rate_ = std::make_shared<rclcpp::Rate>(fps);
    }

    void publish_messages()
    {
        size_t i = 0;
        while (rclcpp::ok())
        {
            rclcpp::Time now = this->get_clock()->now();
            cv::Mat left_image = cv::imread(this->left_paths_[i], cv::IMREAD_GRAYSCALE);
            cv::Mat right_image = cv::imread(this->right_paths_[i], cv::IMREAD_GRAYSCALE);

            cv_bridge::CvImage left_bridge, right_bridge;

            this->create_image_msg(left_image, now, this->get_parameter("left.frame_id").as_string(), left_bridge);
            this->create_image_msg(right_image, now, this->get_parameter("right.frame_id").as_string(), right_bridge);

            this->left_publisher_.publish(left_bridge.toImageMsg());
            this->right_publisher_.publish(right_bridge.toImageMsg());

            RCLCPP_INFO(this->get_logger(), "Image messages published!");

            i++;
            if (i >= this->left_paths_.size())
            {
                break;
            }

            this->rate_->sleep();
        }
    }

private:
    void declare_parameters(const std::string &prefix)
    {
        this->declare_parameter<int>(prefix + ".queue_size", 0);
        this->declare_parameter<std::string>(prefix + ".topic", "");
        this->declare_parameter<std::string>(prefix + ".frame_id", "");
    }

    void create_image_publisher(const std::string &prefix,
                                image_transport::Publisher &publisher)
    {
        publisher = image_transport::create_publisher(this, this->get_parameter(prefix + ".topic").as_string());
    }

    void create_image_msg(const cv::Mat &image,
                          const rclcpp::Time &now,
                          const std::string &frame_id,
                          cv_bridge::CvImage &image_bridge)
    {
        image_bridge.image = image;
        image_bridge.encoding = sensor_msgs::image_encodings::MONO8;
        image_bridge.header.frame_id = frame_id;
        image_bridge.header.stamp = now;
    }
    std::vector<std::string> left_paths_, right_paths_;
    std::shared_ptr<rclcpp::Rate> rate_;
    image_transport::Publisher left_publisher_, right_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto image_publisher_node = std::make_shared<ImagePublisher>();
    image_publisher_node->publish_messages();
    rclcpp::shutdown();
    return 0;
}