#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#include "stereo_odometry_ros/camera.hpp"
#include "stereo_odometry_ros/frame.hpp"
#include "stereo_odometry_ros/stereo_odometry.hpp"

using namespace std::chrono_literals;

// TODO image_transport for compressed images
namespace stereo_odometry
{
    class StereoOdometryNode : public rclcpp::Node
    {
    public:
        StereoOdometryNode() : Node("stereo_odometry_node"),
          odom_(this->get_logger())
          {
         this->declare_parameter<int>("queue_size", 1);
            this->declare_parameter<std::vector<double>>("tf", {});

            std::vector<double> tf = this->get_parameter("tf").as_double_array();

            cv::Mat t = (cv::Mat_<double>(3, 1) << tf[0], tf[1], tf[2]);
            cv::Mat r = (cv::Mat_<double>(3, 1) << tf[3], tf[4], tf[5]);

            cv::Mat R_tf;
            cv::Rodrigues(r, R_tf);

            this->camera_.R = R_tf;
            this->camera_.t = t;

            left_publisher_ = image_transport::create_publisher(this, "/left/image_rect_raw");
            right_publisher_ = image_transport::create_publisher(this, "/right/image_rect_raw");

            for (size_t i = 0; i < CAMERA_COUNT; i++)
            {
                this->declare_params(CAMERA_NAMES[i]);
                subs_[i].subscribe(this,
                                   this->get_parameter(CAMERA_NAMES[i] + ".topic").as_string());
                this->set_params(CAMERA_NAMES[i], this->camera_.cameras[i]);
            }

            camera_.stereoRectify();
            camera_.initUndistortRectifyMap();

            RCLCPP_DEBUG(this->get_logger(), "R ext %f %f %f %f %f %f %f %f %f", R_tf.at<double>(0, 0),
                         R_tf.at<double>(0, 1),
                         R_tf.at<double>(0, 2),
                         R_tf.at<double>(1, 0),
                         R_tf.at<double>(1, 1),
                         R_tf.at<double>(1, 2),
                         R_tf.at<double>(2, 0),
                         R_tf.at<double>(2, 1),
                         R_tf.at<double>(2, 2));

            this->print_camera(camera_.left());
            this->print_camera(camera_.right());

            odom_.camera = camera_;

            this->sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(this->get_parameter("queue_size").as_int()),
                                                                                      this->subs_[0],
                                                                                      this->subs_[1]);

            this->sync_->registerCallback(std::bind(&StereoOdometryNode::image_callback,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2));

            path_.header.frame_id = "map";
            path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        }

    private:
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                           sensor_msgs::msg::Image>;
        void print_camera(const Camera &camera)
        {
            RCLCPP_DEBUG(this->get_logger(), ("R %f %f %f %f %f %f %f %f %f"), camera.R.at<double>(0, 0),
                         camera.R.at<double>(0, 1),
                         camera.R.at<double>(0, 2),
                         camera.R.at<double>(1, 0),
                         camera.R.at<double>(1, 1),
                         camera.R.at<double>(1, 2),
                         camera.R.at<double>(2, 0),
                         camera.R.at<double>(2, 1),
                         camera.R.at<double>(2, 2));
            RCLCPP_DEBUG(this->get_logger(), "K %f %f %f %f %f %f %f %f %f", camera.K.at<double>(0, 0),
                         camera.K.at<double>(0, 1),
                         camera.K.at<double>(0, 2),
                         camera.K.at<double>(1, 0),
                         camera.K.at<double>(1, 1),
                         camera.K.at<double>(1, 2),
                         camera.K.at<double>(2, 0),
                         camera.K.at<double>(2, 1),
                         camera.K.at<double>(2, 2));
            RCLCPP_DEBUG(this->get_logger(), "P %f %f %f %f %f %f %f %f %f %f %f %f", camera.P.at<double>(0, 0),
                         camera.P.at<double>(0, 1),
                         camera.P.at<double>(0, 2),
                         camera.P.at<double>(0, 3),
                         camera.P.at<double>(1, 0),
                         camera.P.at<double>(1, 1),
                         camera.P.at<double>(1, 2),
                         camera.P.at<double>(1, 3),
                         camera.P.at<double>(2, 0),
                         camera.P.at<double>(2, 1),
                         camera.P.at<double>(2, 2),
                         camera.P.at<double>(2, 3));
        }
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
        {
            RCLCPP_DEBUG(this->get_logger(), "Callback");
            if (left_msg->data.empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Left image is empty");
                return;
            }
            if (right_msg->data.empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "Right image is empty");
                return;
            }

            StereoFrame frame;

            msg_to_frame(left_msg, frame.left);
            msg_to_frame(right_msg, frame.right);

            RCLCPP_DEBUG(this->get_logger(), "Frame created");

            cv::remap(frame.left.image, frame.left.image, camera_.left().map_x, camera_.left().map_y, cv::INTER_LINEAR);
            cv::remap(frame.right.image, frame.right.image, camera_.right().map_x, camera_.right().map_y, cv::INTER_LINEAR);

            RCLCPP_DEBUG(this->get_logger(), "Images remapped");

            cv_bridge::CvImage left_processed, right_processed;
            frame_to_msg(frame.left, left_processed);
            frame_to_msg(frame.right, right_processed);
            left_publisher_.publish(left_processed.toImageMsg());
            right_publisher_.publish(right_processed.toImageMsg());

            RCLCPP_DEBUG(this->get_logger(), "Images republished");

            odom_.computeOdometry(frame);
            outfile.open("/root/poses.txt", std::ios::app);
            if (outfile.is_open())
            {
                outfile << frame.pose.at<double>(0, 3) << " " << frame.pose.at<double>(2, 3) << "\n";
            }
            outfile.close();
            std::vector<double> r;
            cv::Rodrigues(frame.pose(cv::Range(0, 3), cv::Range(0, 3)), r);
            tf2::Quaternion q;
            q.setRPY(r[0], r[1], r[2]);

            auto now = this->get_clock()->now();
            geometry_msgs::msg::PoseStamped pose;   
            pose.header.frame_id = "map";
            pose.header.stamp = now;
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.position.x = frame.pose.at<double>(0, 3);
            pose.pose.position.y = frame.pose.at<double>(1, 3);
            pose.pose.position.z = frame.pose.at<double>(2, 3);

            path_.header.stamp = now;
            path_.poses.emplace_back(pose);

            path_publisher_->publish(path_);
        }

        void msg_to_frame(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                          Frame &frame)
        {
            cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(msg, msg->encoding);
            frame.image = bridge->image;
            frame.sec = bridge->header.stamp.sec;
            frame.nanosec = bridge->header.stamp.nanosec;
            frame.encoding = bridge->encoding;
        }

        void frame_to_msg(const Frame &frame,
                          cv_bridge::CvImage &bridge)
        {
            bridge.header.stamp.sec = frame.sec;
            bridge.header.stamp.nanosec = frame.nanosec;
            bridge.image = frame.image;
            bridge.encoding = frame.encoding;
        }

        void declare_params(const std::string &prefix)
        {
            this->declare_parameter<std::string>(prefix + ".topic", "");
            this->declare_parameter<int>(prefix + ".height", 0);
            this->declare_parameter<int>(prefix + ".width", 0);
            this->declare_parameter<std::string>(prefix + ".distortion_model", "");
            this->declare_parameter<std::vector<double>>(prefix + ".D", {});
            this->declare_parameter<std::vector<double>>(prefix + ".K", {});
            this->declare_parameter<std::vector<double>>(prefix + ".R", {});
            this->declare_parameter<std::vector<double>>(prefix + ".P", {});
            this->declare_parameter<int>(prefix + ".binning_x", 0);
            this->declare_parameter<int>(prefix + ".binning_y", 0);
            this->declare_parameter<int>(prefix + ".roi.x_offset", 0);
            this->declare_parameter<int>(prefix + ".roi.y_offset", 0);
            this->declare_parameter<int>(prefix + ".roi.height", 0);
            this->declare_parameter<int>(prefix + ".roi.width", 0);
            this->declare_parameter<bool>(prefix + ".roi.do_rectify", false);
        }

        void set_params(const std::string &prefix,
                        Camera &camera)
        {
            camera.width = this->get_parameter(prefix + ".width").as_int();
            camera.height = this->get_parameter(prefix + ".height").as_int();

            std::vector<double> K = this->get_parameter(prefix + ".K").as_double_array();
            camera.K_raw = cv::Mat(K, true).reshape(1, 3);

            std::vector<double> D = this->get_parameter(prefix + ".D").as_double_array();
            camera.D_raw = cv::Mat(D, true).reshape(1, 5);

            std::vector<double> R = this->get_parameter(prefix + ".R").as_double_array();
            camera.R_raw = cv::Mat(R, true).reshape(1, 3);

            std::vector<double> P = this->get_parameter(prefix + ".P").as_double_array();
            camera.P_raw = cv::Mat(P, true).reshape(1, 3);
        }

        std::array<message_filters::Subscriber<sensor_msgs::msg::Image>, CAMERA_COUNT> subs_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        image_transport::Publisher left_publisher_, right_publisher_;
        StereoCamera camera_;
        StereoOdometry odom_;

        nav_msgs::msg::Path path_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        std::ofstream outfile;
    };
}

using stereo_odometry::StereoOdometryNode;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
