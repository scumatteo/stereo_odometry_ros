#pragma once

#include "rclcpp/rclcpp.hpp"
#include "stereo_odometry_ros/camera.hpp"
#include "stereo_odometry_ros/frame.hpp"

namespace stereo_odometry
{

    enum TrackingStatus
    {
        INIT = 0,
        GOOD = 1,
        LOST = 2
    };

    struct StereoOdometry
    {
        StereoOdometry(const rclcpp::Logger &logger);

        void computeOdometry(StereoFrame &frame);

        cv::Mat r;
        cv::Mat t;
        StereoFrame prev;
        StereoCamera camera;
        TrackingStatus status = TrackingStatus::INIT;
        rclcpp::Logger logger;

    private:
        bool is_inside(const cv::Point2f &point, int width, int height);
        void track(const Frame &prev,
                   const Frame &next,
                   const std::vector<cv::Point2f> &prev_pts,
                   std::vector<cv::Point2f> &next_pts,
                   std::vector<uchar> &status);

        void drawMatches(const cv::Mat &image1,
                      std::vector<cv::Point2f> &points1,
                      const cv::Mat &image2,
                      std::vector<cv::Point2f> &points2,
                      cv::Mat &out);
    };

}
