#pragma once

#include <opencv4/opencv2/opencv.hpp>

namespace stereo_odometry
{
static constexpr size_t SEC_TO_NANOSEC = 1e9;

    struct Frame
    {        
        Frame();

        cv::Mat image;
        int32_t sec;
        int32_t nanosec;
        std::string encoding;

        uint64_t stamp();
    };

    struct StereoFrame
    {
        StereoFrame();

        Frame left;
        Frame right;
        std::vector<cv::Point2f> points2D;
        std::vector<cv::Point3f> points3D;
        cv::Mat pose;
        cv::Mat prev_matches;
        cv::Mat right_matches;
    };
}