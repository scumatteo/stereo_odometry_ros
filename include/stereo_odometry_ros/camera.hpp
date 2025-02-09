#pragma once

#include <opencv4/opencv2/opencv.hpp>

namespace stereo_odometry
{
    static constexpr size_t CAMERA_COUNT = 2;
    static const std::array<std::string, CAMERA_COUNT> CAMERA_NAMES{"left", "right"};
    struct Camera
    {
        using SharedPtr = std::shared_ptr<Camera>;

        std::string name;
        int width;
        int height;
        std::string distortion_model;
        cv::Mat K_raw;
        cv::Mat D_raw;
        cv::Mat R_raw;
        cv::Mat P_raw;
        cv::Mat K;
        cv::Mat D;
        cv::Mat R;
        cv::Mat P;
        cv::Mat map_x;
        cv::Mat map_y;

        Camera();

        void initUndistortRectifyMap();
    };

    struct StereoCamera
    {

        StereoCamera();

        void stereoRectify();
        void initUndistortRectifyMap();

        Camera &left();
        Camera &right();
        std::array<Camera, CAMERA_COUNT> cameras;
        cv::Mat R;
        cv::Mat t;
        cv::Mat Q;
    };

}