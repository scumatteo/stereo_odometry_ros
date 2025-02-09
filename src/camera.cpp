#include "stereo_odometry_ros/camera.hpp"

namespace stereo_odometry
{
    Camera::Camera() {}
    void Camera::initUndistortRectifyMap()
    {
        cv::initUndistortRectifyMap(K_raw,
                                    D_raw,
                                    R,
                                    P,
                                    cv::Size(width, height),
                                    CV_16SC2,
                                    map_x,
                                    map_y);
        K = P(cv::Range(0, 3), cv::Range(0, 3));
        D = cv::Mat();
    }

     StereoCamera::StereoCamera()
        {
            left().name = CAMERA_NAMES[0];
            right().name = CAMERA_NAMES[1];
        }

    void StereoCamera::stereoRectify()
    {
        cv::stereoRectify(left().K_raw,
                          left().D_raw,
                          right().K_raw,
                          right().D_raw,
                          cv::Size(left().width, left().height),
                          R,
                          t,
                          left().R,
                          right().R,
                          left().P,
                          right().P,
                          Q);
    }

    void StereoCamera::initUndistortRectifyMap()
    {
        left().initUndistortRectifyMap();
        right().initUndistortRectifyMap();
    }

    Camera &StereoCamera::left() { return cameras[0]; }
    Camera &StereoCamera::right() { return cameras[1]; }
}