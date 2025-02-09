#include "stereo_odometry_ros/frame.hpp"

namespace stereo_odometry
{
    Frame::Frame() {}

    uint64_t Frame::stamp() { return static_cast<uint64_t>(sec) * SEC_TO_NANOSEC + static_cast<uint64_t>(nanosec); }

    StereoFrame::StereoFrame() {}
}