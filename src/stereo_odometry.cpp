#include "stereo_odometry_ros/stereo_odometry.hpp"
#include <chrono>

namespace stereo_odometry
{
    StereoOdometry::StereoOdometry(const rclcpp::Logger &logger) : logger(logger)
    {
    }

    bool StereoOdometry::is_inside(const cv::Point2f &point, int width, int height)
    {
        return (point.x >= 0 && point.x < width) &&
               (point.y >= 0 && point.y < height);
    }

    void StereoOdometry::track(const Frame &prev,
                               const Frame &next,
                               const std::vector<cv::Point2f> &prev_pts,
                               std::vector<cv::Point2f> &next_pts,
                               std::vector<uchar> &status)
    {
        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(prev.image,
                                 next.image,
                                 prev_pts,
                                 next_pts,
                                 status,
                                 error);
    }

    void StereoOdometry::drawMatches(const cv::Mat &image1,
                                     std::vector<cv::Point2f> &points1,
                                     const cv::Mat &image2,
                                     std::vector<cv::Point2f> &points2,
                                     cv::Mat &out)
    {
        assert(points1.size() == points2.size());
        cv::hconcat(image1, image2, out);
        cv::RNG rng(cv::getTickCount());
        for (size_t i = 0; i < points1.size(); i++)
        {
            int blue = rng.uniform(0, 256);
            int green = rng.uniform(0, 256);
            int red = rng.uniform(0, 256);
            cv::Scalar randomColor(blue, green, red);

            cv::circle(out, points1[i], 4, randomColor);
            cv::Point2f new_point2 = cv::Point2f(points2[i].x + image1.cols, points2[i].y);
            cv::circle(out, new_point2, 4, randomColor);
            cv::line(out, points1[i], new_point2, randomColor);
        }
    }

    void StereoOdometry::computeOdometry(StereoFrame &frame)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<cv::Point2f> left_pts;
        if (this->status == TrackingStatus::INIT)
        {
            cv::goodFeaturesToTrack(frame.left.image, left_pts, 500, 0.01, 10);
            RCLCPP_DEBUG(logger, "DETECTED KPS INIT %d", left_pts.size());
            t = cv::Mat::zeros(3, 1, CV_64F);
            r = cv::Mat::zeros(3, 1, CV_64F);
            frame.pose = cv::Mat::eye(4, 4, CV_64F);

            this->status = TrackingStatus::GOOD;
        }
        else if (this->status == TrackingStatus::GOOD)
        {
            std::vector<cv::Point2f> next_pts;
            // std::vector<cv::Point2f>  prev_pts;
            std::vector<cv::Point3f> prev_points3D;

            std::vector<uchar> status;
            this->track(prev.left,
                        frame.left,
                        prev.points2D,
                        next_pts,
                        status);

            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                {
                    if (this->is_inside(next_pts[i], camera.left().width, camera.left().height))
                    {
                        prev_points3D.emplace_back(prev.points3D[i]);
                        left_pts.emplace_back(next_pts[i]);
                        // prev_pts.emplace_back(prev.points2D[i]);
                    }
                }
            }

            // this->drawMatches(prev.left.image, prev_pts, frame.left.image, left_pts, frame.prev_matches);
            // cv::imwrite("/D/matches/prev/" + std::to_string(counter) + ".png", frame.prev_matches);

            RCLCPP_DEBUG(logger, "TRACKED KPS PREV-NEXT %d", left_pts.size());

            // cv::Mat r = r.clone();
            // cv::Mat t = t.clone();
            cv::Mat r, t;
            std::vector<int> inliers;
            bool solved = cv::solvePnPRansac(prev_points3D,
                                             left_pts,
                                             camera.left().K,
                                             cv::Mat(),
                                             r,
                                             t,
                                             false,
                                             100,
                                             2.0F,
                                             0.99,
                                             inliers,
                                             cv::SOLVEPNP_EPNP);

            if (solved)
            {
                this->r = r;
                this->t = t;
            }
            else
            {
                // frame.pose = prev.pose.clone();
                RCLCPP_DEBUG(logger, "PnP NOT SOLVED! Inliers are %d", inliers.size());
            }

            cv::Mat R, Rt, t_inv;
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            cv::Rodrigues(this->r, R);
            Rt = R.t();
            t_inv = -Rt * this->t;
            Rt.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
            t_inv.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));
            frame.pose = prev.pose * T;

            if (left_pts.size() < 200)
            {
                cv::Mat mask = cv::Mat(frame.left.image.rows, frame.left.image.cols, CV_8UC1, 255);
                for (const auto &p : left_pts)
                {
                    cv::rectangle(mask, cv::Rect(p.x - 7, p.y - 7, 14, 14), 0, cv::FILLED);
                }
                // cv::imwrite("/D/matches/mask/" + std::to_string(counter) + ".png", mask);
                std::vector<cv::Point2f> new_pts;
                cv::goodFeaturesToTrack(frame.left.image, new_pts, 400, 0.01, 10, mask);
                left_pts.insert(left_pts.end(), new_pts.begin(), new_pts.end());
                RCLCPP_DEBUG(logger, "NEW TOTAL KPS %d", left_pts.size());
            }
        }

        std::vector<uchar> status;
        std::vector<cv::Point2f> right_pts, good_left_pts, good_right_pts;
        this->track(frame.left,
                    frame.right,
                    left_pts,
                    right_pts,
                    status);

        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
                if (this->is_inside(right_pts[i], camera.left().width, camera.left().height))
                {
                    good_left_pts.emplace_back(left_pts[i]);
                    good_right_pts.emplace_back(right_pts[i]);
                }
            }
        }

        RCLCPP_DEBUG(logger, "TRACKED KPS LEFT-RIGHT %d", good_right_pts.size());

        // this->drawMatches(frame.left.image, good_left_pts, frame.right.image, good_right_pts, frame.right_matches);
        // cv::imwrite("/D/matches/right/" + std::to_string(counter++) + ".png", frame.right_matches);

        cv::Mat points4D;
        cv::triangulatePoints(camera.left().P,
                              camera.right().P,
                              good_left_pts,
                              good_right_pts,
                              points4D);

        assert(points4D.cols == good_left_pts.size());

        std::vector<cv::Point3f> points3D;
        // points3D.reserve(points4D.cols);
        for (size_t i = 0; i < points4D.cols; i++)
        {
            cv::Mat row = points4D.col(i);
            row /= row.at<float>(3, 0);
            points3D.emplace_back(cv::Point3f(row.at<float>(0, 0), row.at<float>(1, 0), row.at<float>(2, 0)));
        }

        frame.points2D = good_left_pts;
        frame.points3D = points3D;

        RCLCPP_DEBUG(logger, "NEW 3D POINTS %d", points3D.size());

        this->prev = frame;

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        RCLCPP_DEBUG(logger, "compute_odometry took %f second", elapsed.count());
    }
}