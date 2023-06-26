#include "RandomLaneTracking.hpp"

std::pair<cv::Mat, bool> RandomLaneTracking::track_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame, const LanePoints &lane_points)
{
    original_frame_ = original_frame;
    edited_frame_ = edited_frame;

    const bool draw_left_lane = lane_points.left_top != -1 || lane_points.left_bottom != -1;
    const bool draw_right_lane = lane_points.right_top != -1 || lane_points.right_bottom != -1;

    if (draw_left_lane)
        add_left_average(lane_points.left_top, lane_points.left_bottom);

    if (draw_right_lane)
        add_right_average(lane_points.right_top, lane_points.right_bottom);

    draw_from_average(draw_left_lane, draw_right_lane);

    return std::pair<cv::Mat, bool>{original_frame_, true};
}

void RandomLaneTracking::clear_averages()
{
    for (size_t i = 0; i < average_size; ++i)
    {
        left_average[i].first = 0;
        left_average[i].second = 0;

        right_average[i].first = 0;
        right_average[i].second = 0;
    }
}

void RandomLaneTracking::add_average(std::array<std::pair<int, int>, average_size> &source, const int &x1, const int &x2)
{
    for (size_t i = source.size() - 1, end = 1; i >= end; --i)
    {
        source[i] = source[i - 1];
    }

    source[0] = {x1, x2};
}

void RandomLaneTracking::add_left_average(const int &x1, const int &x2)
{
    add_average(left_average, x1, x2);
}

void RandomLaneTracking::add_right_average(const int &x1, const int &x2)
{
    add_average(right_average, x1, x2);
}

std::pair<int, int> RandomLaneTracking::get_average(const std::array<std::pair<int, int>, average_size> &source)
{
    std::pair<int, int> average;

    int divider = 0;

    for (auto cord : source)
    {
        if (cord.first == 0)
            continue;

        average.first += cord.first;
        average.second += cord.second;
        ++divider;
    }

    if (divider == 0)
        return std::pair<int, int>(0, 0);

    average.first /= divider;
    average.second /= divider;

    return average;
}

std::pair<int, int> RandomLaneTracking::get_left_average()
{
    return get_average(left_average);
}

std::pair<int, int> RandomLaneTracking::get_right_average()
{
    return get_average(right_average);
}

void RandomLaneTracking::draw_from_average(bool draw_left_lane, bool draw_right_lane)
{
    const std::pair<int, int> left = get_left_average();
    const std::pair<int, int> right = get_right_average();

    // if (draw_right_lane && draw_left_lane)
    // {
    //     missing_frames_counter = 0;

    //     if (long_missing_flag)
    //     {
    //         long_missing_flag = false;
    //         return;
    //     }
    // }
    // else if (!draw_right_lane && !draw_left_lane)
    // {
    //     ++missing_frames_counter;
    // }

    // if (missing_frames_counter >= average_size)
    // {
    //     long_missing_flag = true;
    //     clear_averages();
    //     return;
    // }

    draw_points(left, right);
}

void RandomLaneTracking::draw_points(const std::pair<int, int> &left, const std::pair<int, int> &right)
{
    const int left_top = left.first;
    const int left_bottom = left.second;
    const int right_top = right.first;
    const int right_bottom = right.second;

    cv::Mat mask = cv::Mat::zeros(original_frame_.size(), original_frame_.type());

    // Draw lines and fill poly made up of the four points described above if both bools are true
    cv::Mat dst; // Holds blended image

    cv::line(original_frame_, cv::Point(left_top, y_top_), cv::Point(left_bottom, y_bottom_), cv::Scalar(255, 0, 0), 7);

    cv::line(original_frame_, cv::Point(right_top, y_top_), cv::Point(right_bottom, y_bottom_), cv::Scalar(255, 0, 0), 7);

    cv::Point pts[4] = {
        cv::Point(left_top, y_top_),       // Starting point left lane
        cv::Point(left_bottom, y_bottom_), // Ending point left lane
        cv::Point(right_top, y_bottom_),   // Ending point right lane
        cv::Point(right_bottom, y_top_)    // Starting point right lane
    };

    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(235, 229, 52)); // Color is light blue

    // Blend the mask and source image together
    addWeighted(original_frame_, 1, mask, 0.3, 0.0, dst);
}