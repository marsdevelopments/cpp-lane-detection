#include "random_lane_tracking.hpp"

cv::Vec4i RandomLaneTracking::track_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame, const cv::Vec4i &line_points_x)
{
    original_frame_ = original_frame;
    edited_frame_ = edited_frame;

    const int left_x1 = line_points_x[0]; // left top
    const int left_x2 = line_points_x[1]; // left bottom
    const int right_x1 = line_points_x[2]; // right top
    const int right_x2 = line_points_x[3]; // right bottom

    const bool draw_left_lane = left_x1 != 0 || left_x2 != 0;
    const bool draw_right_lane = right_x1 != 0 || right_x2 != 0;
    
    if (draw_left_lane)
        add_left_average(left_x1, left_x2);

    if (draw_right_lane)
        add_right_average(right_x1, right_x2);

    draw_from_average(draw_left_lane, draw_right_lane, y1, y2);
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

void RandomLaneTracking::draw_points(const int x1,, int y1, int y2)
{
    const int left_x1 = pointsX[0];
    const int left_x2 = pointsX[1];
    const int rightX1 = pointsX[2];
    const int rightX2 = pointsX[3];

    cv::Mat mask = cv::Mat::zeros(original_frame_.size(), original_frame_.type());

    // Draw lines and fill poly made up of the four points described above if both bools are true
    cv::Mat dst; // Holds blended image

    cv::line(original_frame_, cv::Point(rightX1, y1), cv::Point(rightX2, y2), cv::Scalar(255, 0, 0), 7);

    cv::line(original_frame_, cv::Point(left_x1, y1), cv::Point(left_x2, y2), cv::Scalar(255, 0, 0), 7);

    cv::Point pts[4] = {
        cv::Point(left_x1, y1),  // Starting point left lane
        cv::Point(left_x2, y2),  // Ending point left lane
        cv::Point(rightX2, y2), // Ending point right lane
        cv::Point(rightX1, y1)  // Starting point right lane
    };

    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(235, 229, 52)); // Color is light blue

    // Blend the mask and source image together
    addWeighted(original_frame_, 1, mask, 0.3, 0.0, dst);
}

void RandomLaneTracking::draw_from_average(bool draw_left_lane, bool draw_right_lane, int y1, int y2)
{
    const std::pair<int, int> left = get_left_average();
    const std::pair<int, int> right = get_right_average();

    cv::Vec4i pointsX(left.first, left.second, right.first, right.second);

    if (draw_right_lane && draw_left_lane)
    {
        missing_frames_counter = 0;

        if (long_missing_flag)
        {
            long_missing_flag = false;
            return;
        }
    }
    else if (!draw_right_lane && !draw_left_lane)
    {
        ++missing_frames_counter;
    }

    if (missing_frames_counter >= average_size)
    {
        long_missing_flag = true;
        clear_averages();
        return;
    }

    draw_points(pointsX, y1, y2);
}