#include "random_lane_detection.hpp"

#include "lane_detection.hpp"

#include <vector>
#include <numeric>
#include <random>

#include "constants.hpp"

#define DEBUG_LINE_DET 1

RandomLaneDetection::RandomLaneDetection() {}

cv::Vec4i RandomLaneDetection::find_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame)
{
    original_frame_ = original_frame;
    edited_frame_ = edited_frame;

    populate_lines();

    const std::pair<int, int> left_line = select_best_line(left_lines_);
    const std::pair<int, int> right_line = select_best_line(right_lines_);

    return cv::Vec4i{left_line.first, left_line.second, right_line.first, right_line.second};
}

void RandomLaneDetection::populate_lines()
{
    const int x_center = cst::kVideoWidth / 2;

    cv::imshow("Built Lines", edited_frame_);

    left_lines_ = get_lines_in_range(top_left_x_, x_center, bottom_left_x_, x_center);
    right_lines_ = get_lines_in_range(x_center, top_right_x_, x_center, bottom_right_x_);

#if DEBUG_LINE_DET

    cv::Mat lanes_frame = original_frame_.clone();

    for (const std::pair<int, int> line : left_lines_)
    {
        cv::line(lanes_frame, cv::Point(line.first, y_top_), cv::Point(line.second, y_bottom_), cv::Scalar(0, 0, 255), 2 /* , cv::LINE_AA */);
    }
    for (const std::pair<int, int> line : right_lines_)
    {
        cv::line(lanes_frame, cv::Point(line.first, y_top_), cv::Point(line.second, y_bottom_), cv::Scalar(0, 0, 255), 2 /* , cv::LINE_AA */);
    }
    cv::imshow("Built Lines", lanes_frame);

#endif
}

std::array<std::pair<int, int>, cst::lines_array_size> RandomLaneDetection::get_lines_in_range(const int top_min, const int top_max, const int bottom_min, const int bottom_max)
{
    constexpr int nr_of_lines = 200;

    std::random_device r;
    std::default_random_engine generator{r()};
    std::uniform_int_distribution<int> top_distribution(top_min, top_max);
    std::uniform_int_distribution<int> bottom_distribution(bottom_min, bottom_max);

    std::array<std::pair<int, int>, cst::lines_array_size> output;

    for (int i = 0; i < nr_of_lines; ++i)
    {
        std::pair<int, int> line;
        line.first = top_distribution(generator);
        line.second = bottom_distribution(generator);

        output.at(i) = line;
    }

    return output;
}

std::pair<int, int> RandomLaneDetection::select_best_line(const std::array<std::pair<int, int>, cst::lines_array_size> &lines)
{
    int best_line_score = -1;
    size_t best_line_index = SIZE_MAX;

    for (size_t i = 0, end = lines.size(); i < end; ++i)
    {
        const std::pair<int, int> line = lines.at(i);

        const int line_score = get_line_score(line.first, y_top_, line.second, y_bottom_);

        if (line_score < best_line_score)
            continue;

        best_line_score = line_score;
        best_line_index = i;
    }

    if (best_line_index == SIZE_MAX)
        return std::pair<int, int>(0, 0);

    return lines.at(best_line_index);
}

int RandomLaneDetection::get_line_score(const int x1, const int y1, const int x2, const int y2)
{
    const double length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) + 0.0001;
    double dx = (x2 - x1) / length; // Division by zero safe
    double dy = (y2 - y1) / length;

    int score = 0;

    for (int i = 0; i < length; ++i)
    {
        int x = static_cast<int>(x1 + (dx * i + 0.5)); // 0.5 for rounding when truncating
        int y = static_cast<int>(y1 + (dy * i + 0.5));
        score += edited_frame_.at<uint8_t>(y, x); // White value higher than 200
    }

    return score;
}