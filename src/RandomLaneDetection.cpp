#include "RandomLaneDetection.hpp"

#include <vector>
#include <numeric>
#include <random>

#include "constants.hpp"
#include "RandomLineBuilder.hpp"

#define DEBUG_LINE_DET 1

RandomLaneDetection::RandomLaneDetection() {}

LanePoints RandomLaneDetection::find_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame)
{
    original_frame_ = original_frame;
    edited_frame_ = edited_frame;

    populate_lines();

    const std::pair<int, int> left_line = select_best_line(left_lines_);
    const std::pair<int, int> right_line = select_best_line(right_lines_);

    return LanePoints{left_line.first, left_line.second, right_line.first, right_line.second};
}

void RandomLaneDetection::populate_lines()
{
    const int x_center = cst::kVideoWidth / 2;

    cv::imshow("Built Lines", edited_frame_);

    left_lines_ = RandomLineBuilder::build_lines<cst::lines_array_size>(top_left_x_, x_center, bottom_left_x_, x_center);
    right_lines_ = RandomLineBuilder::build_lines<cst::lines_array_size>(x_center, top_right_x_, x_center, bottom_right_x_);

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
        return std::pair<int, int>(-1, -1);

    return lines.at(best_line_index);
}

int RandomLaneDetection::get_line_score(const int x1, const int y1, const int x2, const int y2)
{
    const double length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) + 0.0001;
    double dx = (x2 - x1) / length; // Division by zero safe
    double dy = (y2 - y1) / length;

    int score = 0;

    for (int i = 0, end = static_cast<int>(length); i < end; ++i)
    {
        int x = static_cast<int>(x1 + (dx * i/*  + 0.5 */)); // 0.5 for rounding when truncating
        int y = static_cast<int>(y1 + (dy * i/*  + 0.5 */));

        score += edited_frame_.at<uint8_t>(y, x); // White value higher than 200
    }

    return score;
}